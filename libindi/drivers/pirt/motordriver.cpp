#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <memory>
#include <cassert>

#include "motordriver.h"

#define DEFAULT_VERBOSITY 1

//namespace PIRT {
	
constexpr std::chrono::milliseconds loop_delay { 10 };
constexpr std::chrono::milliseconds ramp_time { 1000 };
constexpr double ramp_increment { static_cast<double>(loop_delay.count())/ramp_time.count() };
constexpr unsigned int HW_PWM1_PIN { 12 };
constexpr unsigned int HW_PWM2_PIN { 13 };

template<class T>
const T& clamp( const T& v, const T& lo, const T& hi )
{
    assert( !(hi < lo) );
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

template <typename T> constexpr int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

MotorDriver::MotorDriver(std::shared_ptr<GPIO> gpio, Pins pins, std::shared_ptr<ADS1115> adc)
	: fGpio { std::move(gpio) }, fPins { pins }, fAdc { std::move(adc) }
{
	if (fGpio == nullptr) {
		std::cerr<<"Error: no valid GPIO instance.\n";
		return;
	}
	
	fActiveLoop=true;
// since C++14 using std::make_unique
	// fThread = std::make_unique<std::thread>( [this]() { this->readLoop(); } );
// C++11 is unfortunately more unconvenient with move from a locally generated pointer
	std::unique_ptr<std::thread> thread( new std::thread( [this]() { this->threadLoop(); } ));
	fThread = std::move(thread);
}

MotorDriver::~MotorDriver()
{
  fActiveLoop = false;
  if (fThread!=nullptr) fThread->join();
}


// this is the background thread loop
void MotorDriver::threadLoop()
{
	auto lastReadOutTime = std::chrono::system_clock::now();
	bool errorFlag = true;
	while (fActiveLoop) {
		auto currentTime = std::chrono::system_clock::now();
		if (fTargetDutyCycle != fCurrentDutyCycle) {
			setSpeed(fCurrentDutyCycle);
			fCurrentDutyCycle += ramp_increment * sgn( fTargetDutyCycle - fCurrentDutyCycle);
		}
		if ( std::abs(fCurrentDutyCycle) > std::abs(fTargetDutyCycle) ) {
			fCurrentDutyCycle = fTargetDutyCycle;
		}
		
		
		if (errorFlag) {
			std::this_thread::sleep_for(loop_delay);
			errorFlag=false;
			continue;
		}
			
		fMutex.lock();
		fUpdated = true;
		fMutex.unlock();
		std::this_thread::sleep_for(loop_delay);
	}
}

void MotorDriver::setSpeed(float speed_ratio) {
	bool dir = (speed_ratio >= 0);
	speed_ratio = std::abs(clamp(static_cast<double>(speed_ratio), -1., 1.));
	std::uint32_t duty_cycle { 0 };
	if ( fPins.Pwm == HW_PWM1_PIN || fPins.Pwm == HW_PWM2_PIN ) {
		// use hardware pwm
		duty_cycle = speed_ratio * 1000000U;
		int res = fGpio->hw_pwm_set_value(fPins.Pwm, fPwmFreq, duty_cycle);
		return;
	}
	duty_cycle = speed_ratio * fPwmRange;
	int res = fGpio->pwm_set_value(fPins.Pwm, duty_cycle);
}

//} // namespace PIRT
