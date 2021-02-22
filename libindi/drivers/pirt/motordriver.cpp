#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <memory>
#include <cassert>

#include "motordriver.h"

#define DEFAULT_VERBOSITY 1

//namespace PiRaTe {
	
constexpr std::chrono::milliseconds loop_delay { 10 };
constexpr std::chrono::milliseconds ramp_time { 1000 };
constexpr double ramp_increment { static_cast<double>(loop_delay.count())/ramp_time.count() };
constexpr unsigned int HW_PWM1_PIN { 12 };
constexpr unsigned int HW_PWM2_PIN { 13 };

// helper functions for compilation with c++11
// remove, when compiling with c++14 and add std:: to the lines where these functions are used
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
	: fGpio { gpio }, fPins { pins }, fAdc { adc }
{
	if (fGpio == nullptr) {
		std::cerr<<"Error: no valid GPIO instance.\n";
		return;
	}
	
	if (!fGpio->isInitialized()) {
		std::cerr<<"Error: gpio interface not initialized.\n";
		return;
	}

	if ( ( fPins.Dir < 0) || ( fPins.Pwm < 0 ) ) {
		std::cerr<<"Error: mandatory gpio pins for motor control undefined.\n";
		return;
	}
	
	// set pin directions
	fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Dir), true);
	fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Pwm), true);
	if ( fPins.Enable >= 0 ) {
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Enable), true);
	}
	if ( fPins.Fault >= 0 ) {
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Fault), false);
		fGpio->set_gpio_pullup(static_cast<unsigned int>(fPins.Fault));
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
	if (!fActiveLoop) return;
	fActiveLoop = false;
	if (fThread!=nullptr) fThread->join();
	if (fGpio != nullptr && fGpio->isInitialized()) {
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Dir), false);
		fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Pwm), false);
		if ( fPins.Enable >= 0 ) {
			fGpio->set_gpio_direction(static_cast<unsigned int>(fPins.Enable), false);
		}
		if ( fPins.Fault >= 0 ) {
			fGpio->set_gpio_pullup(static_cast<unsigned int>(fPins.Fault), false);
		}
	}
}


// this is the background thread loop
void MotorDriver::threadLoop()
{
	auto lastReadOutTime = std::chrono::system_clock::now();
	bool errorFlag = true;
	while (fActiveLoop) {
		auto currentTime = std::chrono::system_clock::now();
		
		if (hasFaultSense() && isFault()) {
			// fault condition, switch off and deactivate everything
			emergencyStop();
		} else {
			fMutex.lock();
			if (fTargetDutyCycle != fCurrentDutyCycle) {
				setSpeed(fCurrentDutyCycle);
				fCurrentDutyCycle += ramp_increment * sgn( fTargetDutyCycle - fCurrentDutyCycle);
			}
			if ( std::abs(fCurrentDutyCycle) > std::abs(fTargetDutyCycle) ) {
				fCurrentDutyCycle = fTargetDutyCycle;
			}
			fMutex.unlock();
		}
		if ( hasAdc() ) {
			// read current from adc
			fUpdated = true;
		}
		std::this_thread::sleep_for(loop_delay);
	}
}

auto MotorDriver::isFault() -> bool {
	if (!isInitialized() || !fGpio->isInitialized()) return true;
	if ( !hasFaultSense() ) return false;
	return (fGpio->get_gpio_state(static_cast<unsigned int>(fPins.Fault), nullptr) == false);
}

void MotorDriver::setSpeed(float speed_ratio) {
	bool dir = (speed_ratio < 0);
	// set pins
	fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Dir), dir);
	speed_ratio = std::abs(clamp(speed_ratio, -1.f, 1.f));
	std::uint32_t duty_cycle { 0 };
	if ( fPins.Pwm == HW_PWM1_PIN || fPins.Pwm == HW_PWM2_PIN ) {
		// use hardware pwm
		duty_cycle = speed_ratio * 1000000U;
		int res = fGpio->hw_pwm_set_value(static_cast<unsigned int>(fPins.Pwm), fPwmFreq, duty_cycle);
		return;
	}
	duty_cycle = speed_ratio * fPwmRange;
	int res = fGpio->pwm_set_value(static_cast<unsigned int>(fPins.Pwm), duty_cycle);
}

void MotorDriver::move(float speed_ratio) {
	fTargetDutyCycle = clamp(speed_ratio, -1.f, 1.f);
	if (hasEnable()) {
		fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Enable), true);
	}
}

void MotorDriver::stop() {
	this->move(0.f);
}

void MotorDriver::emergencyStop() {
	fTargetDutyCycle = 0.;
	if (hasEnable()) {
		fGpio->set_gpio_state(static_cast<unsigned int>(fPins.Enable), false);
	}
}

void MotorDriver::setPwmFrequency(unsigned int freq)
{ 
	if (freq == fPwmFreq) return;
	if ( !(fPins.Pwm == HW_PWM1_PIN || fPins.Pwm == HW_PWM2_PIN) ) {
		int res = fGpio->pwm_set_frequency(static_cast<unsigned int>(fPins.Pwm), freq);
	}
	fPwmFreq = freq;
}

auto MotorDriver::currentSpeed() const -> float {
	return fCurrentDutyCycle;
}

//} // namespace PiRaTe
