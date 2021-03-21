#ifndef MOTORDRIVERRT_H
#define MOTORDRIVERRT_H


#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <utility>
#include <inttypes.h>  // uint8_t, etc
#include <string>
#include <thread>
#include <chrono>
#include <queue>
#include <list>
#include <mutex>

#include "gpioif.h"
#include "utility.h"

class GPIO;
class ADS1115;

namespace PiRaTe {

constexpr unsigned int DEFAULT_PWM_FREQ { 20000 };

//template <typename T, std::size_t N>
//class Ringbuffer;

class MotorDriver {
public:
	struct Pins {
		int Pwm;
		int Dir;
		int DirA;
		int DirB;
		int Enable;
		int Fault;
    };

    MotorDriver()=delete;

    MotorDriver( std::shared_ptr<GPIO> gpio, Pins pins,
				 bool invertDirection=false, 
				 std::shared_ptr<ADS1115> adc = nullptr, 
				 std::uint8_t adc_channel = 0   );
	
    ~MotorDriver();

    [[nodiscard]] auto getPinConfig() const -> Pins { return fPins; }
	void setPwmFrequency(unsigned int freq);
	
	void move(float speed_ratio);    
    void stop();
    void emergencyStop();
	[[nodiscard]] auto isFault() -> bool;
    [[nodiscard]] auto isInitialized() const -> bool { return fActiveLoop; }
    [[nodiscard]] auto currentSpeed() -> float;
	[[nodiscard]] auto hasFaultSense() const -> bool { return (fPins.Fault > 0); }
	[[nodiscard]] auto hasEnable() const -> bool { return (fPins.Enable > 0); }
	[[nodiscard]] auto hasDualDir() const -> bool { return ( (fPins.DirA > 0) && (fPins.DirB > 0)); }
    [[nodiscard]] auto hasAdc() const -> bool { return (fAdc != nullptr); }
    [[nodiscard]] auto readCurrent() -> double;
  private:
    void threadLoop();
	void setSpeed(float speed_ratio);
    void measureVoltageOffset();
	
	std::shared_ptr<GPIO> fGpio { nullptr };
    Pins fPins;
    std::shared_ptr<ADS1115> fAdc { nullptr };
	unsigned int fPwmFreq { DEFAULT_PWM_FREQ };
	unsigned int fPwmRange { 255 };
	bool fUpdated { false };
	float fCurrentDutyCycle { 0. };
	float fTargetDutyCycle { 0. };
    bool fActiveLoop { false };
	bool fCurrentDir { false };
	bool fInverted { false };
	std::uint8_t fAdcChannel { 0 };
	//double fVoltageOffset { 0. };
	double fCurrent { 0. };

    std::unique_ptr<std::thread> fThread { nullptr };

	std::mutex fMutex;
	
	Ringbuffer<double, 10> fOffsetBuffer { };

};

} // namespace PiRaTe

#endif
