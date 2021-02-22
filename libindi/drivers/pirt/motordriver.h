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

//namespace PiRaTe {

constexpr unsigned int DEFAULT_PWM_FREQ { 20000 };

class GPIO;
class ADS1115;


class MotorDriver {
public:
	struct Pins {
		Pins(int pwm, int dir, int enable=-1, int fault=-1)
			: Enable { enable }, Pwm { pwm }, Dir { dir }, Fault { fault }
			{}
		int Enable { -1 };
		int Pwm { -1 };
		int Dir { -1 };
		int Fault { -1 };
    };

    MotorDriver()=delete;

    MotorDriver(std::shared_ptr<GPIO> gpio, Pins pins, std::shared_ptr<ADS1115> adc = nullptr);
	
    ~MotorDriver();

    void setPinConfig(Pins pins);
    [[nodiscard]] auto getPinConfig() const -> Pins;
	void setPwmFrequency(unsigned int freq);
	
	void move(float speed_ratio);    
    void stop();
    void emergencyStop();
	[[nodiscard]] auto isFault() -> bool;
    [[nodiscard]] auto isInitialized() const -> bool { return fActiveLoop; }
    [[nodiscard]] auto currentSpeed() const -> float;
	[[nodiscard]] auto hasFaultSense() const -> bool { return (fPins.Fault >= 0); }
	[[nodiscard]] auto hasEnable() const -> bool { return (fPins.Enable >= 0); }
    [[nodiscard]] auto hasAdc() const -> bool { return (fAdc != nullptr); }
  private:
    void threadLoop();
	void setSpeed(float speed_ratio);
    
	std::shared_ptr<GPIO> fGpio { nullptr };
    Pins fPins;
    std::shared_ptr<ADS1115> fAdc { nullptr };
	unsigned int fPwmFreq { DEFAULT_PWM_FREQ };
	unsigned int fPwmRange { 255 };
	
	bool fUpdated { false };
	float fCurrentDutyCycle { 0 };
	float fTargetDutyCycle { 0 };
    bool fActiveLoop { false };

    std::unique_ptr<std::thread> fThread { nullptr };

	std::mutex fMutex;

};

//} // namespace PiRaTe

#endif
