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

//namespace PIRT {

constexpr unsigned int DEFAULT_PWM_FREQ { 20000 };

class GPIO;
class ADS1115;


class MotorDriver {
  public:
    struct Pins {
      unsigned int Enable { 0 };
      unsigned int Pwm { 0 };
	  unsigned int Dir { 0 };
	  unsigned int Fault { 0 };
    };

    MotorDriver()=delete;

    MotorDriver(std::shared_ptr<GPIO> gpio, Pins pins, std::shared_ptr<ADS1115> adc = nullptr);
	
    ~MotorDriver();

    void setPinConfig(struct PinConfig config);
    PinConfig getPinConfig();
    
    void stopMovement();
    void stop();
    void go(uint8_t speed, bool dir);
    void go(uint16_t dirspeed);
    
    int getErrorFlags();
    bool isADCpresent();
    //ADS1115* ADC();
    
    
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

//} // namespace PIRT

#endif
