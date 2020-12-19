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

#include <pigpio.h>

class GPIO;
class ADS1115;


class motordriver : public GPIO {
  public:
    struct PinConfig {
      int gpio_pin_A1, gpio_pin_B1, 
	  gpio_pin_A2, gpio_pin_B2,
	  gpio_pin_pwm1, gpio_pin_pwm2,
	  gpio_pin_diag1, gpio_pin_diag2;
    };

    motordriver();
    motordriver(GPIO* gpio) : fGpio(gpio) {}
    motordriver(GPIO* gpio, struct PinConfig config) 
    : fGpio(gpio), fPinConfig(config)
    { }
    ~motordriver();

    void setPinConfig(struct PinConfig config);
    PinConfig getPinConfig();
    
    void stopMovement();
    void stop();
    void go(uint8_t speed, bool dir);
    void go(uint16_t dirspeed);
    
    int getErrorFlags();
    bool isADCpresent();
    ADS1115* ADC();
    
    
  private:
    void listenLoop();
    
    std::thread* fThread;
    GPIO* fGpio;
    PinConfig fPinConfig;
    ADS1115* fAdc;
};


#endif
