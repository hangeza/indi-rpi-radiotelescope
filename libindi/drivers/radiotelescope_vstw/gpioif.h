#ifndef GPIORT_H
#define GPIORT_H


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

class GPIO {
  public:

    GPIO();
    ~GPIO();

    virtual bool isInitialized() const { return (fHandle>=0); }
    
  protected:
    static int fHandle;
};


#endif
