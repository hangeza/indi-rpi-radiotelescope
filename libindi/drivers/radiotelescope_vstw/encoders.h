#ifndef ENCODERSRT_H
#define ENCODERSRT_H


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

class encoder : public GPIO {
  public:
    encoder()=delete;
    explicit encoder(int spiChannel, int nrBitsPerRev, int nrBitsTurns);
    ~encoder();

//     void setPinConfig(struct PinConfig config);
//     PinConfig getPinConfig();
    
    void init();

    virtual bool isInitialized() const { return (fSPIHandle>=0); }
    
    int position() const { return fPos; }
    int nrTurns() const { return fTurns; }
    
    
  private:
    void readLoop();
    
    static int fSPIHandle;
    int fChannel;
    int fNrBitsPerRev;
    int fNrBitsTurns;
    int fPos;
    int fTurns;
    bool fActiveLoop;
    
    static unsigned int fNrInstances;
    std::thread* fThread;
//     PinConfig fPinConfig;
    
    int readDataWord(uint32_t& data);
    bool read(char* buf, int nBytes);
};


#endif
