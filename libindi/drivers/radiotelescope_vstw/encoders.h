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

#include "gpioif.h"

constexpr unsigned int SPI_BAUD_DEFAULT { 500000U };

class GPIO;

class SsiPosEncoder {
  public:
    SsiPosEncoder()=delete;
    SsiPosEncoder(std::shared_ptr<GPIO> gpio, 
				  GPIO::SPI_INTERFACE spi_interface,
				  unsigned int baudrate = SPI_BAUD_DEFAULT,
				  std::uint8_t spi_channel = 0,  
				  GPIO::SPI_MODE spi_mode = GPIO::SPI_MODE::POL1PHA1);
    ~SsiPosEncoder();

    bool isInitialized() const { return (fSpiHandle>=0); }
    
    int position() { fUpdated=false; return fPos; }
    int nrTurns() { fUpdated=false; return fTurns; }
    
    [[nodiscard]] auto isUpdated() const -> bool { return fUpdated; }
    
  private:
    void readLoop();
    
    int fSpiHandle { -1 };
    int fNrBitsPerRev { 12 };
    int fNrBitsTurns { 12 };
    unsigned int fPos { 0 };
    int fTurns { 0 };
	bool fUpdated { false };
    bool fActiveLoop { false };
    
    static unsigned int fNrInstances;
    std::unique_ptr<std::thread> fThread { nullptr };
	std::shared_ptr<GPIO> fGpio { nullptr };

	std::mutex fMutex;
	
	bool readDataWord(uint32_t& data);
};


#endif
