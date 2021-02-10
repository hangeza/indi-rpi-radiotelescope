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

    [[nodiscard]] auto isInitialized() const -> bool { return (fSpiHandle>=0); }
    
    [[nodiscard]] auto position() -> unsigned int { fUpdated=false; return fPos; }
    [[nodiscard]] auto nrTurns() -> int { fUpdated=false; return fTurns; }
    
    [[nodiscard]] auto isUpdated() const -> bool { return fUpdated; }
    void setStBitWidth(std::uint8_t st_bits) { fStBits = st_bits; }
    void setMtBitWidth(std::uint8_t mt_bits) { fMtBits = mt_bits; }
    [[nodiscard]] auto bitErrorCount() const -> unsigned long { return fBitErrors; }
    
  private:
    void readLoop();
	auto readDataWord(std::uint32_t& data) -> bool;
	[[nodiscard]] auto gray_decode(std::uint32_t g) -> std::uint32_t;
    [[nodiscard]] auto intToBinaryString(unsigned long number) -> std::string;

    int fSpiHandle { -1 };
    std::uint8_t fStBits { 12 };
    std::uint8_t fMtBits { 12 };
    unsigned int fPos { 0 };
    int fTurns { 0 };
	unsigned int fLastPos { 0 };
	unsigned int fLastTurns { 0 };
	unsigned long fBitErrors { 0 };
	bool fUpdated { false };
    bool fActiveLoop { false };
    
    static unsigned int fNrInstances;
    std::unique_ptr<std::thread> fThread { nullptr };
	std::shared_ptr<GPIO> fGpio { nullptr };

	std::mutex fMutex;
};


#endif
