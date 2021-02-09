#ifndef GPIO_H
#define GPIO_H

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

class GPIO {
  public:

	enum class SPI_INTERFACE {
		Main, Aux
	};
	
	enum class SPI_MODE : std::uint8_t {
		POL0PHA0=0, POL0PHA1=1, POL1PHA0=2, POL1PHA1=3
	};
	
	GPIO() = delete;
	GPIO(const std::string& host, const std::string& port = "8888");
    virtual ~GPIO();

    virtual bool isInitialized() const { return (fHandle>=0); }
    [[nodiscard]] auto spi_init(SPI_INTERFACE interface, std::uint8_t channel, SPI_MODE mode, unsigned int baudrate, bool lsb_first = 0) -> int;
	[[nodiscard]] auto spi_read(unsigned int spi_handle, unsigned int nBytes) -> std::vector<std::uint8_t>;
	[[nodiscard]] auto spi_write(unsigned int spi_handle, const std::vector<std::uint8_t>& data) -> bool;
    void spi_close(int spi_handle);
	int handle() const { return fHandle; }
  protected:
    int fHandle { -1 };
	std::mutex fMutex;
};


#endif
