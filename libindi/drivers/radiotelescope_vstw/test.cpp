/* simple program to read out absolute position encoder via SSI/SPI interface
 * compile with:
 g++ -std=gnu++14 -Wall -pthread -c gpioif.cpp
 g++ -std=gnu++14 -Wall -pthread -c encoders.cpp
 g++ -std=gnu++14 -Wall -pthread -c test.cpp
 g++ -Wall -pthread -o test gpioif.o encoders.o test.o -lpigpiod_if2 -lrt
 */

#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>

#include "gpioif.h"
#include "encoders.h"

constexpr unsigned int CE0 { 8 };
constexpr unsigned int CE1 { 18 };
constexpr unsigned int CLK1 { 11 };
constexpr unsigned int CLK2 { 21 };
constexpr unsigned int DATA1 { 9 };
constexpr unsigned int DATA2 { 19 };

constexpr unsigned int baud_rate { 500000 };

std::string intToBinaryString(unsigned long number) {
   std::string numStr { };
   for (int i=31; i>=0; i--) {
      numStr += (number & (1<<i))?"1":"0";
   }
  return numStr;
}
/*
uint32_t gray_decode(uint32_t g)
{
    for (uint32_t bit = 1U << 31; bit > 1; bit >>= 1)
    {
        if (g & bit) g ^= bit >> 1;
    }
    return g;
}
*/

int main(void) {
    std::shared_ptr<GPIO> gpio(new GPIO("localhost"));
	
	if (!gpio->isInitialized()) {
        std::cerr<<"Could not connect to pigpio daemon. Is pigpiod running?\n";
        return -1;
    }

	SsiPosEncoder az_encoder(gpio, GPIO::SPI_INTERFACE::Main, baud_rate);
	
	int nIter = 100;
	while (nIter > 0) {
		if (az_encoder.isUpdated()) {
			unsigned int pos = az_encoder.position();
			std::cout<<"Az: "<<pos<<"\n";
			nIter--;
		}
		usleep(1000U);
	}	
	
/*
	int spi1_handle = gpio->spi_init(GPIO::SPI_INTERFACE::Main, 0, GPIO::SPI_MODE::POL1PHA1, baud_rate);
	if (spi1_handle < 0) {
		std::cerr<<"Error opening spi interface.\n";
		return -1;
	}
	
	int nIter = 10;
	while (nIter-- > 0) {
		std::vector<std::uint8_t> data = gpio->spi_read(spi1_handle, 4);

		if (data.size() != 4) {
			std::cout<<"error reading correct number of bytes from spi.\n";
			return -1;
		}
	
		unsigned long dataword = data[3] | (data[2]<<8) | (data[1]<<16) | (data[0]<<24);

		std::cout<<"Az: "<<intToBinaryString(dataword);
	
		std::cout<<"\n";
	}
*/
}

