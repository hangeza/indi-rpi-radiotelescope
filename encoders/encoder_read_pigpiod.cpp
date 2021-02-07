/* simple program to read out absolute position encoder via SSI/SPI interface
 * compile with:
 * g++ -Wall -pthread -o encoder_read_pigpiod encoder_read_pigpiod.cpp -lpigpiod_if2 -lrt
 */


#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>

extern "C" {
#include <pigpiod_if2.h>
}

constexpr unsigned int CE0 { 8 };
constexpr unsigned int CE1 { 18 };
constexpr unsigned int CLK1 { 11 };
constexpr unsigned int CLK2 { 21 };
constexpr unsigned int DATA1 { 9 };
constexpr unsigned int DATA2 { 19 };

constexpr unsigned int baud_rate { 1000000 };

std::string intToBinaryString(unsigned long number) {
   std::string numStr { };
   for (int i=31; i>=0; i--) {
      numStr += (number & (1<<i))?"1":"0";
   }
  return numStr;
}

uint32_t gray_decode(uint32_t g)
{
    for (uint32_t bit = 1U << 31; bit > 1; bit >>= 1)
    {
        if (g & bit) g ^= bit >> 1;
    }
    return g;
}


int main(void) {
    int pi = pigpio_start((char*)"127.0.0.1", (char*)"8888");
    if (pi < 0) {
        std::cerr<<"Could not connect to pigpio daemon. Is pigpiod running?\n";
        return -1;
    }

	constexpr std::uint8_t spi_mode = 0x03;
	constexpr std::uint8_t spi_lsb_first = 0x00;
	unsigned int spi_flags = (spi_mode) | (spi_lsb_first << 15);
	spi_flags |= 0;

	int handle1 = spi_open(pi, 0, baud_rate, spi_flags);
	if (handle1 < 0) {
		std::cout<<"Error opening spi interface.\n";
		return -1;
	}

	int handle2 = spi_open(pi, 0, 1000000UL, spi_flags | (1<<8));
	if (handle2 < 0) {
		std::cout<<"Error opening aux spi interface.\n";
		return -1;
	}
   
	char rx_buffer[] = { 0,0,0,0 };

	int iteration = 0;
	while (iteration++ < 1000) {
		int count = spi_read(pi, handle1, rx_buffer, 4);
		unsigned long dataword = rx_buffer[3] | 
			(rx_buffer[2]<<8) |
			(rx_buffer[1]<<16) |
			(rx_buffer[0]<<24);

		std::cout<<"Az: "<<intToBinaryString(dataword);
	 
		count = spi_read(pi, handle2, rx_buffer, 4);
		dataword = rx_buffer[3] | 
			(rx_buffer[2]<<8) |
			(rx_buffer[1]<<16) |
			(rx_buffer[0]<<24);
	 
		std::cout<<"  El: "<<intToBinaryString(dataword);

		std::uint32_t st = (dataword >> 6) & 0b11111111111111;
		st = gray_decode(st);
		std::cout<<" ST="<<st; 
	    
		std::uint32_t mt = (dataword >> 20);
		mt = gray_decode(mt);

		std::cout<<" MT="<<mt; 
		std::cout<<"\n";

		usleep(250000UL);

      /*
      std::cout<<"received "<<count<<" bytes.\n";
      std::cout<<"data: ";
      
      for (int i=0; i<4; i++) {
         std::cout<<static_cast<int>(rx_buffer[i])<<" ";
      }
      std::cout<<"\n";
      */
	}
	spi_close(pi, handle1);
	spi_close(pi, handle2);
	pigpio_stop(pi);
}

