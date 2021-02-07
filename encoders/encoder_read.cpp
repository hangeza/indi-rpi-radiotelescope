/* simple program to read out absolute position encoder via SSI/SPI interface
 * compile with:
 * g++ -Wall -pthread -o encoder_read encoder_read.cpp -lpigpio -lrt
 */


#include <iostream>
#include <pigpio.h>
#include <string>
#include <cstdlib>
#include <unistd.h>

constexpr unsigned int CE0 { 8 };
constexpr unsigned int CE1 { 18 };
constexpr unsigned int CLK1 { 11 };
constexpr unsigned int CLK2 { 21 };
constexpr unsigned int DATA1 { 9 };
constexpr unsigned int DATA2 { 19 };

constexpr unsigned int baud_rate { 100000 };

std::string intToBinaryString(unsigned long number) {
   std::string numStr { };
   for (int i=31; i>=0; i--) {
      numStr += (number & (1<<i))?"1":"0";
   }
  return numStr;
}

int main(void) {
	int ret = gpioInitialise();
	if (ret < 0) {
		std::cout<<"Error opening pigpio interface.\n";
		return -1;
	}

	constexpr std::uint8_t spi_mode = 0x03;
	constexpr std::uint8_t spi_lsb_first = 0x00;
	unsigned int spi_flags = (spi_mode) | (spi_lsb_first << 15);
	spi_flags |= 0;

	int handle1 = spiOpen(0, 1000000UL, spi_flags);
	if (handle1 < 0) {
		std::cout<<"Error opening spi interface.\n";
		return -1;
	}

   
	//int handle2 = bbSPIOpen(CE1, DATA2, 20, CLK2, 250000, spi_flags);
	int handle2 = spiOpen(0, 1000000UL, spi_flags | (1<<8));
	if (handle2 < 0) {
		std::cout<<"Error opening aux spi interface.\n";
		return -1;
	}
   
   
	char tx_buffer[] = { 0,0,0,0 };
	char rx_buffer[] = { 0,0,0,0 };


	int iteration = 0;
	while (iteration++ < 1000) {
		int count = spiRead(handle1, rx_buffer, 4);
		unsigned long dataword = rx_buffer[0] | 
			(rx_buffer[1]<<8) |
			(rx_buffer[2]<<16) |
			(rx_buffer[3]<<24);

		std::cout<<"Az: "<<intToBinaryString(dataword);
	 
		count = spiRead(handle2, rx_buffer, 4);
		//count = bbSPIXfer(CE1, tx_buffer, rx_buffer, 4);
		dataword = rx_buffer[0] | 
			(rx_buffer[1]<<8) |
			(rx_buffer[2]<<16) |
			(rx_buffer[3]<<24);
	 
		std::cout<<"  El: "<<intToBinaryString(dataword);
	  
		std::cout<<"\n";

		usleep(100000UL);

      /*
      std::cout<<"received "<<count<<" bytes.\n";
      std::cout<<"data: ";
      
      for (int i=0; i<4; i++) {
         std::cout<<static_cast<int>(rx_buffer[i])<<" ";
      }
      std::cout<<"\n";
      */
	}
	spiClose(handle1);
	spiClose(handle2);
	//bbSPIClose(CE1);
	gpioTerminate();
}

