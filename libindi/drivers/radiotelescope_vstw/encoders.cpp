#include <iostream>
#include <stdio.h>
//#include <pigpio.h>

//#include <stdint.h>
#include <unistd.h>
//#include <stdlib.h>
//#include <getopt.h>

#include "encoders.h"

#define DEFAULT_VERBOSITY 1
#define DEFAULT_LD_GPIO 5
//#define SPI_BAUD_DEFAULT 500000UL


unsigned int SsiPosEncoder::fNrInstances = 0;

uint32_t gray_decode(uint32_t g)
{
    for (uint32_t bit = 1U << 31; bit > 1; bit >>= 1)
    {
        if (g & bit) g ^= bit >> 1;
    }
    return g;
}

SsiPosEncoder::SsiPosEncoder(std::shared_ptr<GPIO> gpio, GPIO::SPI_INTERFACE spi_interface, unsigned int baudrate, std::uint8_t spi_channel,  GPIO::SPI_MODE spi_mode)
	: fGpio(gpio)
{
	if (fGpio == nullptr) return;
	fSpiHandle = fGpio->spi_init(spi_interface, spi_channel, spi_mode, baudrate);
	if (fSpiHandle < 0) {
		std::cerr<<"Error opening spi interface.\n";
		return;
	}

	fActiveLoop=true;
	fThread = std::make_unique<std::thread>( [this]() { this->readLoop(); } );
	fNrInstances++;
}

SsiPosEncoder::~SsiPosEncoder()
{
  fActiveLoop = false;
  if (fThread!=nullptr) fThread->join();
  fNrInstances--;
  // close SPI device
  if ( fSpiHandle>=0 && fGpio != nullptr ) fGpio->spi_close(fSpiHandle);
}


// this is the background thread loop
void SsiPosEncoder::readLoop()
{
	while (fActiveLoop) {
		uint32_t data;
		bool ok = readDataWord(data);
		if (ok) {
			std::uint32_t st = (data >> 7) & 0b11111111111111111111111;
			st = gray_decode(st);
			st = st & 0b111111111111;
			//std::cout<<" ST="<<st;
		
			std::int32_t mt = (data >> 19) & 0b11111111111;
			mt = gray_decode(mt);
			// add sign bit to MT value
			// negative counts have to be offset by -1. Otherwise one had to 
			// distinguish between -0 and +0 rotations
			if ( data & (1<<30) ) mt = -mt-1;
			//std::cout<<" MT="<<mt;

			std::lock_guard<std::mutex> guard(fMutex);
			fPos = st;
			fTurns = mt;
			fUpdated = true;
		}
		usleep(100000UL);
	}
}


bool SsiPosEncoder::readDataWord(uint32_t& data)
{
	if (fSpiHandle < 0 && fGpio == nullptr) return false;
	constexpr unsigned int nBytes = 4;
	std::vector<std::uint8_t> bytevec = fGpio->spi_read(fSpiHandle, nBytes);
	if (bytevec.size() != nBytes) {
		std::cout<<"error reading correct number of bytes from encoder.\n";
		return false;
	}
	data = bytevec[3] | (bytevec[2]<<8) | (bytevec[1]<<16) | (bytevec[0]<<24);
	return true;
}



