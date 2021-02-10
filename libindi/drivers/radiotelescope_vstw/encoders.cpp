#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string>
#include <chrono>

#include "encoders.h"

#define DEFAULT_VERBOSITY 1
#define DEFAULT_LD_GPIO 5


unsigned int SsiPosEncoder::fNrInstances = 0;
constexpr unsigned int LOOP_DELAY_MS { 10 };

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

auto SsiPosEncoder::intToBinaryString(unsigned long number) -> std::string 
{
   std::string numStr { };
   for (int i=31; i>=0; i--) {
      numStr += (number & (1<<i))?"1":"0";
   }
  return numStr;
}

auto SsiPosEncoder::gray_decode(std::uint32_t g) -> std::uint32_t
{
    for (std::uint32_t bit = 1U << 31; bit > 1; bit >>= 1)
    {
        if (g & bit) g ^= bit >> 1;
    }
    return g;
}

SsiPosEncoder::SsiPosEncoder(std::shared_ptr<GPIO> gpio, GPIO::SPI_INTERFACE spi_interface, unsigned int baudrate, std::uint8_t spi_channel,  GPIO::SPI_MODE spi_mode)
	: fGpio { std::move(gpio) } 
{
	if (fGpio == nullptr) {
		std::cerr<<"Error: no valid GPIO instance.\n";
		return;
	}
//	std::cout<<"pi handle="<<fGpio->handle()<<"\n";
	fSpiHandle = fGpio->spi_init(spi_interface, spi_channel, spi_mode, baudrate);
//	std::cout<<"spi handle="<<fSpiHandle<<"\n";
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
	bool errorFlag = true;
	while (fActiveLoop) {
		uint32_t data;
		bool ok = readDataWord(data);
		if (ok) {
//			std::cout<<" raw: "<<intToBinaryString(data)<<"\n";
			std::uint32_t temp = data >> (32 - fStBits - fMtBits - 1);
			temp &= (1 << (fStBits + fMtBits - 1))-1;
			temp = gray_decode(temp);
			std::uint32_t st = temp & ((1 << (fStBits)) - 1);
//			std::cout<<" st: "<<intToBinaryString(st)<<"\n";
			
			std::int32_t mt = (temp >> fStBits) & ((1 << (fMtBits)) - 1);
//			std::cout<<" mt: "<<intToBinaryString(mt)<<"\n";
			
			// add sign bit to MT value
			// negative counts have to be offset by -1. Otherwise one had to 
			// distinguish between -0 and +0 rotations
			if ( data & (1<<30) ) mt = -mt-1;
			//std::cout<<" MT="<<mt;
			
			if (errorFlag) {
				fLastPos=st; fLastTurns=mt;
				std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_DELAY_MS/2));
				errorFlag=false;
				continue;
			}
			
			int posDiff = st - fLastPos;
			int turnDiff = mt - fLastTurns;
			if (std::abs(posDiff) > (1<<(fStBits-1))) {
				posDiff -= sgn(posDiff)*(1<<(fStBits-1));
			}
			
			
//			if (std::abs(posDiff) > (1<<(fStBits-2)) && std::abs(turnDiff) > 1 ) {
			if ( ( std::abs(turnDiff) > 1 ) ) {
				//std::cout<<" st diff: "<<posDiff<<"\n";
				fBitErrors++;
				errorFlag = true;
				std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_DELAY_MS/2));
				continue;
			}
			fLastPos=st; fLastTurns=mt;
						
			//std::lock_guard<std::mutex> guard(fMutex);
			fMutex.lock();
			fPos = st;
			fTurns = mt;
			fUpdated = true;
			fMutex.unlock();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(LOOP_DELAY_MS));
		//usleep(LOOP_DELAY_MS*1000U);
	}
}


auto SsiPosEncoder::readDataWord(std::uint32_t& data) -> bool
{
	if (fSpiHandle < 0 || fGpio == nullptr) return false;
	constexpr unsigned int nBytes = 4;
	std::vector<std::uint8_t> bytevec = fGpio->spi_read(fSpiHandle, nBytes);
	if (bytevec.size() != nBytes) {
		std::cout<<"error reading correct number of bytes from encoder.\n";
		return false;
	}
	data = bytevec[3] | (bytevec[2]<<8) | (bytevec[1]<<16) | (bytevec[0]<<24);
	return true;
}



