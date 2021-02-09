#include <iostream>
#include <stdio.h>
//#include <pigpio.h>

//#include <stdint.h>
#include <unistd.h>
//#include <stdlib.h>
//#include <getopt.h>

#include "gpioif.h"

extern "C" {
#include <pigpiod_if2.h>
}

#define DEFAULT_VERBOSITY 1

//GPIO::fHandle = -1;

GPIO::GPIO(const std::string& host, const std::string& port) 
{
    if (fHandle<0) {
		char* addrStr = const_cast<char*>(host.c_str());
		char* portStr = const_cast<char*>(port.c_str());
//		fHandle = pigpio_start((char*)"127.0.0.1", (char*)"8888");
		fHandle = pigpio_start(addrStr, portStr);
		if (fHandle < 0) {
			std::cerr<<"Could not connect to pigpio daemon. Is pigpiod running?\n";
			return;
		}
	}
}


GPIO::~GPIO() {
    if (fHandle>=0) {
		pigpio_stop(fHandle);
	}
    fHandle = -1;
}


auto GPIO::spi_init(SPI_INTERFACE interface, std::uint8_t channel, SPI_MODE mode, unsigned int baudrate, bool lsb_first) -> int
{
	unsigned int spi_flags = static_cast<unsigned int>(mode) | (static_cast<unsigned int>(lsb_first) << 15);
	if (interface == SPI_INTERFACE::Aux) {
		spi_flags |= 1 << 8;
		std::cout<<"spi flags: "<<spi_flags<<"\n";
	}
	int handle = ::spi_open(fHandle, 0, baudrate, spi_flags);
	if (handle < 0) {
		std::cerr<<"Error opening spi interface.\n";
	}
	return handle;
}

auto GPIO::spi_read(unsigned int spi_handle, unsigned int nBytes) -> std::vector<std::uint8_t>
{
	char rx_buffer[nBytes];
	std::lock_guard<std::mutex> guard(fMutex);
	int count = ::spi_read(fHandle, spi_handle, rx_buffer, nBytes);
	if (count<=0) return std::vector<std::uint8_t> {};
	std::vector<std::uint8_t> data(rx_buffer,rx_buffer+std::min(static_cast<unsigned int>(count),nBytes));
	return data;
}

auto GPIO::spi_write(unsigned int spi_handle, const std::vector<std::uint8_t>& data) -> bool
{
	std::lock_guard<std::mutex> guard(fMutex);
	return false;
}

void GPIO::spi_close(int spi_handle)
{
	::spi_close(fHandle, spi_handle);
}
