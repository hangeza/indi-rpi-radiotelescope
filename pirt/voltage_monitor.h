#ifndef VOLTAGEMONITOR_H
#define VOLTAGEMONITOR_H


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
#include <functional>

#include "gpioif.h"
#include "utility.h"

class ADS1115;

namespace PiRaTe {

class VoltageMonitor {
public:
    VoltageMonitor()=delete;

    VoltageMonitor( std::shared_ptr<ADS1115> adc,
					std::uint8_t adc_channel,
					double nominal_voltage = 0.,
					double divider_ratio = 1.
					);
	
    ~VoltageMonitor();

	[[nodiscard]] auto isFault() -> bool;
    [[nodiscard]] auto isInitialized() const -> bool { return fActiveLoop; }
    [[nodiscard]] auto hasAdc() const -> bool { return (fAdc != nullptr); }
    [[nodiscard]] auto currentVoltage() -> double;
    [[nodiscard]] auto meanVoltage() -> double;
	[[nodiscard]] auto nominalVoltage() const -> double { return fNominalVoltage; }
	void registerVoltageReadyCallback(std::function<void(double)> fn) {	fVoltageReadyFn = fn;	}

  private:
    void threadLoop();
    std::shared_ptr<ADS1115> fAdc { nullptr };
	bool fUpdated { false };
    bool fActiveLoop { false };
	std::uint8_t fAdcChannel { 0 };

    std::unique_ptr<std::thread> fThread { nullptr };

	std::mutex fMutex;
	std::function<void(double)> fVoltageReadyFn { };
	
	double fVoltage { 0. };
	Ringbuffer<double, 100> fBuffer { };

	double fNominalVoltage { 0. };
	double fDividerRatio { 1. };
};

} // namespace PiRaTe

#endif // #ifdef VOLTAGEMONITOR_H
