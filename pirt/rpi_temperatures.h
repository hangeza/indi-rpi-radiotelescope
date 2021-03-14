#ifndef RPITEMP_H
#define RPITEMP_H


#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <utility>
#include <inttypes.h>  // uint8_t, etc
#include <string>
#include <functional>
#include <cmath>
#include <thread>
#include <mutex>

namespace PiRaTe {

	
class RpiTemperatureMonitor {
public:
	struct TemperatureItem {
		std::string name { "N/A" };
		std::string id { "0" };
		unsigned int hwMonIndex { };
		std::string value_device_path { "" };
		double temperature { };
		bool valid { false };
	};

	RpiTemperatureMonitor(const std::string device_path = "/sys/class/thermal/thermal_zone0/hwmon0/subsystem/");
	~RpiTemperatureMonitor();
	
	void rescanSources();
	[[nodiscard]] auto nrSources() const -> std::size_t { return fItemList.size(); }
	[[nodiscard]] auto getTemperatureItem(std::size_t source_index) -> TemperatureItem;
	
	void registerTempReadyCallback(std::function<void(TemperatureItem)> fn) {	fTempReadyFn = fn;	}
	
private:
    void threadLoop();

	std::string fDevPath { "" };
    std::vector<TemperatureItem> fItemList { };
	std::unique_ptr<std::thread> fThread { nullptr };
    bool fActiveLoop { false };
	std::mutex fMutex;

	std::function<void(TemperatureItem)> fTempReadyFn { };
};

} // namespace PiRaTe

#endif // #define RPITEMP_H
