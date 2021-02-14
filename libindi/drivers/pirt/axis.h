#ifndef AXISRT_H
#define AXISRT_H


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
#include <functional>
#include <cmath>

namespace PIRT {

constexpr double pi() { return std::acos(-1); }
constexpr double twopi() { return 2.*pi(); }
constexpr double toDeg(double unitVal) { return ( unitVal * 360. ); }
constexpr double toHours(double unitVal) { return ( unitVal * 24. ); }
constexpr double toRadians(double unitVal) { return ( unitVal * twopi() ); }

class RotAxis {
public:
	typedef void(*FunctPtr)();
	
	RotAxis(double min, double max, double ref_unit = twopi()) 
		: fValue { 0. }
	{
		if ( fRefUnit == 0. ) return;
		fRefUnit = ref_unit;
		fMin = min / fRefUnit;
		fMax = max / fRefUnit;
		fValid = true;
	}
	
	RotAxis& operator=(double a_value) { 
		this->setValue(a_value); 
		return *this; 
	}
	
	[[nodiscard]] auto degrees() const -> double { return toDeg(fValue); }
	[[nodiscard]] auto radians() const -> double { return toRadians(fValue); }
	[[nodiscard]] auto hourAngle() const -> double { return toHours(fValue); }

	void setValue(double val) { 
		double princ_val = reduceToPrincipal( val / fRefUnit );
		if (princ_val > fMax) {
			princ_val = 2. * fMax - princ_val;
			if (fGimbalFlipFn) fGimbalFlipFn();
		} else if (princ_val < fMin) {
			princ_val = 2. * fMin - princ_val;
			if (fGimbalFlipFn) fGimbalFlipFn();
		}
		fValue = princ_val;
		fValid = true;
	}
	[[nodiscard]] auto value() const -> double { return fValue * fRefUnit; }
	void registerGimbalFlipCallback(std::function<void()> fn) {	fGimbalFlipFn = fn;	}
	void gimbalFlip() { this->setValue( fRefUnit * (fValue + 0.5) ); }

private:
	double fRefUnit { pi() };
	double fValue { 0. };
	double fMin { -1 };
	double fMax { 1 };
	bool fValid { false };

	double reduceToPrincipal(double arg) {
		while (arg > 1.) arg -= 1.;
		while (arg < -1.) arg += 1.;
		return arg;
	}
	std::function<void()> fGimbalFlipFn { };
};

} // namespace PIRT

#endif
