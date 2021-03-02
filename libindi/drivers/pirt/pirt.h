/*
   INDI Developers Manual
   Tutorial #2

   "Simple Telescope Driver"

   We develop a simple telescope simulator.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file pirt.h
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author HG Zaunick

    \example pirt.h
    A simple GOTO telescope that simulator slewing operation.
*/

#pragma once

#include "inditelescope.h"
#include "axis.h"
//#include "motordriver.h"

struct HorCoords {
	HorCoords() { Alt.registerGimbalFlipCallback( [this]() { this->Az.gimbalFlip(); } ); }
	HorCoords(double az, double alt) {
		Alt.registerGimbalFlipCallback( [this]() { this->Az.gimbalFlip(); } );
		Az.setValue(az);
		Alt.setValue(alt);
	}
	PiRaTe::RotAxis Az { 0., 360., 360. };
	PiRaTe::RotAxis Alt { -90., 90., 360. };
};

struct EquCoords {
	EquCoords() { Dec.registerGimbalFlipCallback( [this]() { this->Ra.gimbalFlip(); } ); }
	EquCoords(double ra, double dec) {
		Dec.registerGimbalFlipCallback( [this]() { this->Ra.gimbalFlip(); } );
		Ra.setValue(ra);
		Dec.setValue(dec);
	}
	PiRaTe::RotAxis Ra { 0., 24., 24. };
	PiRaTe::RotAxis Dec { -90., 90., 360. };
};


class GPIO;
namespace PiRaTe {
	class SsiPosEncoder;
	class MotorDriver;
}


class PiRT : public INDI::Telescope
{
  public:
    enum {
      SYSTEM_EQ,
      SYSTEM_HOR,
      SYSTEM_GAL
    } TargetCoordSystem;
    
    enum {
		AXIS_AZ, 
		AXIS_ALT 
	};
    
    PiRT();
	//~PiRT() override;
	
    bool Connect() override;
    bool Disconnect() override;
    void TimerHit() override;
    virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n) override;
	virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;

    [[nodiscard]] auto isTracking() const -> bool;
    
  protected:
    bool Handshake() override;
    const char *getDefaultName() override;
    bool initProperties() override;
    bool updateProperties() override;

    // Telescope specific functions
    bool ReadScopeStatus() override;
    bool Goto(double, double) override;
    bool GotoHor(double, double);
    bool Abort() override;
	bool SetTrackMode(uint8_t mode) override;
	bool SetTrackEnabled(bool enabled) override;
	bool Park() override;
	bool UnPark() override;
	
  private:
    void Hor2Equ(double az, double alt, double* ra, double* dec);
    void Hor2Equ(const HorCoords& hor_coords, double* ra, double* dec);
    void Equ2Hor(double ra, double dec, double* az, double* alt);
    HorCoords Equ2Hor(const EquCoords& equ_coords);
	EquCoords Hor2Equ(const HorCoords& hor_coords);
	bool isInAbsoluteTurnRange(double absRev);
	
    ILight ScopeStatusL[5];
    ILightVectorProperty ScopeStatusLP;
    INumber HorN[2];
    INumberVectorProperty HorNP;
    INumber JDN;
    INumberVectorProperty JDNP;

    INumber EncoderBitRateN;
    INumberVectorProperty EncoderBitRateNP;

	INumber AzEncoderN[5];
	INumber ElEncoderN[5];
	INumberVectorProperty AzEncoderNP;
	INumberVectorProperty ElEncoderNP;
	INumber AzEncSettingN[2], ElEncSettingN[2];
	INumberVectorProperty AzEncSettingNP, ElEncSettingNP;

	INumber AzAxisSettingN[2], ElAxisSettingN[2];
	INumberVectorProperty AzAxisSettingNP, ElAxisSettingNP;

	INumber MotorStatusN[2];
	INumberVectorProperty MotorStatusNP;

	INumber AxisAbsTurnsN[2];
	INumberVectorProperty AxisAbsTurnsNP;
	
	bool fIsTracking { false };
	
	double axisRatio[2] { 1., 1. };
	double axisOffset[2] { 0., 0. };
	
    IPState lastHorState;
    uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };
	// slew rate, degrees/s
    static const uint8_t SLEW_RATE = 3;
	
	std::shared_ptr<GPIO> gpio { nullptr };
	std::unique_ptr<PiRaTe::SsiPosEncoder> az_encoder { nullptr };
	std::unique_ptr<PiRaTe::SsiPosEncoder> el_encoder { nullptr };
	std::unique_ptr<PiRaTe::MotorDriver> az_motor { nullptr };
	std::unique_ptr<PiRaTe::MotorDriver> el_motor { nullptr };
	//PiRaTe::RotAxis az_axis { 0., 360., 360. };
	//PiRaTe::RotAxis el_axis { -90., 90., 360. };
	HorCoords currentHorizontalCoords { 0. , 90. };
	HorCoords targetHorizontalCoords { 0. , 90. };
	EquCoords targetEquatorialCoords { 0. , 0. };
};
