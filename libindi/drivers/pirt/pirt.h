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


class GPIO;
class SsiPosEncoder;

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

    bool isTracking();
    
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
	
  private:
    void Hor2Equ(double az, double alt, double* ra, double* dec);
    void Equ2Hor(double ra, double dec, double* az, double* alt);
	void updateScopeState();
	
    double currentRA;
    double currentDEC;
    double targetRA;
    double targetDEC;
    
    double currentAz, currentAlt;
    double targetAz, targetAlt;
    
    ILight ScopeStatusL[5];
    ILightVectorProperty ScopeStatusLP;
    INumber HorN[2];
    INumberVectorProperty HorNP;
    
    ISwitch TrackingS[2];
    ISwitchVectorProperty TrackingSP;
    
    INumber JDN;
    INumberVectorProperty JDNP;

	INumber AzEncoderN[2];
	INumber ElEncoderN[2];
	INumberVectorProperty AzEncoderNP;
	INumberVectorProperty ElEncoderNP;
	
    IPState lastHorState;
    uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };
	// slew rate, degrees/s
    static const uint8_t SLEW_RATE = 3;
	
	std::shared_ptr<GPIO> gpio { nullptr };
	std::shared_ptr<SsiPosEncoder> az_encoder { nullptr };
	std::shared_ptr<SsiPosEncoder> el_encoder { nullptr };
};
