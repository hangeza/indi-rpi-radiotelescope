/*
   INDI Developers Manual
   Tutorial #2

   "Simple Telescope Driver"

   We develop a simple telescope simulator.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simplescope.h
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Jasem Mutlaq

    \example simplescope.h
    A simple GOTO telescope that simulator slewing operation.
*/

#pragma once

#include "inditelescope.h"

class SimpleScope : public INDI::Telescope
{
  public:
    enum
    {
      SYSTEM_EQ,
      SYSTEM_HOR,
      SYSTEM_GAL
    } TargetCoordSystem;
    
    enum { AXIS_AZ, AXIS_ALT };
    //bool TrackingOn;
    
    SimpleScope();
    virtual bool Connect();
    virtual bool Disconnect();
    virtual void TimerHit();
    virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n);

    bool isTracking();
    
  protected:
    bool Handshake();
    const char *getDefaultName();
    virtual bool initProperties() override;
    virtual bool updateProperties() override;

    // Telescope specific functions
    bool ReadScopeStatus();
    bool Goto(double, double);
    bool GotoHor(double, double);
    bool Abort();

  private:
    void Hor2Equ(double az, double alt, double* ra, double* dec);
    void Equ2Hor(double ra, double dec, double* az, double* alt);

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

    IPState lastHorState;
    unsigned int DBG_SCOPE;
};
