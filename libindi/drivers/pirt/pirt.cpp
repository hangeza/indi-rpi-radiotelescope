/*
   INDI Developers Manual
   Tutorial #2

   "Simple Telescope Driver"

   We develop a simple telescope simulator.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file simplescope.cpp
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Jasem Mutlaq

    \example simplescope.cpp
    A simple GOTO telescope that simulator slewing operation.
*/

#include "simplescope.h"

#include "indicom.h"

#include <cmath>
#include <memory>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libnova.h>

// #include <math.h>
// #include <unistd.h>
// #include <sys/time.h>
// #include <time.h>

const double SID_RATE = 0.004178; /* sidereal rate, degrees/s */
const double SLEW_RATE = 5.;        /* slew rate, degrees/s */


#define POLLMS 100
#define POS_ACCURACY 0.05
#define TRACK_ACCURACY 0.07
#define MAX_ELEVATION_EXCESS 100.0


std::unique_ptr<SimpleScope> simpleScope(new SimpleScope());





/**************************************************************************************
** Return properties of device.
***************************************************************************************/
void ISGetProperties(const char *dev)
{
    simpleScope->ISGetProperties(dev);
}

/**************************************************************************************
** Process new switch from client
***************************************************************************************/
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    simpleScope->ISNewSwitch(dev, name, states, names, n);
}

/**************************************************************************************
** Process new text from client
***************************************************************************************/
void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    simpleScope->ISNewText(dev, name, texts, names, n);
}

/**************************************************************************************
** Process new number from client
***************************************************************************************/
void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    simpleScope->ISNewNumber(dev, name, values, names, n);
}

/**************************************************************************************
** Process new blob from client
***************************************************************************************/
void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    simpleScope->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

/**************************************************************************************
** Process snooped property from another driver
***************************************************************************************/
void ISSnoopDevice(XMLEle *root)
{
    INDI_UNUSED(root);
}




/*
 * class SimpleScope
 * 
 */
SimpleScope::SimpleScope()
{
    currentRA  = 0;
    currentDEC = 0;
    currentAz = currentAlt = 0;
//     TrackingOn = false;

    // We add an additional debug level so we can log verbose scope status
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

    SetTelescopeCapability(TELESCOPE_CAN_PARK 
			 | TELESCOPE_CAN_ABORT | TELESCOPE_HAS_TIME 
			 | TELESCOPE_HAS_LOCATION  );
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool SimpleScope::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::initProperties();
//     setTelescopeConnection(1);
    IUFillSwitch(&TrackingS[0], "Off", "", ISS_ON);
    IUFillSwitch(&TrackingS[1], "On", "", ISS_OFF);
    IUFillSwitchVector(&TrackingSP, TrackingS, 2, getDeviceName(), "TRACKING", "Tracking", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY,0, IPS_IDLE);
    
    IUFillLight(&ScopeStatusL[0], "SCOPE_IDLE", "Idle", IPS_IDLE);
    IUFillLight(&ScopeStatusL[1], "SCOPE_SLEWING", "Slew", IPS_IDLE);
    IUFillLight(&ScopeStatusL[2], "SCOPE_TRACKING", "Track", IPS_IDLE);
    IUFillLight(&ScopeStatusL[3], "SCOPE_PARKING", "Parking", IPS_IDLE);
    IUFillLight(&ScopeStatusL[4], "SCOPE_PARKED", "Parked", IPS_IDLE);
  // Make sure to set the device name to "Rain Detector" since we are snooping on rain detector device.
    IUFillLightVector(&ScopeStatusLP, ScopeStatusL, 5, getDeviceName(), "SCOPE_STATUS", "Scope Status", MAIN_CONTROL_TAB, IPS_IDLE);

    IUFillTextVector(&TimeTP, TimeT, 2, getDeviceName(), TimeTP.name, TimeTP.label, SITE_TAB, IP_RO, 60, IPS_IDLE);
    IUFillNumberVector(&LocationNP, LocationN, 3, getDeviceName(), LocationNP.name, LocationNP.label, SITE_TAB, IP_RO, 60, IPS_IDLE);

    IUFillNumber(&JDN, "JD", "", "%10.5f", 0, 1e8, 0, 0);
    IUFillNumberVector(&JDNP, &JDN, 1, getDeviceName(), "JD", "Julian Date", SITE_TAB,
           IP_RO, 60, IPS_IDLE);

    IUFillNumber(&HorN[AXIS_AZ], "AZ", "Azimuth (deg:mm:ss)", "%010.6m", 0, 360, 0, 0);
    IUFillNumber(&HorN[AXIS_ALT], "ALT", "Elevation (dd:mm:ss)", "%010.6m", -90, 90, 0, 90);
    IUFillNumberVector(&HorNP, HorN, 2, getDeviceName(), "HORIZONTAL_EOD_COORD", "Hor. Coordinates", MAIN_CONTROL_TAB,
           IP_RW, 60, IPS_IDLE);
    lastHorState = IPS_IDLE;
    
    LocationN[LOCATION_LATITUDE].value  = 50.03;
    LocationN[LOCATION_LONGITUDE].value = 8.57;
    LocationN[LOCATION_ELEVATION].value = 180.;
    LocationNP.s = IPS_OK;
    IDSetNumber(&LocationNP, NULL);
    
    //defineLight(&ScopeStatusLP);
    addDebugControl();

    return true;
}

bool SimpleScope::updateProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::updateProperties();
    // delete the paramters defined for optical scopes - don't need 'em
    deleteProperty(ScopeParametersNP.name);

    if (isConnected()) {
      defineLight(&ScopeStatusLP);
      LocationNP.s = IPS_OK;
      IDSetNumber(&LocationNP, NULL);
      defineNumber(&HorNP);
      defineNumber(&JDNP);
      defineSwitch(&TrackingSP);
    } else {
      deleteProperty(ScopeStatusLP.name);
      deleteProperty(HorNP.name);
      deleteProperty(TrackingSP.name);
      deleteProperty(JDNP.name);
    }
    return true;
}

/**************************************************************************************
**
***************************************************************************************/
bool SimpleScope::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if(strcmp(dev,getDeviceName())==0)
    {
        //  This one is for us
        if(!strcmp(name,TrackingSP.name))
        {
            //  client is telling us what to do with co-ordinate requests
            TrackingSP.s=IPS_OK;
            IUUpdateSwitch(&TrackingSP,states,names,n);
            //  Update client display
            IDSetSwitch(&TrackingSP, NULL);
            if (TrackState == SCOPE_IDLE && isTracking()) {
	      TrackState = SCOPE_TRACKING;
	      Hor2Equ(currentAz, currentAlt, &targetRA, &targetDEC);
	    }
	    else if (!isTracking() && TrackState == SCOPE_TRACKING) TrackState = SCOPE_IDLE;
	    return true;
        }
    }
    //  Nobody has claimed this, so, ignore it
    return INDI::Telescope::ISNewSwitch(dev,name,states,names,n);
}

bool SimpleScope::Connect()
{
  SetTimer(POLLMS);
  return true;
}

bool SimpleScope::Disconnect()
{
  return true;
}

void SimpleScope::TimerHit()
{
//     if(isConnected())
    {
        bool rc;

        rc=ReadScopeStatus();

        if(rc == false)
        {
            //  read was not good
            EqNP.s= IPS_ALERT;
            IDSetNumber(&EqNP, NULL);
        }

//         SetTimer(POLLMS);
    }
    SetTimer(POLLMS);
}


/**************************************************************************************
** INDI is asking us to check communication with the device via a handshake
***************************************************************************************/
bool SimpleScope::Handshake()
{
    // When communicating with a real mount, we check here if commands are receieved
    // and acknolowedged by the mount. For SimpleScope, we simply return true.
    return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *SimpleScope::getDefaultName()
{
    return "Simple Scope";
}

/**************************************************************************************
** is telescope in tracking mode?
***************************************************************************************/
bool SimpleScope::isTracking()
{
  return (TrackingS[1].s == ISS_ON);
}


/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool SimpleScope::Goto(double ra, double dec)
{
    targetRA  = ra;
    targetDEC = dec;
    char RAStr[64]={0}, DecStr[64]={0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;
    TargetCoordSystem = SYSTEM_EQ;
    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to RA: %s - DEC: %s", RAStr, DecStr);

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to slew to a new position (horizontal coordinates)
***************************************************************************************/
bool SimpleScope::GotoHor(double az, double alt)
{
    targetAz  = az;
    targetAlt = alt;
    
    if (alt<=0.) {
      DEBUG(INDI::Logger::DBG_WARNING, "Error: Target below horizon");
      return false;
    }
    char AzStr[64]={0}, AltStr[64]={0};

    // Parse the Az/Alt into strings
    fs_sexa(AzStr, targetAz, 2, 3600);
    fs_sexa(AltStr, targetAlt, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;
    TargetCoordSystem = SYSTEM_HOR;

    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to Az: %s - Alt: %s", AzStr, AltStr);

    // Success!
    return true;
}

/**************************************************************************************
** Client is asking us to abort our motion
***************************************************************************************/
bool SimpleScope::Abort()
{
    return true;
}


void SimpleScope::Hor2Equ(double az, double alt, double* ra, double* dec) {
  struct ln_date date;
  struct tm *utc;

  time_t raw_time;
  time(&raw_time);
  
  utc = gmtime(&raw_time);
  
  /* UT date and time */
  date.years = utc->tm_year+1900;
  date.months = utc->tm_mon+1;
  date.days = utc->tm_mday;
  date.hours = utc->tm_hour;
  date.minutes = utc->tm_min;
  date.seconds = utc->tm_sec;
	
  double JD = ln_get_julian_day(&date);
  
  struct ln_hrz_posn horcoords;
  // 0 deg Az should be S, in libnova it is N
  horcoords.az = ln_range_degrees(az+180.);
  horcoords.alt = alt;
  
  struct ln_lnlat_posn geocoords;
  double x = LocationN[LOCATION_LONGITUDE].value;
  if (x>180.) x-=360.;
  geocoords.lng = x;
  geocoords.lat = LocationN[LOCATION_LATITUDE].value;
  
  struct ln_equ_posn equcoords;
  
  ln_get_equ_from_hrz( &horcoords, &geocoords, JD, &equcoords);
  *ra = equcoords.ra*24./360.;
  *dec = equcoords.dec;
}


void SimpleScope::Equ2Hor(double ra, double dec, double* az, double* alt) {
  struct ln_date date;
  struct tm *utc;

  time_t raw_time;
  time(&raw_time);
  
  utc = gmtime(&raw_time);
  
  /* UT date and time */
  date.years = utc->tm_year+1900;
  date.months = utc->tm_mon+1;
  date.days = utc->tm_mday;
  date.hours = utc->tm_hour;
  date.minutes = utc->tm_min;
  date.seconds = utc->tm_sec;
	
  double JD = ln_get_julian_day(&date);
  
  struct ln_equ_posn equcoords;
  equcoords.ra = 360.*ra/24.;
  equcoords.dec = dec;
  
  struct ln_lnlat_posn geocoords;
  double x = LocationN[LOCATION_LONGITUDE].value;
  if (x>180.) x-=360.;
  geocoords.lng = x;
  geocoords.lat = LocationN[LOCATION_LATITUDE].value;
  
  struct ln_hrz_posn horcoords;
  
  ln_get_hrz_from_equ( &equcoords, &geocoords, JD, &horcoords);
  // 0 deg Az should be S, in libnova it is N
  horcoords.az = ln_range_degrees(horcoords.az-180.);
  *az = horcoords.az;
  *alt = horcoords.alt;
}

/**************************************************************************************
** Client is asking us to report telescope status
***************************************************************************************/
bool SimpleScope::ReadScopeStatus()
{
    static struct timeval ltv { 0, 0 };
    struct timeval tv { 0, 0 };
    double dt = 0, da_ra = 0, da_dec = 0, dx = 0, dy = 0;
    static double dt_time_update = 0.;
    int nlocked;

    static char ts[32]={0};
    struct tm *utc, *local;

    /* update elapsed time since last poll, don't presume exactly POLLMS */
    gettimeofday(&tv, nullptr);

    // this is the first poll after connect
    if (ltv.tv_sec == 0 && ltv.tv_usec == 0) {
      ltv = tv;
      LocationNP.s = IPS_OK;
      IDSetNumber(&LocationNP, NULL);
    }


    // time since last update
    dt  = tv.tv_sec - ltv.tv_sec + 1e-6*(tv.tv_usec - ltv.tv_usec);
    ltv = tv;

//       LocationNP.s = IPS_OK;
//       IDSetNumber(&LocationNP, NULL);

    // update telescope time display only about once per second to save bandwidth
    dt_time_update+=dt;
    if (dt_time_update>0.9) {
      dt_time_update = 0.;
      time_t raw_time;
      time(&raw_time);

      utc = gmtime(&raw_time);
      strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", utc);
      IUSaveText(&TimeT[0], ts);

      local = localtime(&raw_time);
      snprintf(ts, sizeof(ts), "%4.2f", (local->tm_gmtoff / 3600.0));
      IUSaveText(&TimeT[1], ts);
    
      TimeTP.s = IPS_OK;
      IDSetText(&TimeTP, NULL);
      
      struct ln_date date;
      //struct tm *utc;

      /* UT date and time */
      date.years = utc->tm_year+1900;
      date.months = utc->tm_mon+1;
      date.days = utc->tm_mday;
      date.hours = utc->tm_hour;
      date.minutes = utc->tm_min;
      date.seconds = utc->tm_sec;
	
      double JD = ln_get_julian_day(&date);
      JDN.value = JD;
      JDNP.s = IPS_OK;
      IDSetNumber(&JDNP, NULL);
//       LocationNP.s = IPS_OK;
//       IDSetNumber(&LocationNP, NULL);
    }
    
    // Calculate how much we moved since last time
    double dmov = 0.;
    double imot = 0.;
    switch (TrackState)
    {
        case SCOPE_SLEWING:
	  dmov = SLEW_RATE * dt;
	  nlocked=0;
	  if (TargetCoordSystem == SYSTEM_EQ) {
	    Equ2Hor(targetRA, targetDEC, &targetAz, &targetAlt);
	  }
	  
	  dx = targetAz-currentAz;
	  dy = targetAlt-currentAlt;
	  
	  if (dx>180.) dx-=360.;
	  else if (dx<-180.) dx+=360.;
	  if (dy>180.) dy-=360.;
	  else if (dy<-180.) dy+=360.;
	  
	  if (fabs(dx) <= POS_ACCURACY) nlocked++;
	  else {
	   imot = dx/10.;
	   if (fabs(imot)>1.) imot=(dx>=0)?1.:-1.;
	   currentAz += dmov*imot;
	   currentAz = ln_range_degrees(currentAz);
	  }
// 	    if (dx > 0) currentAz += dmov;
// 	  else currentAz -= dmov;
	  
	  if (fabs(dy) <= POS_ACCURACY) nlocked++;
	  else {
	   imot = dy/10.;
	   if (fabs(imot)>1.) imot=(dy>=0)?1.:-1.;
	   currentAlt += dmov*imot;
	  }
// 	  else if (dy > 0) currentAlt += dmov;
// 	  else currentAlt -= dmov;
	  
          // Let's check if we reached position for both axes
          if (nlocked == 2)
          {
                // Let's set state to TRACKING
                TrackState = (isTracking())?SCOPE_TRACKING:SCOPE_IDLE;
		if (isTracking() && TargetCoordSystem == SYSTEM_HOR) {
		  Hor2Equ(currentAz, currentAlt, &targetRA, &targetDEC);
		}
                DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete.");
          }
          break;
	
	case SCOPE_TRACKING:
	  dmov = 2.5*SID_RATE * dt;
	  Equ2Hor(targetRA, targetDEC, &targetAz, &targetAlt);
	  
// 	  DEBUG(INDI::Logger::DBG_SESSION, "tracking...");
// 	  DEBUGF(INDI::Logger::DBG_SESSION, "target Az: %f target Alt: %f", targetAz, targetAlt);

	  dx = targetAz-currentAz;
	  dy = targetAlt-currentAlt;
	  if (dx>180.) dx-=360.;
	  else if (dx<-180.) dx+=360.;
	  if (dy>180.) dy-=360.;
	  else if (dy<-180.) dy+=360.;
	  
	  if (fabs(dx) > TRACK_ACCURACY) {
// 	   currentAz += 0.1*dmov*(dx>=0)?1.:-1.;
	   currentAz += dmov*((dx>=0)?1.:-1.);
// 	   DEBUGF(INDI::Logger::DBG_SESSION, "cur Az+  %f", dmov*((dx>=0)?1.:-1.));
// 	   DEBUGF(INDI::Logger::DBG_SESSION, "dmov = %f", dmov);
	   currentAz = ln_range_degrees(currentAz);
	  }
	  
	  if (fabs(dy) > TRACK_ACCURACY) {
// 	   currentAlt += 0.1*dmov*(dy>=0)?1.:-1.;
	   currentAlt += dmov*((dy>=0)?1.:-1.);
// 	   DEBUGF(INDI::Logger::DBG_SESSION, "cur Alt+  %f", dmov*((dx>=0)?1.:-1.));
	  }
	  
	  break;
	  
	default:
	  break;
	  
    }
    

// 0599546776a










    
    /* Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST and act acoordingly */
//     switch (TrackState)
//     {
//         case SCOPE_SLEWING:
// 	    // Wait until we are "locked" into positon for both RA & DEC axis
//             nlocked = 0;
// 
//             // Calculate diff in RA
//             dx = targetRA - currentRA;
// 
//             // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target RA.
//             if (fabs(dx) * 15. <= da_ra)
//             {
//                 currentRA = targetRA;
//                 nlocked++;
//             }
//             // Otherwise, increase RA
//             else if (dx > 0)
//                 currentRA += da_ra / 15.;
//             // Otherwise, decrease RA
//             else
//                 currentRA -= da_ra / 15.;
// 
//             // Calculate diff in DEC
//             dy = targetDEC - currentDEC;
// 
//             // If diff is very small, i.e. smaller than how much we changed since last time, then we reached target DEC.
//             if (fabs(dy) <= da_dec)
//             {
//                 currentDEC = targetDEC;
//                 nlocked++;
//             }
//             // Otherwise, increase DEC
//             else if (dy > 0)
//                 currentDEC += da_dec;
//             // Otherwise, decrease DEC
//             else
//                 currentDEC -= da_dec;
// 
//             // Let's check if we reached position for both RA/DEC
//             if (nlocked == 2)
//             {
//                 // Let's set state to TRACKING
//                 TrackState = (isTracking())?SCOPE_TRACKING:SCOPE_IDLE;
// 
//                 DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete.");
//             }
//             break;
// 
//         default:
//             break;
//     }

    /* update scope status */
    // update the telescope state lights
    for (int i=0; i<5; i++) ScopeStatusL[i].s=IPS_IDLE;
    ScopeStatusL[TrackState].s=IPS_OK;
    IDSetLight(&ScopeStatusLP, NULL);
    
    if (HorN[AXIS_AZ].value != currentAz || HorN[AXIS_ALT].value != currentAlt)
    {
        HorN[AXIS_AZ].value=currentAz;
        HorN[AXIS_ALT].value=currentAlt;
	HorNP.s = IPS_OK;
        //lastEqState = EqNP.s;
        IDSetNumber(&HorNP, NULL);
    }
  
    Hor2Equ(currentAz, currentAlt, &currentRA, &currentDEC);
    //currentRA*=24./360.;
    
    char RAStr[64]={0}, DecStr[64]={0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, currentRA, 2, 3600);
    fs_sexa(DecStr, currentDEC, 2, 3600);

    DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);

    NewRaDec(currentRA, currentDEC);
//    Equ2Hor(currentRA, currentDEC, &currentAz, &currentAlt);
    
   
    return true;
}
