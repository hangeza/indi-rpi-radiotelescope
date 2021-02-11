#if 0
    LX200 Radio Telescope
    Copyright (C) 2009 HG Zaunick (zhg@gmx.de)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>

#include "indicom.h"
#include "lx200driver.h"
#include "lx200rt.h"

#include <config.h>

#ifdef HAVE_NOVA_H
#include <libnova.h>
#endif

LX200RT *telescope = NULL;
int MaxReticleFlashRate = 3;

/* There is _one_ binary for all LX200 drivers, but each binary is renamed
** to its device name (i.e. lx200gps, lx200_16..etc). The main function will
** fetch from std args the binary name and ISInit will create the appropiate
** device afterwards. If the binary name does not match any known devices,
** we simply create a generic device.
*/
extern char* me;

#define COMM_GROUP	"Communication"
#define BASIC_GROUP	"Main Control"
#define MOTION_GROUP	"Motion Control"
#define DATETIME_GROUP	"Date/Time"
#define SITE_GROUP	"Site Management"
//#define LX16GROUP	"GPS/16 inch Features"
#define FIRMWARE_GROUP "Firmware data"
#define RT_GROUP "RT specific"
#define OBJECT_GROUP "Objects"
#define DEBUG_GROUP "Debug"

#define LX200_TRACK	0
#define LX200_SYNC	1

/* Simulation Parameters */
#define	SLEWRATE	1		/* slew rate, degrees/s */
#define SIDRATE		0.004178	/* sidereal rate, degrees/s */

/* Handy Macros */
#define currentRA	EquatorialCoordsN[0].value
#define currentDEC	EquatorialCoordsN[1].value
#define targetRA	EquatorialCoordsWN[0].value
#define targetDEC	EquatorialCoordsWN[1].value

#define currentAZ	HorizontalCoordsNP.np[0].value
#define currentALT	HorizontalCoordsNP.np[1].value
#define targetAZ	HorizontalCoordsWNP.np[0].value
#define targetALT	HorizontalCoordsWNP.np[1].value

static void ISPoll(void *);
static void retryConnection(void *);

/*INDI Properties */

/**********************************************************************************************/
/************************************ GROUP: Communication ************************************/
/**********************************************************************************************/

/********************************************
 Property: Connection
*********************************************/
static ISwitch ConnectS[]          	= {{"CONNECT" , "Connect" , ISS_OFF, 0, 0},{"DISCONNECT", "Disconnect", ISS_ON, 0, 0}};
ISwitchVectorProperty ConnectSP		= { mydev, "CONNECTION" , "Connection", COMM_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, ConnectS, NARRAY(ConnectS), "", 0};

/********************************************
 Property: Device Port
*********************************************/
/*wildi removed static */
static IText PortT[]			= {{"PORT", "Port", 0, 0, 0, 0}};
ITextVectorProperty PortTP	= { mydev, "DEVICE_PORT", "Ports", COMM_GROUP, IP_RW, 0, IPS_IDLE, PortT, NARRAY(PortT), "", 0};

/********************************************
 Property: Telescope Alignment Mode
*********************************************/
static ISwitch AlignmentS []		= {{"Polar", "", ISS_ON, 0, 0}, {"AltAz", "", ISS_OFF, 0, 0}, {"Land", "", ISS_OFF, 0, 0}};
static ISwitchVectorProperty AlignmentSw= { mydev, "Alignment", "", COMM_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, AlignmentS, NARRAY(AlignmentS), "", 0};


/**********************************************************************************************/
/************************************ GROUP: Main Control *************************************/
/**********************************************************************************************/

/********************************************
 Property: Equatorial Coordinates JNow
 Perm: Transient WO.
 Timeout: 120 seconds.
*********************************************/
INumber EquatorialCoordsWN[] 		= { {"RA",  "RA  H:M:S", "%10.6m",  0., 24., 0., 0., 0, 0, 0}, {"DEC", "Dec D:M:S", "%10.6m", -90., 90., 0., 0., 0, 0, 0} };
INumberVectorProperty EquatorialCoordsWNP= { mydev, "EQUATORIAL_EOD_COORD_REQUEST", "Equatorial JNow", BASIC_GROUP, IP_WO, 120, IPS_IDLE, EquatorialCoordsWN, NARRAY(EquatorialCoordsWN), "", 0};

/********************************************
 Property: Equatorial Coordinates JNow
 Perm: RW
*********************************************/
INumber EquatorialCoordsN[]	 	= { {"RA",  "RA  H:M:S", "%10.6m",  0., 24., 1., 0., 0, 0, 0}, {"DEC", "Dec D:M:S", "%10.6m", -90., 90., 1., 0., 0, 0, 0}};
INumberVectorProperty EquatorialCoordsNP= { mydev, "EQUATORIAL_EOD_COORD", "Equatorial JNow", BASIC_GROUP, IP_RW, 120, IPS_IDLE, EquatorialCoordsN, NARRAY(EquatorialCoordsN), "", 0};

/* Horizontal Coordinates: Read-Write */
static INumber HorizontalCoordsN[] = {
    {"AZ", "Az D:M:S", "%10.6m", 0., 360., 1., 0., 0, 0, 0},
    {"ALT",  "Alt  D:M:S", "%10.6m",  -90., 90., 1., 0., 0, 0, 0}};
static INumberVectorProperty HorizontalCoordsNP = {
    mydev, "HORIZONTAL_COORD", "Horizontal Coords", BASIC_GROUP, IP_RW, 120, IPS_IDLE,
    HorizontalCoordsN, NARRAY(HorizontalCoordsN), "", 0};

//static ISwitches SlewAltAzSw		= { mydev, "AltAzSet", "On Alt/Az Set",  SlewAltAzS, NARRAY(SlewAltAzS), ILS_IDLE, 0, LX16Group};

/********************************************
 Property: On Coord Set
 Description: This property decides what happens
             when we receive a new equatorial coord
             value. We either track, or sync
	          to the new coordinates.
*********************************************/
static ISwitch OnCoordSetS[]		 = {{"SLEW", "Slew", ISS_ON, 0, 0 }, {"SYNC", "Sync", ISS_OFF, 0 , 0}};
ISwitchVectorProperty OnCoordSetSP= { mydev, "ON_COORD_SET", "On Set", BASIC_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, OnCoordSetS, NARRAY(OnCoordSetS), "", 0};

/********************************************
 Property: Abort telescope motion
*********************************************/
static ISwitch AbortSlewS[]		= {{"ABORT", "Abort", ISS_OFF, 0, 0 }};
ISwitchVectorProperty AbortSlewSP= { mydev, "TELESCOPE_ABORT_MOTION", "Abort Slew", BASIC_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, AbortSlewS, NARRAY(AbortSlewS), "", 0};

/********************************************
 Property: HOME Search
*********************************************/
static ISwitch HomeSearchS[]		= { {"Save home", "", ISS_OFF, 0, 0} , {"Set home", "", ISS_OFF, 0, 0}};
static ISwitchVectorProperty HomeSearchSP	= { mydev, "Park", "", BASIC_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, HomeSearchS, NARRAY(HomeSearchS), "", 0};

/********************************************
 Property: Park telescope to HOME
*********************************************/
static ISwitch ParkS[]		 	= { {"PARK", "Park", ISS_OFF, 0, 0} };
ISwitchVectorProperty ParkSP		= {mydev, "TELESCOPE_PARK", "Park Scope", BASIC_GROUP, IP_RW, ISR_ATMOST1, 0, IPS_IDLE, ParkS, NARRAY(ParkS), "", 0 };


/**********************************************************************************************/
/************************************** GROUP: Motion *****************************************/
/**********************************************************************************************/

/********************************************
 Property: Slew Speed
*********************************************/
static ISwitch SlewModeS[]		= {{"Max", "", ISS_ON, 0, 0}, {"Find", "", ISS_OFF, 0, 0}, {"Centering", "", ISS_OFF, 0, 0}, {"Guide", "", ISS_OFF, 0 , 0}};
ISwitchVectorProperty SlewModeSP	= { mydev, "Slew rate", "", MOTION_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, SlewModeS, NARRAY(SlewModeS), "", 0};

/********************************************
 Property: Tracking Mode
*********************************************/
static ISwitch TrackModeS[]		= {{ "Default", "", ISS_ON, 0, 0} , { "Lunar", "", ISS_OFF, 0, 0}, {"Manual", "", ISS_OFF, 0, 0}};
static ISwitchVectorProperty TrackModeSP= { mydev, "Tracking Mode", "", MOTION_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, TrackModeS, NARRAY(TrackModeS), "", 0};

/********************************************
 Property: Tracking Frequency
*********************************************/
static INumber TrackFreqN[] 		 = {{ "trackFreq", "Freq", "%g", 56.4, 60.1, 0.1, 60.1, 0, 0, 0}};
static INumberVectorProperty TrackingFreqNP= { mydev, "Tracking Frequency", "", MOTION_GROUP, IP_RW, 0, IPS_IDLE, TrackFreqN, NARRAY(TrackFreqN), "", 0};

/********************************************
 Property: Movement (Arrow keys on handset). North/South
*********************************************/
static ISwitch MovementNSS[]       = {{"MOTION_NORTH", "North", ISS_OFF, 0, 0}, {"MOTION_SOUTH", "South", ISS_OFF, 0, 0}};
ISwitchVectorProperty MovementNSSP      = { mydev, "TELESCOPE_MOTION_NS", "North/South", MOTION_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, MovementNSS, NARRAY(MovementNSS), "", 0};

/********************************************
 Property: Movement (Arrow keys on handset). West/East
*********************************************/
static ISwitch MovementWES[]       = {{"MOTION_WEST", "West", ISS_OFF, 0, 0}, {"MOTION_EAST", "East", ISS_OFF, 0, 0}};
ISwitchVectorProperty MovementWESP      = { mydev, "TELESCOPE_MOTION_WE", "West/East", MOTION_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, MovementWES, NARRAY(MovementWES), "", 0};

/********************************************
 Property: Slew Accuracy
 Desciption: How close the scope have to be with
	     respect to the requested coords for
	     the tracking operation to be successull
	     i.e. returns OK
*********************************************/
INumber SlewAccuracyN[] = {
    {"SlewRA",  "RA (arcmin)", "%g",  0., 60., 1., 3.0, 0, 0, 0},
    {"SlewkDEC", "Dec (arcmin)", "%g", 0., 60., 1., 3.0, 0, 0, 0},
};
INumberVectorProperty SlewAccuracyNP = {mydev, "Slew Accuracy", "", MOTION_GROUP, IP_RW, 0, IPS_IDLE, SlewAccuracyN, NARRAY(SlewAccuracyN), "", 0};


/**********************************************************************************************/
/*********************************** GROUP: Date & Time ***************************************/
/**********************************************************************************************/

/********************************************
 Property: UTC Time
*********************************************/
static IText TimeT[] = {{"UTC", "UTC", 0, 0, 0, 0}};
ITextVectorProperty TimeTP = { mydev, "TIME_UTC", "UTC Time", DATETIME_GROUP, IP_RW, 0, IPS_IDLE, TimeT, NARRAY(TimeT), "", 0};

/********************************************
 Property: DST Corrected UTC Offfset
*********************************************/
static INumber UTCOffsetN[] = {{"OFFSET", "Offset", "%0.3g" , -12.,12.,0.5,0., 0, 0, 0}};
INumberVectorProperty UTCOffsetNP = { mydev, "TIME_UTC_OFFSET", "UTC Offset", DATETIME_GROUP, IP_RW, 0, IPS_IDLE, UTCOffsetN , NARRAY(UTCOffsetN), "", 0};

/********************************************
 Property: Sidereal Time
*********************************************/
static INumber SDTimeN[] = {{"LST", "Sidereal time", "%10.6m" , 0.,24.,0.,0., 0, 0, 0}};
INumberVectorProperty SDTimeNP = { mydev, "TIME_LST", "Sidereal Time", DATETIME_GROUP, IP_RW, 0, IPS_IDLE, SDTimeN, NARRAY(SDTimeN), "", 0};


/**********************************************************************************************/
/************************************* GROUP: Sites *******************************************/
/**********************************************************************************************/

/********************************************
 Property: Site Management
*********************************************/
static ISwitch SitesS[]          = {{"Site 1", "", ISS_ON, 0, 0}, {"Site 2", "", ISS_OFF, 0, 0},  {"Site 3", "", ISS_OFF, 0, 0},  {"Site 4", "", ISS_OFF, 0 ,0}};
static ISwitchVectorProperty SitesSP  = { mydev, "Sites", "", SITE_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, SitesS, NARRAY(SitesS), "", 0};


/**********************************************************************************************/
/************************************* GROUP: GPS/16 ******************************************/
/**********************************************************************************************/


/* Horizontal Coordinates: Request Only */
static INumber HorizontalCoordsWN[] = {
    {"AZ", "Az D:M:S", "%10.6m", 0., 360., 0., 0., 0, 0, 0},
    {"ALT",  "Alt  D:M:S", "%10.6m",  -90., 90., 0., 0., 0, 0, 0}};
static INumberVectorProperty HorizontalCoordsWNP = {
    mydev, "HORIZONTAL_COORD_REQUEST", "Horizontal Coords", BASIC_GROUP, IP_WO, 120, IPS_IDLE,
    HorizontalCoordsWN, NARRAY(HorizontalCoordsWN), "", 0};


/**********************************************************************************************/
/************************************* GROUP: Firmware ****************************************/
/**********************************************************************************************/

static IText   VersionT[] ={{ "Date", "", 0, 0, 0, 0} ,
			   { "Time", "", 0, 0, 0, 0} ,
			   { "Number", "", 0, 0, 0 ,0} ,
			   { "Full", "", 0, 0, 0, 0} ,
			   { "Name", "" ,0 ,0 ,0 ,0}};

static ITextVectorProperty VersionInfo = {mydev, "Firmware Info", "", FIRMWARE_GROUP, IP_RO, 0, IPS_IDLE, VersionT, NARRAY(VersionT), "" ,0};

/********************************************
 Property: Site Name
*********************************************/
static IText   SiteNameT[] = {{"Name", "", 0, 0, 0, 0}};
static ITextVectorProperty SiteNameTP = { mydev, "Site Name", "", FIRMWARE_GROUP, IP_RO, 0 , IPS_IDLE, SiteNameT, NARRAY(SiteNameT), "", 0};

/********************************************
 Property: Geographical Location
*********************************************/

static INumber geo[] = {
    {"LAT",  "Lat.  D:M:S +N", "%10.6m",  -90.,  90., 0., 0., 0, 0, 0},
    {"LONG", "Long. D:M:S +E", "%10.6m", 0., 360., 0., 0., 0, 0, 0},
    {"HEIGHT", "Height m", "%10.2f", -300., 6000., 0., 610., 0, 0, 0},
};
INumberVectorProperty geoNP = {
    mydev, "GEOGRAPHIC_COORD", "Geographic Location", FIRMWARE_GROUP, IP_RO, 0., IPS_IDLE,
    geo, NARRAY(geo), "", 0};

static INumber cycleCounter[] = {{"CYCLES",  "cycles", "%6.0f",  0.,  100000., 1., 0., 0, 0, 0}};
static INumberVectorProperty cycleCounterNP = {
    mydev, "ERASE_CYCLE_COUNTER", "Flash Erase Cycle Counter", FIRMWARE_GROUP, IP_RO, 0., IPS_IDLE,
    cycleCounter, NARRAY(cycleCounter), "", 0};


/**********************************************************************************************/
/************************************* GROUP: RT **********************************************/
/**********************************************************************************************/

static INumber ControllerTemperatureN[] 		 = {{ "temperature", "Temperature (Deg Celsius)", "%4.1f", -273.16, 500.0, 0.1, 0.0, 0, 0, 0}};
static INumberVectorProperty ControllerTemperatureNP= { mydev, "Controller Temperature", "", RT_GROUP, IP_RO, 0, IPS_IDLE, ControllerTemperatureN, NARRAY(ControllerTemperatureN), "", 0};

static INumber AzPotiN[] 		 = {{ "poti", "RT position (revolutions)", "%6.3f", -2.5, 2.5, 0.1, 0.0, 0, 0, 0}};
static INumberVectorProperty AzPotiNP= { mydev, "Az Poti", "", RT_GROUP, IP_RO, 0, IPS_IDLE, AzPotiN, NARRAY(AzPotiN), "", 0};

static INumber EncodersN[] 		 = {
   { "Az", "Az (0..14399)", "%g", 0.0, 14000.0, 1.0, 0.0, 0, 0, 0},
   { "Alt", "Alt (0..14399)", "%g", 0.0, 14000.0, 1.0, 0.0, 0, 0, 0}};
static INumberVectorProperty EncodersNP= { mydev, "Encoders", "", RT_GROUP, IP_RO, 0, IPS_IDLE, EncodersN, NARRAY(EncodersN), "", 0};

static INumber MotorSpeedsN[] 		 = {
   { "Az", "Az (0..255)", "%g", 0.0, 256.0, 1.0, 0.0, 0, 0, 0},
   { "Alt", "Alt (0..255)", "%g", 0.0, 256.0, 1.0, 0.0, 0, 0, 0}};
static INumberVectorProperty MotorSpeedsNP= { mydev, "Motor Speeds", "", RT_GROUP, IP_RO, 0, IPS_IDLE, MotorSpeedsN, NARRAY(MotorSpeedsN), "", 0};

static ILight RefPosL[] 		 = {
   { "Az", "Az", IPS_IDLE, 0, 0},
   { "Alt", "Alt", IPS_IDLE, 0, 0}};
static ILightVectorProperty RefPosLP= { mydev, "Reference Flags", "", RT_GROUP, IPS_IDLE, RefPosL, NARRAY(RefPosL), "", 0};

static INumber UpTimeN[]       = {{ "uptime", "Up Time (h)", "%6.3f", 0.0, 1000000., 0.1, 0.0, 0, 0, 0}};
static INumberVectorProperty UpTimeNP= { mydev, "Up Time", "", RT_GROUP, IP_RO, 0, IPS_IDLE, UpTimeN, NARRAY(UpTimeN), "", 0};

static ISwitch FanStatusS[]		= { {"On", "", ISS_OFF, 0, 0}, {"Off", "", ISS_OFF, 0, 0}};
static ISwitchVectorProperty FanStatusSP	= { mydev, "Fan", "", RT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, FanStatusS, NARRAY(FanStatusS), "", 0};

/********************************************
 Property: RT Position Offsets
*********************************************/
static INumber HorOffsN[] = {
	{"AzOffset", "Az Offset", "%7.3m" , -180., 180., 1., 0., 0, 0, 0},
	{"AltOffset", "Alt Offset", "%7.3m" , -180., 180., 1., 0., 0, 0, 0}};
INumberVectorProperty HorOffsNP = { mydev, "HOR_OFFS", "Position Corrections", RT_GROUP, IP_RW, 0, IPS_IDLE, HorOffsN, NARRAY(HorOffsN), "", 0};


/**********************************************************************************************/
/************************************* GROUP: Objects *****************************************/
/**********************************************************************************************/

static ISwitch SunS[]		 	= { {"SUN", "GoTo", ISS_OFF, 0, 0} };
ISwitchVectorProperty SunSP		= {mydev, "SUN", "Sun", OBJECT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, SunS, NARRAY(SunS), "", 0 };

static ISwitch CygAS[]		 	= { {"CYGA", "GoTo", ISS_OFF, 0, 0} };
ISwitchVectorProperty CygASP		= {mydev, "CYGA", "Cygnus A", OBJECT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, CygAS, NARRAY(CygAS), "", 0 };

static ISwitch CasAS[]		 	= { {"CASA", "GoTo", ISS_OFF, 0, 0} };
ISwitchVectorProperty CasASP		= {mydev, "CASA", "Cassiopeia A", OBJECT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, CasAS, NARRAY(CasAS), "", 0 };

static ISwitch TauAS[]		 	= { {"TAUA", "GoTo", ISS_OFF, 0, 0} };
ISwitchVectorProperty TauASP		= {mydev, "TAUA", "Taurus A", OBJECT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, TauAS, NARRAY(TauAS), "", 0 };

static ISwitch OriAS[]		 	= { {"ORIA", "GoTo", ISS_OFF, 0, 0} };
ISwitchVectorProperty OriASP		= {mydev, "ORIA", "Orion A", OBJECT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, OriAS, NARRAY(OriAS), "", 0 };

static ISwitch VirAS[]		 	= { {"VIRA", "GoTo", ISS_OFF, 0, 0} };
ISwitchVectorProperty VirASP		= {mydev, "VIRA", "Virgo A", OBJECT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, VirAS, NARRAY(VirAS), "", 0 };

static ISwitch C273S[]		 	= { {"3C273", "GoTo", ISS_OFF, 0, 0} };
ISwitchVectorProperty C273SP		= {mydev, "3C273", "3C273", OBJECT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, C273S, NARRAY(C273S), "", 0 };

static ISwitch C295S[]		 	= { {"3C295", "GoTo", ISS_OFF, 0, 0} };
ISwitchVectorProperty C295SP		= {mydev, "3C295", "3C295", OBJECT_GROUP, IP_RW, ISR_1OFMANY, 0, IPS_IDLE, C295S, NARRAY(C295S), "", 0 };


/**********************************************************************************************/
/************************************* GROUP: DEBUG *******************************************/
/**********************************************************************************************/

/********************************************
 Property: Equatorial Coordinates JNow
 Perm: RO
*********************************************/
static INumber ObjectEquatorialCoordsN[]	 	= {
   {"RA",  "RA  H:M:S", "%10.6m",  0., 24., 1., 0., 0, 0, 0},
   {"DEC", "Dec D:M:S", "%10.6m", -90., 90., 1., 0., 0, 0, 0}};
static INumberVectorProperty ObjectEquatorialCoordsNP= { mydev, "EQUATORIAL_OBJECT_COORD", "Equatorial (Object)", DEBUG_GROUP, IP_RO, 120, IPS_IDLE, ObjectEquatorialCoordsN, NARRAY(ObjectEquatorialCoordsN), "", 0};


/*****************************************************************************************************/
/**************************************** END PROPERTIES *********************************************/
/*****************************************************************************************************/



/* send client definitions of all properties */
void ISInit()
{
   static int isInit=0;

   if (isInit) return;

   isInit = 1;

   IUSaveText(&PortT[0], "/dev/ttyS0");
   IUSaveText(&TimeT[0], "YYYY-MM-DDTHH:MM:SS");

   // We need to check if UTCOffset has been set by user or not
   UTCOffsetN[0].aux0 = (int *) malloc(sizeof(int));
   *((int *) UTCOffsetN[0].aux0) = 0;

   telescope = new LX200RT();
   telescope->setCurrentDeviceName(mydev);
}

void ISIdleTask (void *p) { telescope->ISIdleTask(); p=p;}
void ISGetProperties (const char *dev)
{
   ISInit();
   telescope->ISGetProperties(dev);
   IEAddTimer (POLLMS, ISPoll, NULL);
//   IEAddWorkProc (ISIdleTask, NULL);
}
void ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{ ISInit(); telescope->ISNewSwitch(dev, name, states, names, n);}
void ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{ ISInit(); telescope->ISNewText(dev, name, texts, names, n);}
void ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{ ISInit(); telescope->ISNewNumber(dev, name, values, names, n);}
void ISPoll (void *p) { telescope->ISPoll(); IEAddTimer (POLLMS, ISPoll, NULL); p=p;}
void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
{
  INDI_UNUSED(dev);
  INDI_UNUSED(name);
  INDI_UNUSED(sizes);
  INDI_UNUSED(blobsizes);
  INDI_UNUSED(blobs);
  INDI_UNUSED(formats);
  INDI_UNUSED(names);
  INDI_UNUSED(n);
}

void ISSnoopDevice (XMLEle *root)
{
  telescope->ISSnoopDevice(root);
}

/**************************************************
*** LX200 Generic Implementation
***************************************************/

LX200RT::LX200RT()
{
   currentSiteNum = 1;
   trackingMode   = LX200_TRACK_DEFAULT;
   lastSet        = -1;
   fault          = false;
   simulation     = false;
   currentSet     = 0;
   fd             = -1;

   // Children call parent routines, this is the default
   IDLog("INDI Library v%g\n", INDI_LIBV);
   IDLog("initializing from generic LX200 device...\n");
   char str[64];
   sprintf(str,"Driver Version: %s\n",__DATE__);
//   IDLog("Driver Version: 2009-08-18\n");
   IDLog(str);
   sprintf(str,"compiled with GCC %d.%d.%d\n",__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__);
   IDLog(str);

   //enableSimulation(true);
}

LX200RT::~LX200RT()
{
}

void LX200RT::setCurrentDeviceName(const char * devName)
{
  strcpy(thisDevice, devName);
}

void LX200RT::ISGetProperties(const char *dev)
{

   if (dev && strcmp (thisDevice, dev)) return;

   // COMM_GROUP
   IDDefSwitch (&ConnectSP, NULL);
   IDDefText   (&PortTP, NULL);
   IDDefSwitch (&AlignmentSw, NULL);

   // BASIC_GROUP
//   IDDefNumber (&EquatorialCoordsWNP, NULL);
   IDDefNumber (&EquatorialCoordsNP, NULL);
   IDDefNumber (&HorizontalCoordsNP, NULL);
   IDDefSwitch (&AbortSlewSP, NULL);
   IDDefSwitch (&OnCoordSetSP, NULL);
   IDDefSwitch (&HomeSearchSP, NULL);
   IDDefSwitch (&ParkSP, NULL);

   // MOTION_GROUP
//   IDDefNumber (&TrackingFreqNP, NULL);
   IDDefSwitch (&SlewModeSP, NULL);
   IDDefSwitch (&TrackModeSP, NULL);
   IDDefSwitch (&MovementNSSP, NULL);
   IDDefSwitch (&MovementWESP, NULL);
   IDDefNumber (&SlewAccuracyNP, NULL);

   // DATETIME_GROUP
   #ifdef HAVE_NOVA_H
   IDDefText   (&TimeTP, NULL);
   IDDefNumber(&UTCOffsetNP, NULL);
   #endif
   IDDefNumber (&SDTimeNP, NULL);

   // SITE_GROUP
//   IDDefSwitch (&SitesSP, NULL);

//   IDDefNumber (&HorizontalCoordsWNP, NULL);


   // RT GROUP
   IDDefNumber (&ControllerTemperatureNP, NULL);
   IDDefNumber (&AzPotiNP, NULL);
   IDDefNumber (&EncodersNP, NULL);
   IDDefLight  (&RefPosLP, NULL);
   IDDefNumber (&MotorSpeedsNP, NULL);
   IDDefNumber (&HorOffsNP, NULL);
   IDDefNumber (&UpTimeNP, NULL);
//   IDDefSwitch (&FanStatusSP, NULL);


   // Firmware Group
   IDDefText   (&SiteNameTP, NULL);
   IDDefNumber (&geoNP, NULL);
   IDDefText   (&VersionInfo, NULL);
   IDDefNumber (&cycleCounterNP, NULL);

   // OBJECT_GROUP
   IDDefSwitch (&SunSP, NULL);
   IDDefSwitch (&CasASP, NULL);
   IDDefSwitch (&CygASP, NULL);
   IDDefSwitch (&TauASP, NULL);
   IDDefSwitch (&OriASP, NULL);
   IDDefSwitch (&VirASP, NULL);
   IDDefSwitch (&C273SP, NULL);
   IDDefSwitch (&C295SP, NULL);

   // DEBUG_GROUP
   IDDefNumber (&ObjectEquatorialCoordsNP, NULL);

   /* Send the basic data to the new client if the previous client(s) are already connected. */
   if (ConnectSP.s == IPS_OK) getBasicData();
}

void LX200RT::ISSnoopDevice (XMLEle *root)
{
  INDI_UNUSED(root);
}

void LX200RT::ISNewText (const char *dev, const char *name, char *texts[], char *names[], int n)
{
	int err;
	IText *tp;

	// ignore if not ours //
	if (strcmp (dev, thisDevice)) return;

	// suppress warning
	n=n;

	if (!strcmp(name, PortTP.name) )	{
	   PortTP.s = IPS_OK;
	   tp = IUFindText( &PortTP, names[0] );
	   if (!tp) return;

	   IUSaveText(&PortTP.tp[0], texts[0]);
	   IDSetText (&PortTP, NULL);
	   return;
	}

	if (!strcmp (name, SiteNameTP.name) ) {
	   if (checkPower(&SiteNameTP))
	   return;

	   if ( ( err = setSiteName(fd, texts[0], currentSiteNum) < 0) ) {
	      handleError(&SiteNameTP, err, "Setting site name");
	      return;
	   }
	   SiteNameTP.s = IPS_OK;
	   tp = IUFindText(&SiteNameTP, names[0]);
	   tp->text = new char[strlen(texts[0])+1];
	   strcpy(tp->text, texts[0]);
   	IDSetText(&SiteNameTP , "Site name updated");
	   return;
   }

   #ifdef HAVE_NOVA_H
   if (!strcmp (name, TimeTP.name)) {
	   if (checkPower(&TimeTP))
	   return;

	   if (simulation) {
		   TimeTP.s = IPS_OK;
		   IUSaveText(&TimeTP.tp[0], texts[0]);
		   IDSetText(&TimeTP, "Simulated time updated.");
		   return;
	   }

	   struct ln_date utm;
	   struct ln_zonedate ltm;

      if (*((int *) UTCOffsetN[0].aux0) == 0) {
		   TimeTP.s = IPS_IDLE;
		   IDSetText(&TimeTP, "You must set the UTC Offset property first.");
		   return;
	   }

	   if (extractISOTime(texts[0], &utm) < 0) {
	      TimeTP.s = IPS_IDLE;
	      IDSetText(&TimeTP , "Time invalid");
	      return;
	   }

	   // update JD
      JD = ln_get_julian_day(&utm);
      IDLog("New JD is %f\n", (float) JD);

	   ln_date_to_zonedate(&utm, &ltm, UTCOffsetN[0].value*3600.0);

	   // Set Local Time
	   if ( ( err = setLocalTime(fd, ltm.hours, ltm.minutes, ltm.seconds) < 0) ) {
	      handleError(&TimeTP, err, "Setting local time");
        	return;
	   }

	   if (!strcmp(dev, "LX200 GPS")) {
		   if ( ( err = setCalenderDate(fd, utm.days, utm.months, utm.years) < 0) ) {
		  	   handleError(&TimeTP, err, "Setting TimeT date.");
		  		return;
			}
	   } else {
		   if ( ( err = setCalenderDate(fd, ltm.days, ltm.months, ltm.years) < 0) ) {
		  		handleError(&TimeTP, err, "Setting local date.");
		  		return;
			}
	   }

	   // Everything Ok, save time value
	   if (IUUpdateText(&TimeTP, texts, names, n) < 0)	return;

	   TimeTP.s = IPS_OK;
 	   IDSetText(&TimeTP , "Time updated to %s, updating planetary data...", texts[0]);

	   // Also update telescope's sidereal time
	   getSDTime(fd, &SDTimeN[0].value);
	   IDSetNumber(&SDTimeNP, NULL);
	}
	#endif
}


void LX200RT::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
	int h =0, m =0, s=0, err;
	double newRA =0, newDEC =0;
   double newAlt=0, newAz=0;
   char altStr[64], azStr[64];

	// ignore if not ours //
	if (strcmp (dev, thisDevice))
	    return;

        // Slewing Accuracy
        if (!strcmp (name, SlewAccuracyNP.name))
	{
		if (!IUUpdateNumber(&SlewAccuracyNP, values, names, n))
		{
			SlewAccuracyNP.s = IPS_OK;
			IDSetNumber(&SlewAccuracyNP, NULL);
			return;
		}

		SlewAccuracyNP.s = IPS_ALERT;
		IDSetNumber(&SlewAccuracyNP, "unknown error while setting tracking precision");
		return;
	}

	#ifdef HAVE_NOVA_H
	// DST Correct TimeT Offset
	if (!strcmp (name, UTCOffsetNP.name))
	{
		if (strcmp(names[0], UTCOffsetN[0].name))
		{
			UTCOffsetNP.s = IPS_ALERT;
			IDSetNumber( &UTCOffsetNP , "Unknown element %s for property %s.", names[0], UTCOffsetNP.label);
			return;
		}

		if (!simulation)
			if ( ( err = setUTCOffset(fd, (values[0] * -1.0)) < 0) )
			{
		        	UTCOffsetNP.s = IPS_ALERT;
	        		IDSetNumber( &UTCOffsetNP , "Setting UTC Offset failed.");
				return;
			}

		*((int *) UTCOffsetN[0].aux0) = 1;
		IUUpdateNumber(&UTCOffsetNP, values, names, n);
		UTCOffsetNP.s = IPS_OK;
		IDSetNumber(&UTCOffsetNP, NULL);
		return;
	}
	#endif

	// Equatorial Coordinates
   if (!strcmp (name, EquatorialCoordsNP.name))
	{
	   int i=0, nset=0;

	   if (checkPower(&EquatorialCoordsNP)) return;

	   for (nset = i = 0; i < n; i++) {
		   INumber *eqp = IUFindNumber (&EquatorialCoordsNP, names[i]);
		   if (eqp == &EquatorialCoordsN[0]) {
            newRA = values[i];
		      nset += newRA >= 0 && newRA <= 24.0;
		   } else if (eqp == &EquatorialCoordsN[1]) {
		      newDEC = values[i];
		      nset += newDEC >= -90.0 && newDEC <= 90.0;
		   }
	   }

	   if (nset == 2) {
	      /*EquatorialCoordsWNP.s = IPS_BUSY;*/
	      char RAStr[32], DecStr[32];

	      fs_sexa(RAStr, newRA, 2, 3600);
	      fs_sexa(DecStr, newDEC, 2, 3600);

         #ifdef INDI_DEBUG
	      IDLog("We received JNOW RA %g - DEC %g\n", newRA, newDEC);
	      IDLog("We received JNOW RA %s - DEC %s\n", RAStr, DecStr);
	      #endif

	      if (!simulation) {
	         if ( (err = setObjectRA(fd, newRA)) < 0 || ( err = setObjectDEC(fd, newDEC)) < 0) {
	            EquatorialCoordsNP.s = IPS_ALERT ;
	            IDSetNumber(&EquatorialCoordsNP, NULL);
	            handleError(&EquatorialCoordsNP, err, "Setting RA/DEC");
	            return;
	         }
	      }

         // read back equatorial object coordinates
         if ( (err = getObjectRA(fd, &ObjectEquatorialCoordsNP.np[0].value)) < 0 || (err = getObjectDEC(fd, &ObjectEquatorialCoordsNP.np[1].value)) < 0) {
	         ObjectEquatorialCoordsNP.s = IPS_ALERT;
	         IDSetNumber(&ObjectEquatorialCoordsNP, NULL);
	         handleError(&ObjectEquatorialCoordsNP, err, "Getting Object RA/DEC");
	         return;
	      }

		   IDSetNumber (&ObjectEquatorialCoordsNP, NULL);
			ObjectEquatorialCoordsNP.s = IPS_OK;

         /* wildi In principle this line is according to the discussion */
	      /* In case the telescope is slewing, we have to abort that. No status change here */
         /* EquatorialCoordsWNP.s = IPS_OK; */
         IDSetNumber(&EquatorialCoordsNP, NULL);
	      targetRA  = newRA;
	      targetDEC = newDEC;

	      if (handleCoordSet()) {
	         EquatorialCoordsNP.s = IPS_ALERT;
	         IDSetNumber(&EquatorialCoordsNP, NULL);
	      }
	   } // end nset
	   else {
		   EquatorialCoordsNP.s = IPS_ALERT;
		   IDSetNumber(&EquatorialCoordsNP, "RA or Dec missing or invalid");
	   }

	   return;
   } /* end EquatorialCoordsWNP */

	// Update Sidereal Time
   if ( !strcmp (name, SDTimeNP.name) ) {
	   if (checkPower(&SDTimeNP))
	   return;

	   if (values[0] < 0.0 || values[0] > 24.0) {
	      SDTimeNP.s = IPS_IDLE;
	      IDSetNumber(&SDTimeNP , "Time invalid");
	      return;
	   }

	   getSexComponents(values[0], &h, &m, &s);
	   IDLog("Siderial Time is %02d:%02d:%02d\n", h, m, s);

	   if ( ( err = setSDTime(fd, h, m, s) < 0) ) {
	      handleError(&SDTimeNP, err, "Setting siderial time");
         return;
	   }

	   SDTimeNP.np[0].value = values[0];
	   SDTimeNP.s = IPS_OK;

	   IDSetNumber(&SDTimeNP , "Sidereal time updated to %02d:%02d:%02d", h, m, s);

	   return;
   }

	// Update Geographical Location
	if (!strcmp (name, geoNP.name)) {
	   // new geographic coords
	   double newLong = 0, newLat = 0;
	   int i, nset;
	   char msg[128];

	   if (checkPower(&geoNP)) return;

      for (nset = i = 0; i < n; i++) {
		   INumber *geop = IUFindNumber (&geoNP, names[i]);
		   if (geop == &geo[0])	{
		      newLat = values[i];
		      nset += newLat >= -90.0 && newLat <= 90.0;
		   } else if (geop == &geo[1]) {
		      newLong = values[i];
		      nset += newLong >= 0.0 && newLong < 360.0;
		   }
	   }

	   if (nset == 2) {
		   char l[32], L[32];
		   geoNP.s = IPS_OK;
		   fs_sexa (l, newLat, 3, 3600);
		   fs_sexa (L, newLong, 4, 3600);

		   if (!simulation) {
			   if ( ( err = setSiteLongitude(fd, 360.0 - newLong) < 0) ) {
		   		handleError(&geoNP, err, "Setting site longitude coordinates");
		   		return;
	         }
			   if ( ( err = setSiteLatitude(fd, newLat) < 0) ) {
		   		handleError(&geoNP, err, "Setting site latitude coordinates");
		   		return;
	        	}
		   }

		   geoNP.np[0].value = newLat;
		   geoNP.np[1].value = newLong;
		   snprintf (msg, sizeof(msg), "Site location updated to Lat %.32s - Long %.32s", l, L);
	   } else {
		   geoNP.s = IPS_IDLE;
		   strcpy(msg, "Lat or Long missing or invalid");
	   }
	   IDSetNumber (&geoNP, "%s", msg);
	   return;
   }

	// Update Frequency
	if ( !strcmp (name, TrackingFreqNP.name) ) {
	   if (checkPower(&TrackingFreqNP)) return;

	   IDLog("Trying to set track freq of: %f\n", values[0]);

	   if ( ( err = setTrackFreq(fd, values[0])) < 0) {
         handleError(&TrackingFreqNP, err, "Setting tracking frequency");
	      return;
	   }

	   TrackingFreqNP.s = IPS_OK;
	   TrackingFreqNP.np[0].value = values[0];
	   IDSetNumber(&TrackingFreqNP, "Tracking frequency set to %04.1f", values[0]);
	   if (trackingMode != LX200_TRACK_MANUAL) {
	      trackingMode = LX200_TRACK_MANUAL;
	      TrackModeS[0].s = ISS_OFF;
	      TrackModeS[1].s = ISS_OFF;
	      TrackModeS[2].s = ISS_ON;
	      TrackModeSP.s   = IPS_OK;
	      selectTrackingMode(fd, trackingMode);
	      IDSetSwitch(&TrackModeSP, NULL);
	   }
	   return;
   }

   // Horizontal Coordinates
   if ( !strcmp (name, HorizontalCoordsNP.name) ) {
      int i=0, nset=0;

      if (checkPower(&HorizontalCoordsNP)) return;

      for (nset = i = 0; i < n; i++) {
		   INumber *horp = IUFindNumber (&HorizontalCoordsNP, names[i]);
		   if (horp == &HorizontalCoordsN[1]) {
            newAlt = values[i];
		      nset += newAlt >= -90. && newAlt <= 90.0;
		   } else if (horp == &HorizontalCoordsN[0]) {
		      newAz = values[i];
		      nset += newAz >= 0. && newAz <= 360.0;
		   }
	   }

	   if (nset == 2) {
	      if ( (err = setObjAz(fd, newAz)) < 0 || (err = setObjAlt(fd, newAlt)) < 0) {
	         handleError(&HorizontalCoordsNP, err, "Setting Alt/Az");
	         return;
	   	}

         // read back equatorial object coordinates
         if ( (err = getObjectRA(fd, &ObjectEquatorialCoordsNP.np[0].value)) < 0 || (err = getObjectDEC(fd, &ObjectEquatorialCoordsNP.np[1].value)) < 0) {
	         ObjectEquatorialCoordsNP.s = IPS_ALERT;
	         IDSetNumber(&ObjectEquatorialCoordsNP, NULL);
	         handleError(&ObjectEquatorialCoordsNP, err, "Getting Object RA/DEC");
	         return;
	      }

		   IDSetNumber (&ObjectEquatorialCoordsNP, NULL);
			ObjectEquatorialCoordsNP.s = IPS_OK;


         //HorizontalCoordsWNP.s = IPS_OK;
	      //HorizontalCoordsWNP.n[0].value = values[0];
	      //HorizontalCoordsWNP.n[1].value = values[1];
	      targetAZ  = newAz;
	      targetALT = newAlt;

	      fs_sexa(azStr, targetAZ, 2, 3600);
	      fs_sexa(altStr, targetALT, 2, 3600);

	      //IDSetNumber (&HorizontalCoordsWNP, "Attempting to slew to Alt %s - Az %s", altStr, azStr);
	      handleAltAzSlew();
	   } else {
		   HorizontalCoordsNP.s = IPS_ALERT;
		   IDSetNumber(&HorizontalCoordsNP, "Altitude or Azimuth missing or invalid");
	   }
	   return;
   }

   // Horizontal Offsets
   if ( !strcmp (name, HorOffsNP.name) ) {
      int i=0, nset=0;

      if (checkPower(&HorOffsNP)) return;

      for (nset = i = 0; i < n; i++) {
		   INumber *horp = IUFindNumber (&HorOffsNP, names[i]);
		   if (horp == &HorOffsN[1]) {
            newAlt = values[i];
		      nset += newAlt >= -180. && newAlt <= 180.0;
		   } else if (horp == &HorOffsN[0]) {
		      newAz = values[i];
		      nset += newAz >= -180. && newAz <= 180.0;
		   }
	   }

	   if (nset == 2) {
	      if ( (err = setAzOffs(fd, newAz)) < 0 || (err = setAltOffs(fd, newAlt)) < 0) {
	         handleError(&HorOffsNP, err, "Setting Alt/Az Offsets");
	         return;
	   	}

         // read back horizontal offsets
         if ( 	getAzOffs(fd, &HorOffsN[0].value) < 0
		      || getAltOffs(fd, &HorOffsN[1].value) < 0  )
            IDMessage(thisDevice, "Failed to get position offsets from device.");
         else
            IDSetNumber (&HorOffsNP, NULL);

			HorOffsNP.s = IPS_OK;

	   } else {
		   HorOffsNP.s = IPS_ALERT;
		   IDSetNumber(&HorOffsNP, "Altitude or Azimuth offset missing or invalid");
	   }
	   return;
   }

}


void LX200RT::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
	int err=0, index=0;
	double newRA =0, newDEC =0;
	INDI_UNUSED(names);

	// ignore if not ours //
	if (strcmp (thisDevice, dev)) return;

	// FIRST Switch ALWAYS for power
	if (!strcmp (name, ConnectSP.name)) {
      bool connectionEstablished = (ConnectS[0].s == ISS_ON);
	   if (IUUpdateSwitch(&ConnectSP, states, names, n) < 0) return;
	   if ( (connectionEstablished && ConnectS[0].s == ISS_ON) || (!connectionEstablished && ConnectS[1].s == ISS_ON))
	   {
		   ConnectSP.s = IPS_OK;
		   IDSetSwitch(&ConnectSP, NULL);
		   return;
	   }
   	connectTelescope();
	   return;
	}

	// Coord set
	if (!strcmp(name, OnCoordSetSP.name)) {
  	   if (checkPower(&OnCoordSetSP)) return;

	   if (IUUpdateSwitch(&OnCoordSetSP, states, names, n) < 0) return;
	   currentSet = getOnSwitch(&OnCoordSetSP);
	   OnCoordSetSP.s = IPS_OK;
	   IDSetSwitch(&OnCoordSetSP, NULL);
	}

	// Abort Slew
	if (!strcmp (name, AbortSlewSP.name)) {
	   if (checkPower(&AbortSlewSP)) {
	      AbortSlewSP.s = IPS_IDLE;
	      IDSetSwitch(&AbortSlewSP, NULL);
	      return;
	   }

	   IUResetSwitch(&AbortSlewSP);
	   if (abortSlew(fd) < 0) {
		   AbortSlewSP.s = IPS_ALERT;
		   IDSetSwitch(&AbortSlewSP, NULL);
		   return;
	   }

	   if (EquatorialCoordsNP.s == IPS_BUSY) {
		   AbortSlewSP.s = IPS_OK;
		   EquatorialCoordsNP.s       = IPS_IDLE;
		   IDSetSwitch(&AbortSlewSP, "Slew aborted.");
		   IDSetNumber(&EquatorialCoordsNP, NULL);
      }
	   else if (MovementNSSP.s == IPS_BUSY || MovementWESP.s == IPS_BUSY) {
		   MovementNSSP.s  = MovementWESP.s =  IPS_IDLE;

		   AbortSlewSP.s = IPS_OK;
		   EquatorialCoordsNP.s       = IPS_IDLE;
		   IUResetSwitch(&MovementNSSP);
		   IUResetSwitch(&MovementWESP);
		   IUResetSwitch(&AbortSlewSP);

		   IDSetSwitch(&AbortSlewSP, "Slew aborted.");
		   IDSetSwitch(&MovementNSSP, NULL);
		   IDSetSwitch(&MovementWESP, NULL);
		   IDSetNumber(&EquatorialCoordsNP, NULL);
	   } else {
	      AbortSlewSP.s = IPS_OK;
	      IDSetSwitch(&AbortSlewSP, NULL);
	   }

	   return;
	}

	// Alignment
	if (!strcmp (name, AlignmentSw.name)) {
	   if (checkPower(&AlignmentSw)) return;

	   if (IUUpdateSwitch(&AlignmentSw, states, names, n) < 0) return;
	   index = getOnSwitch(&AlignmentSw);

	   if ( ( err = setAlignmentMode(fd, index) < 0) ) {
	      handleError(&AlignmentSw, err, "Setting alignment");
         return;
	   }

	   AlignmentSw.s = IPS_OK;
         IDSetSwitch (&AlignmentSw, NULL);
	   return;

	}

   // Sites
	if (!strcmp (name, SitesSP.name)) {
	   int dd=0, mm=0;

	   if (checkPower(&SitesSP)) return;

	   if (IUUpdateSwitch(&SitesSP, states, names, n) < 0) return;
	   currentSiteNum = getOnSwitch(&SitesSP) + 1;

	   if ( ( err = selectSite(fd, currentSiteNum) < 0) ) {
   	   handleError(&SitesSP, err, "Selecting sites");
	      return;
	   }

	   if ( ( err = getSiteLatitude(fd, &dd, &mm) < 0)) {
	      handleError(&SitesSP, err, "Selecting sites");
	      return;
	   }

	   if (dd > 0) geoNP.np[0].value = dd + mm / 60.0;
	   else geoNP.np[0].value = dd - mm / 60.0;

	   if ( ( err = getSiteLongitude(fd, &dd, &mm) < 0)) {
	      handleError(&SitesSP, err, "Selecting sites");
		   return;
	   }

	   if (dd > 0) geoNP.np[1].value = 360.0 - (dd + mm / 60.0);
	   else geoNP.np[1].value = (dd - mm / 60.0) * -1.0;

	   getSiteName(fd, SiteNameTP.tp[0].text, currentSiteNum);

	   IDLog("Selecting site %d\n", currentSiteNum);

	   geoNP.s = SiteNameTP.s = SitesSP.s = IPS_OK;

	   IDSetNumber (&geoNP, NULL);
	   IDSetText   (&SiteNameTP, NULL);
      IDSetSwitch (&SitesSP, NULL);
	   return;
	}

	// Slew mode
	if (!strcmp (name, SlewModeSP.name)) {
	   if (checkPower(&SlewModeSP)) return;

	   if (IUUpdateSwitch(&SlewModeSP, states, names, n) < 0) return;
	   index = getOnSwitch(&SlewModeSP);

	   if ( ( err = setSlewMode(fd, index) < 0) ) {
         handleError(&SlewModeSP, err, "Setting slew mode");
         return;
	   }

      SlewModeSP.s = IPS_OK;
	   IDSetSwitch(&SlewModeSP, NULL);
	   return;
	}

	// Movement (North/South)
	if (!strcmp (name, MovementNSSP.name)) {
	   if (checkPower(&MovementNSSP)) return;

	   int last_move=-1;
      int current_move = -1;

	   // -1 means all off previously
	   last_move = getOnSwitch(&MovementNSSP);

	   if (IUUpdateSwitch(&MovementNSSP, states, names, n) < 0)	return;

	   current_move = getOnSwitch(&MovementNSSP);

	   // Previosuly active switch clicked again, so let's stop.
	   if (current_move == last_move) {
		   HaltMovement(fd, (current_move == 0) ? LX200_NORTH : LX200_SOUTH);
		   IUResetSwitch(&MovementNSSP);
	    	//MovementNSSP.s = IPS_IDLE;
	      IDSetSwitch(&MovementNSSP, "Moving toward %s", (current_move == LX200_NORTH) ? "North stopped" : "South stopped");
		   return;
	   }

	   #ifdef INDI_DEBUG
      IDLog("Current Move: %d - Previous Move: %d\n", current_move, last_move);
	   #endif

	   // 0 (North) or 1 (South)
	   last_move      = current_move;

	   // Correction for LX200 Driver: North 0 - South 3
	   current_move = (current_move == 0) ? LX200_NORTH : LX200_SOUTH;

      if ( ( err = MoveTo(fd, current_move) < 0) )	{
	     	handleError(&MovementNSSP, err, "Setting motion direction");
 		   return;
	   }

//	   MovementNSSP.s = IPS_BUSY;
	   IDSetSwitch(&MovementNSSP, "Moving toward %s", (current_move == LX200_NORTH) ? "North" : "South");
	   return;
	}

	// Movement (West/East)
	if (!strcmp (name, MovementWESP.name)) {
	   if (checkPower(&MovementWESP)) return;

	   int last_move=-1;
      int current_move = -1;

	   // -1 means all off previously
	   last_move = getOnSwitch(&MovementWESP);

	   if (IUUpdateSwitch(&MovementWESP, states, names, n) < 0)	return;

	   current_move = getOnSwitch(&MovementWESP);

	   // Previously active switch clicked again, so let's stop.
	   if (current_move == last_move) {
		   HaltMovement(fd, (current_move == 0) ? LX200_WEST : LX200_EAST);
		   IUResetSwitch(&MovementWESP);
	    	//MovementWESP.s = IPS_IDLE;
   	   IDSetSwitch(&MovementWESP, "Moving toward %s", (current_move == LX200_WEST) ? "East stopped" : "West stopped");
		   return;
	   }

	   #ifdef INDI_DEBUG
      IDLog("Current Move: %d - Previous Move: %d\n", current_move, last_move);
	   #endif

	   // 0 (West) or 1 (East)
	   last_move      = current_move;

	   // Correction for LX200 Driver: West 1 - East 2
	   current_move = (current_move == 0) ? LX200_WEST : LX200_EAST;

      if ( ( err = MoveTo(fd, current_move) < 0) ) {
	      handleError(&MovementWESP, err, "Setting motion direction");
 		 	return;
	   }

	   //MovementWESP.s = IPS_BUSY;
	   IDSetSwitch(&MovementWESP, "Moving toward %s", (current_move == LX200_WEST) ? "West" : "East");
	   return;
	}

	// Tracking mode
	if (!strcmp (name, TrackModeSP.name)) {
	   if (checkPower(&TrackModeSP))
	   return;

	   IUResetSwitch(&TrackModeSP);
	   IUUpdateSwitch(&TrackModeSP, states, names, n);
	   trackingMode = getOnSwitch(&TrackModeSP);

	   if ( ( err = selectTrackingMode(fd, trackingMode) < 0) ) {
	      handleError(&TrackModeSP, err, "Setting tracking mode.");
		   return;
	   }

      getTrackFreq(fd, &TrackFreqN[0].value);
	   TrackModeSP.s = IPS_OK;
	   IDSetNumber(&TrackingFreqNP, NULL);
	   IDSetSwitch(&TrackModeSP, NULL);
	   return;
	}


   // Park
   if (!strcmp(name, ParkSP.name)) {
	   if (checkPower(&ParkSP)) return;

   	if (EquatorialCoordsNP.s == IPS_BUSY) {
	      if (abortSlew(fd) < 0) {
		      AbortSlewSP.s = IPS_ALERT;
		      IDSetSwitch(&AbortSlewSP, NULL);
            slewError(err);
		      return;
	      }

	      AbortSlewSP.s = IPS_OK;
	      EquatorialCoordsNP.s       = IPS_IDLE;
         IDSetSwitch(&AbortSlewSP, "Slew aborted.");
	      IDSetNumber(&EquatorialCoordsNP, NULL);

	      if (MovementNSSP.s == IPS_BUSY || MovementWESP.s == IPS_BUSY) {
		      MovementNSSP.s  = MovementWESP.s =  IPS_IDLE;
		      EquatorialCoordsNP.s       = IPS_IDLE;
		      IUResetSwitch(&MovementNSSP);
		      IUResetSwitch(&MovementWESP);
		      IUResetSwitch(&AbortSlewSP);

		      IDSetSwitch(&MovementNSSP, NULL);
		      IDSetSwitch(&MovementWESP, NULL);
	      }

	      // sleep for 200 mseconds
	      usleep(200000);
	   }

      if (slewToPark(fd) < 0) {
		   ParkSP.s = IPS_ALERT;
		   IDSetSwitch(&ParkSP, "Parking Failed.");
		   return;
	   }

	   ParkSP.s = IPS_OK;
	   //ConnectSP.s   = IPS_IDLE;
	   //ConnectSP.sp[0].s = ISS_OFF;
	   //ConnectSP.sp[1].s = ISS_ON;
	   //tty_disconnect(fd);
	   IDSetSwitch(&ParkSP, "The telescope is slewing to park position. Please wait...");
	   //IDSetSwitch(&ConnectSP, NULL);
	   return;
   }

   // Fan
   if (!strcmp(name, FanStatusSP.name)) {
      if (checkPower(&FanStatusSP))
      return;

      IUResetSwitch(&FanStatusSP);
      IUUpdateSwitch(&FanStatusSP, states, names, n);
      index = getOnSwitch(&FanStatusSP);

	   if (index == 0) {
	      if ( (err = turnFanOn(fd)) < 0) {
	         handleError(&FanStatusSP, err, "Changing fan status");
	         return;
	      }
	   } else {
	      if ( (err = turnFanOff(fd)) < 0) {
	         handleError(&FanStatusSP, err, "Changing fan status");
	         return;
	      }
	   }

	   FanStatusSP.s = IPS_OK;
	   IDSetSwitch (&FanStatusSP, index == 0 ? "Fan is ON" : "Fan is OFF");
	   return;
   }

   // Home Search
   if (!strcmp(name, HomeSearchSP.name)) {
      if (checkPower(&HomeSearchSP)) return;

      IUResetSwitch(&HomeSearchSP);
      IUUpdateSwitch(&HomeSearchSP, states, names, n);
      index = getOnSwitch(&HomeSearchSP);

	   if (index == 0) seekHomeAndSave(fd);
	   else seekHomeAndSet(fd);

	   HomeSearchSP.s = IPS_BUSY;
	   IDSetSwitch (&HomeSearchSP, index == 0 ? "Seek Home and Save" : "Seek Home and Set");
	   return;
   }

   // Sun
   if (!strcmp(name, SunSP.name)) {
      if (checkPower(&SunSP)) return;

      IUResetSwitch(&SunSP);
//      IUUpdateSwitch(&SunSP, states, names, n);

      newRA = 0.0;
      newDEC = 0.0;

//	   SunSP.s = IPS_BUSY;
	   IDSetSwitch (&SunSP, "Slew to Sun");
   }

   // Cygnus A
   if (!strcmp(name, CygASP.name)) {
      if (checkPower(&CygASP)) return;

      IUResetSwitch(&CygASP);
//      IUUpdateSwitch(&CygASP, states, names, n);

      newRA = 19.992;
      newDEC = 40.733;

//	   CygASP.s = IPS_BUSY;
	   IDSetSwitch (&CygASP, "Slew to Cygnus A");
   }

   // Cas A
   if (!strcmp(name, CasASP.name)) {
      if (checkPower(&CasASP)) return;

      IUResetSwitch(&CasASP);
//      IUUpdateSwitch(&CasASP, states, names, n);

      newRA = 23.3883;
      newDEC = 58.8167;

//	   CasASP.s = IPS_BUSY;
	   IDSetSwitch (&CasASP, "Slew to Cassiopeia A");
   }


   // Tau A
   if (!strcmp(name, TauASP.name)) {
      if (checkPower(&TauASP)) return;

      IUResetSwitch(&TauASP);

      newRA = 5.575;
      newDEC = 22.016;

//	   TauASP.s = IPS_BUSY;
	   IDSetSwitch (&TauASP, "Slew to Taurus A");
   }

   // Ori A
   if (!strcmp(name, OriASP.name)) {
      if (checkPower(&OriASP)) return;

      IUResetSwitch(&OriASP);

      newRA = 5.55;
      newDEC = -5.425;

//	   OriASP.s = IPS_BUSY;
	   IDSetSwitch (&OriASP, "Slew to Orion A");
   }

   // Vir A
   if (!strcmp(name, VirASP.name)) {
      if (checkPower(&VirASP)) return;

      IUResetSwitch(&VirASP);

      newRA = 12.513;
      newDEC = 12.391;

//	   VirASP.s = IPS_BUSY;
	   IDSetSwitch (&VirASP, "Slew to Virgo A");
   }

   // 3C273
   if (!strcmp(name, C273SP.name)) {
      if (checkPower(&C273SP)) return;

      IUResetSwitch(&C273SP);

      newRA = 12.4834;
      newDEC = 2.05;

//	   C273SP.s = IPS_BUSY;
	   IDSetSwitch (&C273SP, "Slew to 3C273");
   }

   // 3C295
   if (!strcmp(name, C295SP.name)) {
      if (checkPower(&C295SP)) return;

      IUResetSwitch(&C295SP);

      newRA = 12.1883;
      newDEC = 52.205;

//	   C295SP.s = IPS_BUSY;
	   IDSetSwitch (&C295SP, "Slew to 3C295");
   }


   if (!strcmp(name, CygASP.name) || !strcmp(name, CasASP.name) ||
       !strcmp(name, SunSP.name) || !strcmp(name, TauASP.name) ||
       !strcmp(name, OriASP.name) || !strcmp(name, VirASP.name) ||
       !strcmp(name, C273SP.name) || !strcmp(name, C295SP.name))
   {
	   /*EquatorialCoordsWNP.s = IPS_BUSY;*/
	   char RAStr[32], DecStr[32];

	   fs_sexa(RAStr, newRA, 2, 3600);
	   fs_sexa(DecStr, newDEC, 2, 3600);

      #ifdef INDI_DEBUG
	   IDLog("We received JNOW RA %g - DEC %g\n", newRA, newDEC);
	   IDLog("We received JNOW RA %s - DEC %s\n", RAStr, DecStr);
	   #endif

	   if (!simulation) {
	      if ( (err = setObjectRA(fd, newRA)) < 0 || ( err = setObjectDEC(fd, newDEC)) < 0) {
	         EquatorialCoordsNP.s = IPS_ALERT ;
	         IDSetNumber(&EquatorialCoordsNP, NULL);
	         handleError(&EquatorialCoordsNP, err, "Setting RA/DEC");
	         return;
	      }
	   }

      // read back equatorial object coordinates
      if ( (err = getObjectRA(fd, &ObjectEquatorialCoordsNP.np[0].value)) < 0 || (err = getObjectDEC(fd, &ObjectEquatorialCoordsNP.np[1].value)) < 0) {
	      ObjectEquatorialCoordsNP.s = IPS_ALERT;
	      IDSetNumber(&ObjectEquatorialCoordsNP, NULL);
	      handleError(&ObjectEquatorialCoordsNP, err, "Getting Object RA/DEC");
	      return;
	   }

		IDSetNumber (&ObjectEquatorialCoordsNP, NULL);
		ObjectEquatorialCoordsNP.s = IPS_OK;

      /* wildi In principle this line is according to the discussion */
	   /* In case the telescope is slewing, we have to abort that. No status change here */
      /* EquatorialCoordsWNP.s = IPS_OK; */
      IDSetNumber(&EquatorialCoordsNP, NULL);
	   targetRA  = newRA;
	   targetDEC = newDEC;

	   if (handleCoordSet()) {
	      EquatorialCoordsNP.s = IPS_ALERT;
	      IDSetNumber(&EquatorialCoordsNP, NULL);
	   }

   }

}

void LX200RT::handleError(ISwitchVectorProperty *svp, int err, const char *msg)
{

  svp->s = IPS_ALERT;

  /* First check to see if the telescope is connected */
    if (check_lx200_connection(fd))
    {
      /* The telescope is off locally */
      ConnectS[0].s = ISS_OFF;
      ConnectS[1].s = ISS_ON;
      ConnectSP.s = IPS_BUSY;
      IDSetSwitch(&ConnectSP, "Telescope is not responding to commands, will retry in 10 seconds.");

      IDSetSwitch(svp, NULL);
      IEAddTimer(10000, retryConnection, &fd);
      return;
    }

   /* If the error is a time out, then the device doesn't support this property or busy*/
      if (err == -2)
      {
       svp->s = IPS_ALERT;
       IDSetSwitch(svp, "Device timed out. Current device may be busy or does not support %s. Will retry again.", msg);
      }
      else
    /* Changing property failed, user should retry. */
       IDSetSwitch( svp , "%s failed.", msg);

       fault = true;
}

void LX200RT::handleError(INumberVectorProperty *nvp, int err, const char *msg)
{

  nvp->s = IPS_ALERT;

  /* First check to see if the telescope is connected */
    if (check_lx200_connection(fd))
    {
      /* The telescope is off locally */
      ConnectS[0].s = ISS_OFF;
      ConnectS[1].s = ISS_ON;
      ConnectSP.s = IPS_BUSY;
      IDSetSwitch(&ConnectSP, "Telescope is not responding to commands, will retry in 10 seconds.");

      IDSetNumber(nvp, NULL);
      IEAddTimer(10000, retryConnection, &fd);
      return;
    }

   /* If the error is a time out, then the device doesn't support this property */
      if (err == -2)
      {
       nvp->s = IPS_ALERT;
       IDSetNumber(nvp, "Device timed out. Current device may be busy or does not support %s. Will retry again.", msg);
      }
      else
    /* Changing property failed, user should retry. */
       IDSetNumber( nvp , "%s failed.", msg);

       fault = true;
}

void LX200RT::handleError(ITextVectorProperty *tvp, int err, const char *msg)
{

   tvp->s = IPS_ALERT;

   /* First check to see if the telescope is connected */
   if (check_lx200_connection(fd))
   {
      /* The telescope is off locally */
      ConnectS[0].s = ISS_OFF;
      ConnectS[1].s = ISS_ON;
      ConnectSP.s = IPS_BUSY;
      IDSetSwitch(&ConnectSP, "Telescope is not responding to commands, will retry in 10 seconds.");

      IDSetText(tvp, NULL);
      IEAddTimer(10000, retryConnection, &fd);
      return;
   }

   /* If the error is a time out, then the device doesn't support this property */
   if (err == -2)
   {
      tvp->s = IPS_ALERT;
      IDSetText(tvp, "Device timed out. Current device may be busy or does not support %s. Will retry again.", msg);
   }
   else
   /* Changing property failed, user should retry. */
   IDSetText( tvp , "%s failed.", msg);

   fault = true;
}

void LX200RT::correctFault()
{
   fault = false;
   IDMessage(thisDevice, "Telescope is online.");
}

bool LX200RT::isTelescopeOn(void)
{
  //if (simulation) return true;

  return (ConnectSP.sp[0].s == ISS_ON);
}

static void retryConnection(void * p)
{
  int fd = *( (int *) p);

  if (check_lx200_connection(fd))
  {
    ConnectSP.s = IPS_IDLE;
    IDSetSwitch(&ConnectSP, "The connection to the telescope is lost.");
    return;
  }

  ConnectS[0].s = ISS_ON;
  ConnectS[1].s = ISS_OFF;
  ConnectSP.s = IPS_OK;

  IDSetSwitch(&ConnectSP, "The connection to the telescope has been resumed.");

}

void LX200RT::handleAltAzSlew()
{
   int i=0;
	char altStr[64], azStr[64];

	  if (HorizontalCoordsNP.s == IPS_BUSY)
	  {
	     abortSlew(fd);

	     // sleep for 100 mseconds
	     usleep(100000);
	  }

	  if ((i = slewToAltAz(fd)))
	  {
	    HorizontalCoordsNP.s = IPS_ALERT;
	    IDSetNumber(&HorizontalCoordsNP, "Slew is not possible.");
	    return;
	  }

	  HorizontalCoordsNP.s = IPS_BUSY;
	  fs_sexa(azStr, targetAZ, 2, 3600);
	  fs_sexa(altStr, targetALT, 2, 3600);

	  IDSetNumber(&HorizontalCoordsNP, "Slewing to Alt %s - Az %s", altStr, azStr);
//	  IDSetNumber(&HorizontalCoordsNP, NULL);
	  return;
}


void LX200RT::ISPoll()
{
   static int pollcounter = 0;
   double dx, dy;
	/*static int okCounter = 3;*/
	int err=0;
   int searchResult=0;

	if (!isTelescopeOn()) return;

	if (simulation){
		mountSim();
		return;
   }

	if ( (err = getLX200RA(fd, &currentRA)) < 0 || (err = getLX200DEC(fd, &currentDEC)) < 0) {
	   EquatorialCoordsNP.s = IPS_ALERT;
	   IDSetNumber(&EquatorialCoordsNP, NULL);
	   handleError(&EquatorialCoordsNP, err, "Getting RA/DEC");
	   return;
	}

	if (fault) correctFault();

	if (EquatorialCoordsNP.s!=IPS_BUSY) EquatorialCoordsNP.s = IPS_OK;

	if ( fabs(lastRA - currentRA) > (SlewAccuracyN[0].value/(60.0*15.0)) || fabs(lastDEC - currentDEC) > (SlewAccuracyN[1].value/60.0))
   {
	  	lastRA  = currentRA;
		lastDEC = currentDEC;
		IDSetNumber (&EquatorialCoordsNP, NULL);
	}

	switch (EquatorialCoordsNP.s) {
	   case IPS_IDLE:
         break;
      case IPS_BUSY:
         dx = targetRA - currentRA;
         dy = targetDEC - currentDEC;

	      // Wait until acknowledged or within threshold
	      if ( fabs(dx) <= (SlewAccuracyN[0].value/(60.0*15.0)) && fabs(dy) <= (SlewAccuracyN[1].value/60.0))
	      {
	         lastRA  = currentRA;
	         lastDEC = currentDEC;

	         EquatorialCoordsNP.s = IPS_OK;
	         IDSetNumber(&EquatorialCoordsNP, "Slew is complete, target locked...");
	      }
	      break;
	   case IPS_OK:
         break;
	   case IPS_ALERT:
	      break;
	}


   if ( !(pollcounter%5) || HorizontalCoordsNP.s==IPS_BUSY) {
	   if ( (err = getLX200Az(fd, &currentAZ)) < 0 || (err = getLX200Alt(fd, &currentALT)) < 0) {
	     handleError(&HorizontalCoordsNP, err, "Get Alt/Az");
	     return;
	   } else {
	      IDSetNumber (&HorizontalCoordsNP, NULL);
      }
	   if (HorizontalCoordsNP.s!=IPS_BUSY) HorizontalCoordsNP.s = IPS_OK;
   }

  	switch (HomeSearchSP.s)	{
	   case IPS_IDLE:
	      break;
	   case IPS_BUSY:
	      if ( (err = getHomeSearchStatus(fd, &searchResult)) < 0) {
	         handleError(&HomeSearchSP, err, "Home search");
	         return;
	      }

	      if (searchResult == 0) {
	         HomeSearchSP.s = IPS_ALERT;
	         IDSetSwitch(&HomeSearchSP, "Home search failed.");
	      } else if (searchResult == 1) {
	         HomeSearchSP.s = IPS_OK;
	         IDSetSwitch(&HomeSearchSP, "Home search successful.");
	      } else if (searchResult == 2) IDSetSwitch(&HomeSearchSP, "Home search in progress...");
	      else {
	         HomeSearchSP.s = IPS_ALERT;
	         IDSetSwitch(&HomeSearchSP, "Home search error.");
	      }
	      break;
	   case IPS_OK:
	      break;
	   case IPS_ALERT:
	      break;
	}

   // Horizontal Coords
	switch (HorizontalCoordsNP.s){
	   case IPS_IDLE:
	      break;
	   case IPS_BUSY:
// 	    if ( (err = getLX200Az(fd, &currentAZ)) < 0 || (err = getLX200Alt(fd, &currentALT)) < 0)
// 	    {
// 	      handleError(&HorizontalCoordsWNP, err, "Get Alt/Az");
// 	      return;
// 	    }
	      dx = targetAZ - currentAZ;
	      dy = targetALT - currentALT;

         HorizontalCoordsNP.np[1].value = currentALT;
	      HorizontalCoordsNP.np[0].value = currentAZ;

	      // accuracy threshold (3'), can be changed as desired.
	      if ( fabs(dx) <= 0.05 && fabs(dy) <= 0.05)
	      {
		      //HorizontalCoordsWNP.s = IPS_OK;
		      HorizontalCoordsNP.s = IPS_OK;
		      currentAZ = targetAZ;
		      currentALT = targetALT;
            IDSetNumber (&HorizontalCoordsNP, "Slew is complete.");
//		      IDSetNumber (&HorizontalCoordsNP, NULL);
	      } else IDSetNumber (&HorizontalCoordsNP, NULL);
	      break;
	   case IPS_OK:
	      break;
	   case IPS_ALERT:
	      break;
	}

    // Temperature Sensor
    if ((pollcounter%500)==1) {
 		double temp=0.;
 		if ( (err = getTemperature(fd, &temp)) < 0)
          IDMessage(thisDevice, "Failed to get temperature from device.");
       else {
          ControllerTemperatureN[0].value = temp;
 			if (temp>-30. && temp<60.) ControllerTemperatureNP.s = IPS_OK;
 			else if (temp>-60. && temp<85.) ControllerTemperatureNP.s = IPS_BUSY;
 			else ControllerTemperatureNP.s = IPS_ALERT;

 			IDSetNumber (&ControllerTemperatureNP, NULL);
 		}
    }

    // Az Poti
    if ((pollcounter%40)==2) {
       double pot=0.;
 		if ( (err = getAzPoti(fd, &pot)) < 0)
          IDMessage(thisDevice, "Failed to get Az Poti Position from device.");
       else
       {
 			AzPotiN[0].value = pot;
 			double revs=pot;
 			if (fabs(revs)<0.5) AzPotiNP.s=IPS_OK;
 			else if (fabs(revs)<0.75) AzPotiNP.s=IPS_BUSY;
 			else AzPotiNP.s=IPS_ALERT;

    		IDSetNumber (&AzPotiNP, NULL);
 		}
    }

    // Encoders
    if ((pollcounter%15)==3) {
       if ( (err = getEncoders(fd, &EncodersN[0].value, &EncodersN[1].value)) < 0)
       {
          IDMessage(thisDevice, "Failed to get encoder values from device.");
       }
       else {
 			IDSetNumber (&EncodersNP, NULL);
 		}
    }

    // Ref Flags
    int ref1,ref2;
    if ((pollcounter%25)==4) {
       if ( (err = getRefFlags(fd, &ref1, &ref2))<0)
       {
          IDMessage(thisDevice, "Failed to get reference flags from device.");
       }
       else
       {
          if (ref1) RefPosL[0].s=IPS_OK;
			 else RefPosL[0].s=IPS_BUSY;
          if (ref2) RefPosL[1].s=IPS_OK;
 			 else RefPosL[1].s=IPS_BUSY;
 			 if (ref1 && ref2) RefPosLP.s=IPS_OK;
 			 else RefPosLP.s=IPS_BUSY;
          IDSetLight (&RefPosLP, NULL);
       }
    }

    // Motor Speeds
    if ((pollcounter%15)==6)
    {
       if ( (err = getMotorSpeeds(fd, &MotorSpeedsN[0].value, &MotorSpeedsN[1].value)) < 0)
          IDMessage(thisDevice, "Failed to get motor speed values from device.");
       else
          IDSetNumber (&MotorSpeedsNP, NULL);
   	   if (MotorSpeedsN[0].value) MovementWESP.s = IPS_BUSY;
       else MovementWESP.s = IPS_IDLE;
       if (MotorSpeedsN[1].value) MovementNSSP.s = IPS_BUSY;
       else MovementNSSP.s = IPS_IDLE;
       IDSetSwitch(&MovementWESP, NULL);
       IDSetSwitch(&MovementNSSP, NULL);
    }

    // upTime
    if ((pollcounter%2000)==7)
    {
       int value=0;
       if ( (err = getUpTime(fd, &value)) < 0)
          IDMessage(thisDevice, "Failed to get upTime value from device.");
       else {
          UpTimeN[0].value=value/3600.;
          IDSetNumber (&UpTimeNP, NULL);
       }
    }

   if (++pollcounter > 10000) pollcounter = 0;
}


void LX200RT::ISIdleTask()
{
   return;
   if (ConnectSP.sp[0].s!=ISS_ON) return;
	int err=0;
   // Temperature Sensor
		double temp=0.;
		if ( (err = getTemperature(fd, &temp)) < 0)
         IDMessage(thisDevice, "Failed to get temperature from device.");
      else {
         ControllerTemperatureN[0].value = temp;
			if (temp>-30. && temp<60.) ControllerTemperatureNP.s = IPS_OK;
			else if (temp>-60. && temp<85.) ControllerTemperatureNP.s = IPS_BUSY;
			else ControllerTemperatureNP.s = IPS_ALERT;

			IDSetNumber (&ControllerTemperatureNP, NULL);
		}

   // Az Poti
      double pot=0.;
		if ( (err = getAzPoti(fd, &pot)) < 0)
         IDMessage(thisDevice, "Failed to get Az Poti Position from device.");
      else
      {
			AzPotiN[0].value = pot;
			double revs=pot;
			if (fabs(revs)<0.5) AzPotiNP.s=IPS_OK;
			else if (fabs(revs)<0.75) AzPotiNP.s=IPS_BUSY;
			else AzPotiNP.s=IPS_ALERT;

   		IDSetNumber (&AzPotiNP, NULL);
		}

   // Encoders
      if ( (err = getEncoders(fd, &EncodersN[0].value, &EncodersN[1].value)) < 0)
      {
         IDMessage(thisDevice, "Failed to get encoder values from device.");
      }
      else {
			IDSetNumber (&EncodersNP, NULL);
		}

   // Ref Flags
   int ref1,ref2;
      if ( (err = getRefFlags(fd, &ref1, &ref2))<0)
      {
         IDMessage(thisDevice, "Failed to get reference flags from device.");
      }
      else
      {
         if (ref1) RefPosL[0].s=IPS_OK;
         else RefPosL[0].s=IPS_BUSY;
         if (ref2) RefPosL[1].s=IPS_OK;
         else RefPosL[1].s=IPS_BUSY;
         if (ref1 && ref2) RefPosLP.s=IPS_OK;
         else RefPosLP.s=IPS_BUSY;
         IDSetLight (&RefPosLP, NULL);
      }

   // Motor Speeds
      if ( (err = getMotorSpeeds(fd, &MotorSpeedsN[0].value, &MotorSpeedsN[1].value)) < 0)
         IDMessage(thisDevice, "Failed to get motor speed values from device.");
      else
         IDSetNumber (&MotorSpeedsNP, NULL);
  	   if (MotorSpeedsN[0].value) MovementWESP.s = IPS_BUSY;
      else MovementWESP.s = IPS_IDLE;
      if (MotorSpeedsN[1].value) MovementNSSP.s = IPS_BUSY;
      else MovementNSSP.s = IPS_IDLE;
      IDSetSwitch(&MovementWESP, NULL);
      IDSetSwitch(&MovementNSSP, NULL);
	usleep(100000);
}


// wildi nothing changed in LX200Generic::mountSim
void LX200RT::mountSim ()
{
	static struct timeval ltv;
	struct timeval tv;
	double dt, da, dx;
	int nlocked;

	/* update elapsed time since last poll, don't presume exactly POLLMS */
	gettimeofday (&tv, NULL);

	if (ltv.tv_sec == 0 && ltv.tv_usec == 0)
	    ltv = tv;

	dt = tv.tv_sec - ltv.tv_sec + (tv.tv_usec - ltv.tv_usec)/1e6;
	ltv = tv;
	da = SLEWRATE*dt;

	/* Process per current state. We check the state of EQUATORIAL_COORDS and act acoordingly */
	switch (EquatorialCoordsNP.s)
	{

	/* #1 State is idle, update telesocpe at sidereal rate */
	case IPS_IDLE:
	    /* RA moves at sidereal, Dec stands still */
	    currentRA += (SIDRATE*dt/15.);

	   IDSetNumber(&EquatorialCoordsNP, NULL);

	    break;

	case IPS_BUSY:
	    /* slewing - nail it when both within one pulse @ SLEWRATE */
	    nlocked = 0;

	    dx = targetRA - currentRA;

	    if (fabs(dx) <= da)
	    {
		currentRA = targetRA;
		nlocked++;
	    }
	    else if (dx > 0)
	    	currentRA += da/15.;
	    else
	    	currentRA -= da/15.;


	    dx = targetDEC - currentDEC;
	    if (fabs(dx) <= da)
	    {
		currentDEC = targetDEC;
		nlocked++;
	    }
	    else if (dx > 0)
	      currentDEC += da;
	    else
	      currentDEC -= da;

	    if (nlocked == 2)
	    {
		EquatorialCoordsNP.s = IPS_OK;
		//EquatorialCoordsWNP.s = IPS_OK;
		IDSetNumber(&EquatorialCoordsNP, "Now tracking");
//		IDSetNumber(&EquatorialCoordsNP, NULL);
	    } else
		IDSetNumber(&EquatorialCoordsNP, NULL);

	    break;

	case IPS_OK:
	    /* tracking */
	   IDSetNumber(&EquatorialCoordsNP, NULL);
	    break;

	case IPS_ALERT:
	    break;
	}

}

void LX200RT::getBasicData()
{

  int err;
  #ifdef HAVE_NOVA_H
  struct tm *timep;
  time_t ut;
  time (&ut);
  timep = gmtime (&ut);
  strftime (TimeTP.tp[0].text, strlen(TimeTP.tp[0].text), "%Y-%m-%dT%H:%M:%S", timep);

  IDLog("PC UTC time is %s\n", TimeTP.tp[0].text);
  #endif

  getAlignment();

  checkLX200Format(fd);

  if ( (err = getTimeFormat(fd, &timeFormat)) < 0)
     IDMessage(thisDevice, "Failed to retrieve time format from device.");
  else
  {
    timeFormat = (timeFormat == 24) ? LX200_24 : LX200_AM;
    // We always do 24 hours
    if (timeFormat != LX200_24)
      err = toggleTimeFormat(fd);
  }
// wildi proposal
/*
  if ( (err = getLX200RA(fd, &targetRA)) < 0 || (err = getLX200DEC(fd, &targetDEC)) < 0)
  {
     EquatorialCoordsNP.s = IPS_ALERT;
     IDSetNumber(&EquatorialCoordsNP, NULL);
     handleError(&EquatorialCoordsNP, err, "Getting RA/DEC");
     return;
  }
*/
  if (fault)
	correctFault();

//  getLX200RA(fd, &targetRA);
//  getLX200DEC(fd, &targetDEC);

//  EquatorialCoordsNP.np[0].value = targetRA;
//  EquatorialCoordsNP.np[1].value = targetDEC;

//  EquatorialCoordsNP.s = IPS_OK;
//  IDSetNumber (&EquatorialCoordsNP, NULL);

  SiteNameT[0].text = new char[64];

  if ( (err = getSiteName(fd, SiteNameT[0].text, currentSiteNum)) < 0)
    IDMessage(thisDevice, "Failed to get site name from device");
  else
    IDSetText   (&SiteNameTP, NULL);

//  if ( (err = getTrackFreq(fd, &TrackFreqN[0].value)) < 0)
//     IDMessage(thisDevice, "Failed to get tracking frequency from device.");
//  else
//     IDSetNumber (&TrackingFreqNP, NULL);

   //getLX200Az(fd, &currentAZ);
   //getLX200Alt(fd, &currentALT);
   //IDSetNumber (&HorizontalCoordsNP, NULL);

   VersionInfo.tp[0].text = new char[64];
   getVersionDate(fd, VersionInfo.tp[0].text);
   VersionInfo.tp[1].text = new char[64];
   getVersionTime(fd, VersionInfo.tp[1].text);
   VersionInfo.tp[2].text = new char[64];
   getVersionNumber(fd, VersionInfo.tp[2].text);
   VersionInfo.tp[3].text = new char[128];
   getFullVersion(fd, VersionInfo.tp[3].text);
   VersionInfo.tp[4].text = new char[128];
   getProductName(fd, VersionInfo.tp[4].text);

   IDSetText(&VersionInfo, NULL);

/*
   if ( (err = getTemperature(fd, &ControllerTemperatureN[0].value)) < 0)
      IDMessage(thisDevice, "Failed to get temperature from device.");
   else
      IDSetNumber (&ControllerTemperatureNP, NULL);
*/

   if ( 	getAzOffs(fd, &HorOffsN[0].value) < 0
		|| getAltOffs(fd, &HorOffsN[1].value) < 0  )
      IDMessage(thisDevice, "Failed to get position offsets from device.");
   else
      IDSetNumber (&HorOffsNP, NULL);

	int cycles=5;
   if ( (err = getCycleCounter(fd, &cycles)) < 0)
      IDMessage(thisDevice, "Failed to get erase cycle counter from device.");
   else {
		cycleCounter[0].value = (double)cycles;
      if (err) cycleCounterNP.s = IPS_ALERT;
		else cycleCounterNP.s = IPS_OK;

		IDSetNumber (&cycleCounterNP, NULL);
	}

   updateLocation();
   updateTime();
}

int LX200RT::handleCoordSet()
{

  int  err;
  char syncString[256];
  char RAStr[32], DecStr[32];

  switch (currentSet)
  {
    // Slew
    case LX200_TRACK:
          lastSet = LX200_TRACK;
	  if (EquatorialCoordsNP.s == IPS_BUSY)
	  {
	     #ifdef INDI_DEBUG
	     IDLog("Aborting Slew\n");
	     #endif
	     if (abortSlew(fd) < 0)
	     {
		AbortSlewSP.s = IPS_ALERT;
		IDSetSwitch(&AbortSlewSP, NULL);
                slewError(err);
		return (-1);
	     }

	     AbortSlewSP.s = IPS_OK;
	     EquatorialCoordsNP.s       = IPS_IDLE;
             IDSetSwitch(&AbortSlewSP, "Slew aborted.");
	     IDSetNumber(&EquatorialCoordsNP, NULL);

	     if (MovementNSSP.s == IPS_BUSY || MovementWESP.s == IPS_BUSY)
	     {
		MovementNSSP.s  = MovementWESP.s =  IPS_IDLE;
		EquatorialCoordsNP.s       = IPS_IDLE;
		IUResetSwitch(&MovementNSSP);
		IUResetSwitch(&MovementWESP);
		IUResetSwitch(&AbortSlewSP);

		IDSetSwitch(&MovementNSSP, NULL);
		IDSetSwitch(&MovementWESP, NULL);
	      }

	// sleep for 100 mseconds
	usleep(100000);
	}

	if ((err = Slew(fd))) /* Slew reads the '0', that is not the end of the slew */
	{
	    IDMessage(mydev "ERROR Slewing to JNow RA %s - DEC %s\n", RAStr, DecStr);
	    slewError(err);
	    return  -1;
	}

	EquatorialCoordsNP.s = IPS_BUSY;
	fs_sexa(RAStr, targetRA, 2, 3600);
	fs_sexa(DecStr, targetDEC, 2, 3600);
	IDSetNumber(&EquatorialCoordsNP, "Slewing to JNow RA %s - DEC %s", RAStr, DecStr);
	#ifdef INDI_DEBUG
	IDLog("Slewing to JNow RA %s - DEC %s\n", RAStr, DecStr);
   #endif
	break;

    // Sync
    case LX200_SYNC:
          lastSet = LX200_SYNC;

	if (!simulation)
	  if ( ( err = Sync(fd, syncString) < 0) )
	  {
		EquatorialCoordsNP.s = IPS_ALERT;
	        IDSetNumber(&EquatorialCoordsNP , "Synchronization failed.");
		return (-1);
	  }

	  EquatorialCoordsNP.s = IPS_OK;
	  IDLog("Synchronization successful %s\n", syncString);
	  IDSetNumber(&EquatorialCoordsNP, "Synchronization successful.");
	  break;

   }

   return (0);

}

int LX200RT::getOnSwitch(ISwitchVectorProperty *sp)
{
 for (int i=0; i < sp->nsp ; i++)
     if (sp->sp[i].s == ISS_ON)
      return i;

 return -1;
}


int LX200RT::checkPower(ISwitchVectorProperty *sp)
{
  if (simulation) return 0;

  if (ConnectSP.s != IPS_OK)
  {
    if (!strcmp(sp->label, ""))
    	IDMessage (thisDevice, "Cannot change property %s while the telescope is offline.", sp->name);
    else
        IDMessage (thisDevice, "Cannot change property %s while the telescope is offline.", sp->label);

    sp->s = IPS_IDLE;
    IDSetSwitch(sp, NULL);
    return -1;
  }

  return 0;
}

int LX200RT::checkPower(INumberVectorProperty *np)
{
  if (simulation) return 0;

  if (ConnectSP.s != IPS_OK)
  {

    if (!strcmp(np->label, ""))
    	IDMessage (thisDevice, "Cannot change property %s while the telescope is offline.", np->name);
    else
        IDMessage (thisDevice, "Cannot change property %s while the telescope is offline.", np->label);

    np->s = IPS_IDLE;
    IDSetNumber(np, NULL);
    return -1;
  }

  return 0;

}

int LX200RT::checkPower(ITextVectorProperty *tp)
{

  if (simulation) return 0;

  if (ConnectSP.s != IPS_OK)
  {
    if (!strcmp(tp->label, ""))
    	IDMessage (thisDevice, "Cannot change property %s while the telescope is offline.", tp->name);
    else
        IDMessage (thisDevice, "Cannot change property %s while the telescope is offline.", tp->label);

    tp->s = IPS_IDLE;
    IDSetText(tp, NULL);
    return -1;
  }

  return 0;

}

void LX200RT::connectTelescope()
{
   int err;
   int lx200_utc_offset=0;

   switch (ConnectSP.sp[0].s) {
      case ISS_ON:
         if (simulation) {
	         ConnectSP.s = IPS_OK;
	         IDSetSwitch (&ConnectSP, "Simulated telescope is online.");
	         //updateTime();
	         return;
	      }

	      if (tty_connect(PortTP.tp[0].text, 9600, 8, 0, 1, &fd) != TTY_OK) {
	         ConnectS[0].s = ISS_OFF;
	         ConnectS[1].s = ISS_ON;
	         IDSetSwitch (&ConnectSP, "Error connecting to port %s. Make sure you have BOTH write and read permission to your port.\n", PortTP.tp[0].text);
	         return;
	      }
	      if (check_lx200_connection(fd)) {
	         ConnectS[0].s = ISS_OFF;
	         ConnectS[1].s = ISS_ON;
	         IDSetSwitch (&ConnectSP, "Error connecting to Telescope. Telescope is offline.");
	         return;
	      }

         #ifdef INDI_DEBUG
         IDLog("Telescope test successful.\n");
	      #endif

         *((int *) UTCOffsetN[0].aux0) = 0;
	      ConnectSP.s = IPS_OK;
	      IDSetSwitch (&ConnectSP, "Telescope is online. Retrieving basic data...");

// hgz
         // set local time
         struct ln_date utm;
         struct ln_zonedate ltm;

         ln_get_date_from_sys(&utm);

//    if (*((int *) UTCOffsetN[0].aux0) == 0)
// 	{
// 		TimeTP.s = IPS_IDLE;
// 		IDSetText(&TimeTP, "You must set the UTC Offset property first.");
// 		return;
// 	}

	      // update JD
         JD = ln_get_julian_day(&utm);
         IDLog("New JD is %f\n", (float) JD);

         // get UTC Offset from device
         getUTCOffset(fd, &lx200_utc_offset);
         // LX200 TimeT Offset is defined at the number of hours added to LOCAL TIME to get TimeT. This is contrary to the normal definition.
         UTCOffsetN[0].value = lx200_utc_offset*-1;

         // We got a valid value for UTCOffset now
         *((int *) UTCOffsetN[0].aux0) = 1;

	      ln_date_to_zonedate(&utm, &ltm, UTCOffsetN[0].value*3600.0);

	      // Set Local Time
	      if ( ( err = setLocalTime(fd, ltm.hours, ltm.minutes, ltm.seconds) < 0) ) {
	         handleError(&TimeTP, err, "Setting local time");
        	   return;
	      }

	      if (!strcmp(thisDevice, "LX200 GPS")) {
			   if ( ( err = setCalenderDate(fd, utm.days, utm.months, utm.years) < 0) )
	  		   {
		  		   handleError(&TimeTP, err, "Setting TimeT date.");
		  		   return;
			   }
	      } else {
			   if ( ( err = setCalenderDate(fd, ltm.days, ltm.months, ltm.years) < 0) ) {
		  		   handleError(&TimeTP, err, "Setting local date.");
		  		   return;
			   }
	      }

	      getBasicData();
         //updateTime();


      	break;

      case ISS_OFF:
         ConnectS[0].s = ISS_OFF;
	      ConnectS[1].s = ISS_ON;
         ConnectSP.s = IPS_IDLE;
         IDSetSwitch (&ConnectSP, "Telescope is offline.");
	      IDLog("Telescope is offline.");
	      tty_disconnect(fd);
	      break;

    }
}

void LX200RT::slewError(int slewCode)
{
   EquatorialCoordsNP.s = IPS_ALERT;

   if (slewCode == 1) IDSetNumber(&EquatorialCoordsNP, "Object below horizon.");
   else if (slewCode == 2)	IDSetNumber(&EquatorialCoordsNP, "Object below the minimum elevation limit.");
   else IDSetNumber(&EquatorialCoordsNP, "Slew failed.");
}

void LX200RT::getAlignment()
{

   if (ConnectSP.s != IPS_OK) return;

   signed char align = ACK(fd);
   if (align < 0) {
      IDSetSwitch (&AlignmentSw, "Failed to get telescope alignment.");
      return;
   }

   AlignmentS[0].s = ISS_OFF;
   AlignmentS[1].s = ISS_OFF;
   AlignmentS[2].s = ISS_OFF;

   switch (align) {
      case 'P': AlignmentS[0].s = ISS_ON;
         break;
      case 'A': AlignmentS[1].s = ISS_ON;
      	break;
      case 'L': AlignmentS[2].s = ISS_ON;
        	break;
   }

   AlignmentSw.s = IPS_OK;
   IDSetSwitch (&AlignmentSw, NULL);
   IDLog("ACK success %c\n", align);
}

void LX200RT::enableSimulation(bool enable)
{
   simulation = enable;

   if (simulation)
     IDLog("Warning: Simulation is activated.\n");
   else
     IDLog("Simulation is disabled.\n");
}

void LX200RT::updateTime()
{
   #ifdef HAVE_NOVA_H
   char cdate[32];
   double ctime;
   int h, m, s, lx200_utc_offset=0;
   int day, month, year, result;
   struct tm ltm;
   struct tm utm;
   time_t time_epoch;

   if (simulation) {
      sprintf(TimeT[0].text, "%d-%02d-%02dT%02d:%02d:%02d", 1979, 6, 25, 3, 30, 30);
      IDLog("Telescope ISO date and time: %s\n", TimeT[0].text);
      IDSetText(&TimeTP, NULL);
      return;
   }

   getUTCOffset(fd, &lx200_utc_offset);

   // LX200 TimeT Offset is defined at the number of hours added to LOCAL TIME to get TimeT. This is contrary to the normal definition.
   UTCOffsetN[0].value = lx200_utc_offset*-1;

   // We got a valid value for UTCOffset now
   *((int *) UTCOffsetN[0].aux0) = 1;

   #ifdef INDI_DEBUG
   IDLog("Telescope TimeT Offset: %g\n", UTCOffsetN[0].value);
   #endif

   getLocalTime24(fd, &ctime);
   getSexComponents(ctime, &h, &m, &s);

   if ( (result = getSDTime(fd, &SDTimeN[0].value)) < 0)
      IDMessage(thisDevice, "Failed to retrieve siderial time from device.");

   getCalenderDate(fd, cdate);
   result = sscanf(cdate, "%d/%d/%d", &year, &month, &day);
   if (result != 3) return;

   // Let's fill in the local time
   ltm.tm_sec = s;
   ltm.tm_min = m;
   ltm.tm_hour = h;
   ltm.tm_mday = day;
   ltm.tm_mon = month - 1;
   ltm.tm_year = year - 1900;

   // Get time epoch
   time_epoch = mktime(&ltm);

   // Convert to TimeT
   time_epoch -= (int) (UTCOffsetN[0].value * 60.0 * 60.0);

   // Get UTC (we're using localtime_r, but since we shifted time_epoch above by UTCOffset, we should be getting the real UTC time)
   localtime_r(&time_epoch, &utm);

   /* Format it into ISO 8601 */
   strftime(cdate, 32, "%Y-%m-%dT%H:%M:%S", &utm);
   IUSaveText(&TimeT[0], cdate);

   #ifdef INDI_DEBUG
   IDLog("Telescope Local Time: %02d:%02d:%02d\n", h, m , s);
   IDLog("Telescope SD Time is: %g\n", SDTimeN[0].value);
   IDLog("Telescope UTC Time: %s\n", TimeT[0].text);
   #endif

   // Let's send everything to the client
   IDSetText(&TimeTP, NULL);
   IDSetNumber(&SDTimeNP, NULL);
   IDSetNumber(&UTCOffsetNP, NULL);
   #endif
}

void LX200RT::updateLocation()
{

   int dd = 0, mm = 0, err = 0;

   if (simulation) return;

   if ( (err = getSiteLatitude(fd, &dd, &mm)) < 0) IDMessage(thisDevice, "Failed to get site latitude from device.");
   else {
      if (dd > 0)
    	   geoNP.np[0].value = dd + mm/60.0;
      else
         geoNP.np[0].value = dd - mm/60.0;

      IDLog("Autostar Latitude: %d:%d\n", dd, mm);
      IDLog("INDI Latitude: %g\n", geoNP.np[0].value);
   }

   if ( (err = getSiteLongitude(fd, &dd, &mm)) < 0) IDMessage(thisDevice, "Failed to get site longitude from device.");
   else {
      if (dd > 0) geoNP.np[1].value = 360.0 - (dd + mm/60.0);
      else geoNP.np[1].value = (dd - mm/60.0) * -1.0;

      IDLog("Autostar Longitude: %d:%d\n", dd, mm);
      IDLog("INDI Longitude: %g\n", geoNP.np[1].value);
   }

   IDSetNumber (&geoNP, NULL);
}
