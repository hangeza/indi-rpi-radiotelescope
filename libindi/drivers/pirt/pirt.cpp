/*
   INDI Developers Manual
   Tutorial #2

   "Simple Telescope Driver"

   We develop a simple telescope simulator.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

/** \file pirt.cpp
    \brief Construct a basic INDI telescope device that simulates GOTO commands.
    \author Jasem Mutlaq

    \example pirt.cpp
    A simple GOTO telescope that simulator slewing operation.
*/

#include "pirt.h"

#include "indicom.h"

#include <cmath>
#include <memory>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libnova.h>

#include <encoder.h>
#include <gpioif.h>
#include "motordriver.h"

constexpr unsigned int SSI_BAUD_RATE { 1000000 };
constexpr unsigned int POLL_INTERVAL_MS { 250 };
constexpr double DEFAULT_AZ_AXIS_TURNS_RATIO { 152./9. };
constexpr double DEFAULT_EL_AXIS_TURNS_RATIO { 10. };
constexpr double MAX_AZ_OVERTURN { 0.25 };

constexpr double SID_RATE { 0.004178 }; /* sidereal rate, degrees/s */
constexpr double SLEW_RATE { 5. };        /* slew rate, degrees/s */

constexpr double POS_ACCURACY_FINE { 0.25 };
constexpr double POS_ACCURACY_COARSE { 2.0 };
constexpr double TRACK_ACCURACY { 0.05 };

#define MAX_ELEVATION_EXCESS 100.0


static std::unique_ptr<PiRT> pirt(new PiRT());




/**************************************************************************************
** Return properties of device.
***************************************************************************************/
void ISGetProperties(const char *dev)
{
    pirt->ISGetProperties(dev);
}

/**************************************************************************************
** Process new switch from client
***************************************************************************************/
void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    pirt->ISNewSwitch(dev, name, states, names, n);
}

/**************************************************************************************
** Process new text from client
***************************************************************************************/
void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    pirt->ISNewText(dev, name, texts, names, n);
}

/**************************************************************************************
** Process new number from client
***************************************************************************************/
void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    pirt->ISNewNumber(dev, name, values, names, n);
}

/**************************************************************************************
** Process new blob from client
***************************************************************************************/
void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    pirt->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

/**************************************************************************************
** Process snooped property from another driver
***************************************************************************************/
void ISSnoopDevice(XMLEle *root)
{
    pirt->ISSnoopDevice(root);
}




/*
 * class Pi RT
 * 
 */
PiRT::PiRT()
{
    //currentRA  = 0;
    //currentDEC = 0;
///    currentAz = currentAlt = 0;
//     TrackingOn = false;

    // We add an additional debug level so we can log verbose scope status
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

//    setTelescopeConnection(CONNECTION_TCP);
    setTelescopeConnection(CONNECTION_TCP);

	SetTelescopeCapability(
			TELESCOPE_CAN_PARK |
			TELESCOPE_CAN_ABORT |
			TELESCOPE_HAS_TIME | 
			TELESCOPE_HAS_LOCATION |
			TELESCOPE_HAS_TRACK_MODE |
			TELESCOPE_CAN_CONTROL_TRACK /*|
			TELESCOPE_HAS_TRACK_RATE */
	);
	
	///el_axis.registerGimbalFlipCallback( [this]() { this->az_axis.gimbalFlip(); } );
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool PiRT::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::initProperties();

	setDefaultPollingPeriod(POLL_INTERVAL_MS);

	//return true;
/*
	IUFillSwitch(&TrackingS[0], "Off", "", ISS_ON);
    IUFillSwitch(&TrackingS[1], "On", "", ISS_OFF);
    IUFillSwitchVector(&TrackingSP, TrackingS, 2, getDeviceName(), "TRACKING", "Tracking", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY,0, IPS_IDLE);
*/    
    IUFillLight(&ScopeStatusL[0], "SCOPE_IDLE", "Idle", IPS_OK);
    IUFillLight(&ScopeStatusL[1], "SCOPE_SLEWING", "Slew", IPS_IDLE);
    IUFillLight(&ScopeStatusL[2], "SCOPE_TRACKING", "Tracking", IPS_IDLE);
    IUFillLight(&ScopeStatusL[3], "SCOPE_PARKING", "Parking", IPS_IDLE);
    IUFillLight(&ScopeStatusL[4], "SCOPE_PARKED", "Parked", IPS_IDLE);

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

	IUFillNumber(&EncoderBitRateN, "SSI_BITRATE", "SSI Bit Rate", "%5.0f Hz", 0, 5000000, 0, SSI_BAUD_RATE);
    IUFillNumberVector(&EncoderBitRateNP, &EncoderBitRateN, 1, getDeviceName(), "ENC_SPI_SETTINGS", "SPI Interface", "Encoders",
           IP_RW, 60, IPS_IDLE);
	defineProperty(&EncoderBitRateNP);
    IDSetNumber(&EncoderBitRateNP, NULL);

	IUFillNumber(&AzEncoderN[0], "AZ_ENC_POS", "Position", "%5.4f rev", -32767, 32767, 0, 0);
	IUFillNumber(&AzEncoderN[1], "AZ_ENC_ST", "ST", "%5.0f", 0, 65535, 0, 0);
	IUFillNumber(&AzEncoderN[2], "AZ_ENC_MT", "MT", "%5.0f", -32767, 32767, 0, 0);
    IUFillNumber(&AzEncoderN[3], "AZ_ENC_ERR", "Bit Errors", "%5.0f", 0, 0, 0, 0);
    IUFillNumber(&AzEncoderN[4], "AZ_ENC_ROTIME", "R/O Time", "%5.0f us", 0, 0, 0, 0);
    IUFillNumberVector(&AzEncoderNP, AzEncoderN, 5, getDeviceName(), "AZ_ENC", "Azimuth", "Encoders",
           IP_RO, 60, IPS_IDLE);
	IUFillNumber(&ElEncoderN[0], "EL_ENC_POS", "Position", "%5.4f rev", -32767, 32767, 0, 0);
    IUFillNumber(&ElEncoderN[1], "EL_ENC_ST", "ST", "%5.0f", 0, 65535, 0, 3);
    IUFillNumber(&ElEncoderN[2], "EL_ENC_MT", "MT", "%5.0f", -32767, 32767, 0, 4);
    IUFillNumber(&ElEncoderN[3], "EL_ENC_ERR", "Bit Errors", "%5.0f", 0, 0, 0, 0);
    IUFillNumber(&ElEncoderN[4], "EL_ENC_ROTIME", "R/O Time", "%5.0f us", 0, 0, 0, 0);
    IUFillNumberVector(&ElEncoderNP, ElEncoderN, 5, getDeviceName(), "EL_ENC", "Elevation", "Encoders",
           IP_RO, 60, IPS_IDLE);
	IUFillNumber(&AzEncSettingN[0], "AZ_ENC_ST_BITS", "ST bits", "%5.0f", 0, 24, 0, 12);
	IUFillNumber(&AzEncSettingN[1], "AZ_ENC_MT_BITS", "MT bits", "%5.0f", 0, 24, 0, 12);
    IUFillNumberVector(&AzEncSettingNP, AzEncSettingN, 2, getDeviceName(), "AZ_ENC_SETTING", "Az Encoder Settings", "Encoders",
           IP_RW, 60, IPS_IDLE);
	defineProperty(&AzEncSettingNP);
	IDSetNumber(&AzEncSettingNP, NULL);

	IUFillNumber(&ElEncSettingN[0], "EL_ENC_ST_BITS", "ST bits", "%5.0f", 0, 24, 0, 13);
	IUFillNumber(&ElEncSettingN[1], "EL_ENC_MT_BITS", "MT bits", "%5.0f", 0, 24, 0, 12);
    IUFillNumberVector(&ElEncSettingNP, ElEncSettingN, 2, getDeviceName(), "EL_ENC_SETTING", "El Encoder Settings", "Encoders",
           IP_RW, 60, IPS_IDLE);
	defineProperty(&ElEncSettingNP);
    IDSetNumber(&ElEncSettingNP, NULL);
	
	IUFillNumber(&AzAxisSettingN[0], "AZ_AXIS_RATIO", "Enc-to-Axis turns ratio", "%5.3f", 0.0001, 100000, 0, DEFAULT_AZ_AXIS_TURNS_RATIO);
	IUFillNumber(&AzAxisSettingN[1], "AZ_AXIS_OFFSET", "Axis Offset", "%5.4f deg", -180., 180., 0, 0);
    IUFillNumberVector(&AzAxisSettingNP, AzAxisSettingN, 2, getDeviceName(), "AZ_AXIS_SETTING", "Az Axis Settings", "Axes",
           IP_RW, 60, IPS_IDLE);
	defineProperty(&AzAxisSettingNP);
    IDSetNumber(&AzAxisSettingNP, NULL);

	IUFillNumber(&ElAxisSettingN[0], "EL_AXIS_RATIO", "Enc-to-Axis turns ratio", "%5.3f", 0.0001, 100000, 0, DEFAULT_EL_AXIS_TURNS_RATIO);
	IUFillNumber(&ElAxisSettingN[1], "EL_AXIS_OFFSET", "Axis Offset", "%5.4f deg", -90., 90., 0, 0);
    IUFillNumberVector(&ElAxisSettingNP, ElAxisSettingN, 2, getDeviceName(), "El_AXIS_SETTING", "El Axis Settings", "Axes",
           IP_RW, 60, IPS_IDLE);
	defineProperty(&ElAxisSettingNP);
    IDSetNumber(&ElAxisSettingNP, NULL);
	axisRatio[0] = DEFAULT_AZ_AXIS_TURNS_RATIO;
	axisRatio[1] = DEFAULT_EL_AXIS_TURNS_RATIO;

	IUFillNumber(&AxisAbsTurnsN[0], "AZ_AXIS_TURNS", "Az", "%5.4f rev", 0, 0, 0, 0);
	IUFillNumber(&AxisAbsTurnsN[1], "EL_AXIS_TURNS", "Alt", "%5.4f rev", 0, 0, 0, 0);
    IUFillNumberVector(&AxisAbsTurnsNP, AxisAbsTurnsN, 2, getDeviceName(), "AXIS_ABSOLUTE_TURNS", "Absolute Axis Turns", "Axes",
           IP_RO, 60, IPS_IDLE);
	
	IUFillNumber(&AzMotorStatusN[0], "AZ_MOTOR_SPEED", "Speed", "%4.0f %%", -100, 100, 0, 0);
	IUFillNumber(&AzMotorStatusN[1], "AZ_MOTOR_FAULTS", "Fault Counter", "%5.0f", 0, 0, 0, 0);
    IUFillNumberVector(&AzMotorStatusNP, AzMotorStatusN, 2, getDeviceName(), "AZ_MOTOR_STATUS", "Az Motor Status", "Motors",
           IP_RO, 60, IPS_IDLE);
	IUFillNumber(&ElMotorStatusN[0], "EL_MOTOR_SPEED", "Speed", "%4.0f %%", -100, 100, 0, 0);
	IUFillNumber(&ElMotorStatusN[1], "El_MOTOR_FAULTS", "Fault Counter", "%5.0f", 0, 0, 0, 0);
    IUFillNumberVector(&ElMotorStatusNP, ElMotorStatusN, 2, getDeviceName(), "El_MOTOR_STATUS", "El Motor Status", "Motors",
           IP_RO, 60, IPS_IDLE);
	
	addDebugControl();

	return true;
}

bool PiRT::updateProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::updateProperties();
    // delete the paramters defined for optical scopes - don't need 'em
    deleteProperty(ScopeParametersNP.name);
//	deleteProperty("ConnectSP");

	if (isConnected()) {
		defineProperty(&ScopeStatusLP);
		LocationNP.s = IPS_OK;
		IDSetNumber(&LocationNP, NULL);
		defineProperty(&HorNP);
		defineProperty(&JDNP);
		defineProperty(&AzEncoderNP);
		defineProperty(&ElEncoderNP);
		defineProperty(&AzMotorStatusNP);
		defineProperty(&ElMotorStatusNP);
		defineProperty(&AxisAbsTurnsNP);
		EncoderBitRateNP.p = IP_RO;
		//defineProperty(&TrackingSP);
    } else {
		deleteProperty(ScopeStatusLP.name);
		deleteProperty(HorNP.name);
		//deleteProperty(TrackingSP.name);
		deleteProperty(JDNP.name);
		deleteProperty(AzEncoderNP.name);
		deleteProperty(ElEncoderNP.name);
		deleteProperty(AzMotorStatusNP.name);
		deleteProperty(ElMotorStatusNP.name);
		deleteProperty(AxisAbsTurnsNP.name);
		EncoderBitRateNP.p = IP_RW;
	}
	IDSetNumber(&EncoderBitRateNP, nullptr);
    
    return true;
}

/**************************************************************************************
**
***************************************************************************************/
bool PiRT::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
	//return INDI::Telescope::ISNewSwitch(dev,name,states,names,n);
	if(strcmp(dev,getDeviceName())==0)
	{
		//  This one is for us
		//if(!strcmp(name,TrackingSP.name))
		{
			//return true;
		}
	}
	//  Nobody has claimed this, so forward it to the base class' method
	return INDI::Telescope::ISNewSwitch(dev,name,states,names,n);
}

bool PiRT::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
	if(strcmp(dev,getDeviceName())==0)
	{
		//  This one is for us
		if(!strcmp(name, EncoderBitRateNP.name))
		{
			// set Encoder bit rate
			EncoderBitRateNP.s = IPS_OK;
			EncoderBitRateN.value = values[0];
			IDSetNumber(&EncoderBitRateNP, nullptr);
			unsigned int rate = EncoderBitRateNP.np[0].value;
			DEBUGF(DBG_SCOPE, "Setting SSI bit rate to: %u Hz. Please reconnect client!", rate);
			return true;
		} else if(!strcmp(name, AzEncSettingNP.name)) {
			// set Az encoder bit widths
			unsigned int stBits = values[0];
			unsigned int mtBits = values[1];
			if (stBits > 1 && stBits < 20 && mtBits > 1 && mtBits < 20) {
				AzEncSettingNP.s = IPS_OK;
				AzEncSettingN[0].value = stBits;
				AzEncSettingN[1].value = mtBits;
				if (isConnected()) {
					az_encoder->setStBitWidth(stBits);
					az_encoder->setMtBitWidth(mtBits);
				}
				IDSetNumber(&AzEncSettingNP, nullptr);
				return true;
			} else {
				AzEncSettingNP.s = IPS_ALERT;
				return false;
			}
		} else if(!strcmp(name, ElEncSettingNP.name)) {
			// set El encoder bit widths
			unsigned int stBits = values[0];
			unsigned int mtBits = values[1];
			if (stBits > 1 && stBits < 20 && mtBits > 1 && mtBits < 20) {
				ElEncSettingNP.s = IPS_OK;
				ElEncSettingN[0].value = stBits;
				ElEncSettingN[1].value = mtBits;
				if (isConnected()) {
					el_encoder->setStBitWidth(stBits);
					el_encoder->setMtBitWidth(mtBits);
				}
				IDSetNumber(&ElEncSettingNP, nullptr);
				return true;
			} else {
				ElEncSettingNP.s = IPS_ALERT;
				return false;
			}
		} else if(!strcmp(name, AzAxisSettingNP.name)) {
			// Az axis settings: encoder-to-axis turns ratio and offset
			AzAxisSettingNP.s = IPS_OK;
			AzAxisSettingN[0].value = values[0];
			AzAxisSettingN[1].value = values[1];
			IDSetNumber(&AzAxisSettingNP, nullptr);
			axisRatio[0] = values[0];
			axisOffset[0] = values[1];
			DEBUGF(DBG_SCOPE, "Setting Az axis turns ratio to %5.4f rev.", axisRatio[0]);
			DEBUGF(DBG_SCOPE, "Setting Az axis offset %5.4f rev.", axisOffset[0]);
			return true;
		} else if(!strcmp(name, ElAxisSettingNP.name)) {
			// El axis settings: encoder-to-axis turns ratio and offset
			ElAxisSettingNP.s = IPS_OK;
			ElAxisSettingN[0].value = values[0];
			ElAxisSettingN[1].value = values[1];
			IDSetNumber(&ElAxisSettingNP, nullptr);
			axisRatio[1] = values[0];
			axisOffset[1] = values[1];
			DEBUGF(DBG_SCOPE, "Setting El axis turns ratio to %5.4f rev.", axisRatio[1]);
			DEBUGF(DBG_SCOPE, "Setting El axis offset %5.4f rev.", axisOffset[1]);
			return true;
		}
	}	
	//  Nobody has claimed this, so forward it to the base class method
	return INDI::Telescope::ISNewNumber(dev,name,values,names,n);
}

bool PiRT::SetTrackMode(uint8_t mode) {
	return false;
}

bool PiRT::SetTrackEnabled(bool enabled) {
	// TODO: do sth here to enable the tracking loop
	if (enabled) {
		targetEquatorialCoords = Hor2Equ(currentHorizontalCoords);
		/*
		targetRA = currentRA;
		targetDEC = currentDEC;
		*/
	}
	return true;
}

bool PiRT::Park() {
	// TODO: do sth here to start the parking operation
	TrackState = SCOPE_PARKING;
	return true;
}


bool PiRT::Connect()
{
	ITextVectorProperty *tvp = getText("DEVICE_ADDRESS");
	if (tvp == nullptr) {
        DEBUG(INDI::Logger::DBG_ERROR, "No address property found.");
		return false;
	}
	
	//DEBUGF(INDI::Logger::DBG_SESSION, "host: %s:%s", tvp->tp[0].text, tvp->tp[1].text);

	const std::string host { tvp->tp[0].text };
	const std::string port { tvp->tp[1].text };
	
//	std::shared_ptr<GPIO> temp_ptr ( new GPIO("localhost") );
	// before instanciating a new GPIO interface, all objects which carry a reference
	// to the old gpio object must be invalidated, to make sure
	// that noone else uses the shared_ptr<GPIO> when it is newly created
	az_encoder.reset();
	el_encoder.reset();
	az_motor.reset();
	el_motor.reset();
	
	gpio.reset( new GPIO(host, port) );
	if (!gpio->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Could not initialize GPIO interface. Is pigpiod running?");
		return false;
	}
	
	unsigned int bitrate = static_cast<unsigned int>( EncoderBitRateNP.np[0].value );
	if ( bitrate < 80000 || bitrate > 5000000 ) {
        DEBUG(INDI::Logger::DBG_ERROR, "SSI bitrate out of range (80k...5M)");
		return false;
	}

	az_encoder.reset(new SsiPosEncoder(gpio, GPIO::SPI_INTERFACE::Main, bitrate));
	if (!az_encoder->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to connect to Az position encoder.");
		return false;
	}
	az_encoder->setStBitWidth(AzEncSettingN[0].value);
	az_encoder->setMtBitWidth(AzEncSettingN[1].value);
	el_encoder.reset(new SsiPosEncoder(gpio, GPIO::SPI_INTERFACE::Aux, bitrate));
	if (!el_encoder->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to connect to El position encoder.");
		return false;
	}
	el_encoder->setStBitWidth(ElEncSettingN[0].value);
	el_encoder->setMtBitWidth(ElEncSettingN[1].value);

	MotorDriver::Pins az_motor_pins { az_motor_pins.Enable=22, az_motor_pins.Pwm=12, az_motor_pins.Dir=24, az_motor_pins.Fault=5 };
	MotorDriver::Pins el_motor_pins { el_motor_pins.Enable=23, el_motor_pins.Pwm=13, el_motor_pins.Dir=25, el_motor_pins.Fault=6 };
	az_motor.reset( new MotorDriver( gpio, az_motor_pins, nullptr ) );
	if (!az_motor->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to initialize Az motor driver.");
		return false;
	}
	el_motor.reset( new MotorDriver( gpio, el_motor_pins, nullptr ) );
	if (!el_motor->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to initialize El motor driver.");
		return false;
	}

	INDI::Telescope::Connect();
	return true;
}

bool PiRT::Disconnect()
{
	az_encoder.reset();
	el_encoder.reset();
	az_motor.reset();
	el_motor.reset();
	gpio.reset();
	return true;
}


void PiRT::TimerHit()
{
    if(isConnected())
    {
        bool rc;
        rc=ReadScopeStatus();
        if(rc == false)
        {
            //  read was not good
            EqNP.s= IPS_ALERT;
            IDSetNumber(&EqNP, NULL);
        }
		SetTimer(getCurrentPollingPeriod());
    }
}


/**************************************************************************************
** INDI is asking us to check communication with the device via a handshake
***************************************************************************************/
bool PiRT::Handshake()
{
	// When communicating with a real mount, we check here if commands are receieved
	// and acknolowedged by the mount.
	if (isConnected()) {
		if (!gpio->isInitialized()) return false;
		if (!az_encoder->statusOk()) return false;
		if (!el_encoder->statusOk()) return false;
		if (!az_motor->isInitialized()) return false;
		if (!el_motor->isInitialized()) return false;
	}
	return true;
}

/**************************************************************************************
** INDI is asking us for our default device name
***************************************************************************************/
const char *PiRT::getDefaultName()
{
	return "Pi Radiotelescope";
}

/**************************************************************************************
** is telescope in tracking mode?
***************************************************************************************/
bool PiRT::isTracking()
{
	//return true;
	return (TrackingS[1].s == ISS_ON);
}


/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool PiRT::Goto(double ra, double dec)
{
	//targetRA  = ra;
    //targetDEC = dec;
    
	targetEquatorialCoords = EquCoords{ ra , dec };
	
	char RAStr[64]={0}, DecStr[64]={0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, ra, 2, 3600);
    fs_sexa(DecStr, dec, 2, 3600);

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
bool PiRT::GotoHor(double az, double alt)
{
	//targetAz  = az;
	//targetAlt = alt;
    
    if (alt<0.) {
      DEBUG(INDI::Logger::DBG_WARNING, "Error: Target below horizon");
      return false;
    }
    
    targetHorizontalCoords = HorCoords { az, alt };
    
	char AzStr[64]={0}, AltStr[64]={0};

    // Parse the Az/Alt into strings
    fs_sexa(AzStr, az, 2, 3600);
    fs_sexa(AltStr, alt, 2, 3600);

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
bool PiRT::Abort()
{
    TrackState = (isTracking() ? SCOPE_TRACKING : SCOPE_IDLE);
	return true;
}

void PiRT::Hor2Equ(const HorCoords& hor_coords, double* ra, double* dec) {
	Hor2Equ(hor_coords.Az.value(), hor_coords.Alt.value(), ra, dec);
}

EquCoords PiRT::Hor2Equ(const HorCoords& hor_coords) {
	double ra {} , dec {};
	Hor2Equ(hor_coords.Az.value(), hor_coords.Alt.value(), &ra, &dec);
	return EquCoords( ra , dec );
}

void PiRT::Hor2Equ(double az, double alt, double* ra, double* dec) {
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

HorCoords PiRT::Equ2Hor(const EquCoords& equ_coords) {
	double az {} , alt {};
	Equ2Hor(equ_coords.Ra.value(), equ_coords.Dec.value(), &az, &alt);
	return HorCoords( az , alt );
}

void PiRT::Equ2Hor(double ra, double dec, double* az, double* alt) {
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
bool PiRT::ReadScopeStatus()
{
	static struct timeval ltv { 0, 0 };
    struct timeval tv { 0, 0 };
    double dt = 0, dx = 0, dy = 0;
    static double dt_time_update = 0.;

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

    // read pos encoders
    unsigned int st = az_encoder->position();
    int mt = az_encoder->nrTurns();

	AzEncoderN[0].value = az_encoder->absolutePosition();
    AzEncoderN[1].value = static_cast<double>(st);
    AzEncoderN[2].value = static_cast<double>(mt);
	AzEncoderN[3].value = az_encoder->bitErrorCount();
	AzEncoderN[4].value = az_encoder->lastReadOutDuration().count();
    //DEBUGF(INDI::Logger::DBG_SESSION, "Az Encoder values: st=%d mt=%u t_ro=%u us", st, mt, us);
    AzEncoderNP.s = (az_encoder->statusOk())? IPS_OK : IPS_ALERT;
    IDSetNumber(&AzEncoderNP, nullptr);
    ElEncoderN[0].value = el_encoder->absolutePosition();
    ElEncoderN[1].value = static_cast<double>(el_encoder->position());
    ElEncoderN[2].value = static_cast<double>(el_encoder->nrTurns());
	ElEncoderN[3].value = el_encoder->bitErrorCount();
	ElEncoderN[4].value = el_encoder->lastReadOutDuration().count();
    ElEncoderNP.s = (el_encoder->statusOk())? IPS_OK : IPS_ALERT;
    IDSetNumber(&ElEncoderNP, nullptr);

	const double az_revolutions { az_encoder->absolutePosition() };
	const double el_revolutions { el_encoder->absolutePosition() };

	double azAbsTurns = ( az_revolutions / axisRatio[0] ) + axisOffset[0] / 360.;
	double altAbsTurns = ( el_revolutions / axisRatio[1] ) + axisOffset[1] / 360.;

	AxisAbsTurnsN[0].value = azAbsTurns;
	AxisAbsTurnsN[1].value = altAbsTurns;
	if ( std::abs(azAbsTurns) > 1. + MAX_AZ_OVERTURN ) {
		AxisAbsTurnsNP.s = IPS_ALERT;
	} else {
		AxisAbsTurnsNP.s = IPS_OK;
	}
	IDSetNumber(&AxisAbsTurnsNP, nullptr);
	
	currentHorizontalCoords.Az.setValue( 360. * azAbsTurns );
	currentHorizontalCoords.Alt.setValue( 360. * altAbsTurns );
	
	// update motor status
	if (az_motor->isFault()) {
		AzMotorStatusNP.s=IPS_ALERT;
	} else {
		AzMotorStatusNP.s=IPS_OK;
	}
	if (el_motor->isFault()) {
		ElMotorStatusNP.s=IPS_ALERT;
	} else {
		ElMotorStatusNP.s=IPS_OK;
	}
	AzMotorStatusN[0].value = 100. * az_motor->currentSpeed();
	ElMotorStatusN[0].value = 100. * el_motor->currentSpeed();
	IDSetNumber(&AzMotorStatusNP, nullptr);
	IDSetNumber(&ElMotorStatusNP, nullptr);
	
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
	switch (TrackState)
	{
		case SCOPE_SLEWING:
			if (TargetCoordSystem == SYSTEM_EQ) {
				//Equ2Hor(targetRA, targetDEC, &targetAz, &targetAlt);
				targetHorizontalCoords = Equ2Hor(targetEquatorialCoords);
			} else if (TargetCoordSystem == SYSTEM_HOR) {
			} else if (TargetCoordSystem == SYSTEM_GAL) {
			} else {
				// unknown coordinate system - abort
				Abort();
			}

			//PiRaTe::RotAxis diffAz { -180, 180, 360};
			
			dx = targetHorizontalCoords.Az.value() - currentHorizontalCoords.Az.value();
			dy = targetHorizontalCoords.Alt.value() - currentHorizontalCoords.Alt.value();
			
			if ( dx > 180. ) {
				dx -= 360.; 
			} else if ( dx < -180. ) {
				dx += 360.;
			}
			
			if ( dy > 180. ) {
				dy -= 360.;
			} else if ( dy < -180. ) {
				dy += 360.;
			}
			
			if ( std::abs(dx) > POS_ACCURACY_COARSE ) {
				az_motor->move((dx>=0)?1.:-1.);
			} else if ( std::abs(dx) > POS_ACCURACY_FINE ) {				
				az_motor->move(dx/POS_ACCURACY_COARSE);
			} else {
			}

			if ( std::abs(dy) > POS_ACCURACY_COARSE ) {
				az_motor->move((dy>=0)?1.:-1.);
			} else if ( std::abs(dy) > POS_ACCURACY_FINE ) {				
				az_motor->move(dy/POS_ACCURACY_COARSE);
			} else {
			}

			// Let's check if we reached position for both axes
			if ( 	std::abs(dx) < TRACK_ACCURACY 
				&& 	std::abs(dy) < TRACK_ACCURACY	)
			{
				// Let's set state to TRACKING
				Abort();
				/*
				if (isTracking() && TargetCoordSystem == SYSTEM_HOR) {
					Hor2Equ(currentHorizontalCoords, &targetRA, &targetDEC);
				}
				*/
                DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete.");
			}
			break;
	
		case SCOPE_TRACKING:
			break;
/*
			Equ2Hor(targetRA, targetDEC, &targetAz, &targetAlt);
	  
			//DEBUG(INDI::Logger::DBG_SESSION, "tracking...");
			//DEBUGF(INDI::Logger::DBG_SESSION, "target Az: %f target Alt: %f", targetAz, targetAlt);
			dx = targetAz-currentHorizontalCoords.Az.value();
			dy = targetAlt-currentHorizontalCoords.Alt.value();
			if (dx>180.) dx-=360.;
			else if (dx<-180.) dx+=360.;
			if (dy>180.) dy-=360.;
			else if (dy<-180.) dy+=360.;
	  
			if (fabs(dx) > TRACK_ACCURACY) {
				// action to proceed to the new point
			}
	  
			if (fabs(dy) > TRACK_ACCURACY) {
			}
	  
			break;
*/	  
		default:
			break;
	}
    
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
    
	if (HorN[AXIS_AZ].value != currentHorizontalCoords.Az.value()
		|| HorN[AXIS_ALT].value != currentHorizontalCoords.Alt.value())
	{
		HorN[AXIS_AZ].value=currentHorizontalCoords.Az.value();
		HorN[AXIS_ALT].value=currentHorizontalCoords.Alt.value();
		HorNP.s = IPS_OK;
		//lastEqState = EqNP.s;
		IDSetNumber(&HorNP, NULL);
	}
  
	double currentRA { 0. }, currentDEC { 0. }; 
	Hor2Equ(currentHorizontalCoords, &currentRA, &currentDEC);
	//currentRA*=24./360.;
    
	char RAStr[64]={0}, DecStr[64]={0};

	// Parse the RA/DEC into strings
	fs_sexa(RAStr, currentRA, 2, 3600);
	fs_sexa(DecStr, currentDEC, 2, 3600);

	DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);

	NewRaDec(currentRA, currentDEC);
	//Equ2Hor(currentRA, currentDEC, &currentAz, &currentAlt);

	return true;
}
