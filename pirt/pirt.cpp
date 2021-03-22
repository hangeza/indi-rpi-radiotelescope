/*
   INDI Raspberry Pi based mount driver.
   The driver itself acts as a telescope mount reading the positions from SSI-based absolute encoders from
   the on-board SPI interfaces and driving DC motors via PWM over GPIO pins
   "Pi Radiotelescope Driver"

*/

/** \file pirt.cpp
    \brief A customized INDI telescope device with full functionality of a telescope mount.
    \author Hans-Georg Zaunick
*/

#include "pirt.h"

#include "indicom.h"
//#include <connectioninterface.h>

#include <cmath>
#include <memory>
#include <map>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libnova.h>

#include <encoder.h>
#include <gpioif.h>
#include <motordriver.h>
#include <ads1115.h>
//#include <rpi_temperatures.h>

namespace Connection
{
class Interface;
class Serial;
class TCP;
}

constexpr unsigned int SSI_BAUD_RATE { 500000 };
constexpr unsigned int POLL_INTERVAL_MS { 200 };
constexpr double DEFAULT_AZ_AXIS_TURNS_RATIO { 152 /*152./9.*/ };
constexpr double DEFAULT_EL_AXIS_TURNS_RATIO { 20. };
constexpr double MAX_AZ_OVERTURN { 0.5 }; //< maximum overturn in Az in revolutions at both ends
constexpr double MAX_ALT_OVERTURN { 5./360. }; //< maximum overturn in Alt in revolutions at both ends

constexpr double SID_RATE { 0.004178 }; /* sidereal rate, degrees/s */
constexpr double SLEW_RATE { 5. };        /* slew rate, degrees/s */

constexpr double POS_ACCURACY_COARSE { 3.0 };
constexpr double POS_ACCURACY_FINE { 0.1 };
constexpr double TRACK_ACCURACY { 0.017 }; // one arc minute

constexpr double MIN_AZ_MOTOR_THROTTLE_DEFAULT { 0.06 };
constexpr double MIN_ALT_MOTOR_THROTTLE_DEFAULT { 0.14 };
constexpr double AZ_MOTOR_CURRENT_LIMIT_DEFAULT { 2. }; //< absolute motor current limit for Az motor in Ampere
constexpr double ALT_MOTOR_CURRENT_LIMIT_DEFAULT { 1.5 }; //< absolute motor current limit for Alt motor in Ampere
constexpr double MOTOR_CURRENT_FACTOR { 1./0.14 }; //< conversion factor for motor current sense in A/V
constexpr bool AZ_DIR_INVERT { true };
constexpr bool ALT_DIR_INVERT { false };
constexpr PiRaTe::MotorDriver::Pins AZ_MOTOR_PINS { 
	.Pwm=12,
	.Dir=-1,
	.DirA=23,
	.DirB=24,
	.Enable=25,
	.Fault=-1	};
	
constexpr PiRaTe::MotorDriver::Pins ALT_MOTOR_PINS { 
	.Pwm=13,
	.Dir=-1,
	.DirA=5,
	.DirB=6,
	.Enable=26,
	.Fault=-1	};

constexpr std::uint8_t MOTOR_ADC_ADDR { 0x48 };
constexpr std::uint8_t VOLTAGE_MONITOR_ADC_ADDR { 0x49 };

struct Relay {
	std::string name;
	unsigned int gpio_pin;
	bool inverted;
};

const std::vector<Relay> RelayVector { 	{ "Relay1", 17, true },
										{ "Relay2", 27, true },
										{ "Relay3", 22, true },
										{ "Relay4", 16, true } };

struct I2cVoltageDef {
	std::string name;
	double nominal;
	double divider_ratio;
	std::uint8_t adc_address;
	std::uint8_t adc_channel;
};

const std::vector<I2cVoltageDef> voltage_defs { { "+3.3V", 3.3, 2., VOLTAGE_MONITOR_ADC_ADDR, 0 },
												{ "+5V", 5., 2., VOLTAGE_MONITOR_ADC_ADDR, 1 },
												{ "+12V", 12., 11., VOLTAGE_MONITOR_ADC_ADDR, 2 },
												{ "+24V", 24., 11., VOLTAGE_MONITOR_ADC_ADDR, 3 } };

const HorCoords DefaultParkPosition { 180., 89.5 };

// VSTW Radebeul, Google-Maps: 51°06'58.1"N 13°37'17.3"E
const std::map<INDI::Telescope::TelescopeLocation, double> DefaultLocation =
	{ 	{ INDI::Telescope::LOCATION_LATITUDE, 51.116139 },
		{ INDI::Telescope::LOCATION_LONGITUDE, 13.621472 },
		{ INDI::Telescope::LOCATION_ELEVATION, 200. } };



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
    // We add an additional debug level so we can log verbose scope status
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");

//    setTelescopeConnection(CONNECTION_SERIAL);
    setTelescopeConnection(CONNECTION_TCP);
//    setTelescopeConnection(CONNECTION_NONE);
//    setTelescopeConnection(Connection::Interface::CONNECTION_CUSTOM);
//    setTelescopeConnection(1<<15);
	
	SetTelescopeCapability(
			TELESCOPE_CAN_GOTO |
			TELESCOPE_CAN_PARK |
			TELESCOPE_CAN_ABORT |
			TELESCOPE_HAS_TIME | 
			TELESCOPE_HAS_LOCATION |
			TELESCOPE_HAS_TRACK_MODE |
			TELESCOPE_CAN_CONTROL_TRACK /*|
			TELESCOPE_HAS_TRACK_RATE */
	);
}

/**************************************************************************************
** We init our properties here. The only thing we want to init are the Debug controls
***************************************************************************************/
bool PiRT::initProperties()
{
    // ALWAYS call initProperties() of parent first
    INDI::Telescope::initProperties();

	setDefaultPollingPeriod(POLL_INTERVAL_MS);

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
    
    LocationN[LOCATION_LATITUDE].value  = DefaultLocation.at(LOCATION_LATITUDE);
    LocationN[LOCATION_LONGITUDE].value = DefaultLocation.at(LOCATION_LONGITUDE);
    LocationN[LOCATION_ELEVATION].value = DefaultLocation.at(LOCATION_ELEVATION);
    LocationNP.s = IPS_OK;
    IDSetNumber(&LocationNP, NULL);

	IUFillNumber(&EncoderBitRateN, "SSI_BITRATE", "SSI Bit Rate", "%5.0f Hz", 0, 5000000, 0, SSI_BAUD_RATE);
    IUFillNumberVector(&EncoderBitRateNP, &EncoderBitRateN, 1, getDeviceName(), "ENC_SPI_SETTINGS", "SPI Interface", "Encoders",
           IP_RW, 60, IPS_IDLE);
	defineProperty(&EncoderBitRateNP);
    IDSetNumber(&EncoderBitRateNP, nullptr);

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
	IUFillNumber(&AxisAbsTurnsN[1], "ALT_AXIS_TURNS", "Alt", "%5.4f rev", 0, 0, 0, 0);
    IUFillNumberVector(&AxisAbsTurnsNP, AxisAbsTurnsN, 2, getDeviceName(), "AXIS_ABSOLUTE_TURNS", "Absolute Axis Turns", "Axes",
           IP_RO, 60, IPS_IDLE);
	
	IUFillNumber(&MotorStatusN[0], "AZ_MOTOR_SPEED", "Az", "%4.0f %%", -100, 100, 0, 0);
	IUFillNumber(&MotorStatusN[1], "ALT_MOTOR_SPEED", "Alt", "%4.0f %%", -100, 100, 0, 0);
    IUFillNumberVector(&MotorStatusNP, MotorStatusN, 2, getDeviceName(), "MOTOR_STATUS", "Motor Status", "Motors",
           IP_RO, 60, IPS_IDLE);

	IUFillNumber(&MotorThresholdN[0], "AZ_MOTOR_THRESHOLD", "Az", "%4.0f %%", 0, 100, 0, MIN_AZ_MOTOR_THROTTLE_DEFAULT * 100);
	IUFillNumber(&MotorThresholdN[1], "ALT_MOTOR_THRESHOLD", "Alt", "%4.0f %%", 0, 100, 0, MIN_ALT_MOTOR_THROTTLE_DEFAULT * 100);
    IUFillNumberVector(&MotorThresholdNP, MotorThresholdN, 2, getDeviceName(), "MOTOR_THRESHOLD", "Motor Thresholds", "Motors",
           IP_RW, 60, IPS_IDLE);
	
	IUFillNumber(&MotorCurrentN[0], "AZ_MOTOR_CURRENT", "Az", "%4.2f A", 0, 0, 0, 0);
	IUFillNumber(&MotorCurrentN[1], "ALT_MOTOR_CURRENT", "Alt", "%4.2f A", 0, 0, 0, 0);
    IUFillNumberVector(&MotorCurrentNP, MotorCurrentN, 2, getDeviceName(), "MOTOR_CURRENT", "Motor Currents", "Motors",
		IP_RO, 60, IPS_IDLE);

	IUFillNumber(&MotorCurrentLimitN[0], "AZ_MOTOR_CURRENT_LIMIT", "Az", "%4.2f A", 0, 0, 0, AZ_MOTOR_CURRENT_LIMIT_DEFAULT);
	IUFillNumber(&MotorCurrentLimitN[1], "ALT_MOTOR_CURRENT_LIMIT", "Alt", "%4.2f A", 0, 0, 0, ALT_MOTOR_CURRENT_LIMIT_DEFAULT);
    IUFillNumberVector(&MotorCurrentLimitNP, MotorCurrentLimitN, 2, getDeviceName(), "MOTOR_CURRENT_LIMITS", "Motor Current Limits", "Motors",
		IP_RW, 60, IPS_IDLE);

	IUFillNumber(&VoltageMonitorN[0], "VOLTAGE", "+0V", "%4.2f V", 0, 0, 0, 0);
    IUFillNumberVector(&VoltageMonitorNP, VoltageMonitorN, 0, getDeviceName(), "VOLTAGE_MONITOR", "Voltages", "Monitoring",
		IP_RO, 60, IPS_IDLE);

	IUFillNumber(&TempMonitorN[0], "TEMP_SYSTEM", "CPU", "%4.2f °C", 0, 0, 0, 0);
	IUFillNumberVector(&TempMonitorNP, TempMonitorN, 0, getDeviceName(), "TEMPERATURE_MONITOR", "Temperatures", "Monitoring",
		IP_RO, 60, IPS_IDLE);

	
	for ( std::size_t relay_index = 0; relay_index < RelayVector.size(); relay_index++) {
		IUFillSwitch(&RelaySwitchS[relay_index], "SWITCH", "On", ISS_OFF);
		IUFillSwitchVector(&RelaySwitchSP[relay_index], &RelaySwitchS[relay_index], 1, getDeviceName(), std::string("SWITCH"+std::to_string(relay_index)).c_str(), RelayVector[relay_index].name.c_str(), "Switches",
			IP_RW, ISR_NOFMANY, 60, IPS_IDLE);

	}	

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

		deleteProperty(EncoderBitRateNP.name);
		IUFillNumberVector(&EncoderBitRateNP, &EncoderBitRateN, 1, getDeviceName(), "ENC_SPI_SETTINGS", "SPI Interface", "Encoders",
           IP_RO, 60, IPS_IDLE);
		defineProperty(&EncoderBitRateNP);
		IDSetNumber(&EncoderBitRateNP, nullptr);

		defineProperty(&AzEncoderNP);
		defineProperty(&ElEncoderNP);
		defineProperty(&AxisAbsTurnsNP);
		defineProperty(&MotorStatusNP);
		defineProperty(&MotorCurrentNP);
		defineProperty(&MotorThresholdNP);
		defineProperty(&MotorCurrentLimitNP);
		defineProperty(&VoltageMonitorNP);
		defineProperty(&TempMonitorNP);
		for ( std::size_t relay_index = 0; relay_index < RelayVector.size(); relay_index++ ) {
			defineProperty(&RelaySwitchSP[relay_index]);
		}
		//deleteProperty(EncoderBitRateNP.name);
    } else {
		deleteProperty(ScopeStatusLP.name);
		deleteProperty(HorNP.name);
		deleteProperty(JDNP.name);

		deleteProperty(EncoderBitRateNP.name);
		IUFillNumberVector(&EncoderBitRateNP, &EncoderBitRateN, 1, getDeviceName(), "ENC_SPI_SETTINGS", "SPI Interface", "Encoders",
           IP_RW, 60, IPS_IDLE);
		defineProperty(&EncoderBitRateNP);
		IDSetNumber(&EncoderBitRateNP, nullptr);

		deleteProperty(AzEncoderNP.name);
		deleteProperty(ElEncoderNP.name);
		deleteProperty(AxisAbsTurnsNP.name);
		deleteProperty(MotorStatusNP.name);
		deleteProperty(MotorCurrentNP.name);
		deleteProperty(MotorThresholdNP.name);
		deleteProperty(MotorCurrentLimitNP.name);
		deleteProperty(VoltageMonitorNP.name);
		deleteProperty(TempMonitorNP.name);
		for ( std::size_t relay_index = 0; relay_index < RelayVector.size(); relay_index++ ) {
			deleteProperty(RelaySwitchSP[relay_index].name);
		}
//		defineProperty(&EncoderBitRateNP);
	}
    
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
		for ( std::size_t relayIndex = 0; relayIndex < RelayVector.size(); relayIndex++ ) {
			if(!strcmp(name,RelaySwitchSP[relayIndex].name)) {
				IUUpdateSwitch(&RelaySwitchSP[relayIndex], states, names, n);
				if ( RelaySwitchS[relayIndex].s == ISS_ON ) {
					RelaySwitchSP[relayIndex].s = IPS_OK;
					gpio->set_gpio_state(RelayVector[relayIndex].gpio_pin, !RelayVector[relayIndex].inverted );
					std::string tempstr { "Relay"+std::to_string(relayIndex+1)+" switch on" };
					IDSetSwitch( &RelaySwitchSP[relayIndex], tempstr.c_str() );
				} else {
					RelaySwitchSP[relayIndex].s = IPS_IDLE;
					gpio->set_gpio_state(RelayVector[relayIndex].gpio_pin, RelayVector[relayIndex].inverted );
					std::string tempstr { "Relay"+std::to_string(relayIndex+1)+" switch off" };
					IDSetSwitch( &RelaySwitchSP[relayIndex], tempstr.c_str() );
				}
				return true;
			}	
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
		if(!strcmp(name, HorNP.name))
		{
			if (n != 2) return false;
			double az = values[0];
			double alt = values[1];
			if ( (az<0.) || (az>360.) || (alt<-90.) || (alt>90.) ) return false;
			// Remember Track State
			RememberTrackState = TrackState;
			// Issue GOTO
            bool rc = GotoHor(az, alt);
			if (rc)
			{
				HorNP.s = lastHorState = IPS_BUSY;
				//  Now fill in target co-ords, so domes can start turning
				double ra { 0. }, dec {0. };
				Hor2Equ(az, alt, &ra, &dec);
				TargetN[AXIS_RA].value = ra;
				TargetN[AXIS_DE].value = dec;
				IDSetNumber(&TargetNP, nullptr);
			}
			else
			{
				HorNP.s = lastHorState = IPS_ALERT;
			}
			IDSetNumber(&HorNP, nullptr);
			return rc;
		} else if(!strcmp(name, EncoderBitRateNP.name))
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
		} else if(!strcmp(name, MotorCurrentLimitNP.name)) {
			// set motor current limit
			MotorCurrentLimitNP.s = IPS_OK;
			MotorCurrentLimitN[0].value = values[0];
			MotorCurrentLimitN[1].value = values[1];
			IDSetNumber(&MotorCurrentLimitNP, nullptr);
			DEBUGF(DBG_SCOPE, "Setting motor current limits to %5.3f A (Az) and %5.3f A (Alt)", MotorCurrentLimitN[0].value, MotorCurrentLimitN[1].value);
			return true;
		} else if(!strcmp(name, MotorThresholdNP.name)) {
			// set motor thresholds
			MotorThresholdNP.s = IPS_OK;
			MotorThresholdN[0].value = values[0];
			MotorThresholdN[1].value = values[1];
			IDSetNumber(&MotorThresholdNP, nullptr);
			DEBUGF(DBG_SCOPE, "Setting motor thresholds to %4.0f %% (Az) and %4.0f %% (Alt)", MotorThresholdN[0].value, MotorThresholdN[1].value);
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
	if ( enabled && TrackState == SCOPE_PARKED ) {
        DEBUG(INDI::Logger::DBG_ERROR, "Scope in park position - tracking is prohibited.");
		return false;
	}
	if (enabled) {
		targetEquatorialCoords = Hor2Equ(currentHorizontalCoords);
	} else {
		Abort();
	}
	fIsTracking = enabled;
	return true;
}

bool PiRT::Park() {
	if ( TrackState == SCOPE_PARKED ) {
        DEBUG(INDI::Logger::DBG_ERROR, "Scope already parked.");
		return false;
	}
	
	targetHorizontalCoords = DefaultParkPosition;

	char AzStr[64]={0}, AltStr[64]={0};

    // Parse the Az/Alt into strings
    fs_sexa(AzStr, targetHorizontalCoords.Az.value(), 2, 3600);
    fs_sexa(AltStr, targetHorizontalCoords.Alt.value(), 2, 3600);

    // Mark state as parking
	TrackState = SCOPE_PARKING;
	TargetCoordSystem = SYSTEM_HOR;

    // Inform client we are slewing to a new position
    DEBUGF(INDI::Logger::DBG_SESSION, "Slewing to Park Pos ( Az: %s - Alt: %s )", AzStr, AltStr);
	
	return true;
}

bool PiRT::UnPark() {
	if ( TrackState != SCOPE_PARKED ) {
        DEBUG(INDI::Logger::DBG_ERROR, "Scope already unparked.");
		return false;
	}
	SetParked(false);
	
	TrackState = SCOPE_IDLE;
	
	return true;
}

bool PiRT::Connect()
{
	ITextVectorProperty *tvp = getText("DEVICE_ADDRESS");
	if (tvp == nullptr) {
        DEBUG(INDI::Logger::DBG_ERROR, "No address property found.");
		//return false;
	}
	
	//DEBUGF(INDI::Logger::DBG_SESSION, "host: %s:%s", tvp->tp[0].text, tvp->tp[1].text);
/*
	const std::string host { tvp->tp[0].text };
	const std::string port { tvp->tp[1].text };
*/

	// before instanciating a new GPIO interface, all objects which carry a reference
	// to the old gpio object must be invalidated, to make sure
	// that noone else uses the shared_ptr<GPIO> when it is newly created
	az_encoder.reset();
	el_encoder.reset();
	az_motor.reset();
	el_motor.reset();
	
//	gpio.reset( new GPIO(host, port) );
	gpio.reset( new GPIO("localhost", "8888") );
	if (!gpio->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Could not initialize GPIO interface. Is pigpiod running?");
		return false;
	}
	
	// set the baud rate on the SPI interface for communication with the pos encoders
	unsigned int bitrate = static_cast<unsigned int>( EncoderBitRateNP.np[0].value );
	if ( bitrate < 80000 || bitrate > 5000000 ) {
        DEBUG(INDI::Logger::DBG_ERROR, "SSI bitrate out of range (80k...5M)");
		return false;
	}

	// initialize Az pos encoder
	az_encoder.reset(new PiRaTe::SsiPosEncoder(gpio, GPIO::SPI_INTERFACE::Main, bitrate, 0, GPIO::SPI_MODE::POL1PHA1));
	if (!az_encoder->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to connect to Az position encoder.");
		return false;
	}
    DEBUG(INDI::Logger::DBG_SESSION, "Az position encoder ok.");

	az_encoder->setStBitWidth(AzEncSettingN[0].value);
	az_encoder->setMtBitWidth(AzEncSettingN[1].value);

	// initialize Alt pos encoder
	el_encoder.reset(new PiRaTe::SsiPosEncoder(gpio, GPIO::SPI_INTERFACE::Aux, bitrate));
	if (!el_encoder->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to connect to Alt position encoder.");
		return false;
	}
    DEBUG(INDI::Logger::DBG_SESSION, "Alt position encoder ok.");
	el_encoder->setStBitWidth(ElEncSettingN[0].value);
	el_encoder->setMtBitWidth(ElEncSettingN[1].value);

	// search for the ADS1115 ADCs at the specified addresses and initialize them
	// instantiate the first ADS1115 foreseen to read back the motor currents
	std::shared_ptr<ADS1115> adc { new ADS1115(MOTOR_ADC_ADDR) };
	if ( adc != nullptr && adc->devicePresent() ) {
		adc->setPga(ADS1115::PGA4V);
		adc->setRate(ADS1115::RATE860);
		adc->setAGC(true);
		double v1 = adc->readVoltage(0);
		double v2 = adc->readVoltage(1);
		double v3 = adc->readVoltage(2);
		double v4 = adc->readVoltage(3);
		
		i2cDeviceMap.emplace( std::make_pair( MOTOR_ADC_ADDR, std::move (adc) ) );
		DEBUGF(INDI::Logger::DBG_SESSION, "ADC1 values ch0: %f V ch1: %f ch3: %f V ch4: %f", v1,v2,v3,v4);
	} else {
		DEBUGF(INDI::Logger::DBG_ERROR, "ADS1115 at address 0x%02x not found.", MOTOR_ADC_ADDR);
		deleteProperty(MotorCurrentNP.name);
		deleteProperty(MotorCurrentLimitNP.name);
	}
	// instantiate second ADS1115 for voltage monitoring
	adc.reset( new ADS1115(VOLTAGE_MONITOR_ADC_ADDR) );
	if ( adc != nullptr && adc->devicePresent() ) {
		adc->setPga(ADS1115::PGA4V);
		adc->setRate(ADS1115::RATE860);
		adc->setAGC(true);
		double v1 = adc->readVoltage(0);
		double v2 = adc->readVoltage(1);
		double v3 = adc->readVoltage(2);
		double v4 = adc->readVoltage(3);
		
		i2cDeviceMap.emplace( std::make_pair( VOLTAGE_MONITOR_ADC_ADDR, std::move (adc) ) );
		DEBUGF(INDI::Logger::DBG_SESSION, "ADC2 values ch0: %f V ch1: %f ch3: %f V ch4: %f", v1,v2,v3,v4);
	} else {
		DEBUGF(INDI::Logger::DBG_ERROR, "ADS1115 at address 0x%02x not found.", VOLTAGE_MONITOR_ADC_ADDR);
	}
	
	// initialize Az motor driver
	az_motor.reset( new PiRaTe::MotorDriver( gpio, AZ_MOTOR_PINS, AZ_DIR_INVERT, std::dynamic_pointer_cast<ADS1115>( i2cDeviceMap[MOTOR_ADC_ADDR] ), 0 ) );
	if (!az_motor->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to initialize Az motor driver.");
		return false;
	}
	// initialize Alt motor driver
	el_motor.reset( new PiRaTe::MotorDriver( gpio, ALT_MOTOR_PINS, ALT_DIR_INVERT, std::dynamic_pointer_cast<ADS1115>( i2cDeviceMap[MOTOR_ADC_ADDR] ), 1 ) );
	if (!el_motor->isInitialized()) {
        DEBUG(INDI::Logger::DBG_ERROR, "Failed to initialize El motor driver.");
		return false;
	}
	
	// initialize the temperature monitor
	TempMonitorNP.nnp = 0;
	IDSetNumber(&TempMonitorNP, nullptr);
	tempMonitor.reset( new PiRaTe::RpiTemperatureMonitor() );
	if (tempMonitor != nullptr) {
		tempMonitor->registerTempReadyCallback( [this](PiRaTe::RpiTemperatureMonitor::TemperatureItem item) { this->updateTemperatures(item); } );
	}

	// set up the voltages to be monitored
	voltageMonitors.clear();
	int voltage_index = 0;
	for ( auto item: voltage_defs ) {
		auto it = i2cDeviceMap.find( item.adc_address );
		if (  it == i2cDeviceMap.end() ) continue;
		std::shared_ptr<ADS1115> adc( std::dynamic_pointer_cast<ADS1115>(it->second) );
		std::shared_ptr<PiRaTe::Ads1115VoltageMonitor> mon( 
			new PiRaTe::Ads1115VoltageMonitor( item.name, adc, item.adc_channel, item.nominal, item.divider_ratio, item.nominal/10. )
		);
		voltageMonitors.emplace_back( std::move(mon) );
		deleteProperty(VoltageMonitorNP.name);
		IUFillNumber(&VoltageMonitorN[voltage_index], ("VOLTAGE"+std::to_string(voltage_index)).c_str(), (item.name).c_str(), "%4.2f V", item.nominal*0.9 , item.nominal*1.1, 0, 0.);
		IUFillNumberVector(&VoltageMonitorNP, VoltageMonitorN, voltage_index+1, getDeviceName(), "VOLTAGE_MONITOR", "System Voltages", "Monitoring",
			IP_RO, 60, IPS_IDLE);
		defineProperty(&VoltageMonitorNP);
		voltage_index++;
	}
	
	// set up the gpio pins for the relay switches
	for ( unsigned int i = 0; i<RelayVector.size(); i++ ) {
		gpio->set_gpio_direction( RelayVector[i].gpio_pin, true );
		gpio->set_gpio_state( RelayVector[i].gpio_pin, RelayVector[i].inverted );
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
		//DEBUG(INDI::Logger::DBG_SESSION, "Timer hit");
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
auto PiRT::isTracking() const -> bool
{
	//return true;
	//return (TrackStateS[TRACK_ON].s == ISS_ON);
	return fIsTracking;
}


/**************************************************************************************
** Client is asking us to slew to a new position
***************************************************************************************/
bool PiRT::Goto(double ra, double dec)
{
	targetEquatorialCoords = EquCoords{ ra , dec };
	targetHorizontalCoords = Equ2Hor(targetEquatorialCoords);
	
	if ( targetHorizontalCoords.Alt.value() < 0. ) {
      DEBUG(INDI::Logger::DBG_WARNING, "Error: Target below horizon");
      return false;
	}
	
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
	if (isParked())
	{
		DEBUG(INDI::Logger::DBG_WARNING, "Please unpark the mount before issuing any motion/sync commands.");
		return false;
	}

	if ( alt < 0. ) {
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
	az_motor->stop();
	el_motor->stop();
	if ( TrackState == SCOPE_IDLE || TrackState == SCOPE_TRACKING || TrackState == SCOPE_PARKED ) return true;
	else  TrackState = (isTracking() ? SCOPE_TRACKING : SCOPE_IDLE);

	targetEquatorialCoords = Hor2Equ( currentHorizontalCoords );
	return true;
}

bool PiRT::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (command != MOTION_START) {
		el_motor->stop();
		return true;
	}
	constexpr float speed = 0.2;
	switch (dir) {
		case DIRECTION_SOUTH:
			el_motor->move(-speed);
			break;
		case DIRECTION_NORTH:
			el_motor->move(speed);
			break;
		default:
			el_motor->stop();
			break;
	}
	return true;
    //IUResetSwitch(&MovementNSSP);
    MovementNSSP.s = IPS_BUSY;
    IDSetSwitch(&MovementNSSP, nullptr);
    return false;
}

bool PiRT::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if (command != MOTION_START) {
		az_motor->stop();
		return true;
	}
	constexpr float speed = 0.15;
	switch (dir) {
		case DIRECTION_WEST:
			az_motor->move(-speed);
			break;
		case DIRECTION_EAST:
			az_motor->move(speed);
			break;
		default:
			az_motor->stop();
			break;
	}
	return true;

    IUResetSwitch(&MovementWESP);
    MovementWESP.s = IPS_IDLE;
    IDSetSwitch(&MovementWESP, nullptr);
    return false;
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

bool PiRT::isInAbsoluteTurnRange(double absRev) {
	
	if ( (absRev > -0.5 - MAX_AZ_OVERTURN)
		&& (absRev < 0.5 + MAX_AZ_OVERTURN) )
		return true;
	return false;
}

void PiRT::updateMotorStatus() {
	if ( (az_motor->hasFaultSense() && az_motor->isFault()) 
		|| (el_motor->hasFaultSense() && el_motor->isFault()) ) {
		MotorStatusNP.s=IPS_ALERT;
	} else {
		MotorStatusNP.s=IPS_OK;
	}
	MotorStatusN[0].value = 100. * az_motor->currentSpeed();
	MotorStatusN[1].value = 100. * el_motor->currentSpeed();
	IDSetNumber(&MotorStatusNP, nullptr);

	if ( az_motor->hasAdc() || el_motor->hasAdc())
	{
		MotorCurrentNP.s=IPS_OK;
		if ( az_motor->hasAdc() ) {
			MotorCurrentN[0].value = az_motor->readCurrent() + 0.005;
		} else {
			MotorCurrentNP.s=IPS_BUSY;
		}
		if ( el_motor->hasAdc() ) {
			MotorCurrentN[1].value = el_motor->readCurrent() + 0.005;
		} else {
			MotorCurrentNP.s=IPS_BUSY;
		}
		if ( MotorCurrentN[0].value > MotorCurrentLimitN[0].value || MotorCurrentN[1].value > MotorCurrentLimitN[1].value ) {
			MotorCurrentNP.s=IPS_ALERT;
		}
		//DEBUGF(INDI::Logger::DBG_SESSION, "ADC value ch0: %f V ch1: %f ch3: %f V ch4: %f", v1,v2,v3,v4);
		IDSetNumber(&MotorCurrentNP, nullptr);
	}
}

void PiRT::updateMonitoring() {
	if (voltageMonitors.empty()) return;
	int voltage_index = 0;
	bool outsideRange { false };
	VoltageMonitorNP.s=IPS_IDLE;
	for ( auto monitor: voltageMonitors ) {
		if ( !monitor->isInitialized() ) {
			VoltageMonitorN[voltage_index].value = 0.;
			VoltageMonitorNP.s=IPS_ALERT;
		} else {
			double meanVoltage = monitor->meanVoltage();
			if ( meanVoltage < VoltageMonitorN[voltage_index].min || meanVoltage > VoltageMonitorN[voltage_index].max ) {
				outsideRange = true;
			}
			VoltageMonitorN[voltage_index].value = meanVoltage;
		}
		voltage_index++;
	}
	if ( VoltageMonitorNP.s != IPS_ALERT ) {
		if (outsideRange) VoltageMonitorNP.s=IPS_BUSY;
		else VoltageMonitorNP.s = IPS_OK;
	}
	IDSetNumber(&VoltageMonitorNP, nullptr);
/*
	if ( adc != nullptr && adc->devicePresent() ) {
		double v3 = adc->readVoltage(2);
		double v4 = adc->readVoltage(3);
		
		VoltageMonitorN[0].value = v3 * VOLTAGE_MONITORING_RATIO[0];
		VoltageMonitorN[1].value = v4 * VOLTAGE_MONITORING_RATIO[1];
		
		VoltageMonitorNP.s=IPS_OK;
		//DEBUGF(INDI::Logger::DBG_SESSION, "ADC value ch0: %f V ch1: %f ch3: %f V ch4: %f", v1,v2,v3,v4);
		IDSetNumber(&VoltageMonitorNP, nullptr);
	}
*/
}

void PiRT::updateTemperatures( PiRaTe::RpiTemperatureMonitor::TemperatureItem item ) {
	if (!isConnected()) return;
//	DEBUGF( INDI::Logger::DBG_SESSION, " temp measurement from source %s: %f", item.id.c_str(), item.temperature );
	int source = item.sourceIndex;
	if ( source < TempMonitorNP.nnp ) {
		TempMonitorN[source].value = item.temperature;
		IDSetNumber(&TempMonitorNP, nullptr);
		return;
	} 
	deleteProperty(TempMonitorNP.name);
	
	IUFillNumber(&TempMonitorN[source], ("TEMPERATURE"+std::to_string(source)).c_str(), (item.name+":"+item.id).c_str(), "%4.2f \xB0 C", 0, 0, 0, item.temperature);
	IUFillNumberVector(&TempMonitorNP, TempMonitorN, source+1, getDeviceName(), "TEMPERATURE_MONITOR", "Temperatures", "Monitoring",
           IP_RO, 60, IPS_IDLE);
	
	defineProperty(&TempMonitorNP);
	IDSetNumber(&TempMonitorNP, nullptr);
}

void PiRT::updatePosition() {
	double azAbsTurns { 0. };
	double altAbsTurns { 0. };
    
	if ( az_encoder == nullptr || el_encoder == nullptr ) {
		if ( az_encoder == nullptr ) AzEncoderNP.s = IPS_ALERT;
		if ( el_encoder == nullptr ) ElEncoderNP.s = IPS_ALERT;
		return;
	} 
	// read pos encoders
    if ( az_encoder->isUpdated() || el_encoder->isUpdated() ) {
		AzEncoderN[0].value = az_encoder->absolutePosition();
		AzEncoderN[1].value = static_cast<double>(az_encoder->position());
		AzEncoderN[2].value = static_cast<double>(az_encoder->nrTurns());
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

		azAbsTurns = ( az_revolutions / axisRatio[0] ) + axisOffset[0] / 360.;
		altAbsTurns = ( el_revolutions / axisRatio[1] ) + axisOffset[1] / 360.;

		AxisAbsTurnsN[0].value = azAbsTurns;
		AxisAbsTurnsN[1].value = altAbsTurns;
		if ( std::abs(azAbsTurns) > 0.5 + MAX_AZ_OVERTURN || 
			 std::abs(altAbsTurns) > 0.25 + MAX_ALT_OVERTURN ) 
		{
			AxisAbsTurnsNP.s = IPS_ALERT;
		} else {
			AxisAbsTurnsNP.s = IPS_OK;
		}
		IDSetNumber(&AxisAbsTurnsNP, nullptr);
	
		currentHorizontalCoords.Az.setValue( 360. * azAbsTurns );
		currentHorizontalCoords.Alt.setValue( 360. * altAbsTurns );
		//DEBUG(INDI::Logger::DBG_SESSION, "encoders updated");
	}
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

    //DEBUG(INDI::Logger::DBG_SESSION, "before encoder readout");
	double azAbsTurns { 0. };
	double altAbsTurns { 0. };

	// read pos encoders and update the horizontal coordinates, absolute turn values and properties
	updatePosition();
	azAbsTurns = AxisAbsTurnsN[0].value;
	altAbsTurns = AxisAbsTurnsN[1].value;

	// update motor status
	updateMotorStatus();
	
	// update monitoring variables
	updateMonitoring();

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
    
	// the state machine to handle all operation conditions:
	// SCOPE_IDLE, SCOPE_TRACKING, SCOPE_PARKING, SCOPE_PARKED and SCOPE_SLEWING
	switch (TrackState)
	{
		case SCOPE_TRACKING:
			TargetCoordSystem = SYSTEM_HOR;
			targetHorizontalCoords = Equ2Hor(targetEquatorialCoords);
		case SCOPE_PARKING:
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
			
			if ( dx > 180. ) { dx -= 360.; }
			else if ( dx < -180. ) { dx += 360.; }
			
			// this should never happen
			if ( dy > 180. ) { dy -= 360.; } 
			else if ( dy < -180. ) { dy += 360.; }
			
/*
			DEBUGF(INDI::Logger::DBG_SESSION, "trg  Az=%f Alt=%f", targetHorizontalCoords.Az.value(), targetHorizontalCoords.Alt.value());
			DEBUGF(INDI::Logger::DBG_SESSION, "curr Az=%f Alt=%f", currentHorizontalCoords.Az.value(), currentHorizontalCoords.Alt.value());
			DEBUGF(INDI::Logger::DBG_SESSION, "initial dx=%f dy=%f", dx, dy);
*/
			// check, if the absolute position of the target is beyond te allowable limit
			if ( !isInAbsoluteTurnRange( azAbsTurns + dx/360. ) ) {
//				DEBUGF(INDI::Logger::DBG_SESSION, "initial dx=%f dy=%f", dx, dy);
				// if not, we still must be sure to turn into the right direction toward the allowable range
				// e.g. if we are currently far in the forbidden range, make sure to not go further in
				const double alt_dx = ( dx > 0. ) ? ( dx - 360. ) : ( dx + 360. );
				if ( std::abs( azAbsTurns + dx/360. ) > std::abs( azAbsTurns + alt_dx/360. ) ) {
					dx = alt_dx;
				}
//				DEBUGF(INDI::Logger::DBG_SESSION, "allowed dx=%f dy=%f", dx, dy);
			}

			if ( std::abs(dx) > POS_ACCURACY_COARSE ) {
				az_motor->move((dx>=0)?1.:-1.);
			} else if ( std::abs(dx) > POS_ACCURACY_FINE ) {				
				double mot = dx/POS_ACCURACY_COARSE;
				if (std::abs(mot) < MotorThresholdN[0].value/100.) {
					mot = (dx>0.) ? MotorThresholdN[0].value/100. : -MotorThresholdN[0].value/100.;
				}
				az_motor->move( mot );
			} else if ( std::abs(dx) > TRACK_ACCURACY ) {
				az_motor->move( (dx>0.) ? MotorThresholdN[0].value/100. : -MotorThresholdN[0].value/100. );
			} else az_motor->stop();

			if ( std::abs(dy) > POS_ACCURACY_COARSE ) {
				el_motor->move((dy>=0)?1.:-1.);
			} else if ( std::abs(dy) > POS_ACCURACY_FINE ) {				
				double mot = dy/POS_ACCURACY_COARSE;
				if (std::abs(mot) < MotorThresholdN[1].value/100.) {
					mot = (dy>0.) ? MotorThresholdN[1].value/100. : -MotorThresholdN[1].value/100.;
				}
				el_motor->move( mot );
			} else if ( std::abs(dy) > TRACK_ACCURACY ) {
				el_motor->move( (dy>0.) ? MotorThresholdN[1].value/100. : -MotorThresholdN[1].value/100. );
			} else el_motor->stop();

			// Let's check if we reached target position for both axes
			if ( 	std::abs(dx) < TRACK_ACCURACY 
				&& 	std::abs(dy) < TRACK_ACCURACY	)
			{
				if ( TargetCoordSystem == SYSTEM_EQ ) { 
					EqNP.s = IPS_OK;
					IDSetNumber(&EqNP, nullptr);
				} else if ( TargetCoordSystem == SYSTEM_HOR ) { 
					HorNP.s = lastHorState = IPS_OK;
					IDSetNumber(&HorNP, nullptr);
				}
				if ( TrackState == SCOPE_SLEWING ) {
					DEBUG(INDI::Logger::DBG_SESSION, "Telescope slew is complete.");
				}
				// Let's set state to Idle, Tracking or Parked
				if ( TrackState == SCOPE_PARKING) {
					fIsTracking = false;
					//TrackState = SCOPE_PARKED;
					SetParked(true);
				}
				Abort();
			}
			break;
		case SCOPE_PARKED:
		case SCOPE_IDLE:
/*
			if ( std::abs(az_motor->currentSpeed()) < 0.001 && std::abs(el_motor->currentSpeed()) < 0.001 ) {
				measureMotorCurrentOffsets();
			}
*/
		default:
			//Abort();
			break;
	}
    

	/* update scope status */
	// update the telescope state lights
	for (int i=0; i<5; i++) ScopeStatusL[i].s=IPS_IDLE;
	ScopeStatusL[TrackState].s=IPS_OK;
	IDSetLight(&ScopeStatusLP, NULL);
    
	// update horizontal coordinates
	if (HorN[AXIS_AZ].value != currentHorizontalCoords.Az.value()
		|| HorN[AXIS_ALT].value != currentHorizontalCoords.Alt.value())
	{
		HorN[AXIS_AZ].value=currentHorizontalCoords.Az.value();
		HorN[AXIS_ALT].value=currentHorizontalCoords.Alt.value();
		HorNP.s = IPS_IDLE;
		//lastEqState = EqNP.s;
		IDSetNumber(&HorNP, NULL);
	}
  
	double currentRA { 0. }, currentDEC { 0. }; 
	Hor2Equ(currentHorizontalCoords, &currentRA, &currentDEC);
    
	char RAStr[64]={0}, DecStr[64]={0};

	// Parse the RA/DEC into strings
	fs_sexa(RAStr, currentRA, 2, 3600);
	fs_sexa(DecStr, currentDEC, 2, 3600);

//	DEBUGF(DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr);

	NewRaDec(currentRA, currentDEC);

	return true;
}
