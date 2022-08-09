/*
 * Project LoRA-Mesh-Server
 * Description: This device will listen for data from client devices and forward the data to Particle via webhook
 * Author: Chip McClelland
 * Date: 7-28-22
 */

// v1 - Initial attempt - no sleeping but disconnects when not sending data
// v2 - Rough working example for a gateway and a single node

/*
payload[0] = 0;                             // to be replaced/updated
payload[1] = 0;                             // to be replaced/updated
payload[2] = highByte(devID);               // Set for device
payload[3] = lowByte(devID);
payload[4] = firmVersion;                   // Set for code release
payload[5] = highByte(hourly);				// Hourly count
payload[6] = lowByte(hourly); 
payload[7] = highByte(daily);				// Daily Count
payload[8] = lowByte(daily); 
payload[9] = temp;							// Enclosure temp
payload[10] = battChg;						// State of charge
payload[11] = battState; 					// Battery State
payload[12] = resets						// Reset count
Payload[13] = alerts						// Current alerts
payload[14] = highByte(rssi);				// Signal strength
payload[15] = lowByte(rssi); 
payload[16] = msgCnt++;						// Sequential message number
*/

#include <RHMesh.h>
#include <RH_RF95.h>						// https://docs.particle.io/reference/device-os/libraries/r/RH_RF95/
#include "PublishQueuePosixRK.h"			// https://github.com/rickkas7/PublishQueuePosixRK
#include "LocalTimeRK.h"					// https://rickkas7.github.io/LocalTimeRK/

const uint8_t firmware = 2;

// Mesh has much greater memory requirements, and you may need to limit the
// max message length to prevent wierd crashes
#ifndef RH_MAX_MESSAGE_LEN
#define RH_MAX_MESSAGE_LEN 255
#endif

// In this small artifical network of 4 nodes,
#define CLIENT_ADDRESS 1					// This is the node that is sending the data
#define SERVER1_ADDRESS 2					// This server
#define SERVER2_ADDRESS 3					// Other servers in the mesh - to be implemented
#define SERVER3_ADDRESS 4

// System Mode Calls
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.

// For monitoring / debugging, you have some options on the next few lines
SerialLogHandler logHandler(LOG_LEVEL_INFO, { // Logging level for non-application messages
	{ "app.pubq", LOG_LEVEL_TRACE },
	{ "app.seqfile", LOG_LEVEL_TRACE }
});

// Prototype functions
void publishStateTransition(void);
void stayAwakeTimerISR(void);
int setFrequency(String command);
int setLowPowerMode(String command);

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, LoRA_STATE, CONNECTING_STATE, DISCONNECTING_STATE, REPORTING_STATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "LoRA", "Connecting", "Disconnecting", "Reporting"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;
LocalTimeSchedule publishSchedule;							// These allow us to enable a schedule and to use local time
LocalTimeConvert conv;

// Battery conect information - https://docs.particle.io/reference/device-os/firmware/boron/#batterystate-
const char* batteryContext[7] = {"Unknown","Not Charging","Charging","Charged","Discharging","Fault","Diconnected"};

//Define pins for the RFM9x:
#define RFM95_CS D6
#define RFM95_RST A5
#define RFM95_INT D2
#define RF95_FREQ 915.0
const pin_t blueLED = D7;

// Singleton instance of the radio driver
// RH_RF95 driver(D6, D2);
RH_RF95 driver(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(driver, SERVER1_ADDRESS);

// Dont put this on the stack:
uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];

// Variables
const int wakeBoundary = 0*3600 + 5*60 + 0;         // Sets how often we connect to Particle
time_t lastConnectTime = 0;
uint16_t frequencyMinutes = 2;						// Default settings both these can be adjusted via Particle functions
uint16_t updatedFrequencyMinutes = 0;
bool lowPowerMode = false;						

void setup() 
{
	pinMode(blueLED,OUTPUT);

	Particle.variable("Report Frequency", frequencyMinutes);

	Particle.function("LowPower Mode",setLowPowerMode);
	Particle.function("Set Frequency",setFrequency);

	if (!manager.init()) Log.info("init failed");	// Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36

	driver.setFrequency(RF95_FREQ);					// Frequency is typically 868.0 or 915.0 in the Americas, or 433.0 in the EU

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
	driver.setTxPower(23, false);

	// Publish Queue Posix is used exclusively for sending webhooks and update alerts in order to conserve RAM and reduce writes / wear
  	PublishQueuePosix::instance().setup();          // Start the Publish Queie

	if (!Time.isValid()) {							// Until we add a Real Time Clock, I need to make sure the time is valid here.
		Particle.connect();
		if (waitFor(Particle.connected, 600000)) {	// Connect to Particle
			lastConnectTime = Time.now();			// Record the last connection time
			Particle.syncTime();					// Sync time
			waitUntil(Particle.syncTimeDone);		// Makr sure sync is complete
			if (lowPowerMode) state = DISCONNECTING_STATE;
		}
	}
	// Setup local time and set the publishing schedule
	LocalTime::instance().withConfig(LocalTimePosixTimezone("EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"));			// East coast of the US
	conv.withCurrentTime().convert();  				// Convert to local time for use later
    publishSchedule.withMinuteOfHour(frequencyMinutes, LocalTimeRange(LocalTimeHMS("06:00:00"), LocalTimeHMS("21:59:59")));	 // Publish every 15 minutes from 6am to 10pm

    Log.info("Startup complete at %s with battery %4.2f", conv.format(TIME_FORMAT_ISO8601_FULL).c_str(), System.batteryCharge());

	if (state == INITIALIZATION_STATE) state = IDLE_STATE;
}

void loop() {

	uint8_t len = sizeof(buf);
	uint8_t from;

	switch (state) {
		case IDLE_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			
			if (publishSchedule.isScheduledTime()) state = LoRA_STATE;		   // See Time section in setup for schedule

		} break;

		case SLEEPING_STATE:
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

			// if Sleep is enabled, it will occur here - will add a sleep enable particle function
			// Will also look at the low power state

			state = IDLE_STATE;

		break;

		case LoRA_STATE: {
			static time_t startLoRAWindow;

			if (state != oldState) {
				publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

				// Here is where we will power up the LoRA radio module

				startLoRAWindow = Time.now();
			} 

			if (manager.recvfromAck(buf, &len, &from))	{	// We have received a message
				digitalWrite(blueLED,HIGH);			// Signal we are using the radio
				buf[len] = 0;
				Log.info("Received from 0x%02x with rssi=%d msg = %d", from, driver.lastRssi(), buf[16]);

				if (updatedFrequencyMinutes) {
					frequencyMinutes = updatedFrequencyMinutes;
					updatedFrequencyMinutes = 0;
				}
	
				// Send a reply back to the originator client
				uint8_t data[7];
				data[0] = buf[16];								// Message number
				data[1] = ((uint8_t) ((Time.now()) >> 24));		// Fourth byte - current time
				data[2] = ((uint8_t) ((Time.now()) >> 16));		// Third byte
				data[3] = ((uint8_t) ((Time.now()) >> 8));		// Second byte
				data[4] = ((uint8_t) (Time.now()));				// First byte
				data[5] = highByte(frequencyMinutes * 60);		// Time till next report
				data[6] = lowByte(frequencyMinutes * 60);		

				Log.info("Sent response to client message = %d, time = %u, next report = %u", data[0], (data[1] << 24 | data[2] << 16 | data[3] <<8 | data[4]), ( data[5] << 8 | data[6]));

				if (manager.sendtoWait(data, sizeof(data), from) != RH_ROUTER_ERROR_NONE) Log.info("sendtoWait failed");
				digitalWrite(blueLED,LOW);			// Done with the radio
				// Here is where we will power down the LoRA radio module

				state = REPORTING_STATE;
			} 

			if ((Time.now() - startLoRAWindow) > 3600L) state = CONNECTING_STATE;	// This is a fail safe to make sure an off-line client won't prevent gatewat from checking in - and setting its clock

		} break;

		case REPORTING_STATE: {
			if (state != oldState) publishStateTransition();   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

		  	char data[256];                         // Store the date in this character array - not global
  			snprintf(data, sizeof(data), "{\"nodeid\":%u, \"hourly\":%u, \"daily\":%u,\"battery\":%d,\"key1\":\"%s\",\"temp\":%d, \"resets\":%d, \"alerts\":%d,\"rssi\":%d, \"msg\":%d,\"timestamp\":%lu000}",(buf[2] << 8 | buf[3]), (buf[5] << 8 | buf[6]), (buf[7] << 8 | buf[8]), buf[10], batteryContext[buf[11]], buf[9], buf[12], buf[13], ((buf[14] << 8 | buf[15]) - 65535), buf[16], Time.now());
  			PublishQueuePosix::instance().publish("Ubidots-LoRA-Hook-v1", data, PRIVATE | WITH_ACK);

			state = CONNECTING_STATE;
		} break;

		case CONNECTING_STATE:
			if (state != oldState) publishStateTransition();  // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			Particle.connect();
			if (waitFor(Particle.connected, 600000)) {
				lastConnectTime = Time.now();
				if (lowPowerMode) state = DISCONNECTING_STATE;
				else state = SLEEPING_STATE; 
			}
		break;

		case DISCONNECTING_STATE: {
			if (state != oldState) publishStateTransition();  // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			static time_t stayConnectedWindow = Time.now();

			if (Time.now() - stayConnectedWindow > 90) {
				Log.info("Disconnecting from Particle / Cellular");
				Particle.disconnect();
				delay(2000);
				Cellular.off();
				waitFor(Cellular.isOff,10000);
				state = SLEEPING_STATE;
			}
		} break;
	}

	PublishQueuePosix::instance().loop();           // Check to see if we need to tend to the message queue

}

/**
 * @brief Publishes a state transition to the Log Handler and to the Particle monitoring system.
 *
 * @details A good debugging tool.
 */
void publishStateTransition(void)
{
	char stateTransitionString[40];
	snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
	oldState = state;
	if (Particle.connected()) {
		static time_t lastPublish = 0;
		if (millis() - lastPublish > 1000) {
			lastPublish = millis();
			Particle.publish("State Transition",stateTransitionString, PRIVATE);
		}
	}
	Log.info(stateTransitionString);
}

/**
 * @brief Sets the how often the device will report data in minutes.
 *
 * @details Extracts the integer from the string passed in, and sets the closing time of the facility
 * based on this input. Fails if the input is invalid.
 *
 * @param command A string indicating the number of minutes between reporting events.  Note, this function
 * sets an interim value for reporting frequency which takes effect once sent to a new node.
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setFrequency(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 120)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  frequencyMinutes = tempTime;
  snprintf(data, sizeof(data), "Report frequency set to %i minutes",frequencyMinutes);
  Log.info(data);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}

/**
 * @brief Toggles the device into low power mode based on the input command.
 *
 * @details If the command is "1", sets the device into low power mode. If the command is "0",
 * sets the device into normal mode. Fails if neither of these are the inputs.
 *
 * @param command A string indicating whether to set the device into low power mode or into normal mode.
 * Low power mode will enable sleep and only listen at specific times for reports.  Non-low power mode is always connected
 * and is always listening for reports.  
 *
 * @return 1 if able to successfully take action, 0 if invalid command
 */
int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    lowPowerMode = true;
    Log.info("Set Low Power Mode");
    if (Particle.connected()) Particle.publish("Mode",(lowPowerMode) ? "Low Power" :"Not Low Power", PRIVATE);
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    lowPowerMode = false;
    Log.info("Cleared Low Power Mode");
    if (Particle.connected()) Particle.publish("Mode",(lowPowerMode) ? "Low Power" :"Not Low Power", PRIVATE);
  }
  return 1;
}


