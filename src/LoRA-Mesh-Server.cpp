/*
 * Project LoRA-Mesh-Server
 * Description: This device will listen for data from client devices and forward the data to Particle via webhook
 * Author: Chip McClelland
 * Date: 7-28-22
 */

// v1 - Initial attempt - no sleeping but disconnects when not sending data

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

const uint8_t firmware = 1;

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

// State Machine Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, CONNECTING_STATE, REPORTING_STATE, RESP_WAIT_STATE, FIRMWARE_UPDATE};
char stateNames[9][16] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Connecting", "Reporting", "Response Wait", "Firmware Update"};
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;
Timer stayAwakeTimer(90000, stayAwakeTimerISR, true);      // This is how we will ensure the device stays connected for 90 seconds
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
volatile bool disconnectFlag = false;



void setup() 
{
	pinMode(blueLED,OUTPUT);

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
			disconnectFlag = true;					// Raise the disconnect flag - will get serviced in main loop
		}
	}
	// Setup local time and set the publishing schedule
	LocalTime::instance().withConfig(LocalTimePosixTimezone("EST5EDT,M3.2.0/2:00:00,M11.1.0/2:00:00"));			// East coast of the US
	conv.withCurrentTime().convert();  				// Convert to local time for use later
    publishSchedule.withMinuteOfHour(15, LocalTimeRange(LocalTimeHMS("06:00:00"), LocalTimeHMS("21:59:59")));	 // Publish every 15 minutes from 6am to 10pm

    Log.info("Startup complete at %s with battery %4.2f", conv.format(TIME_FORMAT_ISO8601_FULL).c_str(), System.batteryCharge());

	if (state == INITIALIZATION_STATE) state = IDLE_STATE;
}

void loop() {

	uint8_t len = sizeof(buf);
	uint8_t from;

	switch (state) {
		case IDLE_STATE: {
			if (state != oldState) publishStateTransition();                   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			
			if (manager.recvfromAck(buf, &len, &from))	{	// We have received a message
				digitalWrite(blueLED,HIGH);			// Signal we are using the radio
				buf[len] = 0;
				Log.info("Received from 0x%02x with rssi=%d msg = %d", from, driver.lastRssi(), buf[16]);
		
				// Send a reply back to the originator client
				uint8_t data[1];
				data[0] = buf[16];

				if (manager.sendtoWait(data, sizeof(data), from) != RH_ROUTER_ERROR_NONE) Log.info("sendtoWait failed");
				digitalWrite(blueLED,LOW);			// Done with the radio

				state = REPORTING_STATE;
			} 

			if (publishSchedule.isScheduledTime()) state = CONNECTING_STATE;		// See Time section in setup for schedule

		} break;

		case REPORTING_STATE: {
			if (state != oldState) publishStateTransition();   // We will apply the back-offs before sending to ERROR state - so if we are here we will take action

		  	char data[256];                         // Store the date in this character array - not global
  			snprintf(data, sizeof(data), "{\"hourly\":%u, \"daily\":%u,\"battery\":%d,\"key1\":\"%s\",\"temp\":%d, \"resets\":%d, \"alerts\":%d,\"rssi\":%d, \"msg\":%d,\"timestamp\":%lu000}",((buf[5] << 8) | buf[6]), (buf[7] << 8 | buf[8]), buf[10], batteryContext[buf[11]], buf[9], buf[12], buf[13], ((buf[14] << 8 | buf[15]) - 65535), buf[16], Time.now());
  			PublishQueuePosix::instance().publish("Ubidots-LoRA-Hook-v1", data, PRIVATE | WITH_ACK);

			state = IDLE_STATE;
		} break;

		case CONNECTING_STATE:
			if (state != oldState) publishStateTransition();  // We will apply the back-offs before sending to ERROR state - so if we are here we will take action
			Particle.connect();
			if (waitFor(Particle.connected, 600000)) {
				lastConnectTime = Time.now();
				stayAwakeTimer.reset();				// Start a 90 second clock to disconnect
				state = IDLE_STATE;
			}
		break;
	}

	PublishQueuePosix::instance().loop();           // Check to see if we need to tend to the message queue

	if (disconnectFlag) {
		Log.info("Disconnecting from Particle / Cellular");
		Particle.disconnect();
		delay(2000);
		Cellular.off();
		waitFor(Cellular.isOff,10000);
		disconnectFlag = false;
	}
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
 * @brief Interrupt Service Routine for the stay awake timer
 * 
 */
void stayAwakeTimerISR() {
	disconnectFlag = true;
}
