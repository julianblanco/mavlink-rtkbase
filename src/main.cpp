/*
  Send UBX binary commands to enable RTCM sentences on Ublox ZED-F9P module,
  then broadcadst corrections in mavlink format and send over xbee
  
 
  Date: Dec 2nd, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does all steps to configure and enable a ZED-F9P as a base station:
    Begin Survey-In
    Once we've achieved 2m accuracy and 300s have passed, survey is complete
    Enable six RTCM messages
    Begin outputting RTCM bytes

  Hardware Connections:
  Plug a Qwiic cable into the GPS and connect to i2c bus on microcontroller and an xbee on serial 1
   Open the serial monitor at 115200 baud to see the output
*/
#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GPS
#include "mavlink.h"
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS

SFE_UBLOX_GPS myGPS;
#define XBEE Serial1

float mindistance = 0.5;
float surveytime = 300; // seconds
void enableRTK();
// start a survey with min time and distance
void surveyIn(float min_survey_time_local, float min_distance_local);

mavlink_system_t mavlink_system;
float position[6];
bool ManualMode = false;
int32_t Latitude;
int32_t Longitude;
int32_t Altitude;
int16_t Velocity;
int64_t Microseconds;
mavlink_message_t global_position_intMsg;
mavlink_message_t heartbeatMsg;
uint8_t system_type = MAV_TYPE_GROUND_ROVER;    // MAV_TYPE_HELICOPTER;//MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC; // MAV_AUTOPILOT_ARDUPILOTMEGA
uint8_t system_mode = MAV_MODE_MANUAL_DISARMED; // MAV_MODE_MANUAL_ARMED; //MAV_MODE_GUIDED_ARMED //MAV_MODE_GUIDED_DISARMED
uint8_t system_state = MAV_STATE_ACTIVE;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint8_t buf0[MAVLINK_MAX_PACKET_LEN];

mavlink_manual_control_t manual_control;
mavlink_set_mode_t mode;
mavlink_heartbeat_t heartbeat;

void setup()
{
  // enable debug printing to computer
  Serial.begin(115200);
  Serial.println("Ublox Base station example");

  // start serial comms to xbee
  XBEE.begin(115200);

  // start i2c bus to communticate with rtk reciever
  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz

  if (myGPS.begin() == false) // Connect to the Ublox module using Wire port
  {
    Serial.println("Ublox GPS not detected at default I2C address. Please check wiring. Freezing.");
    while (1)
      ;
  }

  // enable rtk basestation mode
  enableRTK();
  // start a survey with min time and distance
  surveyIn(surveytime, mindistance);
  Serial.println("Survey valid!");
  Serial.println("Base survey complete! RTCM now broadcasting.");
  myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); // Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)
}

void loop()
{
  myGPS.checkUblox(); // See if new data is available. Process bytes as they come in.

  delay(250); // Don't pound too hard on the I2C bus
}

// This function gets called from the SparkFun Ublox Arduino Library.
// As each RTCM byte comes in you can specify what to do with it
// Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GPS::processRTCM(uint8_t incoming)
{

  float currentTime = micros();
  memset(buf, 0xFF, sizeof(buf));
  mavlink_system.sysid = 1;
  mavlink_system.compid = MAV_COMP_ID_AUTOPILOT1;
  heartbeat.system_status = MAV_STATE_ACTIVE;
  heartbeat.custom_mode = 65536;
  heartbeat.base_mode = 81;

  // decode the rtcm bytes
  // XBEE.write(incoming);

  // Push the RTCM data to Serial1
  //  // Pack the message
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &heartbeatMsg, 2, 12, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
  // // Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &heartbeatMsg);
  // Serial2.println("Heartbeat");
  // //Write Message
  XBEE.write(buf, len);
  memset(buf, 0xFF, sizeof(buf));

  mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &global_position_intMsg, currentTime, 3, 392919390, -772862310, 10, 0xFFFF, 0xFFFF, Velocity, 0xFFFF, 7, 0, 0, 0, 0, 0); // fix_type must be 3 for some odd reason
  // /// Copy the message to send buffer
  len = mavlink_msg_to_send_buffer(buf, &global_position_intMsg);
  // Write Message
  XBEE.write(buf, len);
  memset(buf, 0xFF, sizeof(buf));

  // Pretty-print the HEX values to Serial
  if (myGPS.rtcmFrameCounter % 16 == 0)
    Serial.println();
  Serial.print(" ");
  if (incoming < 0x10)
    Serial.print("0");
  Serial.print(incoming, HEX);
}

void enableRTK()
{
  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  // myGPS.factoryDefault(); delay(5000);

  myGPS.setI2COutput(COM_TYPE_UBX);                 // Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save the communications port settings to flash and BBR

  while (Serial.available())
    Serial.read(); // Clear any latent chars in serial buffer
  Serial.println("Press any key to send commands to begin Survey-In");
  while (Serial.available() == 0)
    ; // Wait for user to press a key

  boolean response = true;
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); // Enable message 1005 to output through I2C port, message every second
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  response &= myGPS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10); // Enable message every 10 seconds

  // Use COM_PORT_UART1 for the above six messages to direct RTCM messages out UART1
  // COM_PORT_UART2, COM_PORT_USB, COM_PORT_SPI are also available
  // For example: response &= myGPS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 10);

  if (response == true)
  {
    Serial.println("RTCM messages enabled");
  }
  else
  {
    Serial.println("RTCM failed to enable. Are you sure you have an ZED-F9P?");
    while (1)
      ; // Freeze
  }
}

void surveyIn(float min_survey_time_local, float min_distance_local)
{

  // Check if Survey is in Progress before initiating one
  int response = myGPS.getSurveyStatus(2000); // Query module for SVIN status with 2000ms timeout (request can take a long time)
  if (response == false)
  {
    Serial.println("Failed to get Survey In status");
    while (1)
      ; // Freeze
  }

  if (myGPS.svin.active == true)
  {
    Serial.print("Survey already in progress.");
  }
  else
  {
    // Start survey
    // The ZED-F9P is slightly different than the NEO-M8P. See the Integration manual 3.5.8 for more info.
    // response = myGPS.enableSurveyMode(300, 2.000); //Enable Survey in on NEO-M8P, 300 seconds, 2.0m
    response = myGPS.enableSurveyMode(min_survey_time_local, min_distance_local); // Enable Survey in, 60 seconds, 5.0m
    if (response == false)
    {
      Serial.println("Survey start failed");
      while (1)
        ;
    }
    Serial.println("Survey started. This will run until 60s has passed and less than 5m accuracy is achieved.");
  }

  while (Serial.available())
    Serial.read(); // Clear buffer

  // Begin waiting for survey to complete
  while (myGPS.svin.valid == false)
  {
    if (Serial.available())
    {
      byte incoming = Serial.read();
      if (incoming == 'x')
      {
        // Stop survey mode
        response = myGPS.disableSurveyMode(); // Disable survey
        Serial.println("Survey stopped");
        break;
      }
    }

    response = myGPS.getSurveyStatus(2000); // Query module for SVIN status with 2000ms timeout (req can take a long time)
    if (response == true)
    {
      Serial.print("Press x to end survey - ");
      Serial.print("Time elapsed: ");
      Serial.print((String)myGPS.svin.observationTime);

      Serial.print(" Accuracy: ");
      Serial.print((String)myGPS.svin.meanAccuracy);
      Serial.println();
    }
    else
    {
      Serial.println("SVIN request failed");
    }

    delay(1000);
  }
}