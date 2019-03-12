/*
  Code to read and write data using iBus protocol over one serial channel on Arduino. As iBus is one wire bidirectional, this could rx and tx to one iBus
  connection by placing a resistor between the rx and tx pins. Or, it could rx from one iBus connection and tx to another.
  Note: On the ESP32 devkit C, Serial is TX and RX but is used by the USB serial monitor. Instead Serial1 and 2 can be used by specifiying i in MySerial(i). 
  Pin 10 isn't allowed for some reason so if using Serial1 the default is RX=9, TX=10 and the TX pin must be changed.
  iBus reading adapted from: https://gitlab.com/timwilkinson/FlySkyIBus
  iBus writing adapted from: https://github-mirror.open.netease.com/wdcossey/ppm-to-ibus-serial
*/
#define ROSSERIAL_ARDUINO_TCP_WIFI // Required to make the Wifi rosserial work for an ESP32
//#define ESP32_USE_USB // To use USB comms, use this definition

#include "FlySkyiBusCombined.h"
#include <HardwareSerial.h>
#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "PID.h"

HardwareSerial MySerial(2);

#define IBUS_MAXCHANNELS 14  // iBus has a maximum of 14 channels
#define IBUS_LOWER_LIMIT 1000
#define IBUS_UPPER_LIMIT 2000
#define MIN_THRUST 1100
#define PI 3.14159265

uint16_t *channel_data= new uint16_t[IBUS_MAXCHANNELS];
uint8_t channel_count = 0;
uint8_t automated = 0;
uint8_t armed = 0;
uint8_t out_of_bounds[6] = {0};
uint8_t remote_lost = 0;
unsigned long start_time;

HardwareSerial& iBus_serial = MySerial; // Choose which serial port the iBus will be communicating over
PID positionController;

unsigned long old_time = 0;

float pidParams [3]; // = [0.0, 0.0, 0.0]

void set_safe_outputs (); // function prototype
void set_outputs (uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t arming_sw, uint16_t automated_sw);

/***************************************************************************************/
/**************************Wifi stuff***************************************************/
const char* ssid = "test"; // Wifi network name
const char* password =  "abcdefg1"; // Wifi password
IPAddress serverIp(10,42,0,1);      // rosserial socket ROSCORE SERVER IP address
const uint16_t serverPort = 11411; // rosserial socket server port - NOT roscore socket!

void connectToNetwork() {
  WiFi.begin(ssid, password);

  Serial.println("Connecting...");

  while (WiFi.status() != WL_CONNECTED) { // Wait until Wifi is connected
    Serial.println("Still Connecting...");
    delay(1000);
  }

  Serial.println("Connected!");
}
/***************************************************************************************/

/***************************************************************************************/
/**************************ROS Stuff****************************************************/
// Callback for subscriber, receives a 3 element Float32 array
void sub_cb (const std_msgs::Float32MultiArray& data_rec) {
  Serial.println();
  Serial.println("Data from ROS: ");
  
  for (int i=0; i<3; i++) {
    Serial.print (data_rec.data[i]);
    Serial.print ("\t");
    pidParams[i] = data_rec.data[i];
  }
  Serial.println();
  Serial.println();
}

std_msgs::Float32 inc_data; 

ros::Publisher data_pub("ESP_Data", &inc_data);

ros::Subscriber<std_msgs::Float32MultiArray> data_sub("Pos_Data", &sub_cb); 

ros::NodeHandle nh;
/***************************************************************************************/

/***************************************************************************************/
/**************************IMU Stuff****************************************************/
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

unsigned long IMU_Read_Time;
imu::Vector<3> euler;

/***************************************************************************************/


/***************************************************************************************/
/******************************Setup****************************************************/
void setup() {
  
  pinMode (13, OUTPUT); // On-board LED
  Serial.begin(115200); // Serial Monitor
  connectToNetwork(); // Connect to the Wifi - uncomment for Wifi comms, comment for USB

  iBus.begin(iBus_serial); // Open the iBus serial connection

  set_safe_outputs(); // Set all outputs to known safe values
  channel_count = 6; // Must specify how many pieces of data we're explicitly setting, any others will be set to default value

  bno.begin();
  bno.setExtCrystalUse(true);;

  // Initalise node, advertise the pub and subscribe the sub
//  nh.initNode();
//  nh.getHardware()->setConnection(serverIp, serverPort); // Set the rosserial socket server info - uncomment for Wifi comms, comment for USB
//  nh.advertise(data_pub);
//  nh.subscribe(data_sub);
//  
//  inc_data.data = 0.0;

  // Initialise PID Controllers
  // Initialise with (kp,ki,kd,Ie,D)
  positionController.set_x_constants(0,0,0,0,0);
  positionController.set_y_constants(0,0,0,0,0);
  positionController.set_z_constants(0,0,0,0,0);
  
  positionController.set_xspeed_constant(0);
  positionController.set_yspeed_constant(0);
  positionController.set_zspeed_constant(0);

  old_time = micros();
  
  start_time = millis();
  IMU_Read_Time = start_time - BNO055_SAMPLERATE_DELAY_MS;
}
/***************************************************************************************/


/***************************************************************************************/
/**************************************Main Loop****************************************/
void loop() {
  iBus.read_loop(); // Request one frame to be read

/***************************************************************************************/
/**************************Safety checks and automation check***************************/
  remote_lost = 0;
  // Check if any channels are out of bounds, this suggests that the controller has been lost
  for (uint8_t i = 0; i < channel_count; i++) {
    if (iBus.readChannel(i) < IBUS_LOWER_LIMIT || iBus.readChannel(i) > IBUS_UPPER_LIMIT) {
      out_of_bounds[i] = 1;
      remote_lost = 1;
    }
    else {
      out_of_bounds[i] = 0;
    }
  }

  // If remote is connected, then check arming (Channel 5 - SWC)
  if (!remote_lost) {
    if (iBus.readChannel(4) > 1100) {
      armed = 1;
      Serial.println("Armed!");
    }
    else {
      armed = 0;
      Serial.println("Unarmed!");
    }
  }

  // If remote is connected, Channel 6 (SWB) is down, and remote is armed then set automated mode, otherwise fly in manual mode
  if (iBus.readChannel(5) > 1500 && !remote_lost && armed) {
    // If we're entering automated mode, take time.
    if (!automated) {
      start_time = millis(); 
    }
    automated = 1;
  }
  else {
    automated = 0;
  }
/***************************************************************************************/


/***************************************************************************************/
/************************************Read IMU Data**************************************/

  if ((millis() - IMU_Read_Time) >= BNO055_SAMPLERATE_DELAY_MS) {
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  }
//  Serial.print ("X = ");
//  Serial.println (euler.x());
/***************************************************************************************/

/***************************************************************************************/
/***********************************Receive ROS Data************************************/

//  inc_data.data = (float) euler.x();
//  data_pub.publish(&inc_data); // Set the data to be sent back to ROS.
//
//  float oldParams[3];
//  for (int i=0; i<3; i++) {
//    oldParams[i] = pidParams[i];
//  }
//  
//  nh.spinOnce(); // Send data and call subscriber callback to receive any data.

//  if (pidParams != oldParams) {
//    positionController.set_psi_constants(pidParams[0], pidParams[1], pidParams[2], 0, 0);
//  }

// can set desired positions received from ROS

/***************************************************************************************/

  // If we're in manual mode, pass data read from receiver to the FCU
  if (!automated) {
    for (uint8_t i = 0; i < channel_count; i++) {
      uint16_t read_data = iBus.readChannel(i); // Read one frame
      channel_data[i] = read_data;
      switch (i) {
        case 0:
          Serial.print("Roll=");
          break;
        case 1:
          Serial.print("Pitch=");
          break;
        case 2:
          Serial.print("Throttle=");
          break;
        case 3:
          Serial.print("Yaw=");
          break;
        case 4:
          Serial.print("SWC=");
          break;
        case 5:
          Serial.print("SWB=");
          break;
        default:
          break;
      }
      Serial.print(read_data);
      Serial.print(", ");
    }
    Serial.println();
  }
  // If in automated mode, calculate our own channel values to send to FCU
  else if(automated) {
    
    unsigned long current_time = micros();
    unsigned long time_diff = current_time - old_time;

    // for testing, replace with readings
    float current_x = 0.0;
    float current_y = 0.0;
    float current_z = 0.0;
    float current_xspeed = 0.0;
    float current_yspeed = 0.0;
    float current_zspeed = 0.0;
    float current_pitch = 0.0;
    float current_roll = 0.0;

    // format (x, y, z, yaw)
    positionController.set_desired_values(0.0, 0.0, 0.0, 0.0);
    float desired_yaw = 0.0;
    
    // position controller inputs (current_pos, desired_pos, time_diff)
    float xOutput = positionController.compute_x_PID(current_x, time_diff);
    float yOutput = positionController.compute_y_PID(current_y, time_diff);
    float zOutput = positionController.compute_z_PID(current_z, time_diff);

    // speed controller inputs (current_speed, desired_speed, time_diff)
    float xSpeedOutput = positionController.compute_xspeed_PID(current_xspeed, xOutput, time_diff);
    float ySpeedOutput = positionController.compute_yspeed_PID(current_yspeed, yOutput, time_diff);
    float zSpeedOutput = positionController.compute_zspeed_PID(current_zspeed, zOutput, time_diff);

    // orientation conversion input (current angle) (desired angles taken from speed controllers already)
    float pitchOutput = positionController.compute_desired_pitch(current_pitch);
    float rollOutput = positionController.compute_desired_roll(current_roll);

    set_outputs( rollOutput, pitchOutput, zOutput, desired_yaw, 1500, 2000 ); // last 2 values are armed and automated controls
    
    old_time = current_time;
  }
  
  // If we suspect the controller is lost, set all outputs to known safe values
  if (remote_lost) {
    set_safe_outputs();
    Serial.println("Entering Safe State");
  }
  
  iBus.write_one_frame(channel_data, channel_count, iBus_serial); // Send one frame to be written
  
  delay(1); // Set a delay between sending frames
}


void set_safe_outputs () {
  channel_data[0] = 1500; // Roll control - default = 1500
  channel_data[1] = 1500; // Pitch control - default = 1500
  channel_data[2] = 1000; // Throttle control - default = 1000
  channel_data[3] = 1500; // Rudder control - default = 1500
  channel_data[4] = 1000; // Arming control - default = 1000, armed = 1500
  channel_data[5] = 1000; // Selectable control - default = 1000, second mode at 2000
}

void set_outputs (uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t arming_sw, uint16_t automated_sw) {
  channel_data[0] = roll;         // Roll control - default = 1500
  channel_data[1] = pitch;        // Pitch control - default = 1500
  channel_data[2] = throttle;     // Throttle control - default = 1000
  channel_data[3] = yaw;          // Rudder control - default = 1500
  channel_data[4] = arming_sw;    // Arming control - default = 1000, armed = 1500
  channel_data[5] = automated_sw; // Selectable control - default = 1000, second mode at 2000
}

//void planned_path () {
//  unsigned long current_time = millis();
//  unsigned long time = current_time - start_time;
//  if (time < 4000) {
//    set_safe_outputs();
//  }
//  else if (time < 8000) {
//    set_outputs (1500, 1500, 1200, 1500, 1500, 2000);
//  }
//  else if (time < 12000) {
//    set_outputs (1450, 1500, 1200, 1500, 1500, 2000);
//  }
//  else if (time < 16000) {
//    set_outputs (1500, 1500, 1200, 1500, 1500, 2000);
//  }
//  else if (time < 20000) {
//    set_outputs (1550, 1500, 1200, 1500, 1500, 2000);
//  }
//  else if (time < 24000) {
//    set_outputs (1500, 1500, 1200, 1500, 1500, 2000);
//  }
//  else {
//    set_safe_outputs();
//  }
//
//  Serial.print("Time is: ");
//  Serial.println(time);
//}
