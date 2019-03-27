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
#include "PIDOO.h"


#define IBUS_MAXCHANNELS  14    // iBus has a maximum of 14 channels
#define IBUS_LOWER_LIMIT  1000
#define IBUS_UPPER_LIMIT  2000
#define MIN_THRUST        1000
#define MAX_THRUST        1100
#define IMU_SAMPLE_TIME   0.01  // Set the delay between fresh IMU samples (in S)
#define CF_SAMPLE_TIME    0.1   // Set the delay between fresh Crazyflie samples (in S)
#define LOOP_TIME         10    // Set the main loop time (in mS)
#define MAX_DRONE_SPEED   0.75  // If any speeds are above threshold, kill the drone for safety

PIDOO xPosPID;
PIDOO yPosPID;
PIDOO zPosPID;
PIDOO xSpeedPID;
PIDOO ySpeedPID;
PIDOO zSpeedPID;
PIDOO yawPosPID;

HardwareSerial MySerial(2);
HardwareSerial& iBus_serial = MySerial; // Choose which serial port the iBus will be communicating over

static uint16_t* set_safe_outputs (uint16_t *channel_data_in); // function prototype
static uint16_t* set_outputs (uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t arming_sw, uint16_t automated_sw, uint16_t *channel_data_in);
uint16_t saturateiBusCommand (uint16_t val);

/***************************************************************************************/
/**************************Wifi stuff***************************************************/
void connectToNetwork() {
  const char* ssid = "test"; // Wifi network name
  const char* password =  "abcdefg1"; // Wifi password
  
  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) { // Wait until Wifi is connected
    Serial.println("Still Connecting...");
    delay(1000);
  }

  Serial.println("Connected to WiFi!");
}
/***************************************************************************************/

/***************************************************************************************/
/**************************ROS Stuff****************************************************/
static float CF_Vel [3];
static float CF_Yaw;
static float tuning_data [3];
static bool tuning_params_changed = 0;

// Callbacks for subscribers
void sub_cb_loco (const std_msgs::Float32MultiArray& data_rec) {
  CF_Vel[0] = data_rec.data[0];
  CF_Vel[1] = data_rec.data[1];
  CF_Vel[2] = data_rec.data[2];
  CF_Yaw = data_rec.data[3];
}

void sub_cb_kptuning (const std_msgs::Float32& data_rec) {
  tuning_data[0] = data_rec.data;
  tuning_params_changed = 1;
}
void sub_cb_kituning (const std_msgs::Float32& data_rec) {
  tuning_data[1] = data_rec.data;
  tuning_params_changed = 1;
}
void sub_cb_kdtuning (const std_msgs::Float32& data_rec) {
  tuning_data[2] = data_rec.data;
  tuning_params_changed = 1;
}

ros::Subscriber<std_msgs::Float32MultiArray> data_sub_loco("loco_data", &sub_cb_loco); 
ros::Subscriber<std_msgs::Float32> data_sub_kptuning("tuning_data_kp", &sub_cb_kptuning); 
ros::Subscriber<std_msgs::Float32> data_sub_kituning("tuning_data_ki", &sub_cb_kituning); 
ros::Subscriber<std_msgs::Float32> data_sub_kdtuning("tuning_data_kd", &sub_cb_kdtuning); 


static std_msgs::Float32 testRunning;
static std_msgs::Float32 xvel_data; 
static std_msgs::Float32 yvel_data; 
static std_msgs::Float32 xspeedpid_data; 

ros::Publisher data_pub_testing("Testing_Data", &testRunning);
ros::Publisher data_pub_xvel("ESP_Data_xvel", &xvel_data);
ros::Publisher data_pub_yvel("ESP_Data_yvel", &yvel_data);
ros::Publisher data_pub_PID("PID_Output", &xspeedpid_data);


ros::NodeHandle nh;
/***************************************************************************************/

/***************************************************************************************/
/**************************IMU Stuff****************************************************/
Adafruit_BNO055 bno = Adafruit_BNO055();

static unsigned long IMU_Read_Time;
static float desired_xspeed = 0;

void setCalibrationOffsets() {
  adafruit_bno055_offsets_t calibrationData;
  
  // Restore calibration data
  calibrationData.accel_offset_x = 18;
  calibrationData.accel_offset_y = 53;
  calibrationData.accel_offset_z = -2;
  
  calibrationData.mag_offset_x = 1404;
  calibrationData.mag_offset_y = 140;
  calibrationData.mag_offset_z = -2021;
  
  calibrationData.gyro_offset_x = 0;
  calibrationData.gyro_offset_y = -2;
  calibrationData.gyro_offset_z = -2;
  
  calibrationData.accel_radius = 1000;
  calibrationData.mag_radius = 627;

  bno.setSensorOffsets(calibrationData);  
}

//Would like to declare prototype here and define below but isn't working?
class EMA {
public:
  EMA(float a): alpha(a), s_x(0), s_y(0), s_z(0), olds_x(0), olds_y(0), olds_z(0) {} 
  
  float getSx(){
    return s_x;
  }
  float getSy(){
    return s_y;
  }
  float getSz(){
    return s_z;
  }
  float getOldSx(){
    return olds_x;
  }
  float getOldSy(){
    return olds_y;
  }
  float getOldSz(){
    return olds_z;
  }

  void setSx (float accel_x) {
    s_x = alpha*accel_x + (1 - alpha)*s_x; // Averaged x acceleration
  }
  void setSy (float accel_y) {
    s_y = alpha*accel_y + (1 - alpha)*s_y; // Averaged y acceleration
  }
  void setSz (float accel_z) {
    s_z = alpha*accel_z + (1 - alpha)*s_z; // Averaged z acceleration
  }

  void updateOldValues () {
    olds_x = s_x;
    olds_y = s_y;
    olds_z = s_z;
  }

private:
  float alpha;
  float s_x;
  float s_y;
  float s_z;
  float olds_x;
  float olds_y;
  float olds_z;
};
/***************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------------------------------------*/
/***************************************************************************************/
/******************************Setup****************************************************/
void setup() {
  
  Serial.begin(115200); // Serial Monitor
  connectToNetwork(); // Connect to the Wifi - uncomment for Wifi comms, comment for USB

  iBus.begin(iBus_serial); // Open the iBus serial connection

//  channel_data = set_safe_outputs(channel_data); // Set all outputs to known safe values
//  channel_count = 6; // Must specify how many pieces of data we're explicitly setting, any others will be set to default value

  bno.begin();
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  bno.setExtCrystalUse(true);
  setCalibrationOffsets();
  // Set IMU mode to nine degrees of freedom (found in BN055 datasheet)
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);

//  velocities[0] = 0.0; // x vel
//  velocities[1] = 0.0; // y vel
//  velocities[2] = 0.0; // z vel
  CF_Vel[0] = 0.0;
  CF_Vel[1] = 0.0;
  CF_Vel[2] = 0.0;
  CF_Yaw = 0.0;

  Serial.println("Connecting to ROS...");
  // Initalise node, advertise the pub and subscribe the sub
  IPAddress serverIp(10,42,0,1);      // rosserial socket ROSCORE SERVER IP address 
  uint16_t serverPort = 11411; // rosserial socket server port - NOT roscore socket!
  nh.initNode();
  nh.getHardware()->setConnection(serverIp, serverPort); // Set the rosserial socket server info - uncomment for Wifi comms, comment for USB
  nh.advertise(data_pub_testing);
  nh.advertise(data_pub_xvel);
  nh.advertise(data_pub_yvel);
  nh.advertise(data_pub_PID);
  nh.subscribe(data_sub_loco);
  nh.subscribe(data_sub_kptuning);
  nh.subscribe(data_sub_kituning);
  nh.subscribe(data_sub_kdtuning);
  Serial.println("ROS finished setup");
  
  testRunning.data = 0.0;
  xvel_data.data = 0.0;
  yvel_data.data = 0.0;
  xspeedpid_data.data = 0.0;

  // Initialise PID Controllers
  // Initialise with (kp,ki,kd,Ie,D)
//
  tuning_data[0] = 0.11; tuning_data[1] = 0; tuning_data[2] = 0.1;

  xPosPID.set_PID_constants(0,0,0,0,0);
  xPosPID.set_desired_value(0.0);
  yPosPID.set_PID_constants(0,0,0,0,0);
  yPosPID.set_desired_value(0.0);
  zPosPID.set_PID_constants(0,0,0,0,0);
  zPosPID.set_desired_value(0.0);
  xSpeedPID.set_PID_constants(tuning_data[0],tuning_data[1],tuning_data[2],0,0);
  xSpeedPID.set_desired_value(0.0);
  ySpeedPID.set_PID_constants(tuning_data[0],tuning_data[1],tuning_data[2],0,0);
  ySpeedPID.set_desired_value(0.0);
  zSpeedPID.set_PID_constants(0,0,0,0,0);
  zSpeedPID.set_desired_value(0.0);
  yawPosPID.set_PID_constants(0,0,0,0,0);
  yawPosPID.set_desired_value(0.0);  

  Serial.println("Finished Setup");
}
/***************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------------------------------------*/
/***************************************************************************************/
/**************************************Main Loop****************************************/
void loop() {
  static unsigned long start_of_loop = micros();
  static unsigned long start_time = micros();
  static unsigned long old_PID_time = micros();
  static unsigned long old_loop_time = millis();
  static unsigned long IMU_Read_Time = start_time - IMU_SAMPLE_TIME;
  static unsigned long CF_Read_Time = start_time - CF_SAMPLE_TIME;
  static float time_diff = 0.0;

  static float old_z_position = 0;
  static float velocities [3] = {0.0, 0.0, 0.0}; // holds [x vel, y vel, z vel]
  static float yaw = 0.0;
  
  static uint8_t channel_count = 6;
  static uint16_t *channel_data = new uint16_t[channel_count];

  static bool automated = 0;
  static bool armed = 0;
  static bool remote_lost = 0;
  static bool displayStatus = 1; // 0 for no serial output about controller values, 1 for information

  remote_lost = 0;

  // Request one frame to be read
  if (iBus.read_loop() == 1) {
    if (displayStatus) Serial.println("lost"); 
    remote_lost = 1;
  }
  
/***************************************************************************************/
/**************************Safety checks and automation check***************************/
  // Check if any channels are out of bounds, this suggests that the controller has been lost
  for (uint8_t i = 0; i < channel_count; i++) {
    if (iBus.readChannel(i) < IBUS_LOWER_LIMIT || iBus.readChannel(i) > IBUS_UPPER_LIMIT) {
      remote_lost = 1;
    }
  }

  // If remote is connected, then check arming (Channel 5 - SWC)
  if (!remote_lost) {
    if (iBus.readChannel(4) > 1100) {
      armed = 1;
      if (displayStatus)  Serial.println("Armed!");
    }
    else {
      armed = 0;
      if (displayStatus) Serial.println("Unarmed!");
    }
  }

  // If remote is connected, Channel 6 (SWB) is down, and remote is armed then set automated mode, otherwise fly in manual mode
  if (iBus.readChannel(5) > 1500 && !remote_lost && armed) {
    // If we're entering automated mode, take time.
    if (!automated) {
      start_time = micros(); 
    }
    automated = 1;
  }
  else {
    automated = 0;
  }
/***************************************************************************************/

/***************************************************************************************/
/************************************Read CF Data***************************************/
// Correct IMU velocities using readings from the crazyflie
// NOTE: maybe use flags to see if data has changed?
time_diff = (micros() - CF_Read_Time)/1000000.0;
if ((time_diff > CF_SAMPLE_TIME) && (old_z_position != CF_Vel[2])) {
  velocities[0] = CF_Vel[0];
  velocities[1] = CF_Vel[1];
  velocities[2] = (CF_Vel[2] - old_z_position)/time_diff; // CF_Vel[2] coming in is z position not velocity, until this is fixed, differentiate position
  old_z_position = CF_Vel[2];
  yaw = CF_Yaw;
  CF_Read_Time = micros();
}
/***************************************************************************************/
/************************************Read IMU Data**************************************/

  if (iBus.readChannel(4) > 1600) {
    desired_xspeed = 0.0;
    testRunning.data = desired_xspeed;
  }
  else {
    desired_xspeed = 0;
    testRunning.data = 0;
  }

  static EMA myEMA(0.8);
  static imu::Vector<3> accels;
  static imu::Vector<3> orient;

  // NOTE: try with no delay checking? Will IMU return any wrong readings? Is this causing issues?
  time_diff = (micros() - IMU_Read_Time)/1000000.0;
  if(time_diff > IMU_SAMPLE_TIME) {
    // Options for reading are:
    // VECTOR_MAGNETOMETER (uT), VECTOR_GYROSCOPE (rps), VECTOR_EULER (deg)
    // VECTOR_ACCELEROMETER (m/s^2), VECTOR_LINEARACCEL (m/s^2), VECTOR_GRAVITY (m/s^2)
    // NOTE: FUSE ORIENTATION WITH YAW FROM CF
    orient = bno.getVector(Adafruit_BNO055::VECTOR_EULER); // Get IMU's Euler orientation 
    accels = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // Get linear accelerations with gravity accounted for
    
    // Implement moving average filter
    myEMA.setSx(accels.x());
    myEMA.setSy(accels.y());
    myEMA.setSz(accels.z());

//    testRunning.data = myEMA.getSx()/10.0;

    // NOTE: SORT OUT AUTOMATIC CALIBRATION OF IMU
    // Integrate using filtered accelerations and trapezium integration
//    velocities[0] = velocities[0] + (myEMA.getSx() + myEMA.getOldSx())*time_diff/2;
//    velocities[1] = velocities[1] + (myEMA.getSy() + myEMA.getOldSy())*time_diff/2;
//    velocities[2] = velocities[2] + (myEMA.getSz() + myEMA.getOldSz())*time_diff/2;

    if (displayStatus) {
      Serial.print("Velocities, [x, y, z] = \t");
      Serial.print(velocities[0], 4); Serial.print(", \t"); 
      Serial.print(velocities[1], 4); Serial.print(", \t");
      Serial.print(velocities[2], 4); Serial.println(", \t");
    }

    myEMA.updateOldValues();
    
    IMU_Read_Time = micros();
  }
  
  if ( velocities[0] > MAX_DRONE_SPEED || velocities[1] > MAX_DRONE_SPEED || velocities[2] > MAX_DRONE_SPEED ) {
    remote_lost = 1; // If speeds are too high, assume the drone is lost and kill the power
    armed = 0;
    if(displayStatus) Serial.println("TOO FAST!");
  }
/***************************************************************************************/

/***************************************************************************************/
/***********************************Receive ROS Data************************************/

//  testRunning.data = EMA_s[0];
  xvel_data.data = velocities[0];
  yvel_data.data = velocities[1];
  data_pub_testing.publish(&testRunning); // Set the data to be sent back to ROS.
  data_pub_xvel.publish(&xvel_data);
  data_pub_yvel.publish(&yvel_data);
  data_pub_PID.publish(&xspeedpid_data);
  
  nh.spinOnce(); // Send data and call subscriber callback to receive any data.

  if (tuning_params_changed) {
    xSpeedPID.set_PID_constants(tuning_data[0], tuning_data[1], tuning_data[2], 0, 0);
    ySpeedPID.set_PID_constants(tuning_data[0], tuning_data[1], tuning_data[2], 0, 0);
    tuning_params_changed = 0;
  }
/***************************************************************************************/

  // If we're in manual mode, pass data read from receiver to the FCU
  if (!automated) {
    if (displayStatus) Serial.print("Data received is: ");
    
    for (uint8_t i = 0; i < channel_count; i++) {
      uint16_t read_data = iBus.readChannel(i); // Read one frame
      channel_data[i] = read_data;
//      channel_data[i] = iBus.readChannel(i); // Read one frame
      if (displayStatus) {
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
    }
    if (displayStatus)  Serial.println();
  }
  // If in automated mode, calculate our own channel values to send to FCU
  else if(automated) {
    
    unsigned long current_time = micros();
    time_diff = (current_time - old_PID_time)/1000000.0; // micros() time overflows after 70 mins but this difference will still be correct as it also overflows

    // for testing, replace with readings
    float current_x = 0.0;
    float current_y = 0.0;
    float current_z = 0.0;
    float current_xspeed = velocities[0];
    float current_yspeed = velocities[1];
    float current_zspeed = velocities[2];
    float current_pitch = orient[1];
    float current_roll = orient[0];
    float current_yaw = yaw;

/***************************************************************************************/
/****************************Position and Speed Controllers*****************************/
    // position controller inputs (current_pos, time_diff)
    float xPosOutput = xPosPID.compute_PID(current_x, time_diff);
    float yPosOutput = yPosPID.compute_PID(current_y, time_diff);
    float zPosOutput = zPosPID.compute_PID(current_z, time_diff);
    float yawPosOutput = yawPosPID.compute_PID(current_yaw, time_diff);

    // speed controller inputs (current_speed, time_diff)
//    xSpeedPID.set_desired_value(xOutput);
    float xSpeedOutput = xSpeedPID.compute_PID(current_xspeed, time_diff);
//    ySpeedPID.set_desired_value(yOutput);
    float ySpeedOutput = ySpeedPID.compute_PID(current_yspeed, time_diff);
//    zSpeedPID.set_desired_value(zOutput);
    float zSpeedOutput = zSpeedPID.compute_PID(current_zspeed, time_diff);

    xSpeedOutput = xSpeedOutput*500.0 + 1500;
    ySpeedOutput = ySpeedOutput*500.0 + 1500;
//    zSpeedOutput = zSpeedOutput*500.0 + 1500;
    
    xspeedpid_data.data = xSpeedOutput - 1500.0;  
//    yspeedpid_data.data = ySpeedOutput - 1500.0; 

// The flow deck vx and vy are relative to drone's position, not world x and y frame. Is this needed if loco used as well?
//    // orientation conversion input (current angle) (desired angles taken from speed controllers already)
//    float pitchOutput = positionController.compute_desired_pitch(current_pitch)*500 + 1500;
//    float rollOutput = positionController.compute_desired_roll(current_roll)*500 + 1500;
/***************************************************************************************/


//    set_outputs( rollOutput, pitchOutput, zOutput, desired_yaw, 1500, 2000 ); // last 2 values are armed and automated controls
    if (iBus.readChannel(4) > 1600) {
      channel_data = set_outputs( ySpeedOutput, iBus.readChannel(1), iBus.readChannel(2), iBus.readChannel(3), iBus.readChannel(4), 1000, channel_data ); // last 2 values are armed and automated controls
    }
    else {
      channel_data = set_outputs( iBus.readChannel(0), iBus.readChannel(1), iBus.readChannel(2), iBus.readChannel(3), iBus.readChannel(4), 1000, channel_data ); // last 2 values are armed and automated controls
    }
    
    old_PID_time = current_time;
  }
  
  // If we suspect the controller is lost, set all outputs to known safe values
  if (remote_lost) {
    channel_data = set_safe_outputs(channel_data);
    if (displayStatus) Serial.println("Entering Safe State");
  }
  
  // NOTE: Should there be checks to ensure we're writing new data?
  iBus.write_one_frame(channel_data, channel_count, iBus_serial); // Send one frame to be written

  if (displayStatus) {
  Serial.print("Data sent is: ");
    for (uint8_t i = 0; i < channel_count; i++) {
      if (displayStatus) {
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
        Serial.print(channel_data[i]);
        Serial.print(", ");
      }
    }
    Serial.println();
  }

  // Ensure loop runs at the correct rate
  time_diff = millis() - old_loop_time;
  if (time_diff < LOOP_TIME) {
    delay (LOOP_TIME - time_diff);
  }
  old_loop_time = millis();
}
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

// NOTE: Do we need to return the array? If we pass in a pointer to the array and change variables at these memory locations then it is changing the original array.
// NOTE: Set explicit size of the array pointer input?
uint16_t* set_safe_outputs (uint16_t *channel_data_in) {
  channel_data_in[0] = 1500; // Roll control - default = 1500
  channel_data_in[1] = 1500; // Pitch control - default = 1500
  channel_data_in[2] = 1000; // Throttle control - default = 1000
  channel_data_in[3] = 1500; // Rudder control - default = 1500
  channel_data_in[4] = 1000; // Arming control - default = 1000, armed = 1500
  channel_data_in[5] = 1000; // Selectable control - default = 1000, second mode at 2000

  return channel_data_in;
}

uint16_t* set_outputs (uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t arming_sw, uint16_t automated_sw, uint16_t *channel_data_in) {
  channel_data_in[0] = roll;         // Roll control - default = 1500
  channel_data_in[1] = pitch;        // Pitch control - default = 1500
  channel_data_in[2] = throttle;     // Throttle control - default = 1000
  channel_data_in[3] = yaw;          // Rudder control - default = 1500
  channel_data_in[4] = arming_sw;    // Arming control - default = 1000, armed = 1500
  channel_data_in[5] = automated_sw; // Selectable control - default = 1000, second mode at 2000

  for (int i = 0; i<6; i++) {
    channel_data_in[i] = saturateiBusCommand(channel_data_in[i]);
  }
  
  return channel_data_in;
}

uint16_t saturateiBusCommand (uint16_t val) {
  if (val > IBUS_UPPER_LIMIT) val = IBUS_UPPER_LIMIT;
  if (val < IBUS_LOWER_LIMIT) val = IBUS_LOWER_LIMIT;
  return val;
}

