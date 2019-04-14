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
#include "SPIFFS.h"
#include "FS.h"
#include "EMA.h"


#define IBUS_MAXCHANNELS  14    // iBus has a maximum of 14 channels
#define IBUS_LOWER_LIMIT  1000
#define IBUS_UPPER_LIMIT  2000
#define IMU_SAMPLE_TIME   0.01  // Set the delay between fresh IMU samples (in S)
#define CF_SAMPLE_TIME    0.1   // Set the delay between fresh Crazyflie samples (in S)
#define PID_SAMPLE_TIME   0.1   // Set the loop time for the position PID controllers (in S)
#define LOOP_TIME         10    // Set the main loop time (in mS)
#define MAX_DRONE_SPEED   2     // If any speeds are above threshold, kill the drone for safety
#define LOWER_SPEED_LIMIT -0.2   // Limit speed going to speed controllers
#define UPPER_SPEED_LIMIT 0.2   // Limit speed going to speed controllers

PID xPosPID;
PID yPosPID;
PID zPosPID;
PID xSpeedPID;
PID ySpeedPID;
PID zSpeedPID;
PID yawPosPID;

HardwareSerial MySerial(2);
HardwareSerial& iBus_serial = MySerial; // Choose which serial port the iBus will be communicating over

void set_safe_outputs (uint16_t *channel_data_in); // function prototype
void set_outputs (uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t arming_sw, uint16_t automated_sw, uint16_t *channel_data_in);
uint16_t saturateiBusCommand (uint16_t val);
void setupPIDControllers ();

/***************************************************************************************/
/**************************Wifi stuff***************************************************/
void connectToNetwork() {
  const char* ssid = "test"; // Wifi network name
  const char* password =  "abcdefg1"; // Wifi password
//  const char* ssid = "SkyEye"; // Wifi network name
//  const char* password =  "Team8wifi"; // Wifi password
  
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
static float CF_Pos [3];
static float CF_Yaw;
static float tuning_data [3];
static float z_des;
static bool tuning_params_changed = 0;
static bool new_CF_Data = 0;

// Callbacks for subscribers
void sub_cb_loco (const std_msgs::Float32MultiArray& data_rec) {
  CF_Vel[0] = data_rec.data[0];
  CF_Vel[1] = data_rec.data[1];
  CF_Vel[2] = data_rec.data[2];
  
  CF_Yaw = data_rec.data[3];

  CF_Pos[0] = data_rec.data[4];
  CF_Pos[1] = data_rec.data[5];
  CF_Pos[2] = data_rec.data[6];
  
  new_CF_Data = 1;
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
void sub_cb_zdes (const std_msgs::Float32& data_rec) {
  z_des = data_rec.data;
  tuning_params_changed = 1;
}

ros::Subscriber<std_msgs::Float32MultiArray> data_sub_loco("loco_data_drone", &sub_cb_loco); 
ros::Subscriber<std_msgs::Float32> data_sub_kptuning("tuning_data_kp", &sub_cb_kptuning); 
ros::Subscriber<std_msgs::Float32> data_sub_kituning("tuning_data_ki", &sub_cb_kituning); 
ros::Subscriber<std_msgs::Float32> data_sub_kdtuning("tuning_data_kd", &sub_cb_kdtuning); 
ros::Subscriber<std_msgs::Float32> data_sub_zdes("zdes", &sub_cb_zdes);

bool send_vel   = 0;
bool send_pos   = 1;
bool send_acc   = 0;
bool send_x     = 0;
bool send_y     = 0;
bool send_z     = 1;
bool send_pid   = 1;
bool send_extra = 0;


// ChooseTopics
static std_msgs::Float32 xvel_data;
ros::Publisher data_pub_xvel("ESP_Data_xvel", &xvel_data); 

static std_msgs::Float32 yvel_data; 
ros::Publisher data_pub_yvel("ESP_Data_yvel", &yvel_data);

static std_msgs::Float32 zvel_data; 
ros::Publisher data_pub_zvel("ESP_Data_zvel", &zvel_data);

static std_msgs::Float32 xacc_data;
ros::Publisher data_pub_xacc("ESP_Data_xacc", &xacc_data); 

static std_msgs::Float32 yacc_data; 
ros::Publisher data_pub_yacc("ESP_Data_yacc", &yacc_data);

static std_msgs::Float32 zacc_data; 
ros::Publisher data_pub_zacc("ESP_Data_zacc", &zacc_data);
  
static std_msgs::Float32 xspeedpid_data; 
ros::Publisher data_pub_xPID("PID_xOutput", &xspeedpid_data);

static std_msgs::Float32 yspeedpid_data; 
ros::Publisher data_pub_yPID("PID_yOutput", &yspeedpid_data);

static std_msgs::Float32 zspeedpid_data; 
ros::Publisher data_pub_zPID("PID_zOutput", &zspeedpid_data);

static std_msgs::Float32 zpospid_data; 
ros::Publisher data_pub_zposPID("PID_zposOutput", &zpospid_data);

static std_msgs::Float32 testRunning;
ros::Publisher data_pub_testing("Testing_Data", &testRunning);


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
/***************************************************************************************/


/***************************************************************************************/
/**************************************SPIFF Stuff**************************************/

File datalog;
const char filename[10] = "/data.txt";

void beginSPIFFS () {
  Serial.println("Beginning SPIFF setup");
  SPIFFS.begin(true);

//  datalog = SPIFFS.open(filename, FILE_WRITE);
//  
//  datalog.close();

  datalog = SPIFFS.open(filename, FILE_APPEND);
  datalog.println("\n");
  datalog.println("XVelocity\tYVelocity\tZVelocity\tTime");
  
  Serial.println("Finished SPIFF setup");
}

/***************************************************************************************/


/*--------------------------------------------------------------------------------------------------------------------------------------------*/
/***************************************************************************************/
/******************************Setup****************************************************/
void setup() {

  iBus.begin(iBus_serial); // Open the iBus serial connection - do before Serial.begin as occasionally it hangs, possibly interference between the two?
  
  Serial.begin(115200); // Serial Monitor
  connectToNetwork(); // Connect to the Wifi - uncomment for Wifi comms, comment for USB

//  beginSPIFFS();

  Serial.println("Connecting to IMU");
  bno.begin();
  delay(20);
  bno.setExtCrystalUse(true);
  setCalibrationOffsets();
  // Set IMU mode to nine degrees of freedom (found in BN055 datasheet)
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  delay(20);
  Serial.println("Finished Connecting to IMU");

  CF_Vel[0] = 0.0;
  CF_Vel[1] = 0.0;
  CF_Vel[2] = 0.0;
  CF_Yaw = 0.0;

  Serial.println("Connecting to ROS...");
  // Initalise node, advertise the pub and subscribe the sub
  IPAddress serverIp(10,42,0,1);      // rosserial socket ROSCORE SERVER IP address
//  IPAddress serverIp(10,1,0,169);      // rosserial socket ROSCORE SERVER IP address  
  uint16_t serverPort = 11411; // rosserial socket server port - NOT roscore socket!
  nh.initNode();
  nh.getHardware()->setConnection(serverIp, serverPort); // Set the rosserial socket server info - uncomment for Wifi comms, comment for USB

  // ChooseTopics
  if (send_vel) {
    if (send_x) {
      xvel_data.data = 0.0;
      nh.advertise(data_pub_xvel);
    }
    if (send_y) {
      yvel_data.data = 0.0;
      nh.advertise(data_pub_yvel);
    }
    if (send_z) {
      zvel_data.data = 0.0;
      nh.advertise(data_pub_zvel);
    }
  }

  if (send_acc) {
    if (send_x) {
      xacc_data.data = 0.0;
      nh.advertise(data_pub_xacc);
    }
    if (send_y) { 
      yacc_data.data = 0.0;
      nh.advertise(data_pub_yacc);
    }
    if (send_z) {
      zacc_data.data = 0.0;
      nh.advertise(data_pub_zacc);
    }
  }

  if (send_pid) {
    if (send_x) {
      xspeedpid_data.data = 0.0;  
      nh.advertise(data_pub_xPID);
    }
    if (send_y) {
      yspeedpid_data.data = 0.0;
      nh.advertise(data_pub_yPID);
    }
    if (send_z) {
      zspeedpid_data.data = 0.0;
      nh.advertise(data_pub_zPID);
      if (send_pos) {
        zpospid_data.data = 0.0;
        nh.advertise(data_pub_zposPID);
      }
    }
  }

  if (send_extra) {
    testRunning.data = 0.0;
    nh.advertise(data_pub_testing);    
  }
    
  nh.subscribe(data_sub_loco);
  nh.subscribe(data_sub_kptuning);
  nh.subscribe(data_sub_kituning);
  nh.subscribe(data_sub_kdtuning);
  nh.subscribe(data_sub_zdes);

  Serial.println("ROS finished setup");

  // Initialise PID Controllers
  setupPIDControllers();

  Serial.println("Finished Setup");
}
/***************************************************************************************/

/*--------------------------------------------------------------------------------------------------------------------------------------------*/
/***************************************************************************************/
/**************************************Main Loop****************************************/
void loop() {
  static unsigned long start_time = micros();
  static unsigned long old_PID_time = micros();
  static unsigned long old_loop_time = millis();
  static unsigned long IMU_Read_Time = start_time - IMU_SAMPLE_TIME;
  static unsigned long CF_Read_Time = start_time - CF_SAMPLE_TIME;
  static unsigned long pos_PID_time = start_time - PID_SAMPLE_TIME;
  static float time_diff = 0.0;

  static float old_z_position = 0;
  static float velocities [3] = {0.0, 0.0, 0.0}; // holds [x vel, y vel, z vel]
  static float yaw = 0.0;
  static float zAccOffset = 0.0;
  static uint8_t sameZCount = 0;
  
  static EMA accelEMA(0.5);
  static EMA velEMA(0.15);
  static imu::Vector<3> accels;
  static imu::Vector<3> orient;
  
  static uint8_t channel_count = 6;
  static uint16_t channel_data [6] = {1500, 1500, 1000, 1500, 1000, 1000};

  static bool automated = 0;
  static bool armed = 0;
  static bool remote_lost = 0;
  static bool displayStatus = 0; // 0 for no serial output about controller values, 1 for information
  static bool useIMU = 1;
  static bool useCF = 1;
  static uint8_t errorType = 0;

  static char loop_count = 0;
  
  remote_lost = 0;
  errorType = 0;

  // Request one frame to be read
  if (iBus.read_loop() == 1) {
    if (displayStatus) Serial.println("lost"); 
    remote_lost = 1;
    errorType = 1;
  }
  
/***************************************************************************************/
/**************************Safety checks and automation check***************************/
  // Check if any channels are out of bounds, this suggests that the controller has been lost
  for (uint8_t i = 0; i < channel_count; i++) {
    if (iBus.readChannel(i) < IBUS_LOWER_LIMIT || iBus.readChannel(i) > IBUS_UPPER_LIMIT) {
      remote_lost = 1;
      errorType = 2;
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

//  Serial.print("Safety done at:\t"); Serial.println(millis() - old_loop_time);
/***************************************************************************************/

/***************************************************************************************/
/***********************************Receive ROS Data************************************/

//  unsigned long time1, time2, time3, time4, time5;
//    time1 = millis();

  // Set the data to be sent back to ROS.  
  // ChooseTopic

  if (send_vel) {
    if (send_x) {
      xvel_data.data = velocities[0];
      data_pub_xvel.publish(&xvel_data);
    }
    if (send_y) {
      yvel_data.data = velocities[1];
      data_pub_yvel.publish(&yvel_data);
    }
    if (send_z) {
      zvel_data.data = velocities[2];
      data_pub_zvel.publish(&zvel_data);
    }
  }

  if (send_acc) {
    if (send_x) {
      xacc_data.data = accelEMA.getSx();
    //      xacc_data.data = accels.z();
      data_pub_xacc.publish(&xacc_data);
    }
    if (send_y) {
      yacc_data.data = accelEMA.getSy();
    //      yacc_data.data = accels.y();
      data_pub_yacc.publish(&yacc_data);
    }
    if (send_z) {
      zacc_data.data = accelEMA.getSz();
    //      zacc_data.data = accels.z();
      data_pub_zacc.publish(&zacc_data);
    }
  }

  if (send_pid) {
    if (send_x) {
      data_pub_xPID.publish(&xspeedpid_data);
    }
    if (send_y) {
      data_pub_yPID.publish(&yspeedpid_data);
    }
    if (send_z) {
      data_pub_zPID.publish(&zspeedpid_data);
      if (send_pos) {
        data_pub_zposPID.publish(&zpospid_data);
      }
    }
  }

  nh.spinOnce(); // Send data and call subscriber callback to receive any data.
  
  if (tuning_params_changed) {
//    xSpeedPID.set_PID_constants(tuning_data[0], tuning_data[1], tuning_data[2]);
//    ySpeedPID.set_PID_constants(tuning_data[0], tuning_data[1], tuning_data[2]);
//    xSpeedPID.set_desired_value(z_des);

//    zSpeedPID.set_PID_constants(tuning_data[0], tuning_data[1], tuning_data[2]);
//    zSpeedPID.set_desired_value(z_des);

//    zPosPID.set_PID_constants(tuning_data[0], tuning_data[1], tuning_data[2]);
    zPosPID.set_desired_value(z_des);

    xPosPID.set_PID_constants(tuning_data[0], tuning_data[1], tuning_data[2]);
    yPosPID.set_PID_constants(tuning_data[0], tuning_data[1], tuning_data[2]);
    
    tuning_params_changed = 0;
  }

//  time2 = millis();

//  Serial.print("ROS done at:\t"); Serial.println(millis() - old_loop_time);
/***************************************************************************************/

/***************************************************************************************/
/************************************Read CF Data***************************************/
  // Correct IMU velocities using readings from the crazyflie
  // NOTE: maybe use flags to see if data has changed?
  time_diff = (micros() - CF_Read_Time)/1000000.0;
  bool cond1 = 0;
  bool cond2 = 0;
  bool cond3 = 0;
  bool cond4 = 0;
  bool cond5 = 0;
  cond1 = time_diff > CF_SAMPLE_TIME;
  cond2 = time_diff > (2*CF_SAMPLE_TIME);
  cond3 = sq(CF_Vel[2] - old_z_position) > 0.00016; // check if z position changes by more than 4mm
  cond4 = CF_Vel[2] == old_z_position; // measurements should not be identical unless we read before a new one is ready
  if (new_CF_Data) {
    if (useCF) {
      velocities[0] = CF_Vel[0];
      velocities[1] = CF_Vel[1];
      velocities[2] = CF_Vel[2];
      
      velEMA.setSx(velocities[0]);
      velEMA.setSy(velocities[1]);
      velEMA.setSz(velocities[2]);

      velEMA.updateOldValues();
      
      yaw = CF_Yaw;
    }
    CF_Read_Time = micros();
    new_CF_Data = 0;
  }

  time_diff = (micros() - CF_Read_Time)/1000000.0;
  cond5 = time_diff > 1; // check if we haven't had CF data for at least a second, if so disarm drone.
  if (cond5) { 
    armed = 0;
    errorType = 4;
  }
//  
//  Serial.print("CF done at:\t"); Serial.println(millis() - old_loop_time);
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
    accelEMA.calcSx(accels.x());
    accelEMA.calcSy(accels.y());
    accelEMA.calcSz(accels.z()); 
//    if (sameZCount > 2) {
//      zAccOffset = accels.z();
//      velocities[2] = 0.0;
//    }
//    accelEMA.calcSz(accels.z() - zAccOffset);

    // NOTE: SORT OUT AUTOMATIC CALIBRATION OF IMU
    // Integrate using filtered accelerations and trapezium integration
    if (useIMU) {
//      velocities[0] = velocities[0] + (accelEMA.getSx() + accelEMA.getOldSx())*time_diff/2;
//      velocities[1] = velocities[1] + (accelEMA.getSy() + accelEMA.getOldSy())*time_diff/2;
//      velocities[2] = velocities[2] + (accelEMA.getSz() + accelEMA.getOldSz())*time_diff/2;

      float temp1 = velocities[0] + (accelEMA.getSx() + accelEMA.getOldSx())*time_diff/2;
      float temp2 = velocities[1] + (accelEMA.getSy() + accelEMA.getOldSy())*time_diff/2;
      float temp3 = velocities[2] + (accelEMA.getSz() + accelEMA.getOldSz())*time_diff/2;

      velEMA.calcSx(temp1);
      velEMA.calcSy(temp2);
      velEMA.calcSz(temp3);

      velocities[0] = velEMA.getSx();
      velocities[1] = velEMA.getSy();
      velocities[2] = velEMA.getSz();
    }

    if (displayStatus) {
      Serial.print("Velocities, [x, y, z] = \t");
      Serial.print(velocities[0], 4); Serial.print(", \t"); 
      Serial.print(velocities[1], 4); Serial.print(", \t");
      Serial.print(velocities[2], 4); Serial.println(", \t");
    }

    accelEMA.updateOldValues();
    velEMA.updateOldValues();
    
    IMU_Read_Time = micros();
  }

  
  
  if ( velocities[0] > MAX_DRONE_SPEED || velocities[1] > MAX_DRONE_SPEED || velocities[2] > MAX_DRONE_SPEED ) {
    remote_lost = 1; // If speeds are too high, assume the drone is lost and kill the power
    armed = 0;
    if(displayStatus) Serial.println("TOO FAST!");
    errorType = 3;
  }
  
//  time3 = millis();

//  if (loop_count > 3) {
//    if (datalog.size() > 950000) { // check to see if the flash is filling up (950kB)
//      Serial.println(datalog.size());
//      datalog.close();
//      
//      datalog = SPIFFS.open(filename, FILE_WRITE);  
//      datalog.close();
//
//      datalog = SPIFFS.open(filename, FILE_APPEND);
//      datalog.println("\n");
//      datalog.println("XVelocity\tYVelocity\tZVelocity\tTime");
//          
//    }
//
//    String data2print = String(velocities[0], 5) + "\t" + String(velocities[1], 5) + "\t" + String(velocities[2], 5) + "\t" + String(micros() - start_time) + "\n";
//  
////    time4 = millis();
//  
//    datalog.print(data2print);
//    loop_count = 0;
//  }
//
//  loop_count++;
  
//  datalog.print(velocities[0],5); datalog.print("\t");
//  datalog.print(velocities[1],5); datalog.print("\t");
//  datalog.print(velocities[2],5); datalog.print("\t");
//  datalog.print(micros() - start_time); datalog.print("\n");

//    time5 = millis();
  
//  Serial.print("IMU done at:\t"); Serial.println(millis() - old_loop_time);
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
    old_PID_time = micros();
    pos_PID_time = micros();

    xPosPID.reset_integral();
    yPosPID.reset_integral();
    zPosPID.reset_integral();

    xSpeedPID.reset_integral();
    ySpeedPID.reset_integral();
    zSpeedPID.reset_integral();
  }
  // If in automated mode, calculate our own channel values to send to FCU
  else if(automated) {
    
    float current_x = CF_Pos[0];
    float current_y = CF_Pos[1];
    float current_z = CF_Pos[2];
    float current_xspeed = velocities[0];
    float current_yspeed = velocities[1];
    float current_zspeed = velocities[2];
    float current_pitch = orient[2]; // NEED TO CHECK THESE
    float current_roll = orient[1];
    float current_yaw = orient[0];

/***************************************************************************************/
/****************************Position and Speed Controllers*****************************/
    float xPosOutput;
    float yPosOutput;
    float zPosOutput;
    float yawPosOutput;

    // Calculate time difference for position PID controllers    
    unsigned long current_time = micros();
    time_diff = (current_time - pos_PID_time)/1000000.0;

    if (time_diff > PID_SAMPLE_TIME) {
      // position controller inputs (current_pos, time_diff)
      xPosOutput = xPosPID.compute_PID(current_x, time_diff);
      yPosOutput = yPosPID.compute_PID(current_y, time_diff);
      zPosOutput = zPosPID.compute_PID(current_z, time_diff);
      if (zPosOutput > 0.5) {
        Serial.print((zPosPID.get_PID_val(0))); Serial.print("\t");
        Serial.print((zPosPID.get_PID_val(1))); Serial.print("\t");
        Serial.print((zPosPID.get_PID_val(2))); Serial.println("\t");
      }
//      yawPosOutput = yawPosPID.compute_PID(current_yaw, time_diff);
      pos_PID_time = current_time;
      Serial.print(current_z); Serial.print("\t");
      Serial.print(zPosOutput); Serial.println("\t");
    }

    if (zPosOutput > 0.5) {
        Serial.print((zPosPID.get_PID_val(0))); Serial.print("\t");
        Serial.print((zPosPID.get_PID_val(1))); Serial.print("\t");
        Serial.print((zPosPID.get_PID_val(2))); Serial.println("\t");
      }

    // Calculate time difference for speed PID controllers
    current_time = micros();
    time_diff = (current_time - old_PID_time)/1000000.0; // micros() time overflows after 70 mins but this difference will still be correct as it also overflows

    // speed controller inputs (current_speed, time_diff)
    xSpeedPID.set_desired_value(xPosOutput);
    xSpeedPID.limit_des_val(LOWER_SPEED_LIMIT, UPPER_SPEED_LIMIT);
    float xSpeedOutput = xSpeedPID.compute_PID(current_xspeed, time_diff);
    
    ySpeedPID.set_desired_value(yPosOutput);
    ySpeedPID.limit_des_val(LOWER_SPEED_LIMIT, UPPER_SPEED_LIMIT);
    float ySpeedOutput = -ySpeedPID.compute_PID(current_yspeed, time_diff); // IMU and CF define y opposite to the drone
    
    zSpeedPID.set_desired_value(zPosOutput);
    zSpeedPID.limit_des_val(LOWER_SPEED_LIMIT, UPPER_SPEED_LIMIT);
    float zSpeedOutput = zSpeedPID.compute_PID(current_zspeed, time_diff);

    xSpeedOutput = xSpeedOutput*500.0 + 1500.0;
    ySpeedOutput = ySpeedOutput*500.0 + 1500.0;
    zSpeedOutput = zSpeedOutput*500.0 + 1375.0;

    if (send_pid) {
      if (send_x) {
        if (send_vel) {
          xspeedpid_data.data = xSpeedOutput - 1500.0;  
        }
        else if (send_pos) {
          xspeedpid_data.data = xPosOutput;
        }
      }
      if (send_y) {
        if (send_vel) {
          yspeedpid_data.data = ySpeedOutput - 1500.0;  
        }
        else if (send_pos) {
          yspeedpid_data.data = yPosOutput;
        }
      } 
      if (send_z) {
        if (send_vel) {
          zspeedpid_data.data = zSpeedOutput - 1375.0; 
        }
        if (send_pos) {
          zpospid_data.data = zPosOutput;
        }
      }
    }

//    Serial.print("PID done at:\t"); Serial.println(millis() - old_loop_time);

// The flow deck vx and vy are relative to drone's position, not world x and y frame. 
//    // orientation conversion input (current angle) (desired angles taken from speed controllers already)
//    float pitchOutput = positionController.compute_desired_pitch(current_pitch)*500 + 1500;
//    float rollOutput = positionController.compute_desired_roll(current_roll)*500 + 1500;
/***************************************************************************************/


//    set_outputs( rollOutput, pitchOutput, zOutput, desired_yaw, 1500, 2000 ); // last 2 values are armed and automated controls
    if (iBus.readChannel(4) > 1600) { 
      set_outputs( ySpeedOutput, xSpeedOutput, zSpeedOutput, iBus.readChannel(3), armed*iBus.readChannel(4), 1000, &channel_data[0] ); // last 2 values are armed and automated controls
    }
    else {
      set_outputs( iBus.readChannel(0), iBus.readChannel(1), iBus.readChannel(2), iBus.readChannel(3), armed*iBus.readChannel(4), 1000, &channel_data[0] ); // last 2 values are armed and automated controls
      // Reset integrals with switch
      xSpeedPID.reset_integral();
      ySpeedPID.reset_integral();
      zSpeedPID.reset_integral();
    }
    
    old_PID_time = current_time;
  }
  
  // If we suspect the controller is lost, set all outputs to known safe values
  if (remote_lost) {
    set_safe_outputs(&channel_data[0]);
    if (displayStatus) Serial.println("Entering Safe State");
  }
  
  // NOTE: Should there be checks to ensure we're writing new data?
  iBus.write_one_frame(channel_data, channel_count, iBus_serial); // Send one frame to be written

  if (send_extra) {
    testRunning.data = errorType; 
    data_pub_testing.publish(&testRunning);
  }

  nh.spinOnce();

//  if (!armed) {
//    if (errorType != 0) {
//      switch(errorType){
//        case 1:
//          Serial.println("No data rec from RF");
//          break;
//        case 2:
//          Serial.println("Remote lost");
//          break;
//        case 3:
//          Serial.println("Going too fast");
//          break;  
//        case 4:
//          Serial.println("No CF data");
//          break;
//        default:
//          break;
//      }
//    }
//    else Serial.println("Something else has happened");
//
//    if (remote_lost) {
//      Serial.println("Remote lost");
//    }
//  }
  

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

//  Serial.print("\n");
  // Ensure loop runs at the correct rate
  time_diff = millis() - old_loop_time;
//  if (time_diff > 11) {
//    Serial.println("\nHELP *********************************************************");
//    Serial.print("Loop time is: "); Serial.println(time_diff);
//
//    Serial.println("ROS Times: ");
//    Serial.print(time1); Serial.print("\t");
//    Serial.print(time2); Serial.print("\t");
//    Serial.print(time3); Serial.print("\t");
//    Serial.print(time4); Serial.print("\t");
//    Serial.println(time5); 
//    Serial.print("\n");
//  }
    
  if (time_diff < LOOP_TIME) {
    delay (LOOP_TIME - time_diff);
  }
  old_loop_time = millis();
}
/*--------------------------------------------------------------------------------------------------------------------------------------------*/

void set_safe_outputs (uint16_t *channel_data_in) {
  channel_data_in[0] = 1500; // Roll control - default = 1500
  channel_data_in[1] = 1500; // Pitch control - default = 1500
  channel_data_in[2] = 1000; // Throttle control - default = 1000
  channel_data_in[3] = 1500; // Rudder control - default = 1500
  channel_data_in[4] = 1000; // Arming control - default = 1000, armed = 1500
  channel_data_in[5] = 1000; // Selectable control - default = 1000, second mode at 2000
}

void set_outputs (uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw, uint16_t arming_sw, uint16_t automated_sw, uint16_t *channel_data_in) {
  channel_data_in[0] = roll;         // Roll control - default = 1500
  channel_data_in[1] = pitch;        // Pitch control - default = 1500
  channel_data_in[2] = throttle;     // Throttle control - default = 1000
  channel_data_in[3] = yaw;          // Rudder control - default = 1500
  channel_data_in[4] = arming_sw;    // Arming control - default = 1000, armed = 1500
  channel_data_in[5] = automated_sw; // Selectable control - default = 1000, second mode at 2000

  for (int i = 0; i<6; i++) {
    channel_data_in[i] = saturateiBusCommand(channel_data_in[i]);
  }
}

uint16_t saturateiBusCommand (uint16_t val) {
  if (val > IBUS_UPPER_LIMIT) val = IBUS_UPPER_LIMIT;
  if (val < IBUS_LOWER_LIMIT) val = IBUS_LOWER_LIMIT;
  return val;
}

void setupPIDControllers () {
  
  // Initial desired z position
  z_des = 0.5;

  // Initial PID controller constants
  float ini_xy_speed_pid [3] = {0.085, 0.1, 0.0};
  float ini_z_speed_pid [3] = {0.55, 0.65, 0.0002};
  float ini_xy_pos_pid [3] = {0.1, 0.0, 0};
  float ini_z_pos_pid [3] = {0.55, 0.03, 0};
  
  
  for (int i = 0; i<3; i++) {
//    tuning_data[i] =  ini_z_speed_pid[i];
    tuning_data[i] =  ini_z_pos_pid[i]; 
  }
  
  xPosPID.set_PID_constants(ini_xy_pos_pid[0], ini_xy_pos_pid[1], ini_xy_pos_pid[2]);
  xPosPID.set_desired_value(1);
  xPosPID.saturate_integral(false, 0);
  yPosPID.set_PID_constants(ini_xy_pos_pid[0], ini_xy_pos_pid[1], ini_xy_pos_pid[2]);
  yPosPID.set_desired_value(1);
  yPosPID.saturate_integral(false, 0);
  zPosPID.set_PID_constants(ini_z_pos_pid[0], ini_z_pos_pid[1], ini_z_pos_pid[2]);
  zPosPID.set_desired_value(z_des);
  zPosPID.saturate_integral(false, 0);
  xSpeedPID.set_PID_constants(ini_xy_speed_pid[0], ini_xy_speed_pid[1], ini_xy_speed_pid[2]);
  xSpeedPID.set_desired_value(0.0);
  xSpeedPID.limit_des_val(LOWER_SPEED_LIMIT, UPPER_SPEED_LIMIT);
  xSpeedPID.saturate_integral(false, 0);
  ySpeedPID.set_PID_constants(ini_xy_speed_pid[0], ini_xy_speed_pid[1], ini_xy_speed_pid[2]);
  ySpeedPID.set_desired_value(0.0);
  ySpeedPID.limit_des_val(LOWER_SPEED_LIMIT, UPPER_SPEED_LIMIT);
  ySpeedPID.saturate_integral(false, 0);
  zSpeedPID.set_PID_constants(ini_z_speed_pid[0], ini_z_speed_pid[1], ini_z_speed_pid[2]);
  zSpeedPID.set_desired_value(0);
  zSpeedPID.limit_des_val(LOWER_SPEED_LIMIT, UPPER_SPEED_LIMIT);
  zSpeedPID.saturate_integral(false, 150);
  yawPosPID.set_PID_constants(0,0,0);
  yawPosPID.set_desired_value(0.0); 
  yawPosPID.saturate_integral(false, 0); 
}

