// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include <Arduino.h>
#include <SD.h>
#ifdef __linux__
  #include <WiFi.h>
#endif

#include "robot.h"
#include "StateEstimator.h"
#include "Storage.h"
#include "Stats.h"
#include "LineTracker.h"
#include "comm.h"
#include "src/op/op.h"
#ifdef __linux__
  #include <BridgeClient.h>
#else
  #include "src/esp/WiFiEsp.h"
#endif
#include "PubSubClient.h"
#include "RunningMedian.h"
#include "pinman.h"
#include "ble.h"
#include "motor.h"
#include "src/driver/AmRobotDriver.h"
#include "src/driver/CanRobotDriver.h"
#include "src/driver/SerialRobotDriver.h"
#include "src/driver/MpuDriver.h"
#include "src/driver/BnoDriver.h"
#include "src/driver/IcmDriver.h"
#include "battery.h"
#include "gps.h"
#include "src/ublox/ublox.h"
#include "src/skytraq/skytraq.h"
#include "src/lidar/lidar.h"
#include "helper.h"
#include "buzzer.h"
#include "rcmodel.h"
#include "map.h"
#include "config.h"
#include "reset.h"
#include "cpu.h"
#include "i2c.h"
#include "src/test/test.h"
#include "bumper.h"
#include "mqtt.h"

// #define I2C_SPEED  10000
#define _BV(x) (1 << (x))

const signed char orientationMatrix[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};

#ifdef DRV_SIM_ROBOT
  SimImuDriver imuDriver(robotDriver);
#elif defined(GPS_LIDAR)
  LidarImuDriver imuDriver;
#elif defined(BNO055)
  BnoDriver imuDriver;  
#elif defined(ICM20948)
  IcmDriver imuDriver;  
#else
  MpuDriver imuDriver;
#endif
#ifdef DRV_SERIAL_ROBOT
  SerialRobotDriver robotDriver;
  SerialMotorDriver motorDriver(robotDriver);
  SerialBatteryDriver batteryDriver(robotDriver);
  SerialBumperDriver bumperDriver(robotDriver);
  SerialStopButtonDriver stopButton(robotDriver);
  SerialRainSensorDriver rainDriver(robotDriver);
  SerialLiftSensorDriver liftDriver(robotDriver);
  SerialBuzzerDriver buzzerDriver(robotDriver);
#elif defined(DRV_CAN_ROBOT)
  CanRobotDriver robotDriver;
  CanMotorDriver motorDriver(robotDriver);
  CanBatteryDriver batteryDriver(robotDriver);
  CanBumperDriver bumperDriver(robotDriver);
  CanStopButtonDriver stopButton(robotDriver);
  CanRainSensorDriver rainDriver(robotDriver);
  CanLiftSensorDriver liftDriver(robotDriver);
  CanBuzzerDriver buzzerDriver(robotDriver);
#elif defined(DRV_SIM_ROBOT)
  SimRobotDriver robotDriver;
  SimMotorDriver motorDriver(robotDriver);
  SimBatteryDriver batteryDriver(robotDriver);
  SimBumperDriver bumperDriver(robotDriver);
  SimStopButtonDriver stopButton(robotDriver);
  SimRainSensorDriver rainDriver(robotDriver);
  SimLiftSensorDriver liftDriver(robotDriver);
  SimBuzzerDriver buzzerDriver(robotDriver);
#else
  AmRobotDriver robotDriver;
  AmMotorDriver motorDriver;
  AmBatteryDriver batteryDriver;
  AmBumperDriver bumperDriver;
  AmStopButtonDriver stopButton;
  AmRainSensorDriver rainDriver;
  AmLiftSensorDriver liftDriver;
  AmBuzzerDriver buzzerDriver;
#endif
Motor motor;
Battery battery;
PinManager pinMan;
#ifdef DRV_SIM_ROBOT
  SimGpsDriver gps(robotDriver);
#elif GPS_LIDAR
  LidarGpsDriver gps;
#elif GPS_SKYTRAQ
  SKYTRAQ gps;
#else 
  UBLOX gps;
#endif 
BLEConfig bleConfig;
Buzzer buzzer;
Sonar sonar;
Bumper bumper;
VL53L0X tof(VL53L0X_ADDRESS_DEFAULT); //remove me
Map maps;
RCModel rcmodel;
TimeTable timetable;

int stateButton = 0;  
int stateButtonTemp = 0;
unsigned long stateButtonTimeout = 0;

float escapeLawnDistance = ESCAPELAWNDISTANCE; //MrTree
bool escapeFinished = true; //MrTree
bool gpsObstacleNotAllowed = false; //MrTree
unsigned long gpsObstacleNotAllowedTime = 0; //MrTree
unsigned long escapeLawnTriggerTime = 0; //MrTree
bool RC_Mode = false; //MrTree
const long watchdogTime = WATCHDOG_TIME;

OperationType stateOp = OP_IDLE; // operation-mode
Sensor stateSensor = SENS_NONE; // last triggered sensor

unsigned int robot_control_cycle = ROBOT_IDLE_CYCLE;
unsigned long deltaTime = 0; 
unsigned long timeLast = 0;
unsigned long controlLoops = 0;
String stateOpText = "";  // current operation as text
String gpsSolText = ""; // current gps solution as text
float stateTemp = 20; // degreeC
//float stateHumidity = 0; // percent
unsigned long stateInMotionLastTime = 0;
bool stateChargerConnected = false;
bool stateInMotionLP = false; // robot is in angular or linear motion? (with motion low-pass filtering)

unsigned long lastFixTime = 0;
int fixTimeout = 0;
bool absolutePosSource = false;
double absolutePosSourceLon = 0;
double absolutePosSourceLat = 0;
float lastGPSMotionX = 0;
float lastGPSMotionY = 0;
unsigned long nextGPSMotionCheckTime = 0;

bool finishAndRestart = false;

unsigned long nextBadChargingContactCheck = 0;
unsigned long nextToFTime = 0;
unsigned long linearMotionStartTime = 0;
unsigned long angularMotionStartTime = 0;
unsigned long overallMotionTimeout = 0;
unsigned long nextControlTime = 0;
unsigned long lastComputeTime = 0;

unsigned long nextLedTime = 0;
unsigned long nextImuTime = 0;
unsigned long nextTempTime = 0;
unsigned long imuDataTimeout = 0;
unsigned long nextSaveTime = 0;
unsigned long nextOutputTime = 0; //MrTree
unsigned long nextTimetableTime = 0;

//##################################################################################
unsigned long loopTime = millis();
int loopTimeNow = 0;
int loopTimeMax = 0;
float loopTimeMean = 0;
int loopTimeMin = 99999;
unsigned long loopTimeTimer = 0;
unsigned long wdResetTimer = millis();
//##################################################################################


bool wifiFound = false;
char ssid[] = WIFI_SSID;      // your network SSID (name)
char pass[] = WIFI_PASS;        // your network password
WiFiEspServer server(80);
bool hasClient = false;
WiFiEspClient client;
WiFiEspClient espClient;
PubSubClient mqttClient(espClient);
//int status = WL_IDLE_STATUS;     // the Wifi radio's status
#ifdef ENABLE_NTRIP
  NTRIPClient ntrip;  // NTRIP tcp client (optional)
#endif
#ifdef GPS_USE_TCP
  WiFiClient gpsClient; // GPS tcp client (optional)  
#endif

int motorErrorCounter = 0;
int motorMowStallCounter = 0; //MrTree

//RunningMedian<unsigned int,3> tofMeasurements;
//RunningMedian tofMeasurements = RunningMedian(3);

// must be defined to override default behavior
void watchdogSetup (void){} 

void resetMotion(){
  resetLinearMotionMeasurement();
  resetAngularMotionMeasurement();
  resetOverallMotionTimeout();
  resetStateEstimation();
}

void resetStateEstimation(){
  //stateDelta
  //stateDeltaGPS
  //stateDeltaIMU = 0;
  //stateDeltaLast = 0;
  //stateDeltaSpeed = 0;
  stateDeltaSpeedIMU = 0;
  stateDeltaSpeed = 0;
  stateDeltaSpeedLP = 0;
  stateDeltaSpeedWheels = 0;
  diffIMUWheelYawSpeed = 0;
  diffIMUWheelYawSpeedLP = 0;
}

// reset linear motion measurement
void resetLinearMotionMeasurement(){
  linearMotionStartTime = millis();  
}

// reset angular motion measurement
void resetAngularMotionMeasurement(){
  angularMotionStartTime = millis();
}

// reset overall motion timeout
void resetOverallMotionTimeout(){
  overallMotionTimeout = millis() + 10000;      
}

void updateGPSMotionCheckTime(){
  nextGPSMotionCheckTime = millis() + GPS_MOTION_DETECTION_TIMEOUT * 1000;     
}

void sensorTest(){
  CONSOLE.println("testing sensors for 60 seconds...");
  unsigned long stopTime = millis() + 60000;  
  unsigned long nextMeasureTime = 0;
  while (millis() < stopTime){
    sonar.run();
    bumper.run();
    liftDriver.run();
    if (millis() > nextMeasureTime){
      nextMeasureTime = millis() + 1000;      
      if (SONAR_ENABLE){
        CONSOLE.print("sonar (enabled,left,center,right,triggered): ");
        CONSOLE.print(sonar.enabled);
        CONSOLE.print("\t");
        CONSOLE.print(sonar.distanceLeft);
        CONSOLE.print("\t");
        CONSOLE.print(sonar.distanceCenter);
        CONSOLE.print("\t");
        CONSOLE.print(sonar.distanceRight);
        CONSOLE.print("\t");
        CONSOLE.print(((int)sonar.obstacle()));
        CONSOLE.print("\t");
      }
   
      if (BUMPER_ENABLE){
        CONSOLE.print("bumper (left,right,triggered): ");
        CONSOLE.print(((int)bumper.testLeft()));
        CONSOLE.print("\t");
        CONSOLE.print(((int)bumper.testRight()));
        CONSOLE.print("\t");
        CONSOLE.print(((int)bumper.obstacle()));
        CONSOLE.print("\t");       
      }
	    #ifdef ENABLE_LIFT_DETECTION 
        CONSOLE.print("lift sensor (triggered): ");		
        CONSOLE.print(((int)liftDriver.triggered()));	
        CONSOLE.print("\t");							            
      #endif  
	
      CONSOLE.println();  
      watchdogReset();
      robotDriver.run();   
    }
  }
  CONSOLE.println("end of sensor test - please ignore any IMU/GPS errors");
}

void startWIFI(){
#ifdef __linux__
  WiFi.begin();
  wifiFound = true;
#else  
  CONSOLE.println("probing for ESP8266 (NOTE: will fail for ESP32)...");
  int status = WL_IDLE_STATUS;     // the Wifi radio's status
  WIFI.begin(WIFI_BAUDRATE); 
  WIFI.print("AT\r\n");  
  delay(500);
  String res = "";  
  while (WIFI.available()){
    char ch = WIFI.read();    
    res += ch;
  }
  if (res.indexOf("OK") == -1){
    CONSOLE.println("WIFI (ESP8266) not found! If you have ESP8266 and the problem persist, you may need to flash your ESP to firmware 2.2.1");
    return;
  }    
  WiFi.init(&WIFI);  
  if (WiFi.status() == WL_NO_SHIELD) {
    CONSOLE.println("ERROR: WiFi not present");       
    return;
  }   
  wifiFound = true;
  CONSOLE.print("WiFi found! ESP8266 firmware: ");
  CONSOLE.println(WiFi.firmwareVersion());       
  if (START_AP){
    CONSOLE.print("Attempting to start AP ");  
    CONSOLE.println(ssid);
    // uncomment these two lines if you want to set the IP address of the AP
    #ifdef WIFI_IP  
      IPAddress localIp(WIFI_IP);
      WiFi.configAP(localIp);  
    #endif            
    // start access point
    status = WiFi.beginAP(ssid, 10, pass, ENC_TYPE_WPA2_PSK);         
  } else {
    while ( status != WL_CONNECTED) {
      CONSOLE.print("Attempting to connect to WPA SSID: ");
      CONSOLE.println(ssid);      
      status = WiFi.begin(ssid, pass);
      #ifdef WIFI_IP  
        IPAddress localIp(WIFI_IP);
        WiFi.config(localIp);  
      #endif
    }    
  } 
  CONSOLE.print("You're connected with SSID=");    
  CONSOLE.print(WiFi.SSID());
  CONSOLE.print(" and IP=");        
  IPAddress ip = WiFi.localIP();    
  CONSOLE.println(ip);   
#endif         
  #if defined(ENABLE_UDP)
    udpSerial.beginUDP();  
  #endif    
  if (ENABLE_SERVER){
    //server.listenOnLocalhost();
    server.begin();
  }
  if (ENABLE_MQTT){
    CONSOLE.println("MQTT: enabled");
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
  }  
}

// check for RTC module
bool checkAT24C32() {
  byte b = 0;
  int r = 0;
  unsigned int address = 0;
  Wire.beginTransmission(AT24C32_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(AT24C32_ADDRESS);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(AT24C32_ADDRESS, 1);
      while (Wire.available() > 0 && r < 1) {        
        b = (byte)Wire.read();        
        r++;
      }
    }
  }
  #ifdef __linux__  
    return true;
  #else
    return (r == 1);
  #endif
}

void outputConfig(){
  #ifdef ENABLE_PASS
    CONSOLE.println("ENABLE_PASS");
  #endif 
  #ifdef ENABLE_TILT_DETECTION
    CONSOLE.println("ENABLE_TILT_DETECTION");
  #endif
  CONSOLE.print("FREEWHEEL_IS_AT_BACKSIDE: ");
  CONSOLE.println(FREEWHEEL_IS_AT_BACKSIDE);
  CONSOLE.print("WHEEL_BASE_CM: ");
  CONSOLE.println(WHEEL_BASE_CM);
  CONSOLE.print("WHEEL_DIAMETER: ");
  CONSOLE.println(WHEEL_DIAMETER);
  #ifdef ENABLE_LIFT_DETECTION
    CONSOLE.println("ENABLE_LIFT_DETECTION");
    #ifdef LIFT_OBSTACLE_AVOIDANCE
      CONSOLE.println("LIFT_OBSTACLE_AVOIDANCE");
    #endif
  #endif
  CONSOLE.print("ENABLE_ODOMETRY_ERROR_DETECTION: ");
  CONSOLE.println(ENABLE_ODOMETRY_ERROR_DETECTION);
  CONSOLE.print("TICKS_PER_REVOLUTION: ");
  CONSOLE.println(TICKS_PER_REVOLUTION);
  #ifdef MOTOR_DRIVER_BRUSHLESS
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS");
  #endif

  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A");
  #endif
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_A4931
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_A4931");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_JYQD
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_JYQD");
  #endif
  #ifdef MOTOR_DRIVER_BRUSHLESS_MOW_OWL
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_MOW_OWL");
  #endif  

  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308");
  #endif 
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A");
  #endif
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_A4931
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_A4931");
  #endif     
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD");
  #endif
  #ifdef MOTOR_DRIVER_BRUSHLESS_GEARS_OWL
    CONSOLE.println("MOTOR_DRIVER_BRUSHLESS_GEARS_OWL");
  #endif
  
  CONSOLE.print("MOTOR_FAULT_CURRENT: ");
  CONSOLE.println(MOTOR_FAULT_CURRENT);
  CONSOLE.print("MOTOR_OVERLOAD_CURRENT: ");
  CONSOLE.println(MOTOR_OVERLOAD_CURRENT);
  CONSOLE.print("USE_LINEAR_SPEED_RAMP: ");
  CONSOLE.println(USE_LINEAR_SPEED_RAMP);
  CONSOLE.print("MOTOR_PID_KP: ");
  CONSOLE.println(MOTOR_PID_KP);
  CONSOLE.print("MOTOR_PID_KI: ");
  CONSOLE.println(MOTOR_PID_KI);
  CONSOLE.print("MOTOR_PID_KD: ");
  CONSOLE.println(MOTOR_PID_KD);
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    CONSOLE.println("MOTOR_LEFT_SWAP_DIRECTION");
  #endif
  #ifdef MOTOR_RIGHT_SWAP_DIRECTION
    CONSOLE.println("MOTOR_RIGHT_SWAP_DIRECTION");
  #endif
  #if (!USE_MOW_RPM_SET)
    CONSOLE.print("MOW_PWM_NORMAL: ");
    CONSOLE.println(MOW_PWM_NORMAL);
  #else
    CONSOLE.print("MOW_RPM_NORMAL: ");
    CONSOLE.println(MOW_RPM_NORMAL);
  #endif
  CONSOLE.print("MOW_FAULT_CURRENT: ");
  CONSOLE.println(MOW_FAULT_CURRENT);
  CONSOLE.print("MOW_OVERLOAD_CURRENT: ");
  CONSOLE.println(MOW_OVERLOAD_CURRENT);
  CONSOLE.print("ENABLE_OVERLOAD_DETECTION: ");
  CONSOLE.println(ENABLE_OVERLOAD_DETECTION);
  CONSOLE.print("ENABLE_FAULT_DETECTION: ");
  CONSOLE.println(ENABLE_FAULT_DETECTION);
  CONSOLE.print("ENABLE_FAULT_OBSTACLE_AVOIDANCE: ");
  CONSOLE.println(ENABLE_FAULT_OBSTACLE_AVOIDANCE);
  CONSOLE.print("ENABLE_RPM_FAULT_DETECTION: ");
  CONSOLE.println(ENABLE_RPM_FAULT_DETECTION);
  #ifdef SONAR_INSTALLED
    CONSOLE.println("SONAR_INSTALLED");
    CONSOLE.print("SONAR_ENABLE: ");  
    CONSOLE.println(SONAR_ENABLE);
    CONSOLE.print("SONAR_TRIGGER_OBSTACLES: ");
    CONSOLE.println(SONAR_TRIGGER_OBSTACLES);
  #endif
  CONSOLE.print("RAIN_ENABLE: ");
  CONSOLE.println(RAIN_ENABLE);
  CONSOLE.print("BUMPER_ENABLE: ");
  CONSOLE.println(BUMPER_ENABLE);
  CONSOLE.print("BUMPER_DEADTIME: ");
  CONSOLE.println(BUMPER_DEADTIME);
  CONSOLE.print("BUMPER_TRIGGER_DELAY: ");
  CONSOLE.println(BUMPER_TRIGGER_DELAY);
  CONSOLE.print("BUMPER_MAX_TRIGGER_TIME: ");
  CONSOLE.println(BUMPER_MAX_TRIGGER_TIME);  
  CONSOLE.print("CURRENT_FACTOR: ");
  CONSOLE.println(CURRENT_FACTOR);
  CONSOLE.print("GO_HOME_VOLTAGE: ");
  CONSOLE.println(GO_HOME_VOLTAGE);
  CONSOLE.print("BAT_FULL_VOLTAGE: ");
  CONSOLE.println(BAT_FULL_VOLTAGE);
  CONSOLE.print("BAT_FULL_CURRENT: ");
  CONSOLE.println(BAT_FULL_CURRENT);
  CONSOLE.print("BAT_SWITCH_OFF_IDLE: ");
  CONSOLE.println(BAT_SWITCH_OFF_IDLE);
  CONSOLE.print("BAT_SWITCH_OFF_UNDERVOLTAGE: ");
  CONSOLE.println(BAT_SWITCH_OFF_UNDERVOLTAGE);
  #ifdef GPS_USE_TCP
    CONSOLE.println("GPS_USE_TCP");
  #endif
  #ifdef GPS_SKYTRAQ
    CONSOLE.println("GPS_USE_SKYTRAQ");  
  #endif
  CONSOLE.print("REQUIRE_VALID_GPS: ");
  CONSOLE.println(REQUIRE_VALID_GPS);
  CONSOLE.print("GPS_SPEED_DETECTION: ");
  CONSOLE.println(GPS_SPEED_DETECTION);
  CONSOLE.print("GPS_MOTION_DETECTION: ");
  CONSOLE.println(GPS_MOTION_DETECTION);
  CONSOLE.print("GPS_REBOOT_RECOVERY: ");
  CONSOLE.println(GPS_REBOOT_RECOVERY);
  CONSOLE.print("GPS_CONFIG: ");
  CONSOLE.println(GPS_CONFIG);
  CONSOLE.print("GPS_CONFIG_FILTER: ");
  CONSOLE.println(GPS_CONFIG_FILTER);
  CONSOLE.print("CPG_CONFIG_FILTER_MINELEV: ");
  CONSOLE.println(CPG_CONFIG_FILTER_MINELEV);
  CONSOLE.print("CPG_CONFIG_FILTER_NCNOTHRS: ");
  CONSOLE.println(CPG_CONFIG_FILTER_NCNOTHRS);
  CONSOLE.print("CPG_CONFIG_FILTER_CNOTHRS: ");
  CONSOLE.println(CPG_CONFIG_FILTER_CNOTHRS);
  CONSOLE.print("ALLOW_ROUTE_OUTSIDE_PERI_METER: ");
  CONSOLE.println(ALLOW_ROUTE_OUTSIDE_PERI_METER);
  CONSOLE.print("OBSTACLE_DETECTION_ROTATION: ");
  CONSOLE.println(OBSTACLE_DETECTION_ROTATION);
  CONSOLE.print("KIDNAP_DETECT: ");
  CONSOLE.println(KIDNAP_DETECT);
  CONSOLE.print("KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE: ");
  CONSOLE.println(KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE);
  CONSOLE.print("DOCKING_STATION: ");
  CONSOLE.println(DOCKING_STATION);
  CONSOLE.print("DOCK_IGNORE_GPS: ");
  CONSOLE.println(DOCK_IGNORE_GPS);
  CONSOLE.print("DOCK_AUTO_START: ");
  CONSOLE.println(DOCK_AUTO_START);
  CONSOLE.print("TARGET_REACHED_TOLERANCE: ");
  CONSOLE.println(TARGET_REACHED_TOLERANCE);
  CONSOLE.print("STANLEY_CONTROL_P_NORMAL: ");
  CONSOLE.println(STANLEY_CONTROL_P_NORMAL);
  CONSOLE.print("STANLEY_CONTROL_K_NORMAL: ");
  CONSOLE.println(STANLEY_CONTROL_K_NORMAL);
  CONSOLE.print("STANLEY_CONTROL_P_SLOW: ");
  CONSOLE.println(STANLEY_CONTROL_P_SLOW);
  CONSOLE.print("STANLEY_CONTROL_K_SLOW: ");
  CONSOLE.println(STANLEY_CONTROL_K_SLOW);
  CONSOLE.print("BUTTON_CONTROL: ");
  CONSOLE.println(BUTTON_CONTROL);
  CONSOLE.print("USE_TEMP_SENSOR: ");
  CONSOLE.println(USE_TEMP_SENSOR);
  #ifdef BUZZER_ENABLE
    CONSOLE.println("BUZZER_ENABLE");    
  #endif
}

// robot start routine
void start(){    
  pinMan.begin();
  pinMode(pinRemoteSpeed, OUTPUT);      //********* Relais Board K1 LED LIGHT          
  // keep battery switched ON
  batteryDriver.begin();  
  CONSOLE.begin(CONSOLE_BAUDRATE);    
  buzzerDriver.begin();
  buzzer.begin();      
    
  Wire.begin();      
  analogReadResolution(12);  // configure ADC 12 bit resolution
  unsigned long timeout = millis() + 2000;
  while (millis() < timeout){
    if (!checkAT24C32()){
      CONSOLE.println(F("PCB not powered ON or RTC module missing"));      
      I2Creset();  
      Wire.begin();    
      #ifdef I2C_SPEED
        Wire.setClock(I2C_SPEED);     
      #endif
    } else break;
  }  
  
  // give Arduino IDE users some time to open serial console to actually see very first console messages
  #ifndef __linux__
    delay(1500);
  #endif
    
  #if defined(ENABLE_SD)
    #ifdef __linux__
      bool res = SD.begin();
    #else 
      bool res = SD.begin(SDCARD_SS_PIN);
    #endif    
    if (res){
      CONSOLE.println("SD card found!");
      #if defined(ENABLE_SD_LOG)        
        sdSerial.beginSD();  
      #endif
    } else {
      CONSOLE.println("no SD card found");                
    }    
  #endif 
  
  logResetCause();
  
  CONSOLE.println(VER);          
  CONSOLE.print("compiled for: ");
  CONSOLE.println(BOARD);
  
  robotDriver.begin();
  CONSOLE.print("robot id: ");
  String rid = "";
  robotDriver.getRobotID(rid);
  CONSOLE.println(rid);
  motorDriver.begin();
  rainDriver.begin();
  liftDriver.begin();  
  battery.begin();      
  stopButton.begin();

  bleConfig.run();   
  //BLE.println(VER); is this needed? can confuse BLE modules if not connected?  
    
  rcmodel.begin();  
  motor.begin();
  sonar.begin();
  bumper.begin();

  outputConfig();

        
  
  CONSOLE.print("SERIAL_BUFFER_SIZE=");
  CONSOLE.print(SERIAL_BUFFER_SIZE);
  CONSOLE.println(" (increase if you experience GPS checksum errors)");
  //CONSOLE.println("-----------------------------------------------------");
  //CONSOLE.println("NOTE: if you experience GPS checksum errors, try to increase UART FIFO size:");
  //CONSOLE.println("1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom");
  //CONSOLE.println("2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h");
  //CONSOLE.println("   for Grand Central M4 'packages/adafruit/hardware/samd/xxxxx/cores/arduino/RingBuffer.h");  
  //CONSOLE.println("change:     #define SERIAL_BUFFER_SIZE 128     into into:     #define SERIAL_BUFFER_SIZE 1024");
  CONSOLE.println("-----------------------------------------------------");
  
  #ifdef GPS_USE_TCP
    gps.begin(gpsClient, GPS_HOST, GPS_PORT);
  #else 
    gps.begin(GPS, GPS_BAUDRATE);   
  #endif

  maps.begin();      
  //maps.clipperTest();
    
  // initialize ESP module
  startWIFI();
  #ifdef ENABLE_NTRIP
    ntrip.begin();  
  #endif
  
  watchdogEnable(watchdogTime);   // 15 seconds  
  
  startIMU(false);        
  
  buzzer.sound(SND_READY);  
  battery.resetIdle();        
  loadState();

  if (WATCHDOG_CONTINUE) {
    activeOp->checkStop();
    activeOp->run();
  } 

  #ifdef DRV_SIM_ROBOT
    robotDriver.setSimRobotPosState(stateX, stateY, stateDelta);
    tester.begin();
  #endif
}
// should robot wait?
bool robotShouldWait(){
  //motor.waitMowMotor();
  //if (motor.motorMowRpmCheck)

  if (motor.waitMowMotor()) {
    //motor.waitSpinUp = false;
    CONSOLE.println("waitSpinUp triggered");
    //activeOp->onMotorMowStart();
    triggerMotorMowWait();
    return true;
  }
  if (GPS_JUMP_WAIT && gpsJump){
    gpsJump = false;
    //motor.stopImmediately(true);
    //activeOp->onGpsJump();
    triggerGpsJump();
    return true;
  }

  return false;
}

// should robot move forward or backward?
bool robotShouldMove(){
  return (fabs(motor.linearSpeedSet) >= MOTOR_MIN_SPEED); 
}

// should robot move forward?
bool robotShouldMoveForward(){
   return (motor.linearSpeedSet >= MOTOR_MIN_SPEED / 2.0);
}

// should robot move backward?
bool robotShouldMoveBackward(){
   return (motor.linearSpeedSet <= - MOTOR_MIN_SPEED / 2.0);   
}

// should robot rotate? only applies when robot is nearly still
bool robotShouldRotate(){
  if (fabs(motor.linearSpeedSet) < MOTOR_MIN_SPEED && fabs(motor.angularSpeedSet)/PI*180.0 > 4.0) return (true);//MrTree changed to deg/s (returned true before if angularspeedset > 0.57deg/s), reduced linearSpeedSet condition
    else return (false);   
}

// should robot rotate left? only applies when robot is nearly still
bool robotShouldRotateLeft(){
  if (fabs(motor.linearSpeedSet) < (MOTOR_MIN_SPEED*2) && (motor.angularSpeedSet/PI*180.0 < -4.0)) return (true);//MrTree changed to deg/s (returned true before if angularspeedset > 0.57deg/s), reduced linearSpeedSet condition
    else return (false);   
}

// should robot rotate right? only applies when robot is nearly still
bool robotShouldRotateRight(){
  if (fabs(motor.linearSpeedSet) < (MOTOR_MIN_SPEED*2) && (motor.angularSpeedSet/PI*180.0 > 4.0)) return (true);//MrTree changed to deg/s (returned true before if angularspeedset > 0.57deg/s), reduced linearSpeedSet condition
    else return (false);   
}

// should robot be in motion? NOTE: function ignores very short motion pauses (with motion low-pass filtering)
bool robotShouldBeInMotion(){  
  if (robotShouldMove() || (robotShouldRotate())) {
    stateInMotionLastTime = millis();
    stateInMotionLP = true;    
  }
  if (millis() > stateInMotionLastTime + 2000) {
    stateInMotionLP = false;
  }
  return stateInMotionLP;
}

void triggerWaitCommand(unsigned int waitTime){
  waitOp.waitTime = waitTime;
  activeOp->onWaitCommand();
}

void triggerMotorMowWait(){
  resetMotion();
  //resetStateEstimation();
  //resetLinearMotionMeasurement();
  //resetAngularMotionMeasurement();
  //resetOverallMotionTimeout();
  activeOp->onMotorMowStart();
}

// drive reverse on high lawn and retry
void triggerMotorMowStall(){
  resetMotion();
  //resetStateEstimation();
  //resetLinearMotionMeasurement();
  //resetAngularMotionMeasurement();
  activeOp->onMotorMowStall(); 
}

// trigger gps jump action
void triggerGpsJump(){
  resetMotion();
  //resetStateEstimation();
  //resetLinearMotionMeasurement();
  //resetAngularMotionMeasurement();
  activeOp->onGpsJump();
}

// drive reverse if robot cannot move forward
void triggerObstacle(){
  resetMotion();
  //resetStateEstimation();
  //resetLinearMotionMeasurement();
  //resetAngularMotionMeasurement();
  activeOp->onObstacle();
}

// stuck rotate avoidance (drive forward if robot cannot rotate)
void triggerObstacleRotation(){
  //call maps so we dont go forward and then turning again trying to reach the point ?
  //if (!maps.nextPoint(false, stateX, stateY)) {
    // finish
  //  activeOp->onNoFurtherWaypoints();
  //}
  if (robotShouldRotateLeft()) maps.setObstaclePosition(stateX, stateY, -135.0, MOWER_RADIUS_BACK, OBSTACLE_DIAMETER);
  if (robotShouldRotateRight()) maps.setObstaclePosition(stateX, stateY, 135.0, MOWER_RADIUS_BACK, OBSTACLE_DIAMETER);
  resetMotion();
  //resetStateEstimation();
  //resetLinearMotionMeasurement();
  //resetAngularMotionMeasurement();
  
  activeOp->onObstacleRotation();
}

void detectLawn(){ //MrTree
  static unsigned long motorMowStallTime = 0;
  //if (millis() > gpsObstacleNotAllowedTime) gpsObstacleNotAllowed = false;
  if (!motor.switchedOn || motor.waitMowMotor()) return;
  if (ESCAPE_LAWN){ //MrTree option for triggering escapelawn with actual measured rpm stall
    if ((millis() > (escapeLawnTriggerTime + ESCAPELAWN_DEADTIME)) && motor.motorMowStallFlag){
      escapeLawnTriggerTime = millis();                                                                    
      motorMowStallTime += deltaTime;
      //RPM stalled, reverse from lawn after delay
      if (motorMowStallTime > ESCAPELAWN_DELAY){
        if (ESCAPE_LAWN_MODE == 1) CONSOLE.println("detectLawn(): High mow motor power!");
        if (ESCAPE_LAWN_MODE == 2) CONSOLE.println("detectLawn(): RPM of mow motor stalled!");    
        if (ESCAPELAWNDISTANCE > lastTargetDist) escapeLawnDistance = lastTargetDist;            //MrTree(svol0) Wenn die Entfernung zum letzten Wegpunkt kleiner als die Strecke zur Reversieren ist, wird nur bis zum Wegpunkt reversiert
          else escapeLawnDistance = ESCAPELAWNDISTANCE;                                          //MrTree  wegpunkt funktioniert leider nicht, da die lasttarget distance immer zum mäher geupdatet wird??     
        if (escapeFinished){
          escapeFinished = false;
          motorMowStallTime = 0;
          triggerMotorMowStall();
        }
      }
    return;
    }
  }
}

// detect sensor malfunction
void detectSensorMalfunction(){  
  if (ENABLE_ODOMETRY_ERROR_DETECTION){
    if (motor.odometryError){
      CONSOLE.println("odometry error!");    
      activeOp->onOdometryError();
      return;      
    }
  }

  if (ENABLE_OVERLOAD_DETECTION){
    if (motor.motorOverloadDuration > MOW_OVERLOAD_ERROR_TIME){
      // one motor is taking too much current over a long time (too high gras etc.) and we should stop mowing
      CONSOLE.println("overload!");    
      activeOp->onMotorOverload();
      return;
    }  
  }

  if (ENABLE_FAULT_OBSTACLE_AVOIDANCE){
    // there is a motor error (either unrecoverable fault signal or a malfunction) and we should try an obstacle avoidance
    if (motor.motorError){
      CONSOLE.println("motor error!");
      activeOp->onMotorError();
      return;      
    }  
  }
}

// detect lift 
// returns true, if lift detected, otherwise false
bool detectLift(){  
  #ifdef ENABLE_LIFT_DETECTION
    if (liftDriver.triggered()) {
	    CONSOLE.println("LIFT triggered");
      return true;            
    }  
  #endif 
  return false;
}

// detect obstacle (bumper, sonar, ToF)
// returns true, if obstacle detected, otherwise false
bool detectObstacle(){
  static unsigned long lastBumperTime = 0;
  static unsigned long noGPSSpeedTime = 0;

  if (!robotShouldMove()) return false;   
  if (millis() > gpsObstacleNotAllowedTime) gpsObstacleNotAllowed = false; //MrTree
  // lift
  #ifdef ENABLE_LIFT_DETECTION
    #ifdef LIFT_OBSTACLE_AVOIDANCE
      if ( (millis() > linearMotionStartTime + BUMPER_DEADTIME) && (liftDriver.triggered()) ) {
        CONSOLE.println("LIFT SENSOR: lift sensor obstacle!");    
        //statMowBumperCounter++;
        statMowLiftCounter++;
        triggerObstacle();    
        return true;
      }
    #endif
  #endif
  
  // bumper
  if ( millis() > lastBumperTime + BUMPER_DEADTIME && bumper.obstacle() ){ 
    lastBumperTime = millis();
    statMowBumperCounter++;
    //resetLinearMotionMeasurement();
    //resetAngularMotionMeasurement();
    //resetStateEstimation();
    if (bumper.obstacleLeft()){
      CONSOLE.println("BUMPER: bumper left obstacle!");  
      maps.setObstaclePosition(stateX, stateY, 35.0, MOWER_RADIUS_FRONT, OBSTACLE_DIAMETER);  
    } else {
      CONSOLE.println("BUMPER: bumper right obstacle!");
      maps.setObstaclePosition(stateX, stateY, -35.0, MOWER_RADIUS_FRONT, OBSTACLE_DIAMETER);
    }
    maps.setObstaclePosition(stateX, stateY, 0, ESCAPE_REVERSE_WAY, OBSTACLE_DIAMETER);  
    triggerObstacle();    
    return true;
  }
  
  // sonar
  if (sonar.obstacle() && (maps.wayMode != WAY_DOCK)){
    if (SONAR_TRIGGER_OBSTACLES){
      CONSOLE.println("SONAR_TRIGGER_OBSTACLES: sonar obstacle!");            
      statMowSonarCounter++;
      maps.setObstaclePosition(stateX, stateY, 0, MOWER_RADIUS_FRONT, OBSTACLE_DIAMETER);
      triggerObstacle();
      return true;
    }        
  }

  // check GPS stateGroundSpeed difference to linearSpeedSet
  if (millis() > linearMotionStartTime + GPS_SPEED_DEADTIME) {
    if (stateGroundSpeed < fabs(motor.linearSpeedSet/4)) {
      noGPSSpeedTime += deltaTime;
      if (NO_GPS_OBSTACLE && gpsObstacleNotAllowed){
        CONSOLE.println("GPS_SPEED_DETECTION: ignoring gps no groundspeed!");
        return false;
      }
      if ((GPS_SPEED_DETECTION && !maps.isAtDockPath()) && (noGPSSpeedTime > GPS_SPEED_DELAY)){
        CONSOLE.println("GPS_SPEED_DETECTION: gps no groundspeed => assume obstacle!");
        statMowGPSMotionTimeoutCounter++;
        noGPSSpeedTime = 0;
        //resetLinearMotionMeasurement();
        //resetAngularMotionMeasurement();
        //resetStateEstimation();
        maps.setObstaclePosition(stateX, stateY, 0, MOWER_RADIUS_FRONT, OBSTACLE_DIAMETER);
        triggerObstacle();
        return true;
      }
    }     
  }

  // check if GPS motion (obstacle detection)  
  if ((millis() > nextGPSMotionCheckTime) || (millis() > overallMotionTimeout)) {        
    updateGPSMotionCheckTime();
    resetOverallMotionTimeout(); // this resets overall motion timeout (overall motion timeout happens if e.g. 
    // motion between anuglar-only and linar-only toggles quickly, and their specific timeouts cannot apply due to the quick toggling)
    float dX = lastGPSMotionX - stateX;
    float dY = lastGPSMotionY - stateY;
    float delta = sqrt( sq(dX) + sq(dY) );    
    if (delta < GPS_MOTION_DETECTION_DELTA){ //MrTree 0.05
      if (NO_GPS_OBSTACLE && gpsObstacleNotAllowed){
        CONSOLE.println("GPS_MOTION_DETECTION: ignoring gps no groundspeed!");
        return false;
      }
      if (GPS_MOTION_DETECTION){
        //if (robotShouldMoveForward()){
          CONSOLE.println("GPS_MOTION_DETECTION: gps no motion => assume obstacle!");
          statMowGPSMotionTimeoutCounter++;
          //resetLinearMotionMeasurement();
          //resetAngularMotionMeasurement();
          //resetStateEstimation();
          maps.setObstaclePosition(stateX, stateY, 0, MOWER_RADIUS_FRONT, OBSTACLE_DIAMETER);
          triggerObstacle();
          return true;
        
        //if (robotShouldMoveBackward()){
        //  CONSOLE.println("gps no motion while reversing => assume obstacle in back!");
        //  statMowGPSMotionTimeoutCounter++;
          //changeOp(escapeForwardOp, true);
          //triggerObstacle();
          //return true;
        //}
      }
    }
    lastGPSMotionX = stateX;      
    lastGPSMotionY = stateY;      
  }
  // obstacle detection due to deflection of mower during linetracking 
  if (imuDriver.imuFound && targetDist > NEARWAYPOINTDISTANCE/2 && lastTargetDist > NEARWAYPOINTDISTANCE/2 && millis() > linearMotionStartTime + BUMPER_DEADTIME){ // function only starts when mower is going between points 
    // during mowing a line, getting deflected by obstacle while it should not rotate version 1
    if (!robotShouldRotate() && fabs(diffIMUWheelYawSpeedLP) > 12.0/180.0 * PI) {  // yaw speed difference between wheels and IMU more than 8 degree/s, e.g. due to obstacle AND imu shows not enough rotation
      CONSOLE.println("During Linetracking: IMU yaw difference between wheels and IMU while !robotShouldRotate => assuming obstacle at mower side");
      CONSOLE.print("                                                           diffIMUWheelYawSpeedLP = ");CONSOLE.println(fabs(diffIMUWheelYawSpeedLP)*180/PI);
      CONSOLE.print("                                                                    trigger value = ");CONSOLE.println(12.0);
      statMowDiffIMUWheelYawSpeedCounter++;
      //resetStateEstimation();
      //resetAngularMotionMeasurement();
      //resetLinearMotionMeasurement();
      maps.setObstaclePosition(stateX, stateY, 0, MOWER_RADIUS_BACK, OBSTACLE_DIAMETER); //need to add sides            
      triggerObstacle();
      return true;            
    }
    // during mowing a line, getting deflected by obstacle while it should not rotate version 2
    if (!robotShouldRotate() && fabs(stateDeltaSpeedIMU) > 12.0/180.0 * PI && fabs(stateDeltaSpeedWheels) < fabs(stateDeltaSpeedIMU/3)){ 
      //if (millis() > linearMotionStartTime + 2500) {  // give time to straighten and accelerate to track the line after a rotation, could use lastTargetDist and targetDist also!
      CONSOLE.println("During Linetracking: IMU deltaSpeed while !robotShouldRotate => assuming obstacle at mower side");
      CONSOLE.print("                                                                  stateDeltaSpeed = ");CONSOLE.println(fabs(stateDeltaSpeedIMU)*180/PI);
      CONSOLE.print("                                                                    trigger value = ");CONSOLE.println(12.0);
      CONSOLE.print("                                                            stateDeltaSpeedWheels = ");CONSOLE.println(fabs(stateDeltaSpeedWheels)*180/PI);
      CONSOLE.print("                                                                        trigger/2 = ");CONSOLE.println(fabs(stateDeltaSpeedIMU/3)*180/PI);
      statMowDiffIMUWheelYawSpeedCounter++;
      //resetStateEstimation();
      //resetLinearMotionMeasurement();
      //resetAngularMotionMeasurement();
      maps.setObstaclePosition(stateX, stateY, 0, MOWER_RADIUS_FRONT, OBSTACLE_DIAMETER);        
      triggerObstacle();
      return true;           
    }
  }
  return false;
}


// stuck rotate detection (e.g. robot cannot due to an obstacle outside of robot rotation point)
// returns true, if stuck detected, otherwise false
bool detectObstacleRotation(){
  if (!OBSTACLE_DETECTION_ROTATION || !robotShouldRotate()) return false;  

  //MrTree: This is the Situation without an IMU!
  if (millis() > angularMotionStartTime + ROTATION_TIMEOUT) { // too long rotation time (timeout), e.g. due to obstacle
    CONSOLE.println("too long rotation time (timeout) for requested rotation => assuming obstacle");
    statMowRotationTimeoutCounter++;
    if (FREEWHEEL_IS_AT_BACKSIDE){
      //resetStateEstimation();
      //resetLinearMotionMeasurement();
      //resetAngularMotionMeasurement();
      triggerObstacleRotation(); //MrTree changed to trigger freewheel dependent
    } else {
      //resetStateEstimation();
      //resetLinearMotionMeasurement();
      //resetAngularMotionMeasurement();
      maps.setObstaclePosition(stateX, stateY, 0, MOWER_RADIUS_FRONT, OBSTACLE_DIAMETER);     
      triggerObstacle(); //MrTree changed to trigger freewheel dependent        
    }
    return true;
  }

  if (OVERLOAD_ROTATION){       
    if ((motor.motorLeftOverload || motor.motorRightOverload) && millis() > angularMotionStartTime + OVERLOAD_ROTATION_DEADTIME){
      statMowRotationTimeoutCounter++;
      if (FREEWHEEL_IS_AT_BACKSIDE){
        CONSOLE.println("Overload on traction motors while robot should rotate! Assuming obstacle in the back!");
        //resetStateEstimation();
        //resetLinearMotionMeasurement();
        //resetAngularMotionMeasurement();           
        triggerObstacleRotation();
        //maps.nextPoint(false, stateX, stateY); //take next point instead of going back to point mower wanted to reach?
        return true;
      } else {
        CONSOLE.println("Overload on traction motors while robot should rotate! Assuming obstacle in the front!");
        //resetStateEstimation();
        //resetLinearMotionMeasurement();
        //resetAngularMotionMeasurement();
        maps.setObstaclePosition(stateX, stateY, 0, MOWER_RADIUS_FRONT, OBSTACLE_DIAMETER);    
        triggerObstacle();
        return true;
      }
    }       
  }
  if (imuDriver.imuFound){
    if (millis() > angularMotionStartTime + ROTATION_TIME) { 
      // less than 3 degree/s yaw speed, e.g. due to obstacle                 
      if (fabs(stateDeltaSpeedLP) < 3.0/180.0 * PI){ 
        CONSOLE.print("no IMU rotation speed detected for requested rotation => assuming obstacle: stateDeltaSpeedLP = ");
        CONSOLE.println(stateDeltaSpeedLP * 180/PI);
        statMowImuNoRotationSpeedCounter++;
        //resetStateEstimation();
        //resetLinearMotionMeasurement();
        //resetAngularMotionMeasurement(); 
        triggerObstacleRotation();
        //maps.nextPoint(false, stateX, stateY); //take next point instead of going back to point mower wanted to reach?
        return true;      
      }
      // yaw speed difference between wheels and IMU more than 8 degree/s, e.g. due to obstacle AND imu shows not enough rotation
      if (diffIMUWheelYawSpeedLP > 10.0/180.0 * PI ){
        CONSOLE.print("yaw difference between wheels and IMU for requested rotation => assuming obstacle: diffIMUWheelYawSpeedLP = ");
        CONSOLE.println(diffIMUWheelYawSpeedLP * 180/PI);
        statMowDiffIMUWheelYawSpeedCounter++;
        //resetStateEstimation();
        //resetLinearMotionMeasurement();
        //resetAngularMotionMeasurement();  //MrTree reset starttime            
        triggerObstacleRotation();
        //maps.nextPoint(false, stateX, stateY); //take next point instead of going back to point mower wanted to reach?
        return true;            
      }
    }    
  }
  return false;
}

// function to output parameters
void tuningOutput(){
  CONSOLE.println();
  CONSOLE.println("TUNING_LOG (disable in config.h): ");
  CONSOLE.println("---------------------------------------------------->");
  CONSOLE.println("motor.cpp: adaptive_speed()");
      CONSOLE.print("      ");CONSOLE.print("motorMowRpmSet: ");CONSOLE.print(motor.motorMowRpmSet);  CONSOLE.print(" RPM, ");CONSOLE.print("   Driver PWM: ");CONSOLE.print(motor.motorMowPWMCurr);CONSOLE.println(" val ");
      CONSOLE.print("      ");CONSOLE.print("         battV: ");CONSOLE.print(battery.batteryVoltage);CONSOLE.print(" V,   ");CONSOLE.print("motorMowPower: ");CONSOLE.print(motor.mowPowerAct);    CONSOLE.println(" Watt");     
      CONSOLE.print("      ");CONSOLE.print("      gpsSpeed: ");CONSOLE.print(stateGroundSpeed);      CONSOLE.print(" m/s, ");CONSOLE.print("     speedSet: ");CONSOLE.print(motor.linearCurrSet);  CONSOLE.println(" m/s ");
      CONSOLE.print("      ");CONSOLE.print(" ADSpeedfactor: ");CONSOLE.print(motor.SpeedFactor);     CONSOLE.print(" val, ");CONSOLE.print("     actSpeed: ");CONSOLE.print(motor.linearSpeedSet); CONSOLE.println(" m/s ");
      CONSOLE.println();
  CONSOLE.println("motor.cpp: sense()");
      CONSOLE.print("      ");CONSOLE.print("mowPowerAct: ");CONSOLE.print(motor.mowPowerAct);CONSOLE.print(" Watt, motorMowPowerMax: ");CONSOLE.print(motor.motorMowPowerMax);CONSOLE.println(" Watt");
      CONSOLE.print("      ");CONSOLE.print("motorMowSense: ");CONSOLE.print(motor.motorMowSense);CONSOLE.print(" A, motorMowSenseLP: ");CONSOLE.print(motor.motorMowSenseLP);CONSOLE.println(" A");
      CONSOLE.print("      ");CONSOLE.print("motorLeftSense: ");CONSOLE.print(motor.motorLeftSense);CONSOLE.print(" A, motorLeftSenseLP: ");CONSOLE.print(motor.motorLeftSenseLP);CONSOLE.println(" A");
      CONSOLE.print("      ");CONSOLE.print("motorRightSense: ");CONSOLE.print(motor.motorRightSense);CONSOLE.print(" A, motorRightSenseLP: ");CONSOLE.print(motor.motorRightSenseLP);CONSOLE.println(" A");
      CONSOLE.println();
  CONSOLE.println("IMU              -- ");
      CONSOLE.print("      ");CONSOLE.print("diffIMUWheelYawSpeedLP: ");CONSOLE.print(diffIMUWheelYawSpeedLP/PI*180.0);CONSOLE.println(" deg/s");
      CONSOLE.print("      ");CONSOLE.print("    stateDeltaSpeedIMU: ");CONSOLE.print(stateDeltaSpeedIMU/PI*180.0);CONSOLE.println(" deg/s");
      CONSOLE.print("      ");CONSOLE.print(" stateDeltaSpeedWheels: ");CONSOLE.print(stateDeltaSpeedWheels/PI*180.0);CONSOLE.println(" deg/s");
      CONSOLE.print("      ");CONSOLE.print("     stateDeltaSpeedLP: ");CONSOLE.print(stateDeltaSpeedLP/PI*180.0);CONSOLE.println(" deg/s");
      CONSOLE.print("      ");CONSOLE.print("               heading: ");CONSOLE.print(imuDriver.heading);CONSOLE.println(" none");
      CONSOLE.print("      ");CONSOLE.print("                    ax: ");CONSOLE.print(imuDriver.ax);CONSOLE.print(" g, ay: "); CONSOLE.print(imuDriver.ay);CONSOLE.print(" g, az: ");CONSOLE.print(imuDriver.az);CONSOLE.println(" g");
      CONSOLE.print("      ");CONSOLE.print("                  roll: ");CONSOLE.print(imuDriver.roll);CONSOLE.print(" rad, pitch: "); CONSOLE.print(imuDriver.pitch);CONSOLE.print("rad, yaw: ");CONSOLE.print(imuDriver.yaw);CONSOLE.println(" rad");
  CONSOLE.println("Info             -- ");
      CONSOLE.print("      ");CONSOLE.print("Operation: ");CONSOLE.print(stateOp);CONSOLE.println("");
      CONSOLE.println("<----------------------------------------------------");
      CONSOLE.println();
}

// robot main loop
void run(){  
  #ifdef ENABLE_NTRIP
    ntrip.run();
  #endif
  #ifdef DRV_SIM_ROBOT
    tester.run();
  #endif
  robotDriver.run();                //important (interrupts?)
  buzzer.run();                     //unimportant
  buzzerDriver.run();               //unimportant
  stopButton.run();                 //unimportant
  battery.run();                    //semiimportant maybe important because of power??
  batteryDriver.run();              //semiimportant maybe important because of power??
  motorDriver.run();                //important (interrupts?)
  rainDriver.run();                 //unimportant
  liftDriver.run();                 //important (interrupts?)
  //motor.run();
  gps.run();
  sonar.run();                      //semiimportant
  maps.run();                       //maybe important for accurate rtk distances  
  rcmodel.run();                    //unimportant
  bumper.run();                     //important 
  
  // global deltaTime
  deltaTime = timeLast - millis();
  timeLast = millis();


  // LED LIGHTS
  if (stateChargerConnected) { 
    robot_control_cycle = ROBOT_IDLE_CYCLE;         
    digitalWrite(pinRemoteSpeed, HIGH);
  } else {
    digitalWrite(pinRemoteSpeed, LOW);
    robot_control_cycle = ROBOT_CONTROL_CYCLE;          
  }
  
  // state saving
  if (millis() >= nextSaveTime){  
    nextSaveTime = millis() + 5000;
    saveState();
  }

  if (TUNING_LOG){
    if (millis() >= nextOutputTime){
      nextOutputTime = millis() + TUNING_LOG_TIME;
      tuningOutput();
    }
  }
  
  // temp
  if (millis() > nextTempTime){
    nextTempTime = millis() + 60000;    
    float batTemp = batteryDriver.getBatteryTemperature();
    float cpuTemp = robotDriver.getCpuTemperature();    
    if (OUTPUT_ENABLED) {
      CONSOLE.print("batTemp=");
      CONSOLE.print(batTemp,0);
      CONSOLE.print("  cpuTemp=");
      CONSOLE.print(cpuTemp,0);     
    }
    
    //logCPUHealth();
    CONSOLE.println();    
    if (batTemp < -999){
      stateTemp = cpuTemp;
    } else {
      stateTemp = batTemp;    
    }
    statTempMin = min(statTempMin, stateTemp);
    statTempMax = max(statTempMax, stateTemp);    
  }

  // LED states
  if (millis() > nextLedTime){
    nextLedTime = millis() + 1000;
    robotDriver.ledStateGpsFloat = (gps.solution == SOL_FLOAT);
    robotDriver.ledStateGpsFix = (gps.solution == SOL_FIXED);
    robotDriver.ledStateError = (stateOp == OP_ERROR);     
  }

  
   
  if (millis() > nextTimetableTime){
    nextTimetableTime = millis() + 30000;
    gps.decodeTOW();
    timetable.setCurrentTime(gps.hour, gps.mins, gps.dayOfWeek);
    timetable.run();
  }
  
  calcStats();
  
  if (millis() >= nextControlTime) {
    nextControlTime = millis() + robot_control_cycle; 
    controlLoops++; 

    if (imuIsCalibrating) {
      activeOp->onImuCalibration();             
    } else {
      readIMU();   
    }
    if (!robotShouldMove()){
          resetLinearMotionMeasurement();
          updateGPSMotionCheckTime();  
        }
    if (!robotShouldRotate()){
      resetAngularMotionMeasurement();
    }
    if (!robotShouldBeInMotion()){
      resetOverallMotionTimeout();
      lastGPSMotionX = 0;
      lastGPSMotionY = 0;
    }
    motor.run();
    computeRobotState();
    

    

    /*if (gpsJump) {
      // gps jump: restart current operation from new position (restart path planning)
      CONSOLE.println("restarting operation (gps jump)");
      gpsJump = false;
      motor.stopImmediately(true);
      setOperation(stateOp, true);    // restart current operation
    }*/

    if (battery.chargerConnected() != stateChargerConnected) {    
      stateChargerConnected = battery.chargerConnected(); 
      if (stateChargerConnected){      
        // charger connected event        
        activeOp->onChargerConnected();                
      } else {
        activeOp->onChargerDisconnected();
      }            
    }

    if (millis() > nextBadChargingContactCheck) {
      if (battery.badChargerContact()){
        nextBadChargingContactCheck = millis() + 60000; // 1 min.
        activeOp->onBadChargingContactDetected();
      }
    } 

    if (battery.underVoltage()){
      activeOp->onBatteryUndervoltage();
    } 
    else {      
      if (USE_TEMP_SENSOR){
        if (stateTemp > DOCK_OVERHEAT_TEMP){
          CONSOLE.print("Max Temperature triggered: ");
          CONSOLE.print(stateTemp);
          CONSOLE.println(" C°");
          activeOp->onTempOutOfRangeTriggered();
        } 
        else if (stateTemp < DOCK_TOO_COLD_TEMP){
          CONSOLE.print("Min Temperature triggered: ");
          CONSOLE.print(stateTemp);
          CONSOLE.println(" C°");
          activeOp->onTempOutOfRangeTriggered();
        }
      }
      if (RAIN_ENABLE){
        // rain sensor should trigger serveral times to robustly detect rain (robust rain detection)
        // it should not trigger if one rain drop or wet tree leaves touches the sensor  
        if (rainDriver.triggered()){  
          //CONSOLE.print("RAIN TRIGGERED ");
          activeOp->onRainTriggered();                                                                              
        }                           
      }    
      if (battery.shouldGoHome()){
        if (DOCKING_STATION){
           activeOp->onBatteryLowShouldDock();
        }
      }   
       
      if (battery.chargerConnected()){
        if (battery.chargingHasCompleted()){
          activeOp->onChargingCompleted();
        }
      }        
    } 

    //CONSOLE.print("active:");
    //CONSOLE.println(activeOp->name());
    activeOp->checkStop();
    activeOp->run();     
    //motor.run();      
    // process button state
    if (stateButton == 5){
      stateButton = 0; // reset button state
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_DOCK, false);
    } else if (stateButton == 6){ 
      stateButton = 0; // reset button state        
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_MOW, false);
    } 
    //else if (stateButton > 0){  // stateButton 1 (or unknown button state)        
    else if (stateButton == 1){  // stateButton 1                   
      stateButton = 0;  // reset button state
      stateSensor = SENS_STOP_BUTTON;
      setOperation(OP_IDLE, false);                             
    } else if (stateButton == 9){
      stateButton = 0;  // reset button state
      stateSensor = SENS_STOP_BUTTON;
      cmdSwitchOffRobot();
    } else if (stateButton == 12){
      stateButton = 0; // reset button state
      stateSensor = SENS_STOP_BUTTON;
      #ifdef __linux__
        WiFi.startWifiProtectedSetup();
      #endif
    }

    // update operation type      
    stateOp = activeOp->getGoalOperationType();  
  }   // if (millis() >= nextControlTime)
    
  // ----- read serial input (BT/console) -------------
  processComm();
  if (OUTPUT_ENABLED) outputConsole();    

  // reset watchdog, keep calm
  if(millis() > wdResetTimer + 1000){
      watchdogReset();
  } 
  
  if (CALC_LOOPTIME){
    loopTimeNow = millis() - loopTime;
    loopTimeMin = min(loopTimeNow, loopTimeMin); 
    loopTimeMax = max(loopTimeNow, loopTimeMax);
    loopTimeMean = 0.99 * loopTimeMean + 0.01 * loopTimeNow; 
    loopTime = millis();

    if(millis() > loopTimeTimer + 10000){
      if(loopTimeMax > 500){
        CONSOLE.print("WARNING - LoopTime: ");
      }else{
        CONSOLE.print("Info - LoopTime: ");
      }
      CONSOLE.print(loopTimeNow);
      CONSOLE.print(" - ");
      CONSOLE.print(loopTimeMin);
      CONSOLE.print(" - ");
      CONSOLE.print(loopTimeMean);
      CONSOLE.print(" - ");
      CONSOLE.print(loopTimeMax);
      CONSOLE.println("ms");
      loopTimeMin = 99999; 
      loopTimeMax = 0;
      loopTimeTimer = millis();
    }   
  }

  // compute button state (stateButton)
  if (BUTTON_CONTROL){
    if (stopButton.triggered()){
      if (millis() > stateButtonTimeout){
        stateButtonTimeout = millis() + 1000;
        stateButtonTemp++; // next state
        buzzer.sound(SND_READY, true);
        CONSOLE.print("BUTTON ");
        CONSOLE.print(stateButtonTemp);
        CONSOLE.println("s");                                     
      }
                          
    } else {
      if (stateButtonTemp > 0){
        // button released => set stateButton
        stateButtonTimeout = 0;
        stateButton = stateButtonTemp;
        stateButtonTemp = 0;
        CONSOLE.print("stateButton ");
        CONSOLE.println(stateButton);
      }
    }
  }    
}        

// set new robot operation
void setOperation(OperationType op, bool allowRepeat){  
  if ((stateOp == op) && (!allowRepeat)) return;  
  CONSOLE.print("setOperation op=");
  CONSOLE.println(op);
  stateOp = op;
  if (stateOp == OP_IDLE || stateOp == OP_CHARGE || stateChargerConnected){
    robot_control_cycle = ROBOT_IDLE_CYCLE;
    //dmp_set_fifo_rate(robot_control_cycle);
    mpu_reset_fifo();
  } else {
    robot_control_cycle = ROBOT_CONTROL_CYCLE;
    //dmp_set_fifo_rate(robot_control_cycle);
    mpu_reset_fifo();
  }
  activeOp->changeOperationTypeByOperator(stateOp);
  saveState();
}
