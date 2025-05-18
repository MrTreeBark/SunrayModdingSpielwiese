// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

/*
   WARNING: all software, hardware and motor components are designed and optimized as a whole, if you
   try to replace or exclude some component not as designed, you risk to damage your hardware with
   the software.

   see Wiki for installation details:
   http://wiki.ardumower.de/index.php?title=Ardumower_Sunray

   requirements:
   + Ardumower chassis and Ardumower kit mowing and gear motors
   + Ardumower PCB 1.3/1.4
   +   Adafruit Grand Central M4 (highly recommended) or Arduino Due
   +   Ardumower BLE UART module (HM-10/CC2540/CC2541)
   +   optional: Ardumower IMU (MPU6050/MPU9150/MPU9250/MPU9255) - choose your IMU below
   +   optional: Ardumower WIFI (ESP8266 ESP-01 with stock firmware)
   +   optional: HTU21D temperature/humidity sensor
   +   optional: sonar, bumperduino, freewheel sensor
   + Ardumower RTK (ublox F9P)


1. Rename file 'config_example.h' into 'config.h'

2. Configure the options below and finally compile and upload this project.


Adafruit Grand Central M4 NOTE: You have to add SDA, SCL pull-up resistors to the board
and deactivate Due clone reset cicuit (JP13):
https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Adafruit_Grand_Central_M4
NOTE: If you get compilation errors with Adafruit Grand Central M4, you may have to downgrade 'Adafruit SAMD Boards' to version to 1.7.5.

Arduino Due UPLOAD NOTE:

If using Arduino Due 'native' USB port for uploading, choose board 'Arduino Due native' in the
Arduino IDE and COM port 'Arduino Due native port'.

If using Arduino Due 'programming' USB port for uploading, choose board 'Arduino Due programming' in the
Arduino IDE and COM port 'Arduino Due programming port'.

Also, you may choose the serial port below for serial monitor output (CONSOLE).

*/



//#define DRV_SERIAL_ROBOT  1   // Linux (Alfred)
//#define DRV_CAN_ROBOT  1      // Linux (owlRobotics platform)
#define DRV_ARDUMOWER     1   // keep this for Ardumower
//#define DRV_SIM_ROBOT     1   // simulation


// ------- Bluetooth4.0/BLE module -----------------------------------
// see Wiki on how to install the BLE module and configure the jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module
//#define ENABLE_PASS   1        // comment out to disable password authentication
#define PASS          123456   // choose password for WiFi/BLE communication (NOTE: has to match the connection password in the App!)

// -------- IMU sensor  ----------------------------------------------
// choose one MPU IMU (make sure to connect AD0 on the MPU board to 3.3v)
// verify in CONSOLE that your IMU was found (you will hear 8 buzzer beeps for automatic calibration at start)
// see Wiki for wiring, how to position the module, and to configure the I2C pullup jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#IMU.2C_sensor_fusion

#define MPU6050
//#define MPU9150
//#define MPU9250   // also choose this for MPU9255
//#define BNO055
//#define ICM20948
#define MPU_ADDR 0x69  // I2C address (0x68 if AD0=LOW, 0x69 if AD0=HIGH)

// imu fifo rate (Hz)
//#define ROBOT_CONTROL_CYCLE 20

// should the mower turn off if IMU is tilt over? (yes: uncomment line, no: comment line)
#define ENABLE_TILT_DETECTION  1

// --------- lift sensor (e.g. Alfred mower) ---------------------------------------------
// should the lift sensor be enabled? (yes: uncomment line, no: comment line)
//#define ENABLE_LIFT_DETECTION  1
// should the lift sensor be used for obstacle avoidance (if not, mower will simply go into error if lifted)
#define LIFT_OBSTACLE_AVOIDANCE 1

////////////////////////////////////////////////////////////////////////////////////////////////////
//Modsection START
//please keep in mind that there are bugs, but if you encounter bugs that stop the operation of mower on an area completely please tell me about it...
#define ROBOT_CONTROL_CYCLE         20    // (ms) cycle time when mowing
#define ROBOT_IDLE_CYCLE            50    // (ms) cycletime when charging
//Experimental Modfunctions/Options/Speeds/Time etc. for different stuff and movement operations. Please read the descriptions. Cheers.
//Mower general times and speeds with their condition parameters, this section is the easiest to be tuned for one´s needs... so feel free and just tune to what you want to see on the lawn
#define MOWSPINUPTIME               7000  // (ms) Adds time to rotate mowingdisc before starting moving, use high value if you enable ESCAPE_LAWN for good reading of idle mow motor RPM
#define OVERLOADSPEED               0.16  // (m/s) if there is a overloadcurrent of a motordriver, mower will use OVERLOADSPEED
#define TRACKSLOWSPEED              0.25  // (m/s) e.g the docking speed or functions of the future
#define NEARWAYPOINTSPEED           0.25  // (m/s) the speed of mower when reaching/leaving a waypoint
#define NEARWAYPOINTDISTANCE        0.50  // (m) the distance of mower if approaching and leaving a waypoint and thus triggers NEARWAYPOINTSPEED of mower
#define FLOATSPEED                  0.25  // (m/s) on GPS FLOAT SOLUTION, mower will use FLOATSPEED
#define SONARSPEED                  0.15  // (m/s) if sonar enabled and used, on sonar trigger mower will use SONARSPEED
#define OBSTACLEAVOIDANCESPEED      0.35  // (m/s) on obstacle evade operations, mower will use OBSTACLEAVOIDANCESPEED
#define DOCKSPEED                   0.60  // (m/s) the speed mower will use to reach the dock path after mowing or when triggered docking
#define DOCKPATHSPEED               0.30  // (m/s) the speed mower will use on the dockpath
#define ESCAPE_REVERSE_WAY          0.40  // (m) distance in meters the mover will drive reverse in escapeReverseOp if obstacle triggered eg. by bumper org gps no speed
#define ESCAPE_FORWARD_WAY          0.35  // (m) distance in meters the mover will drive forward in escapeForwardOp if obstacle triggered by OBSTACLE_ROTATION (no rotation)
#define GLOBALSPEEDLIMIT            true  // if true, MOTOR_MAX_SPEED and MOTOR_MIN_SPEED will limit possible code bugs or inputs directly in motor.cpp
#define MOTOR_MAX_SPEED             0.60  // (m/s) maximum mower speed --> has not much to to with configuration of speeds: acts as a failsafe
#define MOTOR_MIN_SPEED             0.05  // (m/s) minimal mower speed --> has not much to to with configuration of speeds: acts as a failsafe
#define DISTANCE_RAMP               true  // is using NEARWAYPOINTDISTANCE, MOTOR_MIN_SPEED and the actual setspeed to calculate an indirect deceleration ramp to the next waypoint, if this is true, NEARWAYPOINTSPEED in linetracker.cpp is disabled
#define DISTANCE_RAMP_MINSPEED      0.15  // (m/s) this is the ramp minspeed
#define ROTATION_RAMP               true  // uses a ramp for angletotargetfits
#define ROTATION_RAMP_MAX           100   // (deg/s) maximum rotation speed
#define ROTATION_RAMP_MIN           18    // (deg/s) minimum rotation speed
//rotation speeds, also this is easy to tune for your expectations --> going to be changed to angularRamp() with less parameters
//#define DOCKANGULARSPEED            35.0  // (deg/s) the turning rate of mower when docking (no change needed, keep low; tune at your needs if you must) <-- to be removed
#define ROTATETOTARGETSPEED1        65.0  // (deg/s) if angle difference to point is more than ANGLEDIFF1 then this value will be used...   warning, a high value will result in extreme gearmotor stress (test at own risk, 65deg/s is still safe and fast)
#define ROTATETOTARGETSPEED2        40.0  // (deg/s) if angle difference to point is between ANGLEDIFF1 and ANGLEDIFF2 then this value will be used...
#define ROTATETOTARGETSPEED3        25.0  // (deg/s) if angle difference to point is less than ANGLEDIFF2 then this value will be used...
#define ANGLEDIFF1                  30.0  // (deg) if angle to point is more than ANGLEDIFF1              --> ROTATETOTARGETSPEED1 will be used
#define ANGLEDIFF2                  15.0  // (deg) if angle to point is between ANGLEDIFF1 and ANGLEDIFF2 --> ROTATETOTARGETSPEED2 will be used, if it is less ROTATETOTARGETSPEED3 will be used...
#define ANGLEPRECISE                5.0   // (deg) if Angle to point ist within 5deg, mower will continue with Linetracker.cpp Stanleycode and NEARWAYPOINT setup
#define TRANSITION_ANGLE            25    // (deg) if next point is below this angle, mower will not stop completely but makes a transition with TRANSITION_SPEED
#define TRANSITION_SPEED            0.10  // (m/s) speed for transition to next point angle
//use a PID controller for mowmotor to set an RPM instead of PWM? If you use this (there will be a console output with data after 10sec when you activate the mowmotor and this is enabled)
//CONFIG HINT: for the following options it is important if you have mow motor odometrie: USE_MOW_RPM_SET, ADAPTIVE_SPEED_MODE, ESCAPE_LAWN_MODE. If you do not have odometrie: use mode 1 on both cases and set USE_MOW_RPM_SET = false, if you have odometrie use mode 2 on both cases and set USE_MOW_RPM_SET = true (recommended)
#define USE_MOW_RPM_SET             true  // uses RPM instead of PWM to set mowmotor (RPM_FAULT_DETECTION of orig Sunray is best to be set TRUE for all RPM based functions!!)
#define MOWMOTOR_RPM_OFFSET         270   // compensate small RPM offsets (positive if RPM reading is less then RPM setpoint)
#define MOWMOTOR_PID_KP             0.0032// (0.0024 Mowmotordriver DRV8308) (0.0018 JYQD) this is enough to compensate battery drainage over time and have a slow spinup, there may be a controlleroffset to rpm which has to be thought of... RPM_OFFSET
#define MOWMOTOR_PID_KI             0.01  // (0.04 (Mowmotordriver DRV8308/JYQD))
#define MOWMOTOR_PID_KD             0.00  // (0.0000 (Mowmotordriver DRV8308/JYQD))
//adaptive_speed settings on RPM or LOAD of mowmotor (consider if you have mowmotor odometrie)
#define ADAPTIVE_SPEED              true  // if true, mowing speed will adjust to RPM or MOWMOTORPOWER of mow motor on all forward speed mow operations (best)
#define ADAPTIVE_SPEED_MODE         2     // (1, 2) adaptive speed modes. mode 1 - uses mowmotorpower measurement for speeding up/down; mode 2 - uses rpm measurement of mowmotor for speeding up/down (2: best)
#define ADAPTIVE_SPEED_USE_MINSPEED true  // false: MOTOR_MIN_SPEED is used for slowest possible speed, true: ADAPTIVE_SPEED_MINSPEED is used for slowest possible speed
#define ADAPTIVE_SPEED_MINSPEED     0.15  // (m/s) defines the ramp of speed between actual speedstates eg. mowing speed and ADAPTIVE_SPEED_MINSPEED
#define MOWPOWERMAX_AUTO            false // (expirimental) uses highest actual measured mowPower during operation, if true MOWPOWERMAX is ignored
#define MOWPOWERMIN                 10.0  // (Watt) idle Power of Mowmotor or minimum load power of mowmotor, if under this load mower will have maximum speed
#define MOWPOWERMAX                 35.0  // (Watt) max load power of mowmotor, when hitting this load mower will be at minspeed
#define MOW_RPM_DEADZONE             350  // (rpm) rpm deadzone before speed will be reduced, so if MOW_RPM_NORMAL is 3000, mower will start to reduce speed if rpm is below MOW_RPM_NORMAL - MOW_RPM_DEADZONE
#define MOW_RPM_NORMAL              3200  // (3200)(rpm, only used if USE_MOW_RPM_SET = true) mow motor rpm for mowing (WARNING, you should check if your rpm output works as espected! if it does work, but the reading is wrong, you need to calculate the mowmotorticks per second according to realistic rpm!)
#define MOW_RPM_SLOW                3400  // (3400)(rpm, only used if USE_MOW_RPM_SET = true) mow motor rpm when MOW_RPMtr_SLOW (%) of MOW_RPM_NORMAL (rpm) is met. Should be higher or the same as MOW_RPM_NORMAL
#define MOW_RPM_RETRY               3600  // (3600)(rpm, only used if USE_MOW_RPM_SET = true) mow motor rpm when MOW_RPMtr_RETRY (%) of MOW_RPM_NORMAL (rpm) is met. Should be higher or the same as MOW_RPM_SLOW4 (is only used by ESCAPE_LAWN)
#define MOW_PWM_NORMAL              185   // (pwm, only used if USE_MOW_RPM_SET = false) pwm of mow motor for normal mowing
#define MOW_PWM_SLOW                205   // (pwm, only used if USE_MOW_RPM_SET = false) pwm of mow motor when during mowing a keepslow state is triggered. Should be higher or the same as MOW_PWM_NORMAL
#define MOW_PWM_RETRY               235   // (pwm, only used if USE_MOW_RPM_SET = false) pwm of mow motor when during mowing a retryslow state is triggered (after escape lawn). Should be higher or the same as MOW_PWM_SLOW
//ESCAPELAWN operation, mower will backup on a mowmotor stall and retry with given values... after the first try and still stalling, it backs up and waits for mowmotor to recover
#define ESCAPE_LAWN                 true  // mower drives reverse if true and RPM stall of mow motor is detected by more than MOW_RPMtr_STALL(percentage), only available if ENABLE_RPM_FAULT_DETECTION true (best)
#define ESCAPE_LAWN_MODE             2    // (1, 2) escape lawn modes to trigger escape becouse of high motormow load: mode 1 - uses mowmotorpower for triggering, mode 2 - uses rpm for triggering (2: best)
#define CHANGE_SPEED_SET            true  // if enabled and rpm or power triggers escape lawn or a keepslow state, mower will go slow with RETRYSLOWSPEED or KEEPSLOWSPEED with configured RETRYSLOWTIME or KEEPSLOWTIME. Also, in this states MOW_RPM/PWM for _SLOW/RETRY are used for the mowmotor set
#define MOW_POWERtr_STALL           80    // (%, only used if ESCAPE_LAWN_MODE = 1) if power of mowmotor exceeds e.g 90% of MOWPOWERMAX, escapelawn is triggered
#define MOW_POWERtr_SLOW            70    // (%, only used if ESCAPE_LAWN_MODE = 1) if power of mowmotor exceeds e.g 70% of MOWPOWERMAX, keepslow is triggered
#define MOW_RPMtr_STALL             55    // (70)(%, only used if ESCAPE_LAWN_MODE = 2) if RPM of mowmotor stalls under % of MOW_RPM_NORMAL mower will back up with ESCAPELAWNDISTANCE and ESCAPELAWNSPEED and try again
#define MOW_RPMtr_SLOW              70    // (85)(%, only used if ESCAPE_LAWN_MODE = 2) if RPM of mowmotor stalls under % of MOW_RPM_NORMAL mower will trigger a keepSlow state with KEEPSLOWSPEED
#define NO_GPS_OBSTACLE             true  // ignore gps obstacle when reversing with escapeLawn?
#define NO_GPS_OBSTACLE_DEADTIME    7000  // (ms) time of gps obstacle ignorance after escapeLawnOp 
                          //test
#define TEST_WAIT_BEFORE_REVERSE    false
#define MOW_RPMtr_WAITZONE          10
#define MOW_RECOVERY_WAIT_TIME      3000
                          //test
#define ESCAPELAWNSPEED             0.4  // (m/s) speed of mower reverse due to MOW_RPM_STALL trigger
#define ESCAPELAWNDISTANCE          0.45   // (m) distance mower reverses with ESCAPELAWNSPEED due to MOW_RPM_STALL triggered
#define ESCAPELAWNWAITTIME          5000  // (ms) after reversing the second time within ESCAPELAWNTIMER, mower will wait for this time before continue (recover rpm)
#define ESCAPELAWNTIMER             20000 // (ms) timer to reset retries of ESCAPELAWN, if time is met and retries stay under MAXRETRY triggercounter will reset, otherwise there will be an obstacle error
#define ESCAPELAWN_DEADTIME         4000  // (ms) deadtime between allowed ESCAPELAWN triggers (deadtime should be the reverse time of action and be calculated in code :| )
#define ESCAPELAWN_DELAY            100   // (ms) how long mowmotor needs to be under/over the trigger rpm/power before escape lawn triggers actual reversing of mower
#define MAXRETRY                    5     // number of possible retries of ESCAPELAWN within ESCAPELAWNTIMER until there will be an obstacle error or obstacle avoidance
#define MAXRETRYOBSTACLE            true // if true, ESCAPELAWN will trigger Obstacle avoidance when to many MAXRETRY´s and will not trigger an error
#define RETRYSLOWSPEED              0.15  // (ms) if ESCAPELAWN true, mower will back up with ESCAPELAWNSPEED if RPM stall is detected and retry mowing forward with RETRYSPEED until RETRYSLOWTIME is met, then it will continue with normal Speed
#define KEEPSLOWSPEED               0.30  // mower will use this speed if there is a rpm stall (%) of mowingblades defined by MOW_RPMtr_SLOW
#define RETRYSLOWTIME               20000 // (ms) mower will continue slow with RETRY_SLOW_SPEED after ESCAPELAWN operation (reversing triggered by MOW_RPMtr_RETRY (%)) for RETRYSLOWTIME, if a MOW_RPMtr_SLOW will happen again in this retryslowstate, mower resets this timer until no rpm stall occurs in set time
#define KEEPSLOWTIME                20000 // (ms) mower will continue slow with KEEP_SLOW_SPEED for given Time if MOW_RPMtr_SLOW (%) was met... if a MOW_RPMtr_SLOW will happen again during keepslowstate, mower resets this timer until no rpm stall occurs in set time.
//STANLEY options for experiments
#define MAP_STANLEY_CONTROL         true  // if true, stanley values will be mapped linear from MOTOR_MIN_SPEED-->MOTOR_MAX_SPEED with SC_P_*|SC_K_* to actual speedset of mower (recommended if you use high operation speeds)
#define STANLEYNORMALMUL            false // if true, StanleyNormal parameters in Sunray-App will be multiplied by 10! (0,1 = 1) (for testing)
//GPS reboot for docking/undocking/retrydocking (WARNING: you need at least 2 points before and behind the point indexes! Dockpoint indexes start with 0 (first point), last point is your dock)
#define DOCK_GPS_REBOOT             true  // if false and DOCK_POINT_GPS_REBOOT is not 0, mower will wait at the DOCK_POINT_GPS_REBOOT point for fix without rebooting GPS, if false and DOCK_POINT_GPS_REBOOT = 0 this function is off (hopefully)
#define DOCK_GPS_REBOOT_TIME        45000 // (ms) time to wait after rebooting gps for response from ublox?
//#define GPS_STABLETIME              30000 // (ms) GPS Time with fix solution, before continueing from DOCK_POINT_GPS_REBOOT after undock
#define DOCK_POINT_GPS_REBOOT       2     // dockpath point where GPS will be rebootet when undocking/docking/retrydocking <-- not implemented yet
#define DOCK_SLOW_ONLY_LAST_POINTS  2     // trackslow speed will bes used on last dockingpoint and hitting the charger
#define DOCK_REVERSE_POINT          2     // mower will track reverse if freewheel is at backside (Ardumower) or forward (Landrumower) until at that point
//DOCK options keep mower from rotating in dock by all means, needs situation dependent tuning, so be aware!
#define DOCK_NO_ROTATION            true  // if true, rotation for the mower when reaching or leaving the last dockpoint is not allowed! Make sure mower comes just before the dock in a straight line from the point before, then the last point is the dockposition, on that path angular steering is not allowed!
#define DOCK_NO_ROTATION_DISTANCE   2.00  // (m) distance to from prelast dockpoint to stop angular motion of mower, make sure mower comes straight to dock on a nice straight and long line. Angular will be 0 after the mower surpassed the prelast point to charger contacts and travel for DOCK_NO_ROTATION_DISTANCE, make sure the mower has fix and is not dangling around when this function triggers. Tuning is neccessary, but works very well. Mower is able to cover some Meters straight afterwards.
#define DOCK_NO_ROTATION_TIMER      20000 // (ms) if mower doesnt hit the charger in given time after passing dockpoint before last dockpoint(charger), an obstacle will be triggered and mower will reverse to gps reboot point and try again.
#define DOCK_NO_ROTATION_SPEED      0.20  // (m/s) (original it was 0.10, made it changeable...) when angular is not allowed while going to dockposition, this speed is used
//GPS options
#define GPS_COLD_RESET              true  // perform a cold reset of gps receiver instead a warm reset
#define SOLUTION_TIMEOUT            2000  // (ms) communication timeout with ublox, original it is set to 1000 ms ... long cycle times of code like pathfinder will lead to an invalid solution
#define GPS_RESET_WAIT_FIX          true  // reset GPS if mower is in a float timeout for GPS_RESET_WAIT_FIX_TIME?
#define GPS_RESET_WAIT_FIX_TIME     5     // (min) time in minutes to reset gps if mower is in a float timeout without getting fix within GPS_RESET_WAIT_FIX_TIME
#define NO_GPS_SIGNAL_TIMEOUT       2000 // (ms) mower is allowed to run in an GPS invalid state for given time....
#define GPS_JUMP_WAIT               true  // if a GPS jump is detected, should the mower wait?
#define GPS_JUMP_WAIT_TIME          30000 // (ms) waittime if there was a GPS jump
//#define GPS_JUMP_DISTANCE         0.4   // (m) the sudden difference from last GPS position to new position, that will trigger a GPS jump positive <-- to be added
//OTHER
#define FLOAT_CALC                  1     // better float handling?
#define MOW_START_AT_WAYMOW         true  // mowmotor only starts if way state of mower is waymow for the first time, used for mowmotor not starting directly at dock, but at mow area. This is a onetime trigger that only works when mower is (---> undocking ) ---> wayfree ---> mowarea ---> start mowmotor. After this, mowmotor will behave like it used to be
#define WATCHDOG_CONTINUE           false // set true if you have watchdog reset issues, mower will start mowing after rebooting
#define WATCHDOG_TIME               16000 // (ms) resettimer for watchdog trigger
//OBSTACLES
#define OBSTACLE_DETECTION_ROTATION true  // detect robot rotation stuck (requires IMU) (wheel at backside, popo situation)
#define ROTATION_TIMEOUT            5000  // Timeout of rotation movement that triggers an obstacle with escapeReverse (goal for shorter trigger times)
#define ROTATION_TIME               1500  // Time the code expects to rotate without a high IMU yaw difference (yaw difference configuration to be added)
#define CHANGE_OBSTACLE_ROTATION    true  // if true, after 2 times moving forward due to an IMUyaw difference or OVERLOAD_ROTATION with escapeForward because of FREEWHEEL_IS_AT_BACKSIDE, escapeReverse with obstacle is triggered (prevent mower going forward if it can´t rotate and already tried to evade with escapeForward op)
#define OVERLOAD_ROTATION           true  // is usefull if there is alot of grip of wheels which leads to a high current and can result in a motor error overcurrent, before that happens... we want an evasion of the situation. If FREEWHEEL_IS_AT_BACKSIDE is true mower will drive forward on MOTOROVERLOAD if mower state is shouldrotate... otherwise it will trigger an Obstacle and escapeReverse (front snout of mower is hitting something during rotation)
#define MOWER_RADIUS_FRONT          0.35  // (m) setObstaclePosition relevant (measure chassis from gps antenna and add 5cm´s)
#define MOWER_RADIUS_SIDES          0.25  // (m) setObstaclePosition relevant (measure chassis from gps antenna and add 5cm´s)
#define MOWER_RADIUS_BACK           0.45  // (m) setObstaclePosition relevant (measure chassis from gps antenna and add 5cm´s)
//#define OVERLOAD_ROTATION_AMPS      0.4 // if one motor consumes more than _AMPS and robot is rotating, an obstacle is assumed <-- to be revived
#define OVERLOAD_ROTATION_DEADTIME  1000  // (ms) trigger dead time for OVERLOAD_ROTATION (similar like BUMPER_DEADTIME)
#define OBSTACLE_CHAINING           true  // if true, obstacle and obstaclerotation detection is allowed during an obstacle evasion operation (chaining)
//DRIVER (try to fix 8308 driver with pwm (keep FALSE if you have no issues or no DRV8308, this is for experiments only)) ---> to be removed
#define DRV8308_FIX                 false // only for testing, if true and charger is connected, drivers pwm will be 1 for DRVFIXITERATIONS iteration of code everytime DRVFIXTIMER is met
#define DRVFIXITERATIONS            5     // iterations of code for pwm of drivers to be PWM_GEAR and PWM_MOW (below)
#define DRVFIXTIMER                 60000 // (ms) timer for DRV8308_FIX, everytime timer is met, function will run once for DRVFIXITERATIONS and set pwm of drivers to 1
#define PWM_GEAR                    1     // PWM for DRVFIX efforts..
#define PWM_MOW                     1     // PWM for DRVFIX efforts..
//DRIVER (try to fix 8308 driver with moving, only works with "DOCK_RETRY_TOUCH = true"! tested and seems to be a valid workaround for the DRV8308 related dock freeze/stuck issue (keep FALSE if you have no issues or no DRV8308))
#define MOVE_REGULARLY              true  // if true, mower will move in dock in given MOVE_AGAIN_AFTER time, kk: move mower to keep 8308 alive
#define MOVE_AGAIN_AFTER            9.0  // move the mower every xx minutes
#define MOVING_TIME                 400   // time (ms) for moving back
#define SWITCH_OFF_TRACTION_MOTORS  true  // should tractionmotors be disabled in dock?
//LOG
#define OUTPUT_ENABLED              false // output standard Sunray_FW LOG in serial monitor and SDlog
#define CALC_LOOPTIME               false // calc and output the sunray loop time in serial monitor and SDlog
#define TUNING_LOG                  false // outputs valuable var-states of sunray for debugging tuning functions or just for observation and insights
#define TUNING_LOG_TIME             2000  // (ms) periodic output time of TUNING_LOG
#define DEBUG_STATE_ESTIMATOR       false
#define DEBUG_LINETRACKER           false
#define DEBUG_MOTORCONTROL          false
#define DEBUG_BATTERY               false
//Modsection END
////////////////////////////////////////////////////////////////////////////////////////////////////

// ------- SD card map load/resume and logging ---------------------------------
// all serial console output can be logged to a (FAT32 formatted) SD card
// NOTE: for full log file inspections, we will need your sunray.ino.elf binary
// (you can find out the location of the compiled .elf file while compiling with verbose compilation switched on
//  via 'File->Preferences->Full output during compile') - detailed steps here:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#SD_card_module
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#SD_card_logging
#define ENABLE_SD      1                 // enable SD card services (resuming, logging)? (uncomment to activate)
//#define ENABLE_SD_LOG  1                 // enable SD card logging? uncomment to activate (not recommended - WARNING: may slow down system!)
#define ENABLE_SD_RESUME  1              // enable SD card map load/resume on reset? (uncomment to activate)


// ------ odometry -----------------------------------
// values below are for Ardumower chassis and Ardumower motors
// see Wiki on how to configure the odometry divider:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#PCB1.3_odometry_divider
// NOTE: It is important to verify your odometry is working accurate.
// Follow the steps described in the Wiki to verify your odometry returns approx. 1 meter distance for
// driving the same distance on the ground (without connected GPS):
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Odometry_test
// https://forum.ardumower.de/threads/andere-r%C3%A4der-wie-config-h-%C3%A4ndern.23865/post-41732

// NOTE: if using non-default Ardumower chassis and your freewheel is at frontside (gear motors at backside), have may have to swap motor cables,
// more info here: https://wiki.ardumower.de/index.php?title=Ardumower_Chassis_%27mountain_mod%27)
#define FREEWHEEL_IS_AT_BACKSIDE   true  // default Ardumower: true   (change to false, if your freewheel is at frontside) - this is used for obstacle avoidance
#define WHEEL_BASE_CM         37.5       // standard: 36 ;wheel-to-wheel distance (cm)
#define WHEEL_DIAMETER        250        // (250) wheel diameter (mm)

#define ENABLE_ODOMETRY_ERROR_DETECTION  true    // use this to detect odometry erros
//#define ENABLE_ODOMETRY_ERROR_DETECTION  false

// choose ticks per wheel revolution :
// ...for the 36mm diameter motor (blue cap)  https://www.marotronics.de/2-x-36er-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle
//#define TICKS_PER_REVOLUTION  1310 /2   // odometry ticks per wheel revolution

// ...for the 36mm diameter motor (black cap)  https://www.marotronics.de/MA36-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle-ab-2-Stueck-Staffelpreis
// #define TICKS_PER_REVOLUTION 975 /2

// ...for the newer 42mm diameter motor (green connector) https://www.marotronics.de/MA42-DC-Planeten-Getriebemotor-24-Volt-mit-HallIC-30-33-RPM-8mm-Welle-ab-2-Stueck-Staffelpreis
// #define TICKS_PER_REVOLUTION  696 /2   // odometry ticks per wheel revolution

// ...for the older 42mm diameter motor (white connector)  https://wiki.ardumower.de/images/d/d6/Ardumower_chassis_inside_ready.jpg
//#define TICKS_PER_REVOLUTION  1050 /2  // odometry ticks per wheel revolution

// ...for the brushless motor april 2021   https://wiki.ardumower.de/index.php?title=Datei:BLUnit.JPG
#define TICKS_PER_REVOLUTION 1210 / 2 //640_Tree   // 1330/2 (1194/2)  odometry ticks per wheel revolution //MrTrees: 1210

// #define TICKS_PER_REVOLUTION  304 /2    // odometry ticks per wheel revolution (RM18)

//... Mowmotor ticks per revolution:
#define MOTOR_MOW_TICKS_PER_REVOLUTION 6 //ACT BLDC MOTOR from Shop = 6! (DRV8308/JYQD)

//#define TICKS_PER_REVOLUTION  975     // odometry ticks per wheel revolution (owlRobotics platform)
//#define TICKS_PER_REVOLUTION  90     // odometry ticks per wheel revolution (hoverboard motor)

// ----- gear motors --------------------------------------------------
// for brushless motors, study the sections (drivers, adapter, protection etc.) in the Wiki (https://wiki.ardumower.de/index.php?title=DIY_Brushless_Driver_Board)
#define MOTOR_DRIVER_BRUSHLESS   1     // uncomment this for new brushless motor drivers
//#define MOTOR_DRIVER_BRUSHLESS_MOW_DRV8308  1 // uncomment for brushless DRV8308 driver and mowing motor
#define MOTOR_DRIVER_BRUSHLESS_MOW_A4931  1    // uncomment for brushless A3931 driver and mowing motor
//#define MOTOR_DRIVER_BRUSHLESS_MOW_BLDC8015A 1  // uncomment for brushless BLDC8015A driver and mowing motor
//#define MOTOR_DRIVER_BRUSHLESS_MOW_JYQD 1  // uncomment for brushless JYQD driver and mowing motor (https://forum.ardumower.de/threads/jyqd-treiber-und-sunray.24811/)
//#define MOTOR_DRIVER_BRUSHLESS_MOW_OWL 1  // uncomment for brushless owlDrive mowing motor
#define MOTOR_DRIVER_BRUSHLESS_GEARS_DRV8308  1   // uncomment for brushless DRV8308 driver and gear/traction motors
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_A4931  1    // uncomment for brushless A4931 driver and gear/traction motors
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_BLDC8015A 1   // uncomment for brushless BLDC8015A driver and gear/traction motors
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_JYQD 1   // uncomment for brushless JYQD driver and gears/traction motor
//#define MOTOR_DRIVER_BRUSHLESS_GEARS_OWL 1   // uncomment for brushless owlDrive gears/traction motor

#define MOTOR_FAULT_CURRENT     2.5    // gear motors fault current (amps)
#define MOTOR_TOO_LOW_CURRENT   0.005   // gear motor too low current (amps)
#define MOTOR_OVERLOAD_CURRENT  0.8    // gear motors overload current (amps)

#define USE_LINEAR_SPEED_RAMP  true      // use a speed ramp for the linear speed //MrTree overshooting is reduced, deceleration is more agressive: keep enabled! (recommended)
//#define USE_LINEAR_SPEED_RAMP  false      // do not use a speed ramp

// motor speed control (PID coefficients) - these values are tuned for Ardumower motors
// general information about PID controllers: https://wiki.ardumower.de/index.php?title=PID_control
#define MOTOR_PID_LP      0.0    // encoder low-pass filter (use for low encoder tickcount - use zero to disable)
#define MOWMOTOR_PID_LP   0.0    // encoder low-pass filter (use for low encoder tickcount - use zero to disable)
#define MOTOR_PID_KP      2.0    // do not change 2.0 (for non-Ardumower motors or if the motor speed control is too fast you may try: KP=1.0, KI=0, KD=0)
#define MOTOR_PID_KI      0.03   // do not change 0.03
#define MOTOR_PID_KD      0.03   // do not change 0.03
#define MOTOR_PID_LIMIT   255    // output limit - do not change 255
#define MOTOR_PID_RAMP    0      // output derivative limit - do not change 0
#define MOWMOTOR_PID_RAMP 0      // output derivative limit - do not change 0

#define MOTOR_LEFT_SWAP_DIRECTION 1  // uncomment to swap left motor direction
#define MOTOR_RIGHT_SWAP_DIRECTION 1  // uncomment to swap right motor direction


// ----- mowing motor -------------------------------------------------
// NOTE: motor drivers will indicate 'fault' signal if motor current (e.g. due to a stall on a molehole) or temperature is too high for a
// certain time (normally a few seconds) and the mower will try again and set a virtual obstacle after too many tries
// On the other hand, the overload detection will detect situations the fault signal cannot detect: slightly higher current for a longer time

// shall the mow motor be activated for normal operation? Deactivate this option for GPS tests and path tracking running tests
#define ENABLE_MOW_MOTOR true // Default is true, set false for testing purpose to switch off mow motor permanently

#define MOW_FAULT_CURRENT 4.0         // mowing motor fault current (amps)
#define MOW_TOO_LOW_CURRENT 0.005     // mowing motor too low current (amps)
#define MOW_OVERLOAD_CURRENT 2.0      // mowing motor overload current (amps)
#define MOW_OVERLOAD_ERROR_TIME 10000 // mowing motor overload until error time (ms)

// should the direction of mowing motor toggle each start? (yes: true, no: false)
#define MOW_TOGGLE_DIR       true
//#define MOW_TOGGLE_DIR       false

// should the error on motor overload detection be enabled?
#define ENABLE_OVERLOAD_DETECTION  true    // robot will stop on overload
//#define ENABLE_OVERLOAD_DETECTION  false    // robot will slow down on overload

// should the motor fault (error) detection be enabled?
#define ENABLE_FAULT_DETECTION  true
//#define ENABLE_FAULT_DETECTION  false       // use this if you keep getting 'motor error'

#define ENABLE_RPM_FAULT_DETECTION  true     // use mow rpm signal to detect a motor fault (requires mowing motor with rpm output!)
//#define ENABLE_RPM_FAULT_DETECTION  false     // do not use mow rpm signal to detect a motor fault

// should the robot trigger obstacle avoidance on motor errors if motor recovery failed?
#define ENABLE_FAULT_OBSTACLE_AVOIDANCE true


// ------ WIFI module (ESP8266 ESP-01 with ESP firmware 2.2.1) --------------------------------
// NOTE: all settings (maps, absolute position source etc.) are stored in your phone - when using another
// device for the WIFI connection (PC etc.), you will have to transfer those settings (share maps via app,
// re-enter absolute position source etc) !
// see Wiki on how to install the WIFI module and configure the WIFI jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module

#define START_AP  false             // should WIFI module start its own access point?
#define WIFI_IP   192,168,105,55      // choose IP e.g. 192,168,4,1  (comment out for dynamic IP/DHCP) - NOTE: use commans instead of points
#define WIFI_SSID "kai"            // choose WiFi network ID
#define WIFI_PASS "4453761905356165"      // choose WiFi network password

// client (app) --->  server (robot)
#define ENABLE_SERVER true          // must be enabled if robot should act as server (recommended)
//#define ENABLE_SERVER false           // must be disabled if robot should act as client (requires external relay server)

// a relay server allows to access the robot via the Internet by transferring data from app to robot and vice versa (not available yet, highly experimental)
// client (app) --->  relay server  <--- client (robot)
#define ENABLE_RELAY false            // must be enabled to use relay server
#define RELAY_USER "username"         // choose a unique user name/number!
#define RELAY_MACHINE "robot1"        // choose a unique robot id
#define RELAY_HOST "grauonline.net"   // relay server name
#define RELAY_PORT 5000               // relay server port

//#define ENABLE_UDP 1                // enable console for UDP? (for developers only)
#define UDP_SERVER_IP   192,168,178,5     // remote UDP IP and port to connect to
#define UDP_SERVER_PORT 4210

// --------- NTRIP client (linux only, highly experimental) ---------------------------------
//#define ENABLE_NTRIP 1            // must be activated to use Linux NTRIP
#define NTRIP_HOST "195.227.70.119"   // sapos nrw
#define NTRIP_PORT 2101
#define NTRIP_MOUNT "VRS_3_4G_NW"
#define NTRIP_USER "user"
#define NTRIP_PASS "pass"

// ------ MQTT (for ESP8266 only, highly experimental - ENABLE_SERVER must be set to false for this to work :-/ ) -----------------------------
// you can access your robot using a MQTT broker - choose a topic prefix for your robot below - available MQTT topics:
// robot1/cmd           (cmd can be: start, stop, dock)
// robot1/op            (current robot operation as text)
// robot1/gps/sol       (current gps solution as text)
// robot1/gps/pos       (current gps position as text)
// ... lot of other information -> see comm.cpp or check with your MQTT Explorer
//#define ENABLE_MQTT  true                           // start MQTT client?  (true for yes, false for no)
#define ENABLE_MQTT  false
#define MQTT_TOPIC_PREFIX  "Ardumower"                 // the MQTT topic prefix for your robot
#define MQTT_SERVER  "192.168.178.11"                 // your MQTT broker IP or hostname (e.g. "broker.mqtt-dashboard.com")
#define MQTT_PORT  1883
#define MQTT_USER "ardumower"
#define MQTT_PASS "ardumower"

// ------ ultrasonic sensor -----------------------------
// see Wiki on how to install the ultrasonic sensors:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Ultrasonic_sensor

#define SONAR_INSTALLED 1              // uncomment if ultrasonic sensors are installed
#define SONAR_ENABLE true              // should ultrasonic sensor be used?
//#define SONAR_ENABLE false
#define SONAR_TRIGGER_OBSTACLES false     // should sonar be used to trigger obstacles? if not, mower will only slow down
#define SONAR_LEFT_OBSTACLE_CM   30     // slow down or stop mowing operation below this distance (cm)
#define SONAR_CENTER_OBSTACLE_CM 30      // slow down or stop mowing operation below this distance (cm)
#define SONAR_RIGHT_OBSTACLE_CM  30      // slow down or stop mowing operation below this distance (cm)

// ------ rain sensor ----------------------------------------------------------
#define RAIN_ENABLE true                 // if activated, mower will dock when rain sensor triggers
//#define RAIN_ENABLE false

// ------ time-of-flight distance sensor (VL53L0X) -----------------------------
// do not use this sensor (not recommended)
//#define TOF_ENABLE true
#define TOF_ENABLE false
#define TOF_OBSTACLE_CM 100      // stop mowing operation below this distance (cm)


// ------ bumper sensor (bumperduino, freewheel etc.) ----------------
// see Wiki on how to install bumperduino or freewheel sensor:
// https://wiki.ardumower.de/index.php?title=Bumper_sensor
// https://wiki.ardumower.de/index.php?title=Free_wheel_sensor
#define BUMPER_ENABLE true
//#define BUMPER_ENABLE false
#define BUMPER_DEADTIME 500  // bumper dead-time (ms), when bumper has triggered it can not trigger again for BUMPER_DEADTIME --> give mower time to react to a bumper trigger and release the bumper
#define BUMPER_TRIGGER_DELAY  50 // bumper must be active for (ms) to trigger (do only use if you have a weak bumper(springs)), Bumper has already a code iteration delay of some milliseconds
#define BUMPER_MAX_TRIGGER_TIME 20  // if bumpersensor stays permanently triggered mower will stop with bumper error (time in seconds; 0 = disabled)

// ----- battery charging current measurement (INA169) --------------
// the Marotronics charger outputs max 1.5A
// ( https://www.marotronics.de/Ladegeraete-fuer-den-Ardumower-Akkus-24V-mit-Status-LED-auch-fuer-Li-Ion-Akkus )
// so we are using the INA169 in non-bridged mode (max. 2.5A)
// ( https://www.marotronics.de/INA169-Analog-DC-Current-Sensor-Breakout-60V-25A-5A-Marotronics )

//#define CURRENT_FACTOR 0.5     // PCB1.3 (non-bridged INA169, max. 2.5A)
//#define CURRENT_FACTOR 1.0   // PCB1.3 (bridged INA169, max. 5A)
#define CURRENT_FACTOR 1.98   // PCB1.4 (non-bridged INA169, max. 2.5A)
//#define CURRENT_FACTOR 2.941  // PCB1.4 (bridged INA169, max. 5A)

#define GO_HOME_VOLTAGE   22  // start going to dock below this voltage
// The battery will charge if both battery voltage is below that value and charging current is above that value.
#define BAT_FULL_VOLTAGE  28.8  // start mowing again at this voltage
#define BAT_FULL_CURRENT  0.01   // start mowing again below this charging current (amps)
#define BAT_FULL_SLOPE    0.002  // start mowing again below this voltage slope

// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Automatic_battery_switch_off
#define BAT_SWITCH_OFF_IDLE  true         // switch off if idle (JP8 must be set to autom.)
#define BAT_SWITCH_OFF_IDLE_TIME 1000      // MrTree seconds when switching of in idle state
#define BAT_SWITCH_OFF_UNDERVOLTAGE  true  // switch off if undervoltage (JP8 must be set to autom.)
#define BAT_SWITCH_OFF_VOLTAGE 20.0       //switch off mower if under this voltage

// ------ GPS ------------------------------------------
// ------- RTK GPS module -----------------------------------
// see Wiki on how to install the GPS module and configure the jumpers:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#Bluetooth_BLE_UART_module
//
// NOTE: if you experience GPS checksum errors, try to increase UART FIFO size:
// 1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom
// 2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h'
//    (for Adafruit Grand Central M4: 'packages\adafruit\hardware\samd\xxxxx\cores\arduino\RingBuffer.h')
// change:     #define SERIAL_BUFFER_SIZE 128     into:     #define SERIAL_BUFFER_SIZE 1024

//#define GPS_USE_TCP 1                    // comment out for serial gps, activate for TCP client-based GPS
//#define GPS_SKYTRAQ  1               // comment out for ublox gps, uncomment for skytraq gps/NMEA
//#define GPS_LIDAR 1                    // decomment for LiDAR

#define REQUIRE_VALID_GPS  true       // mower will pause if no float and no fix GPS solution during mowing (recommended)
//#define REQUIRE_VALID_GPS  false    // mower will continue to mow if no float or no fix solution (not recommended)

#define GPS_SPEED_DETECTION true  // will detect obstacles via GPS feedback (no speed)  - recommended
#define GPS_SPEED_DEADTIME     5000  // (ms) give mower time after starting moving to look at the stategroundspeed from ublox
#define GPS_NO_SPEED_TIME      1000  // (ms) time for GPS no speed trigger --> obstacle
#define GPS_SPEED_DELAY        2000  // (ms) how long gps speed needs to be too low to trigger gps obstacle

// detect if robot is actually moving (obstacle detection via GPS feedback)
#define GPS_MOTION_DETECTION          true    // if robot is not moving trigger obstacle avoidance (recommended)
//#define GPS_MOTION_DETECTION        false   // ignore if robot is not moving
#define GPS_MOTION_DETECTION_TIMEOUT  3       // (5) timeout for motion (secs)
#define GPS_MOTION_DETECTION_DELTA    0.05     // (0.20) distance mower must be moving in timeouttime until gps no motion is triggered

// configure ublox f9p with optimal settings (will be stored in f9p RAM only)
// NOTE: due to a PCB1.3 bug GPS_RX pin is not working and you have to fix this by a wire:
// https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#PCB1.3_GPS_pin_fix_and_wire_fix   (see 'GPS wire fix')
#define GPS_REBOOT_RECOVERY  true // allow GPS receiver rebooting (recommended - requires GPS wire fix above! otherwise firmware will stuck at boot!)
//#define GPS_REBOOT_RECOVERY   false  // do not allow rebooting GPS receiver (no GPS wire fix required)

#define GPS_CONFIG   true     // configure GPS receiver (recommended - requires GPS wire fix above! otherwise firmware will stuck at boot!)
//#define GPS_CONFIG   false  // do not configure GPS receiver (no GPS wire fix required)

#define GPS_CONFIG_FILTER   true     // use signal strength filter? (recommended to get rid of 'FIX jumps') - adjust filter settings below
//#define GPS_CONFIG_FILTER   false     // use this if you have difficulties to get a FIX solution (uses ublox default filter settings)
#define CPG_CONFIG_FILTER_MINELEV  3   // Min SV elevation degree: 14 (high elevation, less robust), 10 (low elevation, robust)
#define CPG_CONFIG_FILTER_NCNOTHRS 8   // C/N0 Threshold #SVs: 10 (robust), 6 (less robust)
#define CPG_CONFIG_FILTER_CNOTHRS  17   // 30 dbHz (robust), 13 dbHz (less robust)
#define CPG_CONFIG_FILTER_MINCNO   10   // min satelite signal strenght for observing?


// ------ obstacle detection and avoidance  -------------------------

#define ENABLE_PATH_FINDER  true     // path finder calculates routes around exclusions and obstacles
//#define ENABLE_PATH_FINDER  false
#define ALLOW_ROUTE_OUTSIDE_PERI_METER 1.0   // max. distance (m) to allow routing from outside perimeter
// (increase if you get 'no map route' errors near perimeter)

#define OBSTACLE_AVOIDANCE true   // try to find a way around obstacle
//#define OBSTACLE_AVOIDANCE false  // stop robot on obstacle
#define OBSTACLE_DIAMETER 1.2   // choose diameter of obstacles placed in front of robot (m) for obstacle avoidance

#define DISABLE_MOW_MOTOR_AT_OBSTACLE false // switch off mow motor while escape at detected obstacle; set false if mow motor shall not be stopped at detected obstacles

// detect robot being kidnapped? robot will try GPS recovery if distance to tracked path is greater than a certain value
// (false GPS fix recovery), and if that fails go into error
#define KIDNAP_DETECT true  // recommended
//#define KIDNAP_DETECT false
#define KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE 2.0  // allowed path tolerance (m)
#define KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK 5.0  // allowed path tolerance (m)
#define KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK 10.0  // distance from dock in (m) to use KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK

// ------ docking --------------------------------------
// is a docking station available?
#define DOCKING_STATION true   // use this if docking station available and mower should dock automatically
//#define DOCKING_STATION false    // mower will just stop after mowing instead of docking automatically

//#define DOCK_IGNORE_GPS false     // use GPS fix in docking station and IMU for GPS float/invalid
#define DOCK_IGNORE_GPS true     // ignore GPS fix in docking station and use IMU-only (use this if robot gets false GPS fixes in your docking station)

//#define DOCK_AUTO_START true     // robot will automatically continue mowing after docked automatically
#define DOCK_AUTO_START false      // robot will not automatically continue mowing after docked automatically
#define DOCK_FRONT_SIDE true
#define DOCK_RETRY_TOUCH true   // robot will retry touching docking contacts (max. 1cm) if loosing docking contacts during charging
//#define DOCK_RETRY_TOUCH false   // robot will not retry touching docking contacts (max. 1cm) if loosing docking contacts during charging

#define DOCK_UNDOCK_TRACKSLOW_DISTANCE 4 // set distance (m) from dock for trackslow (speed limit)

#define UNDOCK_IGNORE_GPS_DISTANCE 2 // set distance (m) from dock to ignore gps while undocking

// ---- path tracking -----------------------------------

// below this robot-to-target distance (m) a target is considered as reached
#define TARGET_REACHED_TOLERANCE 0.07

// stanley control for path tracking - determines gain how fast to correct for lateral path errors
#define STANLEY_CONTROL_P_NORMAL  3.5   // 3.4 // 3.0 for path tracking control (angular gain) when mowing
#define STANLEY_CONTROL_K_NORMAL  2.0  // 2.3 // 1.0 for path tracking control (lateral gain) when mowing
#define STANLEY_FLOAT_P_NORMAL         1.5
#define STANLEY_FLOAT_K_NORMAL         0.5

#define STANLEY_CONTROL_P_SLOW    1.0   // 1 // 3.0 for path tracking control (angular gain) when docking tracking
#define STANLEY_CONTROL_K_SLOW    0.5  // 0.05 // 0.1 for path tracking control (lateral gain) when mowing or docking
#define STANLEY_FLOAT_P_SLOW           0.5
#define STANLEY_FLOAT_K_SLOW           0.1




// ----- other options --------------------------------------------

// button control (turns on additional features via the POWER-ON button)
#define BUTTON_CONTROL true      // additional features activated (press-and-hold button for specific beep count:
                                 //  1 beep=stop, 6 beeps=start, 5 beeps=dock, 3 beeps=R/C mode ON/OFF, 9 beeps=shutdown, 12 beeps=WiFi WPS
//#define BUTTON_CONTROL false   // additional features deactivated

//#define USE_TEMP_SENSOR true  // only activate if temp sensor (htu21d) connected
#define USE_TEMP_SENSOR false

#define DOCK_OVERHEAT_TEMP 70    // if temperature above this degreeC, mower will dock (MrTree: CPU temp, if no Battempsensor found)
#define DOCK_TOO_COLD_TEMP 5    // if temperature below this degreeC, mower will dock

// activate support for model R/C control?
// use PCB pin 'mow' for R/C model control speed and PCB pin 'steering' for R/C model control steering,
// also connect 5v and GND and activate model R/C control via PCB P20 start button for 3 sec.
// more details: https://wiki.ardumower.de/index.php?title=Ardumower_Sunray#R.2FC_model
#define RCMODEL_ENABLE 1  // set to 1 to turn on R/C control, 0 = off

#define BUZZER_ENABLE 1 // uncomment to disable


// ------ experimental options  -------------------------

// --------- serial monitor output (CONSOLE) ------------------------
// which Arduino Due USB port do you want to your for serial monitor output (CONSOLE)?
// Arduino Due native USB port  => choose SerialUSB
// Arduino Due programming port => choose Serial
#if defined (__arm__) && defined (__SAM3X8E__) // Arduino Due compatible
  #define BOARD "Arduino Due"
  #define CONSOLE SerialUSB   // Arduino Due: do not change (used for Due native USB serial console)
#elif __SAMD51__
  #define BOARD "Adafruit Grand Central M4"
  #define CONSOLE Serial      // Adafruit Grand Central M4
#elif __linux__
  #define BOARD "Linux"
  #define CONSOLE Console
#else
  #ifdef __cplusplus
    #error "ERROR: you need to choose either Arduino Due or Adafruit GCM4 in Arduino IDE"
  #endif
#endif

// ------- serial ports and baudrates---------------------------------
#define CONSOLE_BAUDRATE    115200    // baudrate used for console
//#define CONSOLE_BAUDRATE    921600  // baudrate used for console
#define BLE_BAUDRATE    115200        // baudrate used for BLE
#define BLE_NAME      "Ardumower"     // name for BLE module
#define GPS_BAUDRATE  115200          // baudrate for GPS RTK module
#define WIFI_BAUDRATE 115200          // baudrate for WIFI module
#define ROBOT_BAUDRATE 115200         // baudrate for Linux serial robot (non-Ardumower)

#ifdef __SAM3X8E__                 // Arduino Due
  #define WIFI Serial1
  #define ROBOT Serial1
  #define BLE Serial2
  #define GPS Serial3
  //#define GPS Serial                // only use this for .ubx logs (sendgps.py)
#elif __SAMD51__                      // Adafruit Grand Central M4
  #define WIFI Serial2
  #define ROBOT Serial2
  #define BLE Serial3
  #define GPS Serial4
#elif __linux__
  #define WIFI SerialWIFI
  #define SERIAL_WIFI_PATH "/dev/null"
  #define LINUX_BLE       // comment to disable BLE
  #define BLE SerialBLE
  #define SERIAL_BLE_PATH "/dev/null"    // dummy serial device
  #define GPS SerialGPS
  #define SERIAL_GPS_PATH "/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00"
  #define GPS_HOST "127.0.0.1"
  #define GPS_PORT 2947
  #define ROBOT SerialROBOT
  #define SERIAL_ROBOT_PATH "/dev/ttyUSB1"
  #define NTRIP SerialNTRIP
  #define SERIAL_NTRIP_PATH "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0"
#endif



// ------- I2C addresses -----------------------------
#define DS1307_ADDRESS B1101000
#define AT24C32_ADDRESS B1010000


// ------- PCB1.3/Due settings -------------------------
#define IOREF 3.3   // I/O reference voltage

// ------ hardware pins---------------------------------------
// no configuration needed here

#ifdef __linux__
  // ...
#else
  #define pinMotorEnable  37         // EN motors enable
  #define pinMotorLeftPWM 5          // M1_IN1 left motor PWM pin
  #define pinMotorLeftDir 31         // M1_IN2 left motor Dir pin
  #define pinMotorLeftSense A1       // M1_FB  left motor current sense
  #define pinMotorLeftFault 25       // M1_SF  left motor fault

  #define pinMotorRightPWM  3        // M2_IN1 right motor PWM pin
  #define pinMotorRightDir 33        // M2_IN2 right motor Dir pin
  #define pinMotorRightSense A0      // M2_FB  right motor current sense
  #define pinMotorRightFault 27      // M2_SF  right motor fault

  #define pinMotorMowPWM 2           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
  #define pinMotorMowDir 29          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
  #define pinMotorMowSense A3        // M1_FB  mower motor current sense
  #define pinMotorMowFault 26        // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
  #define pinMotorMowEnable 28       // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
  #define pinMotorMowRpm A11

  #define pinFreeWheel 8             // front/rear free wheel sensor
  #define pinBumperLeft 39           // bumper pins
  #define pinBumperRight 38

  #define pinDropLeft 45           // drop pins                                                                                          Dropsensor - Absturzsensor
  #define pinDropRight 23          // drop pins                                                                                          Dropsensor - Absturzsensor

  #define pinSonarCenterTrigger 24   // ultrasonic sensor pins
  #define pinSonarCenterEcho 22
  #define pinSonarRightTrigger 30
  #define pinSonarRightEcho 32
  #define pinSonarLeftTrigger 34
  #define pinSonarLeftEcho 36
  #define pinPerimeterRight A4       // perimeter
  #define pinDockingReflector A4     // docking IR reflector
  #define pinPerimeterLeft A5

  #define pinLED 13                  // LED
  #define pinBuzzer 53               // Buzzer
  //#define pinTilt 35                 // Tilt sensor (required for TC-G158 board)
  #define pinLift 35                 // Lift sensor (marked as 'Tilt' on PCB1.3/1.4)
  #define pinButton 51               // digital ON/OFF button
  #define pinBatteryVoltage A2       // battery voltage sensor
  #define pinBatterySwitch 4         // battery-OFF switch
  #define pinChargeVoltage A9        // charging voltage sensor
  #define pinChargeCurrent A8        // charge current sensor
  #define pinChargeRelay 50          // charge relay
  #define pinRemoteMow 12          // remote control CHANNEL1 Linear  (Chan 3 at Receiver)
  #define pinRemoteSteer 11          // remote control CHANNEL2 Angular (Chan 1 at Receiver)
  #define pinRemoteSpeed 10        // remote control PIN3 SwitchOut1 for LED LIGHTS Relaisboard K1
  #define pinRemoteSwitch 52        // remote control PIN4 SwitchOut2 for Powerup RC Reveiver over Relaisboard K2
  #define pinVoltageMeasurement A7   // test pin for your own voltage measurements
  #if defined(_SAM3XA_)              // Arduino Due
    #define pinOdometryLeft DAC0     // left odometry sensor
    #define pinOdometryRight CANRX   // right odometry sensor
    #define pinReservedP46 CANTX     // reserved
    #define pinReservedP48 DAC1      // reserved
  #else                              // Adafruit Grand Central M4
    #define pinOdometryLeft A12      // left odometry sensor
    #define pinOdometryRight A14     // right odometry sensor
    #define pinReservedP46 A15       // reserved
    #define pinReservedP48 A13       // reserved
  #endif
  #define pinLawnFrontRecv 40        // lawn sensor front receive
  #define pinLawnFrontSend 41        // lawn sensor front sender
  #define pinLawnBackRecv 42         // lawn sensor back receive
  #define pinLawnBackSend 43         // lawn sensor back sender
  #define pinUserSwitch1 46          // user-defined switch 1
  #define pinUserSwitch2 47          // user-defined switch 2
  #define pinUserSwitch3 48          // user-defined switch 3
  #define pinRain 44                 // rain sensor
  #define pinReservedP14 A7          // reserved
  #define pinReservedP22 A6          // reserved
  #define pinReservedP26 A10         // reserved

  #ifndef SDCARD_SS_PIN
    #if defined(_SAM3XA_)              // Arduino Due
      #define SDCARD_SS_PIN pinUserSwitch1
    #else
      #define SDCARD_SS_PIN 4
    #endif
  #endif
#endif

// IMU (compass/gyro/accel): I2C  (SCL, SDA)
// Bluetooth: Serial2 (TX2, RX2)
// GPS: Serial3 (TX3, RX3)
// WIFI: Serial1 (TX1, RX1)

#define DEBUG(x) CONSOLE.print(x)
#define DEBUGLN(x) CONSOLE.println(x)

#if defined(ENABLE_SD_LOG)
  #define CONSOLE sdSerial
#elif defined(ENABLE_UDP)
  #define CONSOLE udpSerial
#endif


// the following will be used by Arduino library RingBuffer.h - to verify this Arduino library file:
// 1. Arduino IDE->File->Preferences->Click on 'preferences.txt' at the bottom
// 2. Locate file 'packages/arduino/hardware/sam/xxxxx/cores/arduino/RingBuffer.h

#define SERIAL_BUFFER_SIZE 2048

#ifdef BNO055
  #define MPU9250   // just to make mpu driver happy to compile something
#endif

#ifdef ICM20948
  #define MPU9250   // just to make mpu driver happy to compile something
#endif

#ifdef __cplusplus
  #include "udpserial.h"
  #include "sdserial.h"
  #include "src/agcm4/adafruit_grand_central.h"
  #ifdef __linux__
    #include "src/linux/linux.h"
    #include <Console.h>
  #endif
#endif