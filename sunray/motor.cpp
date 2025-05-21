// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "motor.h"
#include "config.h"
#include "helper.h"
#include "robot.h"
#include "Arduino.h"
#include "rcmodel.h"

void Motor::begin() {
  speedUpTrig = false;
  linearCurrSet = 0;
  waitSpinUp = false;
  motorMowRpmCheck = false;//MrTree
  motorMowStallFlag = false;//MrTree
  motorMowRpmError = false;//MrTree
  motorMowSpunUp = false;//MrTree
	pwmMax = MOTOR_PID_LIMIT;
  switchedOn = false;//MrTree
  mowPowerMax = MOWPOWERMAX;  //MrTree
  mowPowerMin = MOWPOWERMIN;  //MrTree
  mowPwm = MOW_PWM_NORMAL;
  mowRpm = MOW_RPM_NORMAL;
  //ticksPerRevolution = 1060/2;
  ticksPerRevolution = TICKS_PER_REVOLUTION;
  mowticksPerRevolution = MOTOR_MOW_TICKS_PER_REVOLUTION; //MrTree
	wheelBaseCm = WHEEL_BASE_CM;    // wheel-to-wheel distance (cm) 36
  wheelDiameter = WHEEL_DIAMETER; // wheel diameter (mm)
  ticksPerCm = ((float)ticksPerRevolution) / (((float)wheelDiameter)/10.0) / PI;    // computes encoder ticks per cm (do not change)  

  motorLeftPID.Kp       = MOTOR_PID_KP/50.0*ROBOT_CONTROL_CYCLE;  // 2.0;  //calculate according to different controlcyletimes
  motorLeftPID.Ki       = MOTOR_PID_KI/50.0*ROBOT_CONTROL_CYCLE;  // 0.03; 
  motorLeftPID.Kd       = MOTOR_PID_KD/50.0*ROBOT_CONTROL_CYCLE;  // 0.03;
  motorLeftPID.reset(); 
  motorRightPID.Kp       = MOTOR_PID_KP/50.0*ROBOT_CONTROL_CYCLE; //motorLeftPID.Kp;
  motorRightPID.Ki       = MOTOR_PID_KI/50.0*ROBOT_CONTROL_CYCLE; //motorLeftPID.Ki;
  motorRightPID.Kd       = MOTOR_PID_KD/50.0*ROBOT_CONTROL_CYCLE; //motorLeftPID.Kd;
  motorRightPID.reset();
  motorMowPID.Kp       = MOWMOTOR_PID_KP/50.0*ROBOT_CONTROL_CYCLE; //MrTree
  motorMowPID.Ki       = MOWMOTOR_PID_KI/50.0*ROBOT_CONTROL_CYCLE; //MrTree
  motorMowPID.Kd       = MOWMOTOR_PID_KD/50.0*ROBOT_CONTROL_CYCLE; //MrTree
  motorMowPID.reset();     	              //MrTree

  motorLeftLpf.Tf = MOTOR_PID_LP;
  motorLeftLpf.reset();
  motorRightLpf.Tf = MOTOR_PID_LP;  
  motorRightLpf.reset();
  motorMowLpf.Tf = MOWMOTOR_PID_LP;
  motorMowLpf.reset();
 

  robotPitch = 0;
  #ifdef MOTOR_DRIVER_BRUSHLESS
    motorLeftSwapDir = true;
  #else
    motorLeftSwapDir = false;  
  #endif
  motorRightSwapDir = false;
  
  // apply optional custom motor direction swapping 
  #ifdef MOTOR_LEFT_SWAP_DIRECTION
    motorLeftSwapDir = !motorLeftSwapDir;
  #endif
  #ifdef MOTOR_RIGHT_SWAP_DIRECTION
    motorRightSwapDir = !motorRightSwapDir;
  #endif

  motorError = false;
  recoverMotorFault = false;
  recoverMotorFaultCounter = 0;
  nextRecoverMotorFaultTime = 0;
  enableMowMotor = ENABLE_MOW_MOTOR; //Default: true
  tractionMotorsEnabled = true;
  
  motorLeftOverload = false;
  motorRightOverload = false;
  motorMowOverload = false; 
  
  odometryError = false;  
  
  motorLeftSense = 0;
  motorRightSense = 0;
  motorMowSense = 0;  
  motorLeftSenseLP = 0;
  motorRightSenseLP = 0;
  motorMowSenseLP = 0;  
  motorsSenseLP = 0;

  activateLinearSpeedRamp = USE_LINEAR_SPEED_RAMP;
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  SpeedFactor = 1; //MrTree
  motorLeftRpmSet = 0;
  motorRightRpmSet = 0;
  motorMowRpmSet = 0;//MrTree
  mowPowerAct = 0; //MrTree
  mowPowerActLP = 0; //MrTree
  motorMowPWMSet = 0;
  motorMowPowerMax = 35; //MrTree  
  motorMowForwardSet = true;
  toggleMowDir = MOW_TOGGLE_DIR;

  lp005 = 0;
  lp01 = 0;
  lp1 = 0;
  lp2 = 0;
  lp3 = 0;
  lp4 = 0;

  currTime = 0;
  deltaControlTimeMs = 0;
  deltaControlTimeSec = 0;
  lastControlTime = 0;
  //nextSenseTime = 0;
  lastMowStallCheckTime = 0; //MrTree
  motorLeftTicks =0;  
  motorRightTicks =0;
  motorMowTicks = 0;
  motorLeftTicksZero=0;
  motorRightTicksZero=0;
  motorLeftPWMCurr =0;    
  motorRightPWMCurr=0; 
  motorMowPWMCurr = 0;
  motorLeftPWMCurrLP = 0;
  motorRightPWMCurrLP=0;   
  motorMowPWMCurrLP = 0;
  
  motorLeftRpmCurr=0;
  motorRightRpmCurr=0;
  motorMowRpmCurr=0;  
  motorLeftRpmLast = 0;
  motorRightRpmLast = 0;
  motorLeftRpmCurrLP = 0;
  motorRightRpmCurrLP = 0;
  motorMowRpmCurrLP = 0;
  motorMowRpmCurrLPFast = 0;  //MrTree
  
  setLinearAngularSpeedTimeoutActive = false;  
  setLinearAngularSpeedTimeout = 0;
  motorMowSpinUpTime = 0;

  mowRPM_RC = 0; //MrTree
  mowPWM_RC = 0;
  
  motorRecoveryState = false;

  retryslow = false; //MrTree
  keepslow = false;   //MrTree
  y_before = 100;    //MrTree
  keepslow_y = 100;  //MrTree
  
  motorMowStall = false;    //MrTree
  motorMowStallDuration = 0;//MrTree

  drvfixtimer = DRVFIXTIMER; //MrTree
  drvfixreset = false; //MrTree
  drvfixcounter = 0; //MrTree
}

void Motor::setMowPwm( int val ){
  CONSOLE.print("Motor::setMowPwm = ");
  CONSOLE.println(val);
  mowPwm = val;
}

bool Motor::waitMowMotor() {
  if (millis() < motor.motorMowSpinUpTime + MOWSPINUPTIME){
    // wait until mowing motor is running or stopping
    if (!buzzer.isPlaying()) buzzer.sound(SND_MOWSTART, true);
    if (!waitSpinUp) CONSOLE.println("Motor::waitMowMotor() trying to wait for mowmotor....");
    waitSpinUp = true;
    motorMowSpunUp = false;
  } else waitSpinUp = false;
  return waitSpinUp;
}

void Motor::speedPWM ( int pwmLeft, int pwmRight, int pwmMow )
{ 
  //Correct Motor Direction
  if (motorLeftSwapDir) pwmLeft *= -1;
  if (motorRightSwapDir) pwmRight *= -1;

  // ensure pwm is lower than Max
  pwmLeft = min(pwmMax, max(-pwmMax, pwmLeft));
  pwmRight = min(pwmMax, max(-pwmMax, pwmRight));  
  pwmMow = min(pwmMax, max(-pwmMax, pwmMow)); 
  
  motorDriver.setMotorPwm(pwmLeft, pwmRight, pwmMow);
}

// linear: m/s
// angular: rad/s
// -------unicycle model equations----------
//      L: wheel-to-wheel distance
//     VR: right speed (m/s)
//     VL: left speed  (m/s)
//  omega: rotation speed (rad/s)
//      V     = (VR + VL) / 2       =>  VR = V + omega * L/2
//      omega = (VR - VL) / L       =>  VL = V - omega * L/2
void Motor::setLinearAngularSpeed(float linear, float angular, bool useLinearRamp){  
  if (waitMowMotor()) {                     
    linear = 0;
    angular = 0; 
  }
  //if (abs(angular) < 0.05) angular = 0;                          //MrTree
  if (abs(linear) < MOTOR_MIN_SPEED * 0.7) linear = 0;                            //MrTree

  if (angular == 0) resetAngularMotionMeasurement();                          //MrTree
  if (linear == 0) resetLinearMotionMeasurement();                            //MrTree
  
  if (angular && linear != 0) {
    setLinearAngularSpeedTimeout = millis() + 1500;                             //Changed to 2500 from 1000, this possibly causes the stop and go if mower is controlled manually over WiFi 
    setLinearAngularSpeedTimeoutActive = true;
  }

  float linearDelta = linear - linearSpeedSet;                                 //need a delta for triggering ramp
  bool speedChange = (abs(linearDelta) >= 2 * MOTOR_MIN_SPEED);
  if (linear > 0) linear = linear * adaptiveSpeed();
  linearCurrSet = linear;                                                     //MrTree safe linear in a global before messing around with it


  //CONSOLE.print("MOTOR START linearCurrSet: "); CONSOLE.println(linearCurrSet);
  //CONSOLE.print("MOTOR START linearDelta: "); CONSOLE.println(linearDelta);

  if (activateLinearSpeedRamp && useLinearRamp && speedChange ){                              //this is a speed ramp for changes in speed during operation, to smooth transitions a little bit. needs to be quick
    //linearSpeedSet = 0.82 * linearSpeedSet + 0.18 * linear;
    if (linearCurrSet > linearSpeedSet) linear = linearSpeedSet + 0.10/deltaControlTimeMs;         //cm/s² acc ramp with estimated controlfreq of 20Hz (50ms), only trigger this if larger speedchanges
    if (linearCurrSet < linearSpeedSet) linear = linearSpeedSet - 0.20/deltaControlTimeMs;        //cm/s² dec ramp with estimated controlfreq of 20Hz (50ms), only trigger this if larger speedchanges                

    //we want to cut the ramp if linear is zero to improve pointprecision
    if (linearCurrSet == 0 && fabs(linear) < 2 * MOTOR_MIN_SPEED) linear = 0;

    //CONSOLE.print("RAMP   linear: "); CONSOLE.println(linear);

  }
 
  linearSpeedSet = linear;
  
  //Global linear Speedlimit 
  if (GLOBALSPEEDLIMIT) {                                                     //MrTree
    if (linearSpeedSet > 0){
      constrain(linearSpeedSet, MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
    }
    if (linearSpeedSet < 0){
      constrain(linearSpeedSet, -MOTOR_MAX_SPEED, -MOTOR_MIN_SPEED);
    }
  }

  angularSpeedSet = angular;   
  float rspeed = linearSpeedSet + angularSpeedSet * (wheelBaseCm /100.0 /2);          
  float lspeed = linearSpeedSet - angularSpeedSet * (wheelBaseCm /100.0 /2);          
  // RPM = V / (2*PI*r) * 60
  //CONSOLE.print("motorRightRpmSet: "); CONSOLE.println(motorRightRpmSet);
  //CONSOLE.print("motorLeftRpmSet: "); CONSOLE.println(motorLeftRpmSet);
  motorRightRpmSet =  rspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;   
  motorLeftRpmSet = lspeed / (PI*(((float)wheelDiameter)/1000.0)) * 60.0;
}

void Motor::enableTractionMotors(bool enable){
  if (enable == tractionMotorsEnabled) return;
  if (enable)
    CONSOLE.println("traction motors enabled");
  else 
    CONSOLE.println("traction motors disabled");
  tractionMotorsEnabled = enable;
}

void Motor::setMowState(bool switchOn){

  if (!USE_MOW_RPM_SET){
    if ((enableMowMotor) && (switchOn)){       
      if (abs(motorMowPWMSet) > 0) {
        CONSOLE.print("Motor::setMowState PWM - ");
        CONSOLE.println("Motor already switched on!");
        return; // mowing motor already switch ON
      }
      switchedOn = true;
      mowPwm = MOW_PWM_NORMAL; //try circumvent the value load of sd, motor.cpp value at begin seems to get overwritten
      motorMowSpinUpTime = millis();
      //waitMowMotor();
      if (!buzzer.isPlaying()) buzzer.sound(SND_MOWSTART, true);
      if (toggleMowDir){
        // toggle mowing motor direction each mow motor start
        motorMowForwardSet = !motorMowForwardSet;
        if (motorMowForwardSet) motorMowPWMSet = mowPwm;  
          else motorMowPWMSet = -mowPwm;  
      } else {      
        motorMowPWMSet = mowPwm;  
      }
      CONSOLE.print("Motor::setMowState ");
      CONSOLE.print(switchOn);
      CONSOLE.print(" PWM: ");
      CONSOLE.println(motorMowPWMSet);
      triggerMotorMowWait();
    } else {
      motorMowSpinUpTime = millis();
      //waitMowMotor();
      motorMowPWMSet = 0;
      motorMowRpmSet = 0;
      switchedOn = false;
      motorMowSpunUp = false;  
      CONSOLE.println("Motor::setMowState PWM OFF");
      triggerMotorMowWait();  
      //motorMowPWMCurr = 0; MrTree when we switchoff, there is no need for mowdriver to go to full break. its no emergency, we can use the ramp 
    }
  }
   
  if (USE_MOW_RPM_SET){ 
    if ((enableMowMotor) && (switchOn)){   
      if (abs(motorMowRpmSet) > 0) {
        CONSOLE.print("Motor::setMowState RPM - ");
        CONSOLE.println("Motor already switched on!");
        return; // mowing motor already switch ON
      }
      switchedOn = true; 
      motorMowSpinUpTime = millis();
      if (!buzzer.isPlaying()) buzzer.sound(SND_MOWSTART, true);
      if (toggleMowDir){
        // toggle mowing motor direction each mow motor start
        motorMowForwardSet = !motorMowForwardSet;
        if (motorMowForwardSet) motorMowRpmSet = mowRpm;  
          else motorMowRpmSet = -mowRpm;  
      } else {      
        motorMowRpmSet = mowRpm;  
      }
      CONSOLE.print("Motor::setMowState ");
      CONSOLE.print(switchOn);
      CONSOLE.print(" RPM: ");
      CONSOLE.println(motorMowRpmSet);
      triggerMotorMowWait();
    } else {
      CONSOLE.println("Motor::setMowState RPM OFF");
      motorMowSpinUpTime = millis();
      //waitMowMotor();
      motorMowRpmSet = 0;
      switchedOn = false;
      motorMowSpunUp = false;
      triggerMotorMowWait();  
    }
  } 
}

void Motor::stopImmediately(bool includeMowerMotor){
  CONSOLE.println("motor.cpp - STOP IMMEDIATELY");
  linearSpeedSet = 0;
  angularSpeedSet = 0;
  motorRightRpmSet = 0;
  motorLeftRpmSet = 0;      
  motorLeftPWMCurr = 0;
  motorRightPWMCurr = 0;
   
  if (includeMowerMotor) {
    //switchedOn = false;
    motorMowSpunUp = false;  
    motorMowPWMSet = 0;
    //motorMowPWMCurr = 0;//MrTree
    motorMowRpmSet = 0;//MrTree
    //motorMowRpmCurr = 0;//MrTree
    setMowState(false);    
  }
  setLinearAngularSpeed(0, 0, false);  
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr);
  // reset PID
  motorLeftPID.reset();
  motorRightPID.reset();
  motorMowPID.reset();//MrTree
  motorLeftLpf.reset();
  motorRightLpf.reset();
  motorMowLpf.reset();
  // reset unread encoder ticks
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);        
}

void Motor::run() {

  currTime = millis();
  deltaControlTimeMs =  currTime - lastControlTime;
  deltaControlTimeSec = (float)deltaControlTimeMs/1000.0;
  lastControlTime = currTime;
  
  if (deltaControlTimeMs < 15) return;


  //Change to stopimmediately!
  if (setLinearAngularSpeedTimeoutActive){
    if (millis() > setLinearAngularSpeedTimeout){
      setLinearAngularSpeedTimeoutActive = false;
      setLinearAngularSpeed(0, 0);
      linearCurrSet = 0;
      linearSpeedSet = 0;
      motorLeftRpmSet = 0;
      motorRightRpmSet = 0;
    }
  }
    
  sense();
  changeSpeedSet();             //MrTree
  checkMotorMowStall();         //MrTree
  drvfix();
  
  //CONSOLE.print("mowSpinUp: ");
  //CONSOLE.println(motor.waitSpinUp);
  // if motor driver indicates a fault signal, try a recovery   
  // if motor driver uses too much current, try a recovery     
  // if there is some error (odometry, too low current, rpm fault), try a recovery 
  if (!recoverMotorFault) {
    bool someFault = ( (checkFault()) || (checkCurrentTooHighError()) || (checkMowRpmFault()) 
                         || (checkOdometryError()) || (checkCurrentTooLowError()) );
    if (someFault){
      stopImmediately(true);
      recoverMotorFault = true;
      nextRecoverMotorFaultTime = millis() + 1000;                  
      motorRecoveryState = true;
    } 
  } 

  // try to recover from a motor driver fault signal by resetting the motor driver fault
  // if it fails, indicate a motor error to the robot control (so it can try an obstacle avoidance)  
  if (nextRecoverMotorFaultTime != 0){
    if (millis() > nextRecoverMotorFaultTime){
      if (recoverMotorFault){
        nextRecoverMotorFaultTime = millis() + 10000;
        recoverMotorFaultCounter++;                                               
        CONSOLE.print("motor fault recover counter ");
        CONSOLE.println(recoverMotorFaultCounter);
        //stopImmediately(true);
        motorDriver.resetMotorFaults();
        recoverMotorFault = false;  
        if (recoverMotorFaultCounter >= 6){ // too many successive motor faults
          //stopImmediately(true);
          CONSOLE.println("ERROR: motor recovery failed");
          recoverMotorFaultCounter = 0;
          motorError = true;
        }
      } else {
        CONSOLE.println("resetting recoverMotorFaultCounter");
        recoverMotorFaultCounter = 0;
        nextRecoverMotorFaultTime = 0;
        motorRecoveryState = false;
      }        
    }
  }
  
  int ticksLeft;
  int ticksRight;
  int ticksMow;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  
  if (motorLeftPWMCurr < 0) ticksLeft *= -1;
  if (motorRightPWMCurr < 0) ticksRight *= -1;
  if (motorMowPWMCurr < 0) ticksMow *= -1;
  
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;
  
  lp005 = 1 - 0.05*deltaControlTimeSec;
  lp01 = 1 - 0.1*deltaControlTimeSec; //Very Very slow
  lp1 = 1 - 1*deltaControlTimeSec; //slow 1 - 0.02 = 0.98
  lp2 = 1 - 2*deltaControlTimeSec; //medium
  lp3 = 1 - 3*deltaControlTimeSec; //semi fast
  lp4 = 1 - 4*deltaControlTimeSec; //fast

  // calculate speed via tick count
  // 2000 ticksPerRevolution: @ 30 rpm  => 0.5 rps => 1000 ticksPerSec
  // 20 ticksPerRevolution: @ 30 rpm => 0.5 rps => 10 ticksPerSec
  motorLeftRpmCurr = 60.0 * ( ((float)ticksLeft) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorRightRpmCurr = 60.0 * ( ((float)ticksRight) / ((float)ticksPerRevolution) ) / deltaControlTimeSec;
  motorMowRpmCurr = 60.0 * ( ((float)ticksMow) / ((float)mowticksPerRevolution) ) / deltaControlTimeSec;
 
  motorLeftRpmCurrLP = lp1 * motorLeftRpmCurrLP + (1.0-lp1) * motorLeftRpmCurr;
  motorRightRpmCurrLP = lp1 * motorRightRpmCurrLP + (1.0-lp1) * motorRightRpmCurr;
  motorMowRpmCurrLP = lp1 * motorMowRpmCurrLP + (1.0-lp1) * motorMowRpmCurr; 
 
  motorMowRpmCurrLPFast = lp3 * motorMowRpmCurrLPFast + (1.0-lp3) * motorMowRpmCurr;

  if (ticksLeft == 0) {
    motorLeftTicksZero++;
    if (motorLeftTicksZero > 2) motorLeftRpmCurr = 0;
  } else motorLeftTicksZero = 0;

  if (ticksRight == 0) {
    motorRightTicksZero++;
    if (motorRightTicksZero > 2) motorRightRpmCurr = 0;
  } else motorRightTicksZero = 0;

  // speed controller
  
  control();    
  motorLeftRpmLast = motorLeftRpmCurr;
  motorRightRpmLast = motorRightRpmCurr;

  //DEBUG OUTPUT
  if (DEBUG_MOTORCONTROL) {
    CONSOLE.println("     motor.cpp --------------------------------> ");
    CONSOLE.println(" ");
    CONSOLE.print("                   PWM (l,r,m) = ");  CONSOLE.print(motorLeftPWMCurr);CONSOLE.print(", ");  CONSOLE.print(motorRightPWMCurr);CONSOLE.print(", ");CONSOLE.println(motorMowPWMCurr);
    CONSOLE.print("       motor*PWMCurrLP (l,r,m) = ");CONSOLE.print(motorLeftPWMCurrLP);CONSOLE.print(", ");CONSOLE.print(motorRightPWMCurrLP);CONSOLE.print(", ");CONSOLE.println(motorMowPWMCurrLP);
    CONSOLE.print("         motor*SenseLP (l,r,m) = ");  CONSOLE.print(motorLeftSenseLP);CONSOLE.print(", ");  CONSOLE.print(motorRightSenseLP);CONSOLE.print(", ");CONSOLE.println(motorMowSenseLP);
    CONSOLE.println("     <------------------------------------------ ");
  }
}  

// check if motor current too high
bool Motor::checkCurrentTooHighError(){
  bool motorLeftFault = (motorLeftSenseLP > MOTOR_FAULT_CURRENT);
  bool motorRightFault = (motorRightSenseLP > MOTOR_FAULT_CURRENT);
  bool motorMowFault = (motorMowSenseLP > MOW_FAULT_CURRENT);
  if (motorLeftFault || motorRightFault || motorMowFault){
    CONSOLE.print("ERROR motor current too high: ");
    CONSOLE.print("  current=");
    CONSOLE.print(motorLeftSenseLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSenseLP);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSenseLP);
    return true;
  } 
  return false; 
}

// check if motor current too low
bool Motor::checkCurrentTooLowError(){

  if  (    ( (abs(motorMowPWMCurr) > 100) && (abs(motorMowPWMCurrLP) > 100) && (motorMowSenseLP < MOW_TOO_LOW_CURRENT)) 
        ||  ( (abs(motorLeftPWMCurr) > 100) && (abs(motorLeftPWMCurrLP) > 100) && (motorLeftSenseLP < MOTOR_TOO_LOW_CURRENT))    
        ||  ( (abs(motorRightPWMCurr) > 100) && (abs(motorRightPWMCurrLP) > 100) && (motorRightSenseLP < MOTOR_TOO_LOW_CURRENT))  ){        
    // at least one motor is not consuming current      
    // first try recovery, then indicate a motor error to the robot control (so it can try an obstacle avoidance)    
    CONSOLE.print("ERROR: motor current too low: PWM (left,right,mow)=");
    CONSOLE.print(motorLeftPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorRightPWMCurr);
    CONSOLE.print(",");
    CONSOLE.print(motorMowPWMCurr);
    CONSOLE.print("  motor*PWMCurrLP (left,right,mow)=");
    CONSOLE.print(motorLeftPWMCurrLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightPWMCurrLP);
    CONSOLE.print(",");
    CONSOLE.print(motorMowPWMCurrLP);
    CONSOLE.print("  average current amps (left,right,mow)=");
    CONSOLE.print(motorLeftSenseLP);
    CONSOLE.print(",");
    CONSOLE.print(motorRightSenseLP);
    CONSOLE.print(",");
    CONSOLE.println(motorMowSenseLP);
    return true;
  }
  return false;
}

// check motor driver (signal) faults
bool Motor::checkFault(){
  bool fault = false;
  bool leftFault = false;
  bool rightFault = false;
  bool mowFault = false;
  if (ENABLE_FAULT_DETECTION){    
    motorDriver.getMotorFaults(leftFault, rightFault, mowFault);
  }
  if (leftFault) {
    CONSOLE.println("Error: motor driver left signaled fault");
    fault = true;
  }
  if  (rightFault) {
    CONSOLE.println("Error: motor driver right signaled fault"); 
    fault = true;
  }
  if (mowFault) {
    CONSOLE.println("Error: motor driver mow signaled fault");
    fault = true;
  }
  return fault;
}

// check odometry errors
bool Motor::checkOdometryError(){
  if (ENABLE_ODOMETRY_ERROR_DETECTION){
    if  (   ( (abs(motorLeftPWMCurr) > 100) && (abs(motorLeftPWMCurrLP) > 100) && (abs(motorLeftRpmCurrLP) < 0.001))    
        ||  ( (abs(motorRightPWMCurr) > 100) && (abs(motorRightPWMCurrLP) > 100) && (abs(motorRightRpmCurrLP) < 0.001))  )
    {               
      // odometry error
      CONSOLE.print("ERROR: odometry error - rpm too low (left, right)=");
      CONSOLE.print(motorLeftRpmCurrLP);
      CONSOLE.print(",");
      CONSOLE.println(motorRightRpmCurrLP);     
      return true;        
    }
  }
  return false;
}

// check motor overload
void Motor::checkOverload(){
  motorLeftOverload = (motorLeftSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorRightOverload = (motorRightSenseLP > MOTOR_OVERLOAD_CURRENT);
  motorMowOverload = (motorMowSenseLP > MOW_OVERLOAD_CURRENT);
  //CONSOLE.print("motorLeftSenseLP    ");CONSOLE.print(motorLeftSenseLP);
  //CONSOLE.print("  motorRightSenseLP   ");CONSOLE.println(motorRightSenseLP);
  if (motorLeftOverload || motorRightOverload || motorMowOverload){                   //MrTree
    if (motorOverloadDuration == 0){
      CONSOLE.print("ERROR motor overload (average current too high) - duration=");
      CONSOLE.print(motorOverloadDuration);
      CONSOLE.print("  avg current amps (left,right,mow)=");
      CONSOLE.print(motorLeftSenseLP);
      CONSOLE.print(",");
      CONSOLE.print(motorRightSenseLP);
      CONSOLE.print(",");
      CONSOLE.println(motorMowSenseLP);
    }
    motorOverloadDuration += 20;     
  } else {
    motorOverloadDuration = 0;
  }
}

// Adaptive_Speed: ramp mowingspeed with mow motor rpm or mow motor power, handles also 3 speed stages (retryslow, keepslow, normal)         
float Motor::adaptiveSpeed(){
  if (ADAPTIVE_SPEED){
    //returns
    if (!switchedOn) {
      return 1;
    }

    //prepare variables
    float x = 0;
    float x1 = 0;
    float x2 = 0;                                                    
    float y = 0;
    float y1 = 0;
    float y2 = 0;

    if (MOWPOWERMAX_AUTO) mowPowerMax = motorMowPowerMax;

    if (ADAPTIVE_SPEED_MODE == 1) {
      x = mowPowerAct * 1000;
      x1 = mowPowerMin * 1000;
      x2 = mowPowerMax * 1000; //reach minspeed before powermax
      //if (x2 < x1) x2 = motorMowPowerMax * 1000;    //safety first
      //y1 = MOTOR_MIN_SPEED/linearCurrSet*100; //linearSpeedSet
      if (ADAPTIVE_SPEED_USE_MINSPEED) y1 = ADAPTIVE_SPEED_MINSPEED/linearCurrSet*100; //linearSpeedSet
      else y1 = MOTOR_MIN_SPEED/linearCurrSet*100; //linearSpeedSet
      y2 = 100; 
      //CONSOLE.print(x);
      x = ((x2 - x1) * sqrt(x / x2)); //test
      //CONSOLE.println(x);
    } 

    if (ADAPTIVE_SPEED_MODE == 2) {
      x = abs(motorMowRpmCurrLPFast) * 1000;
      x1 = (abs(motorMowRpmSet) * (MOW_RPMtr_SLOW+5)/100) * 1000; //add some offset to trigger slow, because we dont want to trigger slowstate but become slow before that happens
      x2 = (abs(motorMowRpmSet) - MOW_RPM_DEADZONE) * 1000;         
      y1 = 100;
      if (ADAPTIVE_SPEED_USE_MINSPEED) y2 = ADAPTIVE_SPEED_MINSPEED/linearCurrSet*100; //linearSpeedSet
      else y2 = MOTOR_MIN_SPEED/linearCurrSet*100; //linearSpeedSet
      if (TEST_WAIT_BEFORE_REVERSE && (abs(motorMowRpmCurrLPFast) > (abs(motorMowRpmSet) * (MOW_RPMtr_STALL)/100)) && (abs(motorMowRpmCurrLPFast) < (abs(motorMowRpmSet) * (MOW_RPMtr_STALL+MOW_RPMtr_WAITZONE)/100))){
        //waitOp.waitTime = 3000;
        triggerWaitCommand(3000);
      }
    }

    y = map(x, x1, x2, y2, y1);
    //CONSOLE.print("y= "); CONSOLE.print(y);CONSOLE.print("   x= "); CONSOLE.print(x); CONSOLE.print(" x1= ");CONSOLE.print(x1); CONSOLE.print(" x2= ");CONSOLE.print(x2);CONSOLE.print(" y2= ");CONSOLE.print(y2);CONSOLE.print(" y1= ");CONSOLE.println(y1);                             
    y = constrain(y, MOTOR_MIN_SPEED/linearCurrSet*100, 100);     //limit val
    
    if (y > y_before) y = y_before + 5*deltaControlTimeSec; //0.2;//y = 0.995 * y_before + 0.005 * y; //if speed was reduced use a ramp for getting faster only (smoothing)
    //if ((speedUpTrig)&&(linear >= linearCurrSet)) linear = 0.998 * linearSpeedSet + 0.002 * linear; //Speed up slow if triggered from adaptive speed function
    y_before = y;
    SpeedFactor = y/100.0;                                        //used for linear modifier as: MOTOR_MIN_SPEED/setSpeed*100 < Speedfactor <= 1

    return SpeedFactor;

  } else {                                                        //MrTree adaptive speed is not activated
    return 1;
  }
}  

void Motor::changeSpeedSet(){

  if (!CHANGE_SPEED_SET) return;
  if (!switchedOn || (millis() < motor.motorMowSpinUpTime + MOWSPINUPTIME)) {
      keepslow = false;
      retryslow = false;
      keepSlowTime = 0;
      retrySlowTime = 0;
      speedUpTrig = false;
      return;
    }

  //prepare variables
  float slowtrig = 0;
  //float retrytrig = 0;
  float controlval = 0;
  int mownormal = 0;
  int mowslow = 0;
  int mowretry = 0;
  int mowset = 0;

  if (USE_MOW_RPM_SET){
    slowtrig = MOW_RPM_NORMAL * MOW_RPMtr_SLOW/100;
    //retrytrig = 0;
    controlval = abs(motorMowRpmCurrLPFast);
    mownormal = MOW_RPM_NORMAL;
    mowslow = MOW_RPM_SLOW;
    mowretry = MOW_RPM_RETRY;
    mowset = motorMowRpmSet;
  } else {
    slowtrig = mowPowerAct;                           //because we are rising with mowpower not falling like rpm, we flip the controlval to be the trigger
    //retrytrig = 0;
    controlval = mowPowerMax * MOW_POWERtr_SLOW/100;  //set the trigger to be the controlval....
    mownormal = MOW_PWM_NORMAL;
    mowslow = MOW_PWM_SLOW;
    mowretry = MOW_PWM_RETRY;
    mowset = motorMowPWMSet;
  }

  if (keepslow && retryslow) keepslow = false;             //reset keepslow because retryslow is prior

  if (controlval < slowtrig){                              //trigger and set timer once, trigger by rpm percentage
    //CONSOLE.print("controlval = ");CONSOLE.print(controlval);CONSOLE.print("slowtrig = ");CONSOLE.println(slowtrig);
    if ((!keepslow) && (!retryslow)){                      //only if not already trigged
      CONSOLE.println("Adaptive_Speed: Keeping slow!");
      keepslow = true;                                          //enable keepslow state
      if (abs(mowset) != mowslow){                              //set the keepslow rpm
        if (USE_MOW_RPM_SET){
          CONSOLE.println("Adaptive_Speed: Using MOW_RPM_SLOW");
          if (motorMowRpmSet > 0) motorMowRpmSet = mowslow;                    //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing?
          if (motorMowRpmSet < 0) motorMowRpmSet = -mowslow;                   //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing?    
        } else {
          CONSOLE.println("Adaptive_Speed: Using MOW_PWM_SLOW");
          if (motorMowPWMSet > 0) motorMowPWMSet = mowslow;                    //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing?
          if (motorMowPWMSet < 0) motorMowPWMSet = -mowslow;                   //set new rpm. should use max pwm, cause no one knows if controller can reach rpmset with drained batterie without testing? 
        }                                                   
      }
    }                                                                  //step out of trigger condition
    if (keepslow) keepSlowTime = millis()+KEEPSLOWTIME;                //set or refresh keepslowtimer
    if (retryslow) retrySlowTime = millis()+RETRYSLOWTIME;             //if we already are in retryslow condition, we refresh the timer of retry also                               
  }                                          
  
  if (retryslow){                                                 //retryslow is triggered by the end of escapelawnop                       
    if (abs(mowset) != mowretry) {
      if (USE_MOW_RPM_SET){
        CONSOLE.println("Adaptive_Speed: Using MOW_RPM_RETRY");
        if (motorMowRpmSet > 0) motorMowRpmSet = mowretry; 
        if (motorMowRpmSet < 0) motorMowRpmSet = -mowretry;
      } else {
        CONSOLE.println("Adaptive_Speed: Using MOW_RPM_RETRY");
        if (motorMowPWMSet > 0) motorMowPWMSet = mowretry; 
        if (motorMowPWMSet < 0) motorMowPWMSet = -mowretry; 
      }                      
    }            
    if (millis() > retrySlowTime) {
      CONSOLE.println("Adaptive_Speed: Retryslow done! Going to Keepslow!");
      //CONSOLE.println("Adaptive_Speed: Speeding up!");
      retryslow = false;
      keepslow = true;
      keepSlowTime = millis()+KEEPSLOWTIME;
    }
  }

  if (keepslow){                                                //keepslow is triggered in this function, retryslow in escapelawnOp   
    if (millis() > keepSlowTime) {
      CONSOLE.println("Adaptive_Speed: Keepslow done!");
      CONSOLE.println("Adaptive_Speed: Speeding up!");
      keepslow = false;                                         //reset keepslow if no rpm stall      
      speedUpTrig = true;
    }
  }

  if (speedUpTrig){
    if (millis() > keepSlowTime + KEEPSLOWTIME) {
      CONSOLE.println("Adaptive_Speed: Speeding up done!");
      CONSOLE.println("Adaptive_Speed: Using MOW_RPM_NORMAL");
      if (USE_MOW_RPM_SET){
        if (motorMowRpmSet > 0) motorMowRpmSet = mownormal;
        if (motorMowRpmSet < 0) motorMowRpmSet = -mownormal;
      } else {
        if (motorMowPWMSet > 0) motorMowPWMSet = mownormal;
        if (motorMowPWMSet < 0) motorMowPWMSet = -mownormal;
      }
      speedUpTrig = false;
    }
  }
}

// check mow motor RPM stalls                                                          
void Motor::checkMotorMowStall(){ 
  if (ESCAPE_LAWN && switchedOn && !RC_Mode && !robotShouldWait()) {
    static unsigned long lastStalltime = 0;
    unsigned long deltaControlTimeSec = (millis() - lastMowStallCheckTime);
    lastMowStallCheckTime = millis();
    
    //test workaround not moving around without switched on mowmotor
    //if switchedOn && (abs(motorMowRpmCurrLP) < 500){
    //  robotShouldWait();
    //  motor.setMowState(true)
    //  return;
    //}
    
    //return
    if (millis() < lastStalltime + BUMPER_DEADTIME || bumper.obstacle()) return;
    if (millis() < linearMotionStartTime + 2000) return;

    if (ESCAPE_LAWN_MODE == 1){
          motorMowStall = (mowPowerAct > mowPowerMax *MOW_POWERtr_STALL/100);
          if (!motorMowSpunUp){
            motorMowSpunUp = true;    
            CONSOLE.println("checkMotorMowStall: Using mow motor power for triggering escape lawn.");
          }
    }

    if (ESCAPE_LAWN_MODE == 2) {  //MrTree in RPM mode we put out some messages with rpm data to use for configuration and information 
      if (!motorMowSpunUp){
        if (!USE_MOW_RPM_SET) motorMowRpmSet = motorMowRpmCurrLPFast;
        if ((abs(motorMowRpmCurrLPFast) < abs(motorMowRpmSet)-150) || (abs(motorMowRpmCurrLPFast) > abs(motorMowRpmSet)+150)){
          motorMowSpunUp = true;    
          CONSOLE.println("checkMotorMowStall: WARNING mow motor did not spun up to RPM set, ignore small values.");
          CONSOLE.println("Increase SPINUPTIME or consider mow motor not reaching or overspinning specific RPM set by more than +- 150 RPM.");
          CONSOLE.print("DATA: SPINUPTIME (ms), driverPWM, mowRPM, mowRPMSet, RPM difference: ");
          CONSOLE.print(MOWSPINUPTIME);
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowPWMCurr));
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowRpmCurrLPFast));
          CONSOLE.print(", ");
          CONSOLE.print(abs(motorMowRpmSet));
          CONSOLE.print(", ");
          CONSOLE.println(abs(motorMowRpmSet)-abs(motorMowRpmCurrLPFast));
        } else {
            motorMowSpunUp = true;
            CONSOLE.println("checkMotorMowStall: Mow motor Spun up!");
            CONSOLE.print("DATA: SPINUPTIME (ms), driverPWM, mowRPM, mowRPMSet: ");
            CONSOLE.print(MOWSPINUPTIME);
            CONSOLE.print(", ");
            CONSOLE.print(abs(motorMowPWMCurr));
            CONSOLE.print(", ");
            CONSOLE.print(abs(motorMowRpmCurrLPFast));
            CONSOLE.print(", ");
            CONSOLE.println(abs(motorMowRpmSet));
        }
      }

      if (USE_MOW_RPM_SET) motorMowStall = (abs(motorMowRpmCurrLPFast) < (abs(MOW_RPM_NORMAL)*MOW_RPMtr_STALL/100));  //LPFast? // changed motorMowRpmSet to mowRpm
      else motorMowStall = (abs(motorMowRpmCurrLPFast) < (abs(motorMowRpmSet)*MOW_RPMtr_STALL/100));
    }
    
    if ((motorMowStall) ){
      lastStalltime = millis();
      if (motorMowStallDuration == 0)CONSOLE.println("checkMotorMowStall:: mow motor stalled!");
      motorMowStallFlag = true;
      motorMowStallDuration += deltaControlTimeSec;     
    } else {
      if (motorMowStallDuration != 0){
        motorMowStallFlag = false;         
        CONSOLE.print("checkMotorMowStall: WARNING mow motor stalled for duration(ms) >= ");
        CONSOLE.println(motorMowStallDuration);
        CONSOLE.print("Data: Trigger RPM, Mow RPM: ");
        CONSOLE.print((abs(MOW_RPM_NORMAL)*MOW_RPMtr_STALL/100));
        CONSOLE.print(", ");
        CONSOLE.println(abs(motorMowRpmSet));      
      }
      motorMowStallDuration = 0;
    }     
    return;
  }
  motorMowStall = false;
  return;
}

void Motor::drvfix(){
  if (DRV8308_FIX) {
    if (drvfixreset) {
      drvfixcounter++;
      if (drvfixcounter >= DRVFIXITERATIONS) {
        speedPWM(0, 0, 0);
        drvfixreset = false;
        drvfixcounter = 0;
      }
    }
    if ((millis() >= drvfixtimer) && (battery.chargerConnected()) && (motorLeftPWMCurr == 0) && (motorRightPWMCurr == 0) && (motorMowPWMCurr == 0)){
      drvfixtimer = millis() + DRVFIXTIMER;
      speedPWM(PWM_GEAR, PWM_GEAR, PWM_MOW);
      drvfixreset = true;
    }  
    return;
  } else {
    return;
  }
}

bool Motor::checkMowRpmFault(){
  if (ENABLE_RPM_FAULT_DETECTION){
    if  ( (abs(motorMowPWMCurr) > 100) && (abs(motorMowPWMCurrLP) > 100) && (abs(motorMowRpmCurrLP) < 10.0)) {        
      CONSOLE.print("ERROR: mow motor, average rpm too low: pwm=");
      CONSOLE.print(motorMowPWMCurr);
      CONSOLE.print("  pwmLP=");      
      CONSOLE.print(motorMowPWMCurrLP);      
      CONSOLE.print("  rpmLP=");
      CONSOLE.print(motorMowRpmCurrLP);
      CONSOLE.println("  (NOTE: choose ENABLE_RPM_FAULT_DETECTION=false in config.h, if your mowing motor has no rpm sensor!)");
      return true;
    }
  }  
  return false;
}

// measure motor currents
void Motor::sense(){
  motorDriver.getMotorCurrent(motorLeftSense, motorRightSense, motorMowSense);

  motorRightSenseLP = lp2 * motorRightSenseLP + (1.0-lp2) * motorRightSense;
  motorLeftSenseLP = lp2 * motorLeftSenseLP + (1.0-lp2) * motorLeftSense;
  motorMowSenseLP = lp2 * motorMowSenseLP + (1.0-lp2) * motorMowSense; 

  motorsSenseLP = motorRightSenseLP + motorLeftSenseLP + motorMowSenseLP;

  motorRightPWMCurrLP = lp01 * motorRightPWMCurrLP + (1.0-lp01) * ((float)motorRightPWMCurr);
  motorLeftPWMCurrLP = lp01 * motorLeftPWMCurrLP + (1.0-lp01) * ((float)motorLeftPWMCurr);  
  motorMowPWMCurrLP = lp01 * motorMowPWMCurrLP + (1.0-lp01) * ((float)motorMowPWMCurr); 
 
  // compute normalized current (normalized to 1g gravity)
  //float leftAcc = (motorLeftRpmCurr - motorLeftRpmLast) / deltaControlTimeSec;
  //float rightAcc = (motorRightRpmCurr - motorRightRpmLast) / deltaControlTimeSec;
  float cosPitch = cos(robotPitch); 
	float pitchfactor;
  float robotMass = 1.0;
	// left wheel friction
	if (  ((motorLeftPWMCurr >= 0) && (robotPitch <= 0)) || ((motorLeftPWMCurr < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch; // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch;  // increase by angle
	motorLeftSenseLPNorm = abs(motorLeftSenseLP) * robotMass * pitchfactor;  
	// right wheel friction
	if (  ((motorRightPWMCurr >= 0) && (robotPitch <= 0)) || ((motorRightPWMCurr < 0) && (robotPitch >= 0)) )
		pitchfactor = cosPitch;  // decrease by angle
	else 
		pitchfactor = 2.0-cosPitch; // increase by angle
  motorRightSenseLPNorm = abs(motorRightSenseLP) * robotMass * pitchfactor; 

  //////////////////////////////// MrTree need more data
  mowPowerAct = battery.batteryVoltage * motorMowSense;             //actual mow motor Power
  mowPowerActLP = battery.batteryVoltage * motorMowSenseLP;
  motorRightPowerAct = battery.batteryVoltage * motorRightSenseLP;      //actual right motor power
  motorLeftPowerAct = battery.batteryVoltage * motorLeftSenseLP;        //actual left motor power
  motorMowPowerMax = max(motorMowPowerMax, mowPowerActLP);              //save mow motor max power during mower activated
  motorRightPowerMax = max(motorRightPowerMax, motorRightPowerAct);   //save right motor max power during mower activated
  motorLeftPowerMax = max(motorLeftPowerMax, motorLeftPowerAct);      //save left motor max power during mower activated
  ///////////////////////////////
  checkOverload();  
}


void Motor::control(){  

  motorLeftPID.Kp       = MOTOR_PID_KP/50.0*robot_control_cycle;  // 2.0;  //calculate according to different controlcyletimes
  motorLeftPID.Ki       = MOTOR_PID_KI/50.0*robot_control_cycle;  // 0.03; 
  motorLeftPID.Kd       = MOTOR_PID_KD/50.0*robot_control_cycle;  // 0.03;
  //motorLeftPID.reset(); 
  motorRightPID.Kp       = MOTOR_PID_KP/50.0*robot_control_cycle; //motorLeftPID.Kp;
  motorRightPID.Ki       = MOTOR_PID_KI/50.0*robot_control_cycle;
  motorRightPID.Kd       = MOTOR_PID_KD/50.0*robot_control_cycle; //motorLeftPID.Kd;
  //motorRightPID.reset();
  motorMowPID.Kp       = MOWMOTOR_PID_KP/50.0*robot_control_cycle; //MrTree
  motorMowPID.Ki       = MOWMOTOR_PID_KI/50.0*robot_control_cycle; //MrTree
  motorMowPID.Kd       = MOWMOTOR_PID_KD/50.0*robot_control_cycle; //MrTree
  //motorMowPID.reset();     	              //MrTree
 
  //########################  Calculate PWM for left driving motor ############################
  //CONSOLE.print("motorLeftLpf(motorLeftRpmCurr):  ");CONSOLE.println(valuesome);  
  //CONSOLE.print("motorLeftRpmSet:                 ");CONSOLE.println(motorLeftRpmSet);
  //CONSOLE.print("motorLeftRpmCurr:                ");CONSOLE.println(motorLeftRpmCurr);
  //CONSOLE.print("pwmMax:                          ");CONSOLE.println(pwmMax);
  //CONSOLE.print("motorLeftPID.Kp:                 ");CONSOLE.println(motorLeftPID.Kp);
  //CONSOLE.print("motorRightPID.Ki:                ");CONSOLE.println(motorRightPID.Ki);
  //CONSOLE.print("motorRightPID.Kd:                ");CONSOLE.println(motorRightPID.Kd);
  
  motorLeftPID.TaMax = 0.25;
  //motorLeftPID.x = motorLeftLpf(motorLeftRpmCurr); //gives a NaN after some time..................
  motorLeftPID.x = motorLeftRpmCurr;
  motorLeftPID.w = motorLeftRpmSet;
  motorLeftPID.y_min = -pwmMax;
  motorLeftPID.y_max = pwmMax;
  motorLeftPID.max_output = pwmMax;
  motorLeftPID.output_ramp = MOTOR_PID_RAMP;
  motorLeftPID.compute();
  //CONSOLE.print("motorLeftPID.y:                  ");CONSOLE.println(motorLeftPID.y);
  motorLeftPWMCurr = motorLeftPWMCurr + motorLeftPID.y;
  //CONSOLE.print("pwmcurr:  ");CONSOLE.println(motorLeftPWMCurr);  
  //if (motorLeftRpmSet > 0) motorLeftPWMCurr = min( max(0, (int)motorLeftPWMCurr), pwmMax); // 0.. pwmMax  //MrTree Bugfix deleted =
  //if (motorLeftRpmSet < 0) motorLeftPWMCurr = max(-pwmMax, min(0, (int)motorLeftPWMCurr));  // -pwmMax..0

  if (motorLeftRpmSet > 0) motorLeftPWMCurr = constrain(motorLeftPWMCurr,20,255); // 0.. pwmMax  //MrTree Bugfix deleted =
  if (motorLeftRpmSet < 0) motorLeftPWMCurr = constrain(motorLeftPWMCurr,-255,-20);  // -pwmMax..0
  //if (abs(motorLeftPWMCurr) < 10) motorLeftPWMCurr = 0;
  //########################  Calculate PWM for right driving motor ############################
  
  motorRightPID.TaMax = 0.25;
  //motorRightPID.x = motorRightLpf(motorRightRpmCurr); //gives a NaN after some time..................
  motorRightPID.x = motorRightRpmCurr;
  motorRightPID.w = motorRightRpmSet;
  motorRightPID.y_min = -pwmMax;
  motorRightPID.y_max = pwmMax;
  motorRightPID.max_output = pwmMax;
  motorRightPID.output_ramp = MOTOR_PID_RAMP;
  motorRightPID.compute();
  motorRightPWMCurr = motorRightPWMCurr + motorRightPID.y;

  //if (motorRightRpmSet > 0) motorRightPWMCurr = min( max(0, (int)motorRightPWMCurr), pwmMax);  // 0.. pwmMax //MrTree Bugfix deleted =
  //if (motorRightRpmSet < 0) motorRightPWMCurr = max(-pwmMax, min(0, (int)motorRightPWMCurr));   // -pwmMax..0  

  if (motorRightRpmSet > 0) motorRightPWMCurr = constrain(motorRightPWMCurr,20,255);  // 0.. pwmMax //MrTree Bugfix deleted =
  if (motorRightRpmSet < 0) motorRightPWMCurr = constrain(motorRightPWMCurr,-255,-20);   // -pwmMax..0  
  //if (abs(motorRightPWMCurr) < 10) motorRightPWMCurr = 0;
  //########################  Calculate PWM for mow motor ############################

  if (USE_MOW_RPM_SET) {                                                               
    if ((mowRPM_RC != 0)&&(RC_Mode)) {
      if (motorMowRpmSet < 0)motorMowRpmSet = -mowRPM_RC; //MrTree
      if (motorMowRpmSet > 0)motorMowRpmSet = mowRPM_RC; //MrTree
    }
    
    float comp = MOWMOTOR_RPM_OFFSET;
    if (motorMowRpmSet < 0) comp = -comp;
    
    motorMowPID.TaMax = 0.25;
    //motorMowPID.x = motorMowLpf(motorMowRpmCurr) - comp; //gives a NaN after some time..................
    motorMowPID.x = motorMowRpmCurr - comp;
    motorMowPID.w = motorMowRpmSet;
    motorMowPID.y_min = -pwmMax;
    motorMowPID.y_max = pwmMax;
    motorMowPID.max_output = pwmMax;
    motorMowPID.output_ramp = MOWMOTOR_PID_RAMP;
    motorMowPID.compute();
    motorMowPWMCurr = motorMowPWMCurr + motorMowPID.y;

    //if (motorMowRpmSet > 0) motorMowPWMCurr = min( max(0, (int)motorMowPWMCurr), pwmMax);  // 0.. pwmMax
    //if (motorMowRpmSet < 0) motorMowPWMCurr = max(-pwmMax, min(0, (int)motorMowPWMCurr));   // -pwmMax..0  

    if (motorMowRpmSet > 0) motorMowPWMCurr = constrain(motorMowPWMCurr,20,255);  // 0.. pwmMax
    if (motorMowRpmSet < 0) motorMowPWMCurr = constrain(motorMowPWMCurr,-255,-20);   // -pwmMax..0 

    if (motorMowRpmSet == 0){ //we use simple low pass when stopping mow motor
      motorMowPWMCurr = lp1 * motorMowPWMCurr + (1 - lp1) * motorMowPWMSet;
      if (abs(motorMowRpmCurrLPFast) < 400) motorMowPWMCurr = 0;    
    }      
  } else { //if !USE_RPM_SET
    if (mowPWM_RC != 0 && RC_Mode) {
      if (motorMowPWMSet < 0)motorMowPWMSet = -mowPWM_RC; //MrTree
      if (motorMowPWMSet > 0)motorMowPWMSet = mowPWM_RC; //MrTree
    } 
    motorMowPWMCurr = lp4 * motorMowPWMCurr + (1 - lp4) * motorMowPWMSet;     //MrTree slightly increased spinuprate (0.99|0.01) before)
    if (motorMowPWMCurr > 0) motorMowPWMCurr = constrain(motorMowPWMCurr,20,255);  // 0.. pwmMax
    if (motorMowPWMCurr < 0) motorMowPWMCurr = constrain(motorMowPWMCurr,-255,-20);   // -pwmMax..0 
  }                                                                                                            //MrTree
  
  //set PWM for all motors
  if (!tractionMotorsEnabled){
    motorLeftPWMCurr = motorRightPWMCurr = 0;
    //CONSOLE.println("traction off!");
  }
  //CONSOLE.println();
  //CONSOLE.print("deltaTime ");CONSOLE.print(deltaControlTimeMs);CONSOLE.print("   motorLeftPWMCurr ");CONSOLE.print(motorLeftPWMCurr);CONSOLE.print("   motorRightPWMCurr ");CONSOLE.println(motorRightPWMCurr);
  speedPWM(motorLeftPWMCurr, motorRightPWMCurr, motorMowPWMCurr); 
}

void Motor::dumpOdoTicks(int seconds){
  int ticksLeft=0;
  int ticksRight=0;
  int ticksMow=0;
  motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
  motorLeftTicks += ticksLeft;
  motorRightTicks += ticksRight;
  motorMowTicks += ticksMow;
  CONSOLE.print("t=");
  CONSOLE.print(seconds);
  CONSOLE.print("  ticks Left=");
  CONSOLE.print(motorLeftTicks);  
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightTicks);             
  CONSOLE.print("  current Left=");
  CONSOLE.print(motorLeftSense);
  CONSOLE.print("  Right=");
  CONSOLE.print(motorRightSense);
  CONSOLE.println();               
}

void Motor::test(){
  CONSOLE.println("motor test - 10 revolutions");
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  unsigned long nextInfoTime = 0;
  int seconds = 0;
  int pwmLeft = 200;
  int pwmRight = 200; 
  bool slowdown = true;
  unsigned long stopTicks = ticksPerRevolution * 10;
  unsigned long nextControlTime = 0;
  while (motorLeftTicks < stopTicks || motorRightTicks < stopTicks){
    if (millis() > nextControlTime){
      nextControlTime = millis() + 20;
      if ((slowdown) && ((motorLeftTicks + ticksPerRevolution  > stopTicks)||(motorRightTicks + ticksPerRevolution > stopTicks))){  //Letzte halbe drehung verlangsamen
        pwmLeft = pwmRight = 20;
        slowdown = false;
      }    
      if (millis() > nextInfoTime){      
        nextInfoTime = millis() + 1000;            
        dumpOdoTicks(seconds);
        seconds++;      
      }    
      if(motorLeftTicks >= stopTicks)
      {
        pwmLeft = 0;
      }  
      if(motorRightTicks >= stopTicks)
      {
        pwmRight = 0;      
      }
      
      speedPWM(pwmLeft, pwmRight, 0);
      sense();
      //delay(50);         
      watchdogReset();     
      robotDriver.run();
    }
  }  
  speedPWM(0, 0, 0);
  CONSOLE.println("motor test done - please ignore any IMU/GPS errors");
}

void Motor::plot(){
  CONSOLE.println("motor plot (left,right,mow) - NOTE: Start Arduino IDE Tools->Serial Plotter (CTRL+SHIFT+L)");
  delay(5000);
  CONSOLE.println("pwmLeft,pwmRight,pwmMow,ticksLeft,ticksRight,ticksMow");
  motorLeftTicks = 0;  
  motorRightTicks = 0;  
  motorMowTicks = 0;
  int pwmLeft = 0;
  int pwmRight = 0; 
  int pwmMow = 0;
  int cycles = 0;
  int acceleration = 1;
  bool forward = true;
  unsigned long nextPlotTime = 0;
  unsigned long stopTime = millis() + 1 * 60 * 1000;
  unsigned long nextControlTime = 0;

  while (millis() < stopTime){   // 60 seconds...
    if (millis() > nextControlTime){
      nextControlTime = millis() + 20; 

      int ticksLeft=0;
      int ticksRight=0;
      int ticksMow=0;
      motorDriver.getMotorEncoderTicks(ticksLeft, ticksRight, ticksMow);  
      motorLeftTicks += ticksLeft;
      motorRightTicks += ticksRight;
      motorMowTicks += ticksMow;

      if (millis() > nextPlotTime){ 
        nextPlotTime = millis() + 100;
        CONSOLE.print(300+pwmLeft);
        CONSOLE.print(",");  
        CONSOLE.print(300+pwmRight);
        CONSOLE.print(",");
        CONSOLE.print(pwmMow);
        CONSOLE.print(",");        
        CONSOLE.print(300+motorLeftTicks);    
        CONSOLE.print(",");
        CONSOLE.print(300+motorRightTicks);
        CONSOLE.print(",");
        CONSOLE.print(motorMowTicks);        
        CONSOLE.println();
        motorLeftTicks = 0;
        motorRightTicks = 0;
        motorMowTicks = 0;      
      }

      speedPWM(pwmLeft, pwmRight, pwmMow);
      if (pwmLeft >= 255){
        forward = false;
        cycles++; 
      }      
      if (pwmLeft <= -255){
        forward = true;
        cycles++;               
      } 
      if ((cycles == 2) && (pwmLeft >= 0)) {
        if (acceleration == 1) acceleration = 20;
          else acceleration = 1;
        cycles = 0;
      }         
      if (forward){
        pwmLeft += acceleration;
        pwmRight += acceleration;
        pwmMow += acceleration;
      } else {
        pwmLeft -= acceleration;
        pwmRight -= acceleration;
        pwmMow -= acceleration;
      }
      pwmLeft = min(255, max(-255, pwmLeft));
      pwmRight = min(255, max(-255, pwmRight));          
      pwmMow = min(255, max(-255, pwmMow));                
    }  
    //sense();
    //delay(10);
    watchdogReset();     
    robotDriver.run(); 
  }
  speedPWM(0, 0, 0);
  CONSOLE.println("motor plot done - please ignore any IMU/GPS errors");
}
