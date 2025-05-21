// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#ifndef MOTOR_H
#define MOTOR_H

#include "pid.h"
#include "lowpass_filter.h"

// selected motor
enum MotorSelect {MOTOR_LEFT, MOTOR_RIGHT, MOTOR_MOW} ;
typedef enum MotorSelect MotorSelect;

extern unsigned int robot_control_cycle;

class Motor {
  public:
    bool waitSpinUp;
    bool motorMowStallFlag;//MrTree
    bool speedUpTrig;
    bool switchedOn;    //MrTree
    int mowRPM_RC; //MrTree
    int mowPWM_RC;
    bool motorMowRpmError;//MrTree
    float robotPitch;  // robot pitch (rad)
    float wheelBaseCm;  // wheel-to-wheel diameter
    int wheelDiameter;   // wheel diameter (mm)
    int ticksPerRevolution; // ticks per revolution
    int mowticksPerRevolution; //MrTree
    float ticksPerCm;  // ticks per cm
    bool activateLinearSpeedRamp;  // activate ramp to accelerate/slow down linear speed?
    bool toggleMowDir; // toggle mowing motor direction each mow motor start?    
    bool motorLeftSwapDir;
    bool motorRightSwapDir;
    bool motorError;
    bool motorLeftOverload; 
    bool motorRightOverload; 
    bool motorMowOverload;
    bool motorMowStall;       //MrTree has RPM of mowmotor Stalled?
    bool tractionMotorsEnabled;       
    bool enableMowMotor;
    bool odometryError;       
    unsigned long motorOverloadDuration; // accumulated duration (ms)
    unsigned long motorMowStallDuration; //MrTree RPM of mowmotor stalled duration (ms)
    int pwmMax;
    int mowPwm; 
    bool motorMowSpunUp; //MrTree 
    float mowRpm;//mrTree
    float mowMotorCurrentAverage;
    float mowPowerAct; //MrTree
    float mowPowerActLP; //MrTree
    float mowPowerMax; //MrTree
    float mowPowerMin; //MrTree
    float motorMowPowerMax; //MrTree
    float motorLeftPowerAct; //MrTree
    float motorLeftPowerMax = 0; //MrTree
    float motorRightPowerAct; //MrTree
    float motorRightPowerMax = 0; //MrTree
    float currentFactor;
    float SpeedFactor; //MrTree
    bool keepslow;   //MrTree
    bool retryslow; //MrTree
    float y_before;//MrTree
    float keepslow_y;//MrTree
    bool pwmSpeedCurveDetection;
    unsigned long motorLeftTicks;
    unsigned long motorRightTicks;
    unsigned long motorMowTicks;
    int motorMowPWMCurr;
    bool motorMowRpmCheck; //MrTree    
    float motorMowRpmSet;//MrTree
    float linearCurrSet;// MrTree helper  
    float linearSpeedSet; // m/s
    float angularSpeedSet; // rad/s
    float motorLeftSense; // left motor current (amps)
    float motorRightSense; // right  motor current (amps)
    float motorMowSense;  // mower motor current (amps)         
    float motorLeftSenseLP; // left motor current (amps, low-pass)
    float motorRightSenseLP; // right  motor current (amps, low-pass)
    float motorMowSenseLP;  // mower motor current (amps, low-pass)       
    float motorsSenseLP; // all motors current (amps, low-pass)
    float motorLeftSenseLPNorm;
    float motorRightSenseLPNorm;
    unsigned long motorMowSpinUpTime;
    unsigned long keepSlowTime; //MrTree adaptive_Speed keep slow speed after RPM stall of mow motor
    unsigned long retrySlowTime;  //MrTree
    bool motorRecoveryState;
    bool motorMowForwardSet;
    PID motorLeftPID;
    PID motorRightPID;
    PID motorMowPID;       //MrTree    
    LowPassFilter motorLeftLpf;
    LowPassFilter motorRightLpf;
    LowPassFilter motorMowLpf; //MrTree 
    void begin();
    void run();      
    void test();
    void plot();
    void enableTractionMotors(bool enable);
    void setLinearAngularSpeed(float linear, float angular, bool useLinearRamp = true);
    void setMowState(bool switchOn); 
    void setMowPwm( int val );
    bool waitMowMotor();  
    void stopImmediately(bool includeMowerMotor);    
     
  protected:
    float lp005;
    float lp01; //Very very slow
    float lp1;
    float lp2;
    float lp3;
    float lp4;

    float motorLeftRpmSet;                //set speed
    float motorRightRpmSet;
    
    float motorMowPWMSet;  
    float motorLeftRpmCurr;
    float motorRightRpmCurr;
    float motorMowRpmCurr;    
    float motorLeftRpmCurrLP;
    float motorRightRpmCurrLP;    
    float motorMowRpmCurrLP;
    float motorMowRpmCurrLPFast;    
    float motorLeftRpmLast;
    float motorRightRpmLast;
    int motorLeftPWMCurr;
    int motorRightPWMCurr;    
    float motorMowPWMCurrLP; 
    float motorLeftPWMCurrLP;
    float motorRightPWMCurrLP;
    unsigned long currTime;
    unsigned long deltaControlTimeMs;
    float deltaControlTimeSec;    
    unsigned long lastControlTime;    
    //unsigned long nextSenseTime;
    unsigned long lastMowStallCheckTime;  //MrTree
    unsigned long drvfixtimer;            //MrTree 
    bool drvfixreset;                     //MrTree 
    unsigned int drvfixcounter;      
    bool recoverMotorFault;
    int recoverMotorFaultCounter;
    unsigned long nextRecoverMotorFaultTime;
    int motorLeftTicksZero;    
    int motorRightTicksZero;  
    bool setLinearAngularSpeedTimeoutActive;
    unsigned long setLinearAngularSpeedTimeout;    
    void speedPWM ( int pwmLeft, int pwmRight, int pwmMow );
    void control();    
    bool checkFault();
    void checkOverload();   
    bool checkOdometryError();
    bool checkMowRpmFault();
    void drvfix();                    //MrTree
    void checkMotorMowStall();        //MrTree
    float adaptiveSpeed();            //MrTree
    //float distanceRamp(float linear); //MrTree
    void changeSpeedSet();            //MrTree
    bool checkCurrentTooHighError();    
    bool checkCurrentTooLowError();
    void sense();
    void dumpOdoTicks(int seconds);    
};


#endif
