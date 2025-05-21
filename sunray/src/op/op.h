// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#ifndef OPS_H
#define OPS_H

#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"


// operations - the robot starts in operation 'IdleOp' and (depending on events) enters new operations (MowOp, EscapeReverseOp, GpsWaitFixOp etc.)

// base class for all operations (op's)
class Op {
  public:     
    virtual String name();
    // -------- transitions ----------------------------------       
    // op inititated by operator?
    bool initiatedByOperator;
    // should this operation stop?
    bool shouldStop;
    // op start time
    unsigned long startTime;
    // previous op
    Op *previousOp;
    // next op to call after op exit
    Op *nextOp; 

    // returns chained op's as a string (starting with active op, going until goal op) 
    // (example: "ImuCalibration->GpsWaitFix->Mow")
    String getOpChain();
	  String OpChain;

    // op's can be chained, this returns the current goal op:
    // examples:    
    // ImuCalibrationOp (active)-->GpsWaitFixOp-->mowOp            -->  mowOp    
    // ImuCalibrationOp (active)-->dockOp                          -->  dockOp
    Op* getGoalOp();

    Op();
    // trigger op exit (optionally allow returning back on called operation exit, e.g. generate an op chain)
    virtual void changeOp(Op &anOp, bool returnBackOnExit = false);

    // trigger op exit (optionally allow returning back on called operation exit, e.g. generate an op chain)
    virtual void changeOperationTypeByOperator(OperationType op);
    virtual OperationType getGoalOperationType();

    virtual void setInitiatedByOperator(bool flag);    
    // op entry code
    virtual void begin();
    // checks if active operation should stop and if so, makes transition to new one
    virtual void checkStop();
    // op run code 
    virtual void run();    
    // op exit code
    virtual void end();        
    // --------- events --------------------------------------
    virtual void onImuCalibration();
    virtual void onGpsJump();
    virtual void onGpsNoSignal();
    virtual void onGpsFixTimeout();
    virtual void onDockGpsReboot();  
    virtual void onRainTriggered();
    virtual void onTempOutOfRangeTriggered();
    virtual void onLiftTriggered();
    virtual void onOdometryError();
    virtual void onMotorOverload();
    virtual void onMotorMowStart();
	  virtual void onMotorMowStall();		//MrTree
    virtual void onMotorError();
    virtual void onObstacle();
    virtual void onObstacleRotation();
    virtual void onNoFurtherWaypoints();    
    virtual void onTargetReached();
    virtual void onKidnapped(bool state);
    virtual void onBatteryUndervoltage();
    virtual void onBatteryLowShouldDock();  
    virtual void onTimetableStopMowing();
    virtual void onTimetableStartMowing();      
    virtual void onChargerDisconnected();
    virtual void onBadChargingContactDetected();    
    virtual void onChargerConnected();    
    virtual void onChargingCompleted();
    virtual void onWaitCommand();              
    virtual void onImuTilt();
    virtual void onImuError();
    virtual float getDockDistance();
};


// idle op
class IdleOp: public Op {
  public:        
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

// IMU calibration op
class ImuCalibrationOp: public Op {
  public:        
    unsigned long nextImuCalibrationSecond;
    int imuCalibrationSeconds;
    virtual String name() override;
    virtual void changeOp(Op &anOp, bool returnBackOnExit = false) override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

// mowing op (optionally, also undocking dock points)
class MowOp: public Op {
  public:	
    bool lastMapRoutingFailed;
    int mapRoutingFailedCounter;
    unsigned int gpsNoSignalTime = 0;

    MowOp();
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
    virtual void onGpsJump() override;
    virtual void onGpsNoSignal() override;
    virtual void onGpsFixTimeout() override;
    virtual void onDockGpsReboot() override;  
    virtual void onOdometryError() override;
    virtual void onMotorOverload() override;
    virtual void onMotorMowStart() override;
	  virtual void onMotorMowStall() override;		//MrTree
    virtual void onMotorError() override;
    virtual void onRainTriggered() override;
    virtual void onTempOutOfRangeTriggered() override;    
    virtual void onBatteryLowShouldDock() override;
	  virtual void onTimetableStartMowing() override;    
    virtual void onTimetableStopMowing() override; 
    virtual void onObstacle() override;
    virtual void onObstacleRotation() override;
    virtual void onTargetReached() override;    
    virtual void onKidnapped(bool state) override;   
    virtual void onNoFurtherWaypoints() override;
    virtual void onWaitCommand() override;     
    virtual void onImuTilt() override;
    virtual void onImuError() override;
};

// dock op (driving to first dock point and following dock points until charging point)
class DockOp: public Op {
  public:        
    bool dockReasonRainTriggered;
    unsigned long dockReasonRainAutoStartTime;
    bool lastMapRoutingFailed;
    int mapRoutingFailedCounter;
    DockOp();
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
    virtual void onObstacle() override;
    virtual void onObstacleRotation() override;
    virtual void onTargetReached() override;    
    virtual void onGpsFixTimeout() override;
    virtual void onNoFurtherWaypoints() override;
    virtual void onDockGpsReboot() override;              
    virtual void onGpsNoSignal() override;
    virtual void onKidnapped(bool state) override;
    virtual void onChargerConnected() override;
    virtual void onWaitCommand() override;   
};

// charging op
class ChargeOp: public Op {
  public:
	unsigned long retryTouchDockSpeedTime;
    unsigned long retryTouchDockStopTime;
    unsigned long betterTouchDockStopTime;
	unsigned long nextMoveTime;
	unsigned long movingTime;
    bool retryTouchDock;
    bool betterTouchDock;
	bool Moving;
	bool Vor;
	bool Once;
    unsigned long nextConsoleDetailsTime;   
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
    virtual void onChargerDisconnected() override;
    virtual void onBadChargingContactDetected() override;
    virtual void onBatteryUndervoltage() override;    
    virtual void onRainTriggered() override;   
    virtual void onChargerConnected() override;
	  virtual void onTimetableStartMowing() override;    
    virtual void onTimetableStopMowing() override;   
};

// wait for undo kidnap (gps jump) 
class KidnapWaitOp: public Op {
  public:
    unsigned long recoverGpsTime;
    int recoverGpsCounter;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
    virtual void onKidnapped(bool state) override;
    virtual void onGpsNoSignal() override;    
};

// dock reboot gps
class DockGpsRebootOp: public Op {
  public:
    unsigned long rebootGpsTime;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

// reboot gps recovery
class GpsRebootRecoveryOp: public Op {
  public:
    unsigned long retryOperationTime;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

// wait for gps fix
class GpsWaitFixOp: public Op {
  public:
    unsigned long resetGpsTimer = 0;  //MrTree
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

// wait for gps signal (float or fix)
class GpsWaitFloatOp: public Op {
  public:
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

// just wait
class WaitOp: public Op {
  public:
    unsigned long waitStartTime = 0;  //MrTree
    unsigned long waitTime = 0;
    //WaitOp();
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

// escape high lawn (drive backwards without virtual obstace)
class EscapeLawnOp: public Op {					//MrTree
  public:        								//**
	  int escapeLawnCounter = 0;							
	  unsigned long escapeLawnStartTime;
    unsigned long driveReverseStopTime;
	  unsigned long escapeLawnWaitTime;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
    virtual void onImuTilt() override;
    virtual void onImuError() override;
};												//**

// escape rotation (drive backwards)
class EscapeRotationOp: public Op {
  public:        
    unsigned long driveReverseStopTime;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
    virtual void onImuTilt() override;
    virtual void onImuError() override;
};

// escape obstacle (drive backwards)
class EscapeReverseOp: public Op {
  public:        
    unsigned long driveReverseStopTime;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
    virtual void onImuTilt() override;
    virtual void onImuError() override;
};

// escape obstacle (drive forward)
class EscapeForwardOp: public Op {
  public:
    int escapeForwardCounter = 0;		        //MrTree				
	  unsigned long escapeForwardStartTime;   //MrTree
    unsigned long driveForwardStopTime;
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
    virtual void onImuTilt() override;
    virtual void onImuError() override;
};

// error op
class ErrorOp: public Op {
  public:        
    virtual String name() override;
    virtual void begin() override;
    virtual void end() override;
    virtual void run() override;
};

extern unsigned long deltaTime;

extern ChargeOp chargeOp;
extern ErrorOp errorOp;
extern DockOp dockOp;
extern IdleOp idleOp;
extern MowOp mowOp;
extern EscapeLawnOp escapeLawnOp;			//MrTree
extern EscapeRotationOp escapeRotationOp;
extern EscapeReverseOp escapeReverseOp;
extern EscapeForwardOp escapeForwardOp;
extern WaitOp waitOp;
extern KidnapWaitOp kidnapWaitOp;
extern GpsWaitFixOp gpsWaitFixOp;
extern GpsWaitFloatOp gpsWaitFloatOp;
extern GpsRebootRecoveryOp gpsRebootRecoveryOp;
extern DockGpsRebootOp dockGpsRebootOp;
extern ImuCalibrationOp imuCalibrationOp;

// active op
extern Op *activeOp;

#endif


