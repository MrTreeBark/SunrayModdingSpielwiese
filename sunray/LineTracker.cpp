// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)


#include "LineTracker.h"
#include "robot.h"
#include "StateEstimator.h"
#include "helper.h"
#include "pid.h"
#include "src/op/op.h"
#include "Stats.h"

float stanleyTrackingNormalK = STANLEY_CONTROL_K_NORMAL;
float stanleyTrackingNormalP = STANLEY_CONTROL_P_NORMAL;
float stanleyTrackingSlowK = STANLEY_CONTROL_K_SLOW;
float stanleyTrackingSlowP = STANLEY_CONTROL_P_SLOW;

float targetDist = 0;     //MrTree
float lastTargetDist = 0; //MrTree

float setSpeed = 0.1; //externally controlled (app) linear speed (m/s)
float CurrSpeed = 0;  //actual used speed from motor.linearSpeedSet
float CurrRot = 0;  //actualk rotation from mow.motor
float linear = 0;
float angular = 0;

float x_old = 0;
float y_old = 0;
float x_new = 0;
float y_new = 0;
Point lastPoint;
Point target;
Point lastTarget;

bool mow = false;
bool trackslow_allowed = false;
bool straight = false;
bool shouldRotate = false;                      //MrTree
bool shouldRotatel = false;                     //MrTree
bool angleToTargetFits = false;
bool angleToTargetPrecise = true;
bool langleToTargetFits = false;
bool targetReached = false;
bool stateKidnapped = false;
bool dockTimer = false;                         //MrTree
bool unDockTimer = false;
bool oneTrigger = false;                        //MrTree
bool printmotoroverload = false;
float trackerDiffDelta = 0;
float targetDelta = 0;
float distToPath = 0;
int dockGpsRebootState;                     // Svol0: status for gps-reboot at specified docking point by undocking action
int counterCheckPos = 0;                    // check if gps position is reliable
bool blockKidnapByUndocking;                // Svol0: kidnap detection is blocked by undocking without gps
unsigned long dockGpsRebootTime;            // Svol0: retry timer for gps-fix after gps-reboot
unsigned long dockGpsRebootFixCounter;      // Svol0: waitingtime for fix after gps-reboot
unsigned long dockGpsRebootFeedbackTimer;   // Svol0: timer to generate acustic feedback
unsigned long reachedPointBeforeDockTime = 0;   //MrTree
bool dockGpsRebootDistGpsTrg = false;       // Svol0: trigger to check solid gps-fix position (no jump)
bool allowDockLastPointWithoutGPS = false;  // Svol0: allow go on docking by loosing gps fix
bool allowDockRotation = true;              //MrTree: disable rotation on last dockingpoint
bool warnDockWithoutGpsTrg = false;         // Svol0: Trigger for warnmessage
float stateX_1 = 0;                         // Svol0
float stateY_1 = 0;                         // Svol0
float stateX_2 = 0;                         // Svol0
float stateY_2 = 0;                         // Svol0
float stateX_3 = 0;                         // Svol0
float stateY_3 = 0;                         // Svol0

bool AngleToTargetFits() {
  // allow rotations only near last or next waypoint or if too far away from path
  if (targetDist < 0.3 || lastTargetDist < 0.3 || fabs(distToPath) > 1.0 ) {
    angleToTargetFits = (fabs(trackerDiffDelta) / PI * 180.0 <= TRANSITION_ANGLE);              //MrTree we have more than TRANSITION_ANGLE difference to point, else linetracker stanley angular faktor p will sort things out
  } else {
	// while tracking the mowing line do allow rotations if angle to target increases (e.g. due to gps jumps)
    angleToTargetFits = (fabs(trackerDiffDelta)/PI*180.0 < 45);       
  }

  if ((!angleToTargetFits || !angleToTargetPrecise)) angleToTargetFits = false;   //MrTree added !dockTimer to prevent the jumping gps point to cause linear=0 because of !angleToTargetFits, added !angleToTargetPrecise 
  if (dockTimer || unDockTimer) angleToTargetFits = true;
  return angleToTargetFits;  
}

void rotateToTarget() {
  // angular control (if angle to far away, rotate to next waypoint)
    if (!angleToTargetFits) angleToTargetPrecise = false;
    linear = 0; //MrTree while turning from >= 20/45 deg difference, linear is set to 0... still decelerating or accelerating on stepin/out
    if (ROTATION_RAMP) {
      //angular = lerp(ROTATION_RAMP_MIN / 180.0 * PI, ROTATION_RAMP_MAX / 180.0 * PI, fabs(trackerDiffDelta));
      //CONSOLE.print("lerp angular:  ");CONSOLE.println(angular*180/PI);
      angular = fabs(trackerDiffDelta) + ROTATION_RAMP_MIN / 180.0 * PI;
      //CONSOLE.print("raw angular +5 :  ");CONSOLE.println(angular*180/PI);
      
      angular = constrain(angular, ROTATION_RAMP_MIN / 180.0 * PI, ROTATION_RAMP_MAX / 180.0 * PI);
      //CONSOLE.print("constrain angular:  ");CONSOLE.println(angular*180/PI);
    } else {
      if (fabs(trackerDiffDelta)/PI*180.0 >= ANGLEDIFF1) angular = ROTATETOTARGETSPEED1 / 180.0 * PI;   //MrTree set angular to fast defined in config.h
      if (fabs(trackerDiffDelta)/PI*180.0 < ANGLEDIFF1) angular = ROTATETOTARGETSPEED2  / 180.0 * PI;    //MrTree slow down turning when near desired angle     
      if (fabs(trackerDiffDelta)/PI*180.0 <= ANGLEDIFF2) angular = ROTATETOTARGETSPEED3 / 180.0 * PI;    //MrTree slow down turning even more when almost at desired angle     
    }
    if (trackerDiffDelta < 0) {     //MrTree set rotation direction and do not keep it :)
      angular *= -1;
    }                     
    if (fabs(trackerDiffDelta)/PI*180.0 < ANGLEPRECISE){
      angular = 0;
      resetStateEstimation();
      if (CurrRot == 0) angleToTargetPrecise = true;                          //MrTree Step out of everything when angle is precise... and we stopped rotating 
    }
    //add option to disable and start rotsatating even if still moving
    if (fabs(CurrSpeed) > 0.0) angular = 0;                //MrTree reset angular if current speed is over given value (still deccelerating)
    
}

void stanleyTracker() {
   
  //Stanley parameters
  static float k;
  static float p;

  if (MAP_STANLEY_CONTROL) {
    //Mapping of Stanley Control Parameters in relation to actual Setpoint value of speed
    //Values need to be multiplied, because map() function does not work well with small range decimals
    // linarSpeedSet needed as absolut value for mapping
    
    //do not use agressive stanley if floatsittuaion
    if (gps.solution == SOL_FLOAT || gps.solution == SOL_INVALID) {
      stanleyTrackingNormalK = STANLEY_FLOAT_K_NORMAL; 
      stanleyTrackingNormalP = STANLEY_FLOAT_P_NORMAL;
      stanleyTrackingSlowK = STANLEY_FLOAT_K_SLOW;
      stanleyTrackingSlowP = STANLEY_FLOAT_P_SLOW;
    } 

    k = map(fabs(CurrSpeed) * 1000, MOTOR_MIN_SPEED * 1000, MOTOR_MAX_SPEED * 1000, stanleyTrackingSlowK * 1000, stanleyTrackingNormalK * 1000); //MOTOR_MIN_SPEED and MOTOR_MAX_SPEED from config.h
    p = map(fabs(CurrSpeed) * 1000, MOTOR_MIN_SPEED * 1000, MOTOR_MAX_SPEED * 1000, stanleyTrackingSlowP * 1000, stanleyTrackingNormalP * 1000); //MOTOR_MIN_SPEED and MOTOR_MAX_SPEED from config.h
    k = k / 1000;
    p = p / 1000;
    k = max(stanleyTrackingSlowK, min(stanleyTrackingNormalK, k));  // limitation for value if out of range
    p = max(stanleyTrackingSlowP, min(stanleyTrackingNormalP, p));  // limitation for value if out of range
  } else {
    k = stanleyTrackingNormalK;                                     // STANLEY_CONTROL_K_NORMAL;
    p = stanleyTrackingNormalP;                                     // STANLEY_CONTROL_P_NORMAL;
    if (maps.trackSlow && trackslow_allowed) {
      k = stanleyTrackingSlowK;                                     //STANLEY_CONTROL_K_SLOW;
      p = stanleyTrackingSlowP;                                     //STANLEY_CONTROL_P_SLOW;
    }
  }
                                                                                                                            //MrTree
  angular =  p * trackerDiffDelta + atan2(k * lateralError, (0.001 + fabs(CurrSpeed)));       //MrTree, use actual speed correct for path errors
  // restrict steering angle for stanley  (not required anymore after last state estimation bugfix)
  angular = max(-PI/6, min(PI/6, angular)); //MrTree still used here because of gpsfix jumps that would lead to an extreme rotation speed
  //if (!maps.trackReverse && motor.linearCurrSet < 0) angular *= -1;   // it happens that mower is reversing without going to a map point (obstacle escapes) but trying to get straight to the next point angle (transition angle), for this case angular needs to be reversed
  //After all, we want to use stanley for transition angles as well, too have a smooth operation between points without coming to a complete stop
  //For that we will scale down the actual linear speed set dependent to the angle difference to the NEXT targetpoint, we need also to deactivate distance ramp for nextangletotargetfits = true
  //so distanceramp is basically just a function for !angletotargetfits and can be moved into this function permanent without possibility to activate or deactivate? This would also eliminate the need in distanceramp to try and compensate small angles to not come to a stop
  //which is now anyway forced by angletotargetfits and has no effect.

  //linear = linearSpeedState() * angletonexttarget
}

void linearSpeedState(){
  const int aLen = 10;                                           //array length of linearSpeed[]
  const String linearSpeedNames[aLen] = {                       //strings for message output accordingly to state
                                    "FLOATSPEED",
                                    "NEARWAYPOINTSPEED",
                                    "SONARSPEED",
                                    "OVERLOADSPEED",
                                    "KEEPSLOWSPEED",
                                    "RETRYSLOWSPEED",
                                    "TRACKSLOWSPEED",
                                    "DOCK_NO_ROTATION_SPEED",
                                    "DOCKPATHSPEED",
                                    "DOCKSPEED"
                                  };
  const float linearSpeed[aLen] = {
                                    FLOATSPEED,
                                    NEARWAYPOINTSPEED,
                                    SONARSPEED,
                                    OVERLOADSPEED,
                                    KEEPSLOWSPEED,
                                    RETRYSLOWSPEED,
                                    TRACKSLOWSPEED,
                                    DOCK_NO_ROTATION_SPEED,
                                    DOCKPATHSPEED,
                                    DOCKSPEED
                                  };
  static bool linearBool[aLen];     //helper array to choose lowest value from sensor/mower states
  static int chosenIndex;           //helper to know what speed is used
  static int chosenIndexl;          //helper to compare a index change and trigger a meassage in console
  int speedIndex = 0;               //used for linearBool and linearSpeed array index 
  trackslow_allowed = true;

  /*
  // in case of docking or undocking - check if trackslow is allowed
  if ( maps.isUndocking() || maps.isDocking() ) {
    static float dockX = 0;
    static float dockY = 0;
    static float dockDelta = 0;
    maps.getDockingPos(dockX, dockY, dockDelta);
    // only allow trackslow if we are near dock (below DOCK_UNDOCK_TRACKSLOW_DISTANCE)
    if (distance(dockX, dockY, stateX, stateY) > DOCK_UNDOCK_TRACKSLOW_DISTANCE) {
      trackslow_allowed = false;
    }
  }
  */


  linear = setSpeed;                //always compare speeds against desired setSpeed 
  
  //which states can be true in runtime?
  linearBool[0] = (gps.solution == SOL_FLOAT);                                                      // [0] FLOATSPEED
  linearBool[1] = (targetDist < NEARWAYPOINTDISTANCE || lastTargetDist < NEARWAYPOINTDISTANCE);     // [1] NEARWAYPOINTSPEED
  linearBool[2] = (sonar.nearObstacle());                                                           // [2] SONARSPEED
  linearBool[3] = (motor.motorLeftOverload || motor.motorRightOverload || motor.motorMowOverload);  // [3] OVERLOADSPEED
  linearBool[4] = (motor.keepslow);                                                                 // [4] KEEPSLOWSPEED
  linearBool[5] = (motor.retryslow);                                                                // [5] RETRYSLOWSPEED
  linearBool[6] = (maps.trackSlow && trackslow_allowed);                                            // [6] TRACKSLOWSPEED
  linearBool[7] = (dockTimer || unDockTimer);                                                       // [7] DOCK_NO_ROTATION_SPEED
  linearBool[8] = (maps.isAtDockPath());                                                            // [8] DOCKPATHSPEED
  linearBool[9] = (maps.isGoingToDockPath());                                                       // [8] DOCKSPEED
  //disable near way point speed if we use the distance ramp
  if (DISTANCE_RAMP) linearBool[1] = false;

  //choose the lowest speed of the true states set before in the bool array
  for(speedIndex = 0; speedIndex < aLen; speedIndex ++){
    if (linearBool[speedIndex] == true){
      if (linearSpeed[speedIndex] < linear){
        linear = linearSpeed[speedIndex];
        chosenIndex = speedIndex;
      }
    }
  }

  //trigger a message if speed changes
  if (chosenIndex != chosenIndexl){
    CONSOLE.print("Linetracker.cpp - linearSpeedState(): ");
    CONSOLE.print(linearSpeedNames[chosenIndex]);
    CONSOLE.print(" = ");
    CONSOLE.print(linearSpeed[chosenIndex]);
    CONSOLE.println(" m/s");
  }

  //consider the distance ramp wih the chosen speed if we are approaching or leaving a waypoint
  if (DISTANCE_RAMP) {
    if (targetDist < 2 * NEARWAYPOINTDISTANCE || lastTargetDist < 2 * NEARWAYPOINTDISTANCE) { //start computing before reaching point distance (maybe not neccessary)
      linear = distanceRamp(linear);
    }
  }

  chosenIndexl = chosenIndex;

  if (maps.trackReverse) linear *= -1;   // reverse line tracking needs negative speed
}

float distanceRamp(float linear){
    float maxSpeed = linear*1000;
    float minSpeed = DISTANCE_RAMP_MINSPEED * 1000;
    float maxDist = (linear * NEARWAYPOINTDISTANCE /setSpeed) * 1000;     //if we are going slow for example because of float, the ramp will kick in when mower is nearer to point
    float minDist = 0;                                                    //TARGET_REACHED_TOLERANCE*1000;
    float actDist = 0;
    float rampSpeed = 0;
    static bool wasStraight;

    if (targetDist <= lastTargetDist) {                                  //need to decide what ramp, leaving or aproaching? --> approaching
      maxDist += maxSpeed;                                               //add an speed dependent offset to target distance when approaching, because mower comes with high speed that causes a timing issue
      actDist = targetDist;
      if (straight) minSpeed = TRANSITION_SPEED * 1000; //maxSpeed * 0.5;                           //if we don´t need to rotate, do not decellarate too much
      wasStraight = straight;
    } else {
      if (wasStraight) minSpeed = TRANSITION_SPEED * 1000; //maxSpeed * 0.5;
      actDist = lastTargetDist;
      //actDist += 0.05; //add an offset      
    }

    actDist *= 1000;

    if (targetDist + lastTargetDist < maxDist) { //points are not far away from each other
      actDist *= 2;                              //multiply the actDist to trick the map function (hurryup because we wont reach full speed anyway)
    }

    rampSpeed = map(actDist, minDist, maxDist, minSpeed, maxSpeed);
    rampSpeed = constrain(rampSpeed, minSpeed, maxSpeed);
    rampSpeed /= 1000;
    //CONSOLE.print(straight); CONSOLE.print(" "); CONSOLE.println(rampSpeed);
    return rampSpeed;
}

void gpsConditions() {
  // check some pre-conditions that can make linear+angular speed zero
  if (fixTimeout != 0) {
    if (millis() > lastFixTime + fixTimeout * 1000.0) {
      activeOp->onGpsFixTimeout();
    }
  }
  //CONSOLE.print(maps.shouldGpsReboot); CONSOLE.print(" <- shouldreboot | isatgpsrebootpoint -> "); CONSOLE.print(maps.isAtGpsRebootPoint()); CONSOLE.print(" | dockPointIdx: ");CONSOLE.println(maps.dockPointsIdx);
  if (maps.shouldGpsReboot && maps.isAtGpsRebootPoint()){
    activeOp->onDockGpsReboot();
  }

  // gps-jump/false fix check
  if (KIDNAP_DETECT) {
    float allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE;
    if ( maps.isUndocking() || maps.isDocking() ) {
      float dockX = 0;
      float dockY = 0;
      float dockDelta = 0;
      maps.getDockingPos(dockX, dockY, dockDelta);
      float dist = distance(dockX, dockY, stateX, stateY);
      // check if current distance to docking station is below
      // KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK to trigger KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK
      if (dist < KIDNAP_DETECT_DISTANCE_DOCK_UNDOCK) {
        allowedPathTolerance = KIDNAP_DETECT_ALLOWED_PATH_TOLERANCE_DOCK_UNDOCK;
      }
    }// MrTree integrated with keeping new sunray code Svol0: changed for GPS-Reboot at a
    if (fabs(distToPath) > allowedPathTolerance) { // actually, this should not happen (except on false GPS fixes or robot being kidnapped...)
      if (!stateKidnapped) {
        stateKidnapped = true;
        activeOp->onKidnapped(stateKidnapped);
      }
    } else {
      if (stateKidnapped) {
        stateKidnapped = false;
        activeOp->onKidnapped(stateKidnapped);
      }
    }
  }
}

void noDockRotation() {
  if (maps.wayMode != WAY_DOCK) return;
  if ((maps.isTargetingLastDockPoint() && !maps.isUndocking())){        //MrTree step in algorithm if allowDockRotation (computed in maps.cpp) is false and mower is not undocking
    if (!dockTimer){                                                  //set helper bool to start a timer and print info once
      reachedPointBeforeDockTime = millis();                          //start a timer when going to last dockpoint
      dockTimer = true;                                               //enables following code
      CONSOLE.println("allowDockRotation = false, timer to successfully dock startet. angular = 0, turning not allowed");
    }
    if (dockTimer){
      //resetLinearMotionMeasurement();                                         //need to test if this is still neccessary
      if (lastTargetDist > DOCK_NO_ROTATION_DISTANCE){                          //testing easier approach for DOCK_NO_ROTATION setup
        angular = 0;
        linear = DOCK_NO_ROTATION_SPEED;
        targetReached = false;
        if (!buzzer.isPlaying()) buzzer.sound(SND_ERROR, true);                  
      }
      if (millis() > reachedPointBeforeDockTime+DOCK_NO_ROTATION_TIMER){      //check the time until mower has to reach the charger and triger obstacle if not reached
        CONSOLE.println("noDockRotation(): not docked in given time, triggering maps.retryDocking!");
        dockTimer = false;
        triggerObstacle();     
      } 
    }
  } else {
      dockTimer = false;     
  }
  return;
}

void noUnDockRotation(){
  if (maps.wayMode != WAY_DOCK) return;
  if (maps.isBetweenLastAndNextToLastDockPoint() && maps.isUndocking()){
    if (!unDockTimer){                                                  //set helper bool to start a timer and print info once
      reachedPointBeforeDockTime = millis();                          //start a timer when going to last dockpoint
      unDockTimer = true;                                               //enables following code
      CONSOLE.println("noUnDockRotation(): timer to successfully undock startet. angular = 0, turning not allowed");
    }
    if (unDockTimer){
      //resetLinearMotionMeasurement();                                         //need to test if this is still neccessary
                               //testing easier approach for DOCK_NO_ROTATION setup
      angular = 0;
      linear = -DOCK_NO_ROTATION_SPEED;
      if (!buzzer.isPlaying()) buzzer.sound(SND_ERROR, true);                  
      if (millis() > reachedPointBeforeDockTime+DOCK_NO_ROTATION_TIMER){      //check the time until mower has to reach the charger and triger obstacle if not reached
        CONSOLE.println("noUnDockRotation(): reversed for given Time, triggering Wait before further retreating to reboot gps point!");
        unDockTimer = false;
        maps.dockPointsIdx--;
        //targetReached = true;
        //waitOp.waitTime = 15000;
        triggerWaitCommand(15000);    
      } 
    }
  } else {
      unDockTimer = false;     
  }
  return;  
}

void checkMowAllowed() {
  mow = false;                                //MrTree changed to false
  if (MOW_START_AT_WAYMOW &! oneTrigger) {                                                             
    if (maps.wayMode == WAY_MOW) {            //MrTree do not activate mow until there is a first waymow 
      mow = true;                             //MrTree this will only work directly after undocking and way free, the first time it is in waymow, mow will be true forever like before     
      oneTrigger = true;
    }                                              
  } else {
    if (maps.wayMode == WAY_MOW || WAY_EXCLUSION || WAY_PERIMETER){
      mow = true;                               //MrTree --> original condition, mow will be true here and is maybe changed by a condition later in linetracker
    }
  }

  if (stateOp == OP_DOCK || maps.shouldDock == true) {
    mow = false;
    oneTrigger = false;
  }
}

// control robot velocity (linear,angular) to track line to next waypoint (target)
// uses a stanley controller for line tracking
// https://medium.com/@dingyan7361/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
void trackLine(bool runControl) {
  target = maps.targetPoint;
  lastTarget = maps.lastTargetPoint;
  CurrSpeed = motor.linearSpeedSet;           //MrTree take the real speed from motor.linearSpeedSet
  CurrRot = motor.angularSpeedSet;
  linear = 0;                                 //MrTree Changed from 1.0
  angular = 0;
  
  targetDelta = pointsAngle(stateX, stateY, target.x(), target.y());
  if (maps.trackReverse) targetDelta = scalePI(targetDelta + PI);
  targetDelta = scalePIangles(targetDelta, stateDelta);
  trackerDiffDelta = distancePI(stateDelta, targetDelta);
  lateralError = distanceLineInfinite(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());
  distToPath = distanceLine(stateX, stateY, lastTarget.x(), lastTarget.y(), target.x(), target.y());
  targetDist = maps.distanceToTargetPoint(stateX, stateY);
  lastTargetDist = maps.distanceToLastTargetPoint(stateX, stateY);
  targetReached = (targetDist < TARGET_REACHED_TOLERANCE);
  //float lineDist = maps.distanceToTargetPoint(lastTarget.x(), lastTarget.y());
  
  if (!AngleToTargetFits()) { 
    rotateToTarget();
  } else {
    linearSpeedState();  //compares the linear Speed to use according to configured mower state (maybe this should be first in line)
    stanleyTracker();    //track the path
  }
  gpsConditions();      //check for gps conditions to eg. trigger obstacle or fixtimeout (shouldn´t that be in mowop???)
  noDockRotation();     //disable angular for dock/undock situations
  noUnDockRotation();
  checkMowAllowed();

  /*CONSOLE.print("     linear: ");
  CONSOLE.print(linear);
  CONSOLE.print("        angleToTargetFits: ");
  CONSOLE.println(angleToTargetFits);
  CONSOLE.print("    angular: ");
  CONSOLE.print(angular*180.0/PI);
  CONSOLE.print("         trackerDiffDelta: ");
  CONSOLE.println(trackerDiffDelta*180/PI);
  CONSOLE.print(" distToPath: ");
  CONSOLE.print(distToPath);
  CONSOLE.print("             distToTarget: ");
  CONSOLE.println(targetDist);
  */
  if (runControl) {

    shouldRotate = robotShouldRotate();

    if (DEBUG_LINETRACKER) {
      // ouput target point change
      x_new = target.x();
      y_new = target.y();
      if (x_old != x_new || y_old != y_new) {
        CONSOLE.print("LineTracker.cpp targetPoint  x = ");
        CONSOLE.print(x_new);
        CONSOLE.print(" y = ");
        CONSOLE.println(y_new);
        x_old = x_new;
        y_old = y_new;
      }
      // output rotate state change
      if (shouldRotate != shouldRotatel) {
        CONSOLE.print("Linetracker.cpp ShouldRotate = ");
        CONSOLE.println(shouldRotate);
        shouldRotatel = shouldRotate;
      }
      // output tracking data permanently
      CONSOLE.println("DEBUG_LINETRACKER START -->");
      CONSOLE.print(" angleToTargetFits: ");CONSOLE.println(angleToTargetFits);
      CONSOLE.print("           angular: ");CONSOLE.println(angular*180.0/PI);
      CONSOLE.print("  trackerDiffDelta: ");CONSOLE.println(trackerDiffDelta*180/PI);
      CONSOLE.print("     distToPath --> ");CONSOLE.print(distToPath);CONSOLE.print(" | ");CONSOLE.print(targetDist);CONSOLE.println(" <-- targetDist");
      CONSOLE.println("<-- DEBUG_LINETRACKER END");
    }
    
    if (detectLift()){ // in any case, turn off mower motor if lifted  
      mow = false;  // also, if lifted, do not turn on mowing motor so that the robot will drive and can do obstacle avoidance 
      linear = 0;
      angular = 0; 
    }


    if (mow != motor.switchedOn && motor.enableMowMotor){
      CONSOLE.print("Linetracker.cpp changes mow status: ");
      CONSOLE.println(mow);
      motor.setMowState(mow); 
    }
    motor.setLinearAngularSpeed(linear, angular, true);    
  }

  if (targetReached) {
    activeOp->onTargetReached();
    straight = maps.nextPointIsStraight();
    if (!maps.nextPoint(false, stateX, stateY)) {
      // finish
      activeOp->onNoFurtherWaypoints();
    }
  }
}
