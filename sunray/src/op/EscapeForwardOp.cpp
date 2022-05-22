// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"


String EscapeForwardOp::name(){
    return "EscapeForward";
}

void EscapeForwardOp::begin(){
    // rotate stuck avoidance
    motor.setLinearAngularSpeed(0.1,0);
    motor.setMowState(false);                
    driveForwardStopTime = millis() + 2000;
}


void EscapeForwardOp::end(){
}


void EscapeForwardOp::run(){
    battery.resetIdle();
    if (millis() > driveForwardStopTime){
        CONSOLE.println("driveForwardStopTime");
        motor.stopImmediately(false);  
        driveForwardStopTime = 0;
        /*maps.addObstacle(stateX, stateY);
        Point pt;
        if (!maps.findObstacleSafeMowPoint(pt)){
        setOperation(OP_DOCK, true); // dock if no more (valid) mowing points
        } else*/ 
        changeOp(*nextOp);    // continue current operation              
    }
}
