// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"

String GpsWaitFixOp::name(){
    return "GpsWaitFix";
}

void GpsWaitFixOp::begin(){
    CONSOLE.println("GpsWaitFixOp::begin - WARN: no gps solution!");
    stateSensor = SENS_GPS_INVALID;
    //setOperation(OP_ERROR);
    //buzzer.sound(SND_STUCK, true);          
    
    //linear = 0;
    //angular = 0;      
    //mow = false;
	CONSOLE.println("GpsWaitFixOp::begin - switch OFF all motors --> no gps solution!");
    motor.setLinearAngularSpeed(0,0, false); 
    motor.setMowState(false);
    if (GPS_RESET_WAIT_FIX){
        CONSOLE.println("GpsWaitFixOp::begin - reset timer for getting fix again startet!");
        resetGpsTimer = millis() + (GPS_RESET_WAIT_FIX_TIME*1000*60);     //MrTree strart timer for wait fix gps reset
    }
}


void GpsWaitFixOp::end(){
}

void GpsWaitFixOp::run(){
    battery.resetIdle();
    if (!buzzer.isPlaying()) buzzer.sound(SND_GPSWAITFIX);
    if (GPS_RESET_WAIT_FIX){
        if (millis() > resetGpsTimer) {
            CONSOLE.println("GpsWaitFixOp::run - reset gps timer triggered! Resetting GPS!");
            resetGpsTimer = millis() + (GPS_RESET_WAIT_FIX_TIME*1000*60);
            gps.reboot();
        }
    }
    if (gps.solution == SOL_FIXED){
        buzzer.sound(SND_READY);
        motor.setMowState(true);
        changeOp(*nextOp);
    }     
}


