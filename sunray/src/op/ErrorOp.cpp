// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "op.h"
#include <Arduino.h>
#include "../../robot.h"
#include "../../map.h"


String ErrorOp::name(){
    return "Error";
}


void ErrorOp::begin(){
    CONSOLE.println("OP_ERROR - stop immediately"); 
    //if (stateOp == OP_CHARGE){
    //  
    //  op = stateOp;
    //} else {        
    motor.stopImmediately(true); // do not use PID to get to stop
    motor.setLinearAngularSpeed(0,0);
    motor.setMowState(false);
    resetMotion();      
    //}  
    buzzer.sound(SND_ERROR, true);
}


void ErrorOp::end(){
}

void ErrorOp::run(){
    if (!buzzer.isPlaying()) buzzer.sound(SND_SOS, true);
    resetMotion();
    if (battery.chargerConnected()){        
        changeOp(chargeOp);
    }    
}


