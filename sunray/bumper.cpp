// Ardumower Sunray
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "bumper.h"
#include "config.h"
#include "robot.h"
#include <Arduino.h>

volatile bool inputLeftPressed = false;
volatile bool inputRightPressed = false;

volatile bool outputLeftPressed = false;
volatile bool outputRightPressed = false;

static bool bumperLeft   = false;
static bool bumperRight  = false;

unsigned long leftTrigTime = 0; // on delay timer (BUMPER_TRIGGER_DELAY) for the bumper inputs
unsigned long rightTrigTime = 0; // on delay timer (BUMPER_TRIGGER_DELAY) for the bumper inputs
unsigned long rightOnTimer = 0; // on delay timer (BUMPER_TRIGGER_DELAY) for the bumper inputs
unsigned long leftOnTimer = 0; // on delay timer (BUMPER_TRIGGER_DELAY) for the bumper inputs
unsigned long lastBumperTime = 0;


void Bumper::begin(){
  bumperDriver.begin();
}

void Bumper::run() {
  bumperDriver.run();
  inputLeftPressed   = bumperDriver.getLeftBumper();
  inputRightPressed  = bumperDriver.getRightBumper();

  if (BUMPER_ENABLE){
    //outputLeftPressed = inputLeftPressed;
    //outputRightPressed = inputRightPressed;  


    //MrTree: FIXED!
    //FIXME: code does not seem to work properly in all cases
    //https://github.com/Ardumower/Sunray/pull/103#issuecomment-1215526140
    //IDEA: do not check bumper errors here, check them (like for the lift-sensor) in 'src/EscapeReverseOp.cpp/run'

    // delay and dead time for the bumper inputs
    //if (millis() > (lastBumperTime + BUMPER_DEADTIME)){

      if (inputLeftPressed){
        if (!bumperLeft) { //state change and write vars once (not every iteration in code)!
          leftTrigTime = millis(); //remind the first time it is switched on
          leftOnTimer = 0;          //reset on timer
        }
        bumperLeft = true;
        leftOnTimer += millis() - lastBumperTime;   //add the timer for stuck detection
        if (millis() >= leftTrigTime + BUMPER_TRIGGER_DELAY) outputLeftPressed = true;
      } else {
        bumperLeft = false;
        outputLeftPressed = false;
      }

      if (inputRightPressed){
        if (!bumperRight) { //state change and write vars once (not every iteration in code)!
          rightTrigTime = millis(); //remind the first time it is switched on
          rightOnTimer = 0;          //reset on timer
        }
        bumperRight = true;
        rightOnTimer += millis() - lastBumperTime;   //add the timer for stuck detection
        if (millis() >= rightTrigTime + BUMPER_TRIGGER_DELAY) outputRightPressed = true;
      } else { 
        bumperRight = false;
        outputRightPressed = false;
      }
    
      // check for stuck condition and throw an error if stuck
      if (bumperRight || bumperLeft) {
        if (max(leftOnTimer, rightOnTimer) > max(leftTrigTime, rightTrigTime) + BUMPER_MAX_TRIGGER_TIME * 1000 ) {
          if (stateOp != OP_ERROR){
            stateSensor = SENS_BUMPER;
            CONSOLE.println("ERROR BUMPER BLOCKED - BUMPER_MAX_TRIGGER_TIME exceeded. See config.h for further information");
            CONSOLE.print("leftBumper triggered for: ");CONSOLE.print(leftOnTimer);CONSOLE.print(" ms, rightBumper triggered for: ");CONSOLE.print(rightOnTimer);CONSOLE.println(" ms");
            setOperation(OP_ERROR);
            leftTrigTime = 0;
            rightTrigTime = 0;
            leftOnTimer = 0;
            rightOnTimer = 0;
          }
        }
      }
      lastBumperTime = millis();
    //} else {
    //  outputLeftPressed = false;
    //  outputRightPressed = false;
  }
}


bool Bumper::obstacle(){
  if (BUMPER_ENABLE){
    return (outputLeftPressed || outputRightPressed);
  }
  else return false;
}

bool Bumper::obstacleLeft(){
  if (BUMPER_ENABLE){
    return (outputLeftPressed);
  }
  else return false;
}

bool Bumper::obstacleRight(){
  if (BUMPER_ENABLE){
    return (outputRightPressed);
  }
  else return false;
}

// send separated signals without delay to sensortest
bool Bumper::testLeft(){
  return (inputLeftPressed);
}

bool Bumper::testRight(){
  return (inputRightPressed);
}
