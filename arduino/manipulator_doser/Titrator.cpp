#include "Titrator.h"


Titrator::Titrator(){
  stepper->setMaxSpeed(MAX_SPEED);
  stepper->setAcceleration(ACCEL);

  pinMode(CALIBRATE_PIN, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, 0); //притягиваем EN к земле

  #ifdef DEBUG
    Serial.begin(BAUD_RATE);
  #endif
}

Titrator::~Titrator(){}

void Titrator::calibrater(){
  #ifdef DEBUG
    Serial.println("Calibrating");
  #endif
  int calibrate = digitalRead(CALIBRATE_PIN);
  int calibrateOld = 1; //необходимо для фильтрации помех.
  int startTime = millis();
  while (!(calibrate == 0 && calibrateOld == 0)){
    stepper->move(-10);//микрошаг
    int currentTime = millis() - startTime;
    while (abs(stepper->distanceToGo()) > 1){
      stepper->setSpeed(-10000);
      stepper->runSpeed();
    }
    if (currentTime > 50) { //импульс сигнала не меньше 10мс
      calibrateOld = calibrate;
      startTime = millis();
    }
    calibrate = digitalRead(CALIBRATE_PIN);
    #ifdef DEBUG
      Serial.println(calibrate);
    #endif
  }
}

void Titrator::dropLiquid(int volume){
  #ifdef DEBUG
    Serial.println("Dropping liquid");
  #endif
  double s = (3.14159265 *D*D/ 4);
  int steps = (int)(1600 * (volume / s));
  #ifdef DEBUG
    Serial.println(volume/s);
    Serial.println(steps);
  #endif
  stepper->move(steps);
  while (abs(stepper->distanceToGo()) > 2){
    stepper->setSpeed(10000);
    stepper->runSpeed();
  }
  delay(1000);
}

void Titrator::pickLiquid(int volume){
  #ifdef DEBUG
    Serial.println("Taking liquid");
  #endif
  double s = (3.14159265 *D*D/ 4);
  int steps = (int)(-1600 * (volume / s));
  stepper->move(steps);
  #ifdef DEBUG
    Serial.println(volume/s);
    Serial.println(steps);
  #endif
  while (abs(stepper->distanceToGo()) > 2){
    stepper->setSpeed(-10000);
    stepper->runSpeed();
  }
  delay(1000);
}

void Titrator::dropTube(){
  #ifdef DEBUG
    Serial.println("Dropping tube");
  #endif
  stepper->move(1600 * 15);
  while (abs(stepper->distanceToGo()) > 2){
    stepper->setSpeed(10000);
    stepper->runSpeed();
  }
  delay(1000);
}
