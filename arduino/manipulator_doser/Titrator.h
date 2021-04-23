#pragma once
#ifndef TITRATOR

#include <Arduino.h>
#include <AccelStepper.h>

#define CALIBRATE_PIN       19
#define ENABLE_PIN          24
#define D                   20
#define MOTOR_TYPE           1
#define MOTOR_STEP_PIN      26
#define MOTOR_DIR_PIN       28

#define MAX_SPEED        10000
#define ACCEL               10
#define BAUD_RATE         9600

class Titrator{

  public:
  Titrator();
  ~Titrator();

  AccelStepper *stepper = new AccelStepper(MOTOR_TYPE, MOTOR_STEP_PIN, MOTOR_DIR_PIN);//Объявляем мотор

  void calibrater();
  void dropLiquid(int volume);
  void pickLiquid(int volume);
  void dropTube();

};

#endif /* end of include guard:  */
