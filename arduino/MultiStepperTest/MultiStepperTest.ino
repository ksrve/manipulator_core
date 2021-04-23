// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.

#include <AccelStepper.h>
#include <MultiStepper.h>

#define E1_STEP_PIN                       36
#define E1_DIR_PIN                        34

#define Z_STEP_PIN                        46
#define Z_DIR_PIN                         48

#define Y_STEP_PIN                        60
#define Y_DIR_PIN                         61

#define X_STEP_PIN                        54
#define X_DIR_PIN                         55

#define E0_STEP_PIN                       26
#define E0_DIR_PIN                        28

#define MOTOR_TYPE                         1

AccelStepper joint1(MOTOR_TYPE, Z_STEP_PIN,  Z_DIR_PIN);
AccelStepper joint2(MOTOR_TYPE, Y_STEP_PIN,  Y_DIR_PIN);
AccelStepper joint3(MOTOR_TYPE, X_STEP_PIN, X_DIR_PIN);
AccelStepper joint4(MOTOR_TYPE, E1_STEP_PIN, E1_DIR_PIN);


// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

//test with uint8 converted to long
unsigned int x = 1000;

void setup() {
  Serial.begin(250000);

  // Configure each stepper
  joint1.setMaxSpeed(1000);
  joint2.setMaxSpeed(300);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
}

void loop() {
  long positions[4]; // Array of desired stepper positions

  // Back of the envelope calculation for microsteps/revolution, where positions[i] is the number of steps (or microsteps).
  positions[0] = 0; //4100 microsteps is 1/8 revolutions ----> 32800 microsteps/rev
  positions[1] = 0; //2000 is 40/360 revolutions ---> 18000 microsteps/rev
  positions[2] = 12000; //4000 is 20/360 revolutions ---> 72000 microsteps/rev
  positions[3] = 0; //820 is 1/4 revolution (200steps/revolution * 16microsteps/step (since microstepping) ~= 32800 microsteps/rev)
  
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1);
  
  // Move to a different coordinate
  positions[0] = 0;
  positions[1] = 0;
  positions[2] = 0;
  positions[3] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1);
}
