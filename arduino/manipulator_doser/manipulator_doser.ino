#include "Manipulator.h"
#include "Titrator.h"

Titrator *_titrator = NULL;

void setup() {

    _titrator = new Titrator();
    //_titrator->calibrater();
    //_titrator->dropLiquid(1);

    nh.initNode();
    nh.subscribe(required_angle_sub);

    nh.advertise(current_pose_pub);
    nh.advertise(is_in_pose_pub);
    nh.advertise(required_step_pub);

    pinMode(JOINT_1_ENDSTOP, INPUT_PULLUP);
    pinMode(JOINT_2_ENDSTOP, INPUT_PULLUP);
    pinMode(JOINT_3_ENDSTOP, INPUT_PULLUP);
    pinMode(JOINT_4_ENDSTOP, INPUT_PULLUP);

    joint1.setMaxSpeed(JOINT_1_AVERAGE_SPEED);
    joint2.setMaxSpeed(JOINT_2_AVERAGE_SPEED);
    joint3.setMaxSpeed(JOINT_3_AVERAGE_SPEED);
    joint4.setMaxSpeed(JOINT_4_AVERAGE_SPEED);

    joint1.setAcceleration(JOINT_1_AVERAGE_ACCEL);
    joint2.setAcceleration(JOINT_2_AVERAGE_ACCEL);
    joint3.setAcceleration(JOINT_3_AVERAGE_ACCEL);
    joint4.setAcceleration(JOINT_4_AVERAGE_ACCEL);

    steppers_1_pair.addStepper(joint1);
    steppers_1_pair.addStepper(joint2);
    steppers_1_pair.addStepper(joint3);
    steppers_1_pair.addStepper(joint4);

    /*
    while (!calibrate(1)){}
    go_to_initial_position(1);
    move_to_pose(joint4, -2000);
    move_to_pose(joint3, -20000);

    while (!calibrate(2)){}
    go_to_initial_position(2);
    move_to_pose(joint3, 20000);
    */
    



}


void loop() {

  joints_control[0] = -req_step[0];
  joints_control[1] = req_step[1];
  joints_control[2] = -req_step[2];
  joints_control[3] = -req_step[3];

  current_position_publisher();
  current_distance_status_publisher();

  steppers_1_pair.moveTo(joints_control);
  //steppers_2_pair.moveTo(joints_control);

  //steppers_2_pair.runSpeedToPosition();
  steppers_1_pair.runSpeedToPosition();

  nh.spinOnce();
  delay(1);

}
