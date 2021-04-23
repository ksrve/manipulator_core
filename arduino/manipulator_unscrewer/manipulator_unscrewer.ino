#include "Manipulator.h"

void setup() {

    nh.initNode();
    nh.subscribe(required_angle_sub);
    nh.subscribe(gripper_sub);
    nh.subscribe(unscrewer_sub);

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

    steppers.addStepper(joint1);
    steppers.addStepper(joint2);
    steppers.addStepper(joint3);
    steppers.addStepper(joint4);
    
    init_gripper();
    init_unscrewer();

    while (!calibrate(1)){}
    go_to_initial_position(1);
    move_to_pose(joint3, 12000);
  
    while (!calibrate(2)){}
    go_to_initial_position(2);
    move_to_pose(joint3, -12000);

}


void loop() {

  joints[0] = -req_step[0];
  joints[1] = -req_step[1];
  joints[2] = req_step[2];
  joints[3] = req_step[3];

  current_position_publisher();
  current_distance_status_publisher();

  steppers.moveTo(joints);
  steppers.runSpeedToPosition();
  nh.spinOnce();
  delay(1);

}
