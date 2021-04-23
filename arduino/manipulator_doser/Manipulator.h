#ifndef MANIPULATOR_H_
#define MANIPULATOR_H_

#include <Arduino.h>
#include <ros.h>
#include <manipulator/ArmCurrentPosition.h>

#include <Servo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <std_msgs/Int64.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

#define E1_STEP_PIN           36
#define E1_DIR_PIN            34

#define Z_STEP_PIN            46
#define Z_DIR_PIN             48

#define Y_STEP_PIN            60
#define Y_DIR_PIN             61

#define X_STEP_PIN            54
#define X_DIR_PIN             55

#define MOTOR_TYPE             1

#define JOINT_1_ENDSTOP        3
#define JOINT_2_ENDSTOP        2
#define JOINT_3_ENDSTOP       14
#define JOINT_4_ENDSTOP       15

#define JOINT_1_AVERAGE_SPEED   -2000
#define JOINT_2_AVERAGE_SPEED   -2000
#define JOINT_3_AVERAGE_SPEED   9000
#define JOINT_4_AVERAGE_SPEED   -1000

#define JOINT_1_AVERAGE_ACCEL   2000
#define JOINT_2_AVERAGE_ACCEL   5000
#define JOINT_3_AVERAGE_ACCEL   2000
#define JOINT_4_AVERAGE_ACCEL   1000

#define JOINT_1_CALIB_VAL     8900
#define JOINT_2_CALIB_VAL     5350
#define JOINT_3_CALIB_VAL    -24000
#define JOINT_4_CALIB_VAL     3900

#define STEP_1                32800
#define STEP_2                17600
#define STEP_3                70800
#define STEP_4                14000

AccelStepper joint1(MOTOR_TYPE, Z_STEP_PIN,  Z_DIR_PIN);
AccelStepper joint2(MOTOR_TYPE, Y_STEP_PIN,  Y_DIR_PIN);
AccelStepper joint3(MOTOR_TYPE, X_STEP_PIN, X_DIR_PIN);
AccelStepper joint4(MOTOR_TYPE, E1_STEP_PIN, E1_DIR_PIN);

MultiStepper steppers_1_pair;
MultiStepper steppers_2_pair;

long req_step[4];
long joints_control[4];

ros::NodeHandle nh;

manipulator::ArmCurrentPosition current_position;
manipulator::ArmCurrentPosition required_step;
std_msgs::Bool is_in_pose;

// ROS Publishers
ros::Publisher current_pose_pub("manipulator2/current_position",
&current_position);
ros::Publisher is_in_pose_pub("manipulator2/is_in_pose",
&is_in_pose);
ros::Publisher required_step_pub("manipulator2/current_step",
&required_step);

void required_angle_callback(const manipulator::ArmCurrentPosition& required_angle){
  long stepsPerRevolution[4] = {(long)STEP_1, (long)STEP_2, (long)STEP_3, (long)STEP_4};
  required_step.position1 = req_step[0] = required_angle.position1 * stepsPerRevolution[0] / (2 * M_PI);
  required_step.position2 = req_step[1] = required_angle.position2 * stepsPerRevolution[1] / (2 * M_PI);
  required_step.position3 = req_step[2] = required_angle.position3 * stepsPerRevolution[2] / (2 * M_PI);
  required_step.position4 = req_step[3] = required_angle.position4 * stepsPerRevolution[3] / (2 * M_PI);

  required_step_pub.publish(&required_step);
}

ros::Subscriber<manipulator::ArmCurrentPosition> required_angle_sub("manipulator2/required_angle",
required_angle_callback); //subscribes to joint_steps on arm

// Публикация текущего положения в шагах
void current_position_publisher() {
  current_position.position1 = joint1.currentPosition();
  current_position.position2 = joint2.currentPosition();
  current_position.position3 = joint3.currentPosition();
  current_position.position4 = joint4.currentPosition();

  current_pose_pub.publish(&current_position);
}
// Публикация статуса доезд до точки
void current_distance_status_publisher() {
  int64_t  _distances[4];
  _distances[0] = (int)joint1.distanceToGo();
  _distances[1] = (int)joint2.distanceToGo();
  _distances[2] = (int)joint3.distanceToGo();
  _distances[3] = (int)joint4.distanceToGo();
  if (_distances[0] == 0 && _distances[1] == 0 &&
     _distances[2] == 0 && _distances[3] == 0){
       is_in_pose.data = true;
     }else{
       is_in_pose.data = false;
     }
  is_in_pose_pub.publish(&is_in_pose);
}

void init_position(AccelStepper motor, int position){
  motor.setCurrentPosition(0);
  motor.moveTo(position);
  motor.runToPosition();
  motor.setCurrentPosition(0);
}

void follow_speed(AccelStepper motor, int speed){
  motor.setSpeed(speed);
  motor.runSpeed();
}

void go_to_initial_position(int motors_pair){
  if (motors_pair == 1){
    init_position(joint1, JOINT_1_CALIB_VAL);
    init_position(joint3, JOINT_3_CALIB_VAL);
  }else{
    init_position(joint2, JOINT_2_CALIB_VAL);
    init_position(joint4, JOINT_4_CALIB_VAL);
  }

}

void move_to_pose(AccelStepper motor, int position){
  motor.moveTo(position);
  motor.runToPosition();
}

bool calibrate(int motor_pair){

  bool AXIS_1 = digitalRead(JOINT_1_ENDSTOP);
  bool AXIS_2 = digitalRead(JOINT_2_ENDSTOP);
  bool AXIS_3 = digitalRead(JOINT_3_ENDSTOP);
  bool AXIS_4 = digitalRead(JOINT_4_ENDSTOP);
  Serial.println(AXIS_2);

  if (motor_pair == 1){
    if(AXIS_1){
      follow_speed(joint1, JOINT_1_AVERAGE_SPEED);
    } else{
      joint1.stop();
          if(AXIS_3){
            follow_speed(joint3, JOINT_3_AVERAGE_SPEED);
          } else{
            joint3.stop();
        }
      }
    return (!AXIS_1 && !AXIS_3);// && !AXIS_2 && !AXIS_3 && !AXIS_4);
  }else{

      if(AXIS_4){
        follow_speed(joint4, JOINT_4_AVERAGE_SPEED);
      }else{
        joint4.stop();

        if(AXIS_2){
            follow_speed(joint2, JOINT_2_AVERAGE_SPEED);
        } else{
            joint2.stop();
        }
      }
    return (!AXIS_2 && !AXIS_4);
  }
}

#endif
