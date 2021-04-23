#!/usr/bin/env python3
import time
import sys
import rospy
import numpy as np
import math

from manipulator.msg import ArmCurrentPosition
from std_msgs.msg import Int64, Float64MultiArray, String, Int32MultiArray


ROSTOPIC_PREFIX = "manipulator1/"


class RobotArm:
    def __init__(self):

        rospy.init_node('manipulator_kinematics_node', anonymous=True)

        self.required_angle_pub = rospy.Publisher(ROSTOPIC_PREFIX + 'required_angle',
                                                ArmCurrentPosition, queue_size=1)
        self.gripper_pub     = rospy.Publisher(ROSTOPIC_PREFIX + 'gripper', Int64,
                                                queue_size=1)
        self.unscrewer_pub   = rospy.Publisher(ROSTOPIC_PREFIX + 'unscrewer', Int64,
                                                queue_size=1)
        self.calibrater_pub  = rospy.Publisher(ROSTOPIC_PREFIX + 'calibrater', Int64,
                                                queue_size=1)
        self.rate = rospy.Rate(50)

        rospy.Subscriber(ROSTOPIC_PREFIX + "current_angle", ArmCurrentPosition,
                        self.set_angles)

        self.angles = [0, 0, 0, 0]
        self.offsets = [0, 0, 0, 0]

        self.kc = 0.01
        self.tp = 1

        self.status = False

        self.d1 = 0.2315
        self.a2 = 0.225
        self.a3 = 0.223
        self.no_ang = True
        self.limits = [[-3*math.pi/2, 3*math.pi/2], [-3*math.pi/2, 3*math.pi/2],
                       [-3*math.pi/2, 3*math.pi/2], [-3*math.pi/2, 3*math.pi/2]]

    def set_angles(self, data):
        self.angles[0] = data.position1 - self.offsets[0]
        self.angles[1] = data.position2 - self.offsets[1]
        self.angles[2] = data.position3 - self.offsets[2]
        self.angles[3] = data.position4 - self.offsets[3]
        self.no_ang = False
        rospy.spin()

    def offset(self):
        while self.no_ang:
            continue
        self.offsets = self.angles

    def inverse_kinematics(self, x, y, z):

        alpha = -math.pi * 1.03
        r1 = math.sqrt(x**2 + y**2)
        r3 = math.sqrt(x**2 + y**2 + (z - self.d1) **2)
        if r3 > self.a2+self.a3:
            return False, {}

        desired_angle = {}
        if x == 0:
            if y == 0:
                desired_angle[0] = 0
            elif y > 0:
                desired_angle[0] = math.pi/2
            else:
                desired_angle[0] = -math.pi/2
        elif y == 0:
            if x > 0:
                desired_angle[0] = 0
            else:
                desired_angle[0] = math.pi
        else:
            desired_angle[0] = math.atan2(y, x)

        if z - self.d1 == 0:
            desired_angle[1] = math.pi/2 - math.acos((self.a2**2+r3**2-self.a3**2)/(2*self.a2*r3))
        elif z - self.d1 > 0:
            desired_angle[1] = math.atan(r1/(z-self.d1)) - math.acos((self.a2**2+r3**2-self.a3**2)/(2*self.a2*r3))
        else:
            desired_angle[1] = math.pi + math.atan(r1/(z-self.d1)) - math.acos((self.a2**2+r3**2-self.a3**2)/(2*self.a2*r3))

        desired_angle[2] = math.pi - math.acos((self.a2**2+self.a3**2-r3**2)/(2*self.a2*self.a3))
        desired_angle[3] = alpha + desired_angle[1] + desired_angle[2]
        for i in range(len(desired_angle)):
            if (desired_angle[i] < self.limits[i][0]) or (desired_angle[i] > self.limits[i][1]):
                return False, {}
        return True, desired_angle

    def go_to(self, x, y, z):
        print("Going to")
        required_angles = ArmCurrentPosition()
        reachable, desired_angle = self.inverse_kinematics(x, y, z)
        if reachable:
            while not rospy.is_shutdown():
                required_angles.position1 = desired_angle[0]
                required_angles.position2 = desired_angle[1]
                required_angles.position3 = desired_angle[2]
                required_angles.position4 = desired_angle[3]
                while (self.required_angle_pub.get_num_connections() < 0):
                    self.rate.sleep()
                self.required_angle_pub.publish(required_angles)
                self.rate.sleep()

                break
        else:
            print("unreachable point (" + str(x) + ", " + str(y) + ", " + str(z) + ")")

    def return_empty_array(self):
        info = [[[]] * 4 for i in range(3)]
        return info

    def calibrater_publish(self):
        while (self.calibrater.get_num_connections() < 0):
            self.rate.sleep()
        self.calibrater.publish(1)
        self.rate.sleep()

    def return_coordinates(self, x, y, delta_x, delta_y):
        print()
        delta_x_ = 0
        delta_y_ = 0
        coords = self.return_empty_array()
        for i in range(len(coords)):
            for j in range(len(coords[i])):
                coords[i][j] = [round(x + delta_x_, 2) ,
                                round(y + delta_y_, 2)]
                delta_y_ -= delta_y
            delta_x_ += delta_x

            print(coords[i])
        return coords

    def gripper_publisher(self, state):
        while (self.gripper_pub.get_num_connections() < 0):
            self.rate.sleep()
        self.gripper_pub.publish(state)
        self.rate.sleep()


if __name__ == '__main__':



    robot = RobotArm()
    robot.gripper_publisher(2)
    rospy.sleep(5)
    robot.gripper_publisher(2)
    rospy.sleep(5)
    robot.gripper_publisher(1)
    rospy.sleep(5)

    reagents_x       = 0.2455
    reagents_y       = 0.063
    reagents_delta_x = 0.055
    reagents_delta_y = 0.05
    reagents_z_high  = 0.3
    reagents_z_low   = 0.24

    mixing_x         = 0.2
    mixing_y         = 0.1
    mixing_delta_x   = 0.05
    mixing_delta_y   = 0.05
    mixing_z_high    = 0.4
    mixing_z_low     = 0.4

    container_x      = 0.2
    container_y      = 0.1
    container_z      = 0.4

    unscrewer_x      = 0.12
    unscrewer_y      = 0.33
    unscrewer_z_high = 0.30
    unscrewer_z_low  = 0.27

    is_exist = [[1, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0]]

    reagents_coordinates = robot.return_coordinates(reagents_x, reagents_y, reagents_delta_x, reagents_delta_y)
    mixing_coordinates   = robot.return_coordinates(mixing_x, mixing_y, mixing_delta_x, mixing_delta_y)

    for line in reagents_coordinates:
        for xy in line:
            if is_exist[reagents_coordinates.index(line)][line.index(xy)] == 1:

                robot.go_to(xy[0], xy[1], reagents_z_high)
                robot.rate.sleep()
                rospy.sleep(4)

                robot.go_to(xy[0], xy[1], reagents_z_low)
                robot.rate.sleep()
                rospy.sleep(3)
                robot.gripper_publisher(2)
                robot.rate.sleep()
                rospy.sleep(3)
                robot.go_to(xy[0], xy[1], reagents_z_high)
                robot.rate.sleep()
                rospy.sleep(3)
                robot.go_to(unscrewer_x, unscrewer_y, unscrewer_z_high)
                robot.rate.sleep()
                rospy.sleep(3)
                robot.go_to(unscrewer_x, unscrewer_y, unscrewer_z_low)
                robot.rate.sleep()
                rospy.sleep(3)
                robot.gripper_publisher(1)
                robot.rate.sleep()
                rospy.sleep(3)
                robot.go_to(unscrewer_x, unscrewer_y, unscrewer_z_high)
                robot.rate.sleep()
                rospy.sleep(3)

    '''

    while not rospy.is_shutdown():

        x, y, z = input().split()

        robot.go_to(float(x), float(y), float(z)) #0.204
        rospy.sleep(2)

        robot.rate.sleep()
    '''

    '''

    tubes = []
    tube_reagents_coords = [0.245, 0.065]
    for i in range(len(tubes)):
        print("H")
        #ubes[i] = tube_reagents_coords

    tubes = [[(x0-0.05-0.001,  y0), (x0-0.05-0.001,  y0-0.055), (x0-0.05-0.001,  y0-0.11)],
             [(x0+0.00,  y0-0.003), (x0+0.00,  y0-0.055-0.003), (x0+0.00,  y0-0.11-0.003)],
             [(x0+0.05+0.001,  y0-0.005), (x0+0.05+0.001,  y0-0.055-0.005), (x0+0.05+0.001,  y0-0.11-0.005)],
             [(x0+0.1,   y0), (x0+0.10,  y0-0.055), (x0+0.10,  y0-0.11)]]
    x1 = 0.254
    y1 = 0.065 - 0.18
    tubes2 = [[(x1-0.05, y1), (x1-0.05,  y1-0.055), (x1-0.05,  y1-0.11)],
             [(x1+0.00,  y1-0.003), (x1+0.00,  y1-0.055-0.003), (x1+0.00,  y1-0.11-0.003)],
             [(x1+0.05+0.001,  y1-0.003), (x1+0.05+0.001,  y1-0.055-0.003), (x1+0.05+0.001,  y1-0.11-0.003)],
             [(x1+0.1,   y1), (x1+0.10,  y1-0.055), (x1+0.10,  y1-0.11)]]

    is_exist = [[1, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]]
    try:
        h = 0.194 # 0.192
        h_205 = 0.205

        unscrewer_cords = [0.145+0.003, 0.315-0.001]
        container_cords = [0.260+0.02, 0.260+0.01]

        unscrewer.publish(0)
        rate.sleep()
        rospy.sleep(1)

        gripper.publish(1)
        rate.sleep()
        gripper.publish(1)
        rate.sleep()
        gripper.publish(2)
        rate.sleep()
        gripper.publish(4)
        rate.sleep()

        rospy.sleep(3)


        for line in tubes:
            for xy in line:
                if is_exist[tubes.index(line)][line.index(xy)] == 1:

                    print("----------------------------------------")
                    print("Go to the tube " + str(tubes.index(line)) + "," + str(line.index(xy)))
                    robot.go_to(xy[0], xy[1], 0.3+0.02, -math.pi) #0.204
                    rospy.sleep(3)
                    robot.go_to(xy[0], xy[1], 0.3+0.02, -math.pi) #0.204
                    rospy.sleep(3)
                    robot.go_to(xy[0], xy[1], 0.25+0.02, -math.pi) #0.204
                    rospy.sleep(2)
                    robot.go_to(xy[0], xy[1], 0.210+0.02, -math.pi) #0.204
                    rospy.sleep(2)

                    # screw
                    print("Screw the tube")
                    gripper.publish(2)
                    rate.sleep()
                    gripper.publish(2)
                    rate.sleep()
                    rospy.sleep(3)

                    print("Go higher")
                    robot.go_to(xy[0], xy[1], 0.30+0.02, -math.pi) #0.204
                    rospy.sleep(2)
                    robot.go_to(xy[0], xy[1], 0.35+0.02, -math.pi) #0.204
                    rospy.sleep(2)
                    print("Go to unscrewer...")

                    robot.go_to(unscrewer_cords[0], unscrewer_cords[1], 0.35+0.025, -math.pi) #0.204   #225
                    rospy.sleep(3)
                    robot.go_to(unscrewer_cords[0], unscrewer_cords[1], 0.35+0.025, -math.pi) #0.204   #225
                    rospy.sleep(3)
                    print("Go to unscrewer...")
                    robot.go_to(unscrewer_cords[0], unscrewer_cords[1], 0.228, -math.pi) #0.204   #225
                    rospy.sleep(2)
                    print("Open end-effector")
                    gripper.publish(2)
                    rate.sleep()
                    gripper.publish(5)
                    rate.sleep()
                    gripper.publish(5)
                    rate.sleep()
                    gripper.publish(5)
                    rate.sleep()
                    rospy.sleep(2)

                    unscrewer.publish(1)
                    rate.sleep()
                    rospy.sleep(2)

                    print("Unscrew the cap")
                    gripper.publish(3)
                    rate.sleep()
                    gripper.publish(3)
                    rate.sleep()
                    rospy.sleep(2)

                    robot.go_to(unscrewer_cords[0], unscrewer_cords[1], 0.35+0.02, -math.pi) #0.204   #225
                    rospy.sleep(2)

                    print("Go to container")
                    robot.go_to(container_cords[0], container_cords[1], 0.35+0.02, -math.pi) #0.204   #225
                    rospy.sleep(3)
                    gripper.publish(5)
                    rate.sleep()
                    gripper.publish(5)
                    rate.sleep()
                    rospy.sleep(3)

                    unscrewer.publish(0)
                    rate.sleep()
                    rospy.sleep(2)

                    print("Go to unscrewer...")
                    robot.go_to(unscrewer_cords[0], unscrewer_cords[1], 0.35+0.02, -math.pi) #0.204   #225
                    rospy.sleep(3)

                    robot.go_to(unscrewer_cords[0], unscrewer_cords[1], 0.215+0.025, -math.pi) #0.204   #225
                    rospy.sleep(3)

                    gripper.publish(2)
                    rate.sleep()
                    rospy.sleep(3)
                    robot.go_to(unscrewer_cords[0], unscrewer_cords[1], 0.35+0.02, -math.pi) #0.204   #225
                    rospy.sleep(3)

                    second = tubes2[tubes.index(line)][line.index(xy)]
                    robot.go_to(second[0], second[1], 0.35+0.02, -math.pi) #0.204
                    rospy.sleep(3)
                    robot.go_to(second[0], second[1], 0.24+0.023, -math.pi) #0.204
                    rospy.sleep(3)
                    robot.go_to(second[0], second[1], 0.21+0.023, -math.pi) #0.204
                    rospy.sleep(3)

                    gripper.publish(1)
                    rate.sleep()
                    rospy.sleep(2)

                    robot.go_to(second[0], second[1], 0.35+0.02, -math.pi) #0.204
                    rospy.sleep(3)
                    robot.go_to(second[0], second[1], 0.40+0.02, -math.pi) #0.204
                    rospy.sleep(3)

                    gripper.publish(1)
                    rate.sleep()
                    gripper.publish(1)
                    rate.sleep()
                    gripper.publish(2)
                    rate.sleep()

                    rospy.sleep(3)
        '''
