#!/usr/bin/env python3
import time
import sys
import rospy
import numpy as np
import math

from manipulator.msg import ArmCurrentPosition
from manipulator.msg import DoserStatus
from std_msgs.msg import Int64, Float64MultiArray, String, Int32MultiArray


ROSTOPIC_PREFIX = "manipulators/"
ROSTOPIC_PREFIX_ = "manipulator2/"

class RobotArm:
    def __init__(self):

        rospy.init_node('manipulator_2', anonymous=True)

        rospy.Subscriber(ROSTOPIC_PREFIX_ + "current_angle", ArmCurrentPosition,
                        self.set_angles)
        rospy.Subscriber(ROSTOPIC_PREFIX + "reagents",          Float64MultiArray,
                        self.set_reagents)
        rospy.Subscriber(ROSTOPIC_PREFIX + "mixing",            Float64MultiArray,
                        self.set_mixing)

        self.required_angle_pub = rospy.Publisher(ROSTOPIC_PREFIX_ + 'required_angle',
                                                ArmCurrentPosition, queue_size=10)
        self.calibrater_pub  = rospy.Publisher(ROSTOPIC_PREFIX_ + 'calibrater', Int64,
                                                queue_size=1)
        self.doser_pub  = rospy.Publisher(ROSTOPIC_PREFIX_ + 'doser', DoserStatus,
                                                queue_size=1)
        self.rate = rospy.Rate(10)

        self.angles = [0, 0, 0, 0]
        self.offsets = [0, 0, 0, 0]

        self.status = False
        self.existance_array = []
        self.info_array = []
        self.mixing_exist_array = []
        self.mixing_info_array = []

        self.REAGENTS_DESCRIPTION = ["ReagentNum", "C", "W", "p", "volume", "row", "column"]
        self.MIXING_DESCRIPTION = ["ReagentNum1", "ReagentNum2", "C", "W", "p", "volume", "row", "column"]
        self.MISSION_DESCRIPTION = ["Reagent_row", "Reagent_column", "Water_row", "Water_column", "Mixs_row",
                                    "Mixs_column", "Reags_volume" ,"Water_volume"]

        self.end = []

        self.REAGENTS_DESCRIPTION_1 = [[1, 3.667, 13, 1, 35000, 0, 0], [2, 3.667, 13, 1, 35000, 0, 1],
                                      [-1, -1, -1, -1, 45000, 0, 2], [3, 3.667, 13, 1, 35000, 1, 0]]
        self.MIXING_DESCRIPTION_1 = [[3, -1, 1.41, 5, 1, 20000, 0, 2, 1],
                                    [1, -1, 3.103, 11, 1, 10000, 0, 0, 1],
                                    [2, -1, 1.41, 5, 1, 20000, 0, 1, 1],
                                    [1, -1, 1.41, 5, 1, 5000, 1, 1, 1]]

        self.d1 = 0.2315
        self.a2 = 0.225
        self.a3 = 0.223
        self.no_ang = True
        self.limits = [[-3*math.pi/2, 3*math.pi/2], [-3*math.pi/2, 3*math.pi/2],
                       [-3*math.pi/2, 3*math.pi/2], [-3*math.pi/2, 3*math.pi/2]]

    def return_empty_array(self):
        info = [[[]] * 3 for i in range(4)]
        return info

    def set_reagents(self, data):
        self.status = data.status_server
        self.reagents = (np.reshape(np.array(data.data.data), (-1, 6))).tolist()
        rospy.spin()

    def set_mixing(self, data):
        self.mixing = (np.reshape(np.array(data.data.data), (-1, 8))).tolist()
        rospy.spin()

    def mixing(self, c_in, v_water, v_reagent, c_out, v_out):
        v_reagent_needed = c_out / c_in * v_out
        v_water_needed = v_out * (1 - c_out / c_in)
        if v_reagent_needed <= v_reagent and v_water_needed <= v_water:
            return True, int(v_reagent_needed), int(v_water_needed)
        return False, 0, 0


    def data_parser(self, data, value1, value2, dic):
        exist_array = []
        info_array  = []
        _info = []
        id = 0
        for key in data:
            exist_array.append(key[value1:value2])
            id_keys = key[0:value2]
            info_array.append(id_keys)
            id+=1
        info = self.return_empty_array()
        for (i, j) in zip(exist_array, info_array):
            info[i[0]][i[1]] = dict(zip(dic, j))
        return info

    def data_restructarization(self, arr1, arr2):
        waters = []
        mission = []
        for i in arr2:
            for j in i:
                if any(j):
                    if j["ReagentNum"] == -1:
                        waters.append(j)
        for arr1_iter in arr1:
            for mixs in arr1_iter:
                if any(mixs):
                    for arr2_iter in arr2:
                        for reags in arr2_iter:
                            if any(reags) and any(mixs):
                                if reags["ReagentNum"] == mixs["ReagentNum1"]:
                                    for water_iter in waters:
                                        _, v_reag, v_water = self.mixing(reags["C"], water_iter["volume"], reags["volume"],
                                                                                mixs["C"], mixs["volume"])
                                        if _:
                                            water_iter["volume"] = water_iter["volume"] - int(v_water)
                                            reags["volume"] = reags["volume"] - int(v_reag)
                                            mission.append([reags["row"], reags["column"],
                                                                water_iter["row"],     water_iter["column"],
                                                                mixs["row"],  mixs["column"],
                                                                v_reag,  v_water])
                                            mixs = {}
                                        else:
                                            continue
        return mission

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

        alpha = -math.pi/2.5
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

    def goto_point(self, x, y, z):
        reachable, desired_angle = self.inverse_kinematics(x, y, z)
        print("Going to point  " + str(x) + ", " + str(y) + ", " + str(z))
        if reachable:
            self.required_angles_publisher(desired_angle)
        else:
            print("unreachable point (" + str(x) + ", " + str(y) + ", " + str(z) + ")")

    def required_angles_publisher(self, angles):
        required_angles = ArmCurrentPosition()
        required_angles.position1=angles[0]
        required_angles.position2=angles[1]
        required_angles.position3=angles[2]
        required_angles.position4=angles[3]
        rospy.sleep(5)
        while (self.required_angle_pub.get_num_connections() < 0):
            self.rate.sleep()
        self.required_angle_pub.publish(required_angles)
        self.rate.sleep()


    def doser_publish(self, action, volume):
        doser_msg = DoserStatus()
        doser_msg.action = action
        doser_msg.volume = volume
        while (self.doser_pub.get_num_connections() < 0):
            self.rate.sleep()
        self.doser_pub.publish(doser_msg)
        self.rate.sleep()

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

    def takeliquid(self, x_reagents, y_reagents, z_high, z_low,
                         x_mix, y_mix, z_high, z_low,
                         volume):
        while volume != 0:
            self.goto_point(x, y, z_high)
            self.goto_point(x, y, z_low)
            if volume <= 5000:
                doser_publish(1, volume)
            else:
                doser_publish(1, 5000)
            self.goto_point(x, y, z_high)

    def take_pipete(self):
        self.goto_point(x, y, z_high)

if __name__ == '__main__':

    robot = RobotArm()

    reagents_matrix = robot.data_parser(robot.REAGENTS_DESCRIPTION_1, 5, 7, robot.REAGENTS_DESCRIPTION)
    mixing_matrix = robot.data_parser(robot.MIXING_DESCRIPTION_1, 6, 8, robot.MIXING_DESCRIPTION)
    mission = robot.data_restructarization(mixing_matrix, reagents_matrix)

    reagents_x       = 0.2
    reagents_y       = 0.1
    reagents_delta_x = 0.05
    reagents_delta_y = 0.05
    reagents_z_high  = 0.4
    reagents_z_low   = 0.3

    pipets_x         = 0.05
    pipets_y         = 0.2
    pipets_delta_x   = 0.05
    pipets_delta_y   = 0.05
    pipets_z_high    = 0.4
    pipets_z_low     = 0.4

    mixing_x         = 0.2
    mixing_y         = 0.1
    mixing_delta_x   = 0.05
    mixing_delta_y   = 0.05
    mixing_z_high    = 0.4
    mixing_z_low     = 0.4

    container_x      = 0.2
    container_y      = 0.1
    mixing_z_high    = 0.4

    pipets_exist = [[1, 1, 1, 1, 1, 1],
                    [1, 1, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0]]

    reagents_coordinates = robot.return_coordinates(reagents_x, reagents_y, reagents_delta_x, reagents_delta_y)
    pipets_coordinates   = robot.return_coordinates(pipets_x, pipets_y, pipets_delta_x, pipets_delta_y)
    mixing_coordinates   = robot.return_coordinates(mixing_x, mixing_y, mixing_delta_x, mixing_delta_y)

    step_old = None

    for (step, reag, mix) in zip(mission, reagents_coordinates, mixing_coordinates):
        for (i, j) in zip(reag, mix):
            if any(step):
                if step_old == step:
                    #take liquid
                    #robot.goto_point(i[step[0]], i[step[1]], reagents_z_high)
                    #robot.goto_point(i[step[0]], i[step[1]], reagents_z_low)
                else:
                    #take pipete
                step_old = step
