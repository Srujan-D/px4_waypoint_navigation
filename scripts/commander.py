#!/usr/bin/env python


import math
import time

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String


class Commander:
    def __init__(self):
        rospy.init_node("commander_node")
        self.rate = rospy.Rate(20)
        self.position_target_pub = rospy.Publisher(
            "gi/set_pose/position", PoseStamped, queue_size=10
        )
        self.yaw_target_pub = rospy.Publisher(
            "gi/set_pose/orientation", Float32, queue_size=10
        )
        self.custom_activity_pub = rospy.Publisher(
            "gi/set_activity/type", String, queue_size=10
        )
        self.armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def move(self, pos, BODY_OFFSET_ENU=True):
        self.position_target_pub.publish(self.set_pose(pos, BODY_OFFSET_ENU))

    def turn(self, yaw_degree):
        self.yaw_target_pub.publish(yaw_degree)

    def land(self):
        self.custom_activity_pub.publish(String("LAND"))

    def set_pose(self, pos, BODY_FLU=True):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"

        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]

        return pose


if __name__ == "__main__":
    pos = []  # [(0, 0, 2), (1, 1, 2), (0, 1, 2), (0, 0, 2)]
    o = Commander()
    i = 0
    f = open("waypoints.txt", "r")
    for line in f:
        coord = line.split(",")
        pos.append([float(coord[0]), float(coord[1]), float(coord[2])])
    end = len(pos)
    while not rospy.is_shutdown():
        for i in range(len(pos)):
            if o.arm():
                o.move(pos[i])
                rospy.sleep(3)
            if i != 0:
                o.turn(
                    (1 / math.pi)
                    * 180
                    * math.atan2(
                        (pos[i][1] - pos[i - 1][1]), (pos[i][0] - pos[i - 1][0])
                    )
                )
            rospy.sleep(3)
        if i == (end - 1):
            o.land()
            rospy.sleep(3)
            break
