#!/usr/bin/env python


import math
import time

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped, Twist
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String


class Controller:
    def __init__(self):
        rospy.init_node("controller_node")
        self.rate = rospy.Rate(20)
        self.imu = None
        self.local_pose = None
        self.current_state = None
        self.lla = [0, 0, 0]
        self.rpy = None
        self.takeoff_height = 2
        self.cur_target = None
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"
        self.dist = 0
        self.dist_time = 3.0
        self.state = None
        self.target_raw_pose = PositionTarget()
        self.local_pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.local_pose_cb
        )
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_cb)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        self.gps_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.gps_cb
        )
        self.set_target_position_sub = rospy.Subscriber(
            "gi/set_pose/position", PoseStamped, self.set_target_position_cb
        )
        self.set_target_yaw_sub = rospy.Subscriber(
            "gi/set_pose/orientation", Float32, self.set_target_yaw_cb
        )
        self.custom_activity_sub = rospy.Subscriber(
            "gi/set_activity/type", String, self.custom_activity_cb
        )
        self.local_target_pub = rospy.Publisher(
            "mavros/setpoint_raw/local", PositionTarget, queue_size=10
        )
        self.armService = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        print("Controller Initialized....")

    def start(self):
        while self.rpy is None:
            print("Waiting for commander....")
            time.sleep(1)
        starting = PoseStamped()
        starting.pose.position.z = self.takeoff_height
        self.cur_target = self.target(starting, self.rpy[2])

        for _ in range(10):
            self.local_target_pub.publish(self.cur_target)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            rospy.sleep(0.1)

        if self.takeoff_detection():
            print("Take off was successful")
        else:
            print("Take off failed")
            return

        while (
            self.takeoff_detection()
            and self.arm_state
            and self.offboard_state
            and (rospy.is_shutdown() is False)
        ):
            self.local_target_pub.publish(self.cur_target)

            if (self.state is "LAND") and (self.local_pose.pose.position.z <= 0.10):
                if self.disarm():
                    self.state = "DISARMED"
            rospy.sleep(0.1)

    def target(self, pos, yaw, yaw_rate=1):

        self.target_raw_pose.header.stamp = rospy.Time.now()

        self.target_raw_pose.coordinate_frame = 9

        self.target_raw_pose.position.x = pos.pose.position.x
        self.target_raw_pose.position.y = pos.pose.position.y
        self.target_raw_pose.position.z = self.takeoff_height  # constant altitude

        if round(pos.pose.position.z) == 2:
            self.target_raw_pose.velocity.x = self.dist / 3.0  # time = 3 sec
            self.target_raw_pose.velocity.z = 0
        elif self.state == "LAND":
            self.target_raw_pose.velocity.x = 0
            self.target_raw_pose.velocity.z = -self.takeoff_height / 30.0
        else:
            self.target_raw_pose.velocity.x = 0
            self.target_raw_pose.velocity.z = self.takeoff_height / 30.0

        self.target_raw_pose.type_mask = 0b101111000000

        self.target_raw_pose.yaw = yaw
        self.target_raw_pose.yaw_rate = yaw_rate

        return self.target_raw_pose

    def distance(self, cur_p, target_p):
        if isinstance(cur_p, PoseStamped) and isinstance(target_p, PoseStamped):
            delta_x = math.fabs(cur_p.pose.position.x - target_p.pose.position.x)
            delta_y = math.fabs(cur_p.pose.position.y - target_p.pose.position.y)
            delta_z = math.fabs(cur_p.pose.position.z - target_p.pose.position.z)
        elif isinstance(cur_p, PositionTarget) and isinstance(target_p, PositionTarget):
            delta_x = math.fabs(cur_p.position.x - target_p.position.x)
            delta_y = math.fabs(cur_p.position.y - target_p.position.y)
            delta_z = math.fabs(cur_p.position.z - target_p.position.z)
        elif isinstance(cur_p, PositionTarget) and isinstance(target_p, PoseStamped):
            delta_x = math.fabs(cur_p.position.x - target_p.pose.position.x)
            delta_y = math.fabs(cur_p.position.y - target_p.pose.position.y)
            delta_z = math.fabs(cur_p.position.z - target_p.pose.position.z)
        else:
            delta_x = math.fabs(cur_p.pose.position.x - target_p.pose.position.x)
            delta_y = math.fabs(cur_p.pose.position.y - target_p.pose.position.y)
            delta_z = math.fabs(cur_p.pose.position.z - target_p.pose.position.z)
        self.dist = math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)

    def local_pose_cb(self, msg):
        self.local_pose = msg

    def mavros_state_cb(self, msg):
        self.mavros_state = msg.mode

    def imu_cb(self, msg):
        self.imu = msg
        self.rpy = self.q2yaw(self.imu.orientation)
        self.received_imu = True

    def gps_cb(self, msg):
        self.gps = msg
        self.lla[0] = msg.latitude
        self.lla[1] = msg.longitude
        self.lla[2] = msg.altitude

    def set_target_position_cb(self, msg):
        print("Target Position")

        if msg.header.frame_id == "base_link":
            self.frame = "BODY"

            self.distance(self.local_pose, msg)
            ned = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

            ecef = self.ned2ecef(ned, self.lla[0], self.lla[1])
            self.local_pose.pose.position.x = ecef[0] + msg.pose.position.x
            self.local_pose.pose.position.y = ecef[1] + msg.pose.position.y
            self.local_pose.pose.position.z = ecef[2] + msg.pose.position.z
            self.cur_target = self.target(self.local_pose, self.rpy[2])

        else:
            self.frame = "ECEF"
            self.cur_target = self.target(msg, self.rpy[2])

    def custom_activity_cb(self, msg):
        print("Custom Activity:", msg.data)
        if msg.data == "LAND":
            print("LANDING")
            self.state = "LAND"
            self.local_pose.pose.position.z = 0.1
            print(self.local_pose)
            self.cur_target = self.target(
                self.local_pose,
                self.rpy[2],
            )

        else:
            print("Custom Activity:", msg.data, "not supported yet!")

    def set_target_yaw_cb(self, msg):
        print("Yaw")

        yaw_deg = msg.data
        self.cur_target = self.target(
            self.local_pose,
            yaw_deg,
        )

    def q2yaw(self, q):
        quat = [q.x, q.y, q.z, q.w]
        r, p, y = tf.transformations.euler_from_quaternion(quat)
        rpy = [r, p, y]

        return rpy

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode="OFFBOARD"):
            return True
        else:
            print("Offboard failed")
            return False

    def ned2ecef(self, ned, lat_ref, lon_ref):  # Code taken for navpy.
        np_ned = np.array(ned)
        np_ned = np_ned.T
        C = np.zeros((3, 3))
        C[0, 0] = -np.sin(lat_ref) * np.cos(lon_ref)
        C[0, 1] = -np.sin(lat_ref) * np.sin(lon_ref)
        C[0, 2] = np.cos(lat_ref)

        C[1, 0] = -np.sin(lon_ref)
        C[1, 1] = np.cos(lon_ref)
        C[1, 2] = 0

        C[2, 0] = -np.cos(lat_ref) * np.cos(lon_ref)
        C[2, 1] = -np.cos(lat_ref) * np.sin(lon_ref)
        C[2, 2] = -np.sin(lat_ref)

        # C defines transoformation: ned = C * ecef.  Hence used transpose.
        ecef = np.dot(C.T, np_ned)
        ecef = ecef.T

        return ecef.tolist()

    def takeoff_detection(self):
        if (
            self.local_pose.pose.position.z > 0.1
            and self.offboard_state
            and self.arm_state
        ):
            return True
        else:
            return False


if __name__ == "__main__":

    o = Controller()
    while not rospy.is_shutdown():
        o.start()
        o.rate.sleep()
