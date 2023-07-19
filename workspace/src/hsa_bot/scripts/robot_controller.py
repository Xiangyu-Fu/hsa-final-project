#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import tf
import serial
import time
from math import sin, cos, pi, log
from typing import Tuple
import numpy as np


def remap_speed(n: int):
    if n < 0:
        return int((n / 255) * 185 - 70)
    elif n > 0:
        return int((n / 255) * 185 + 70)
    return 0

def rotate_clamp(left_command, right_command)-> Tuple[float, float]:
    if left_command*right_command <0.0:
        return left_command, right_command
    else:
        return left_command, right_command


class DiffDriveController:
    """Given the angular velocity and linear one, compute the wheel speed on left and right."""

    def __init__(self, wheel_radius, wheel_separation):
        # ROS publishers for the wheel velocities
        # self.left_wheel_pub = rospy.Publisher(
        #    "left_wheel_velocity", Int16, queue_size=10
        # )
        # self.right_wheel_pub = rospy.Publisher(
        #    "right_wheel_velocity", Int16, queue_size=10
        # )

        self.max_speed = 10  # maximum speed in m/s

        # Parameters
        self.wheel_radius = wheel_radius
        self.wheel_separation = wheel_separation

        # Subscribe to cmd_vel
        # rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

    def compute(self, linear_vel: float, angular_vel: float) -> Tuple[int, int]:
        # Compute the wheel velocities from linear and angular velocities
        """

        Compute the wheel velocities from linear and angular velocities

        :param linear_vel: linear velocity in m/s
        :param angular_vel: angular velocity in rad/s
        :return: left and right wheel velocities in the range -255 to 255
        """
        left_wheel_vel: float = (
            2 * linear_vel - angular_vel * self.wheel_separation
        ) / (2 * self.wheel_radius)
        right_wheel_vel: float = (
            2 * linear_vel + angular_vel * self.wheel_separation
        ) / (2 * self.wheel_radius)

        print("lr:",left_wheel_vel, right_wheel_vel)

        left_wheel_vel, right_wheel_vel = rotate_clamp(left_wheel_vel, right_wheel_vel)

        # Scale the wheel velocities to be in the range -255 to 255
        # print("compute:", left_wheel_vel, right_wheel_vel)
        return self.scale_velocity(left_wheel_vel), self.scale_velocity(right_wheel_vel)

    def scale_velocity(self, velocity: float) -> int:
        # Scale the velocity from m/s to be in the range -255 to 255
        # This assumes a maximum possible speed; adjust as necessary
        max_speed: float = 10.0  # maximum speed in m/s
        return remap_speed(int(255 * velocity / self.max_speed))


class RobotController:
    def __init__(self):
        self.serial_port = "/dev/ttyUSB0"  # Change to your serial port
        self.baud_rate = 115200  # Change to your baud rate
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        self.ser_init = False

        # self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = TransformBroadcaster()

        self.diffdrive = DiffDriveController(wheel_radius=0.035, wheel_separation=0.16)

        # new tf broadcaster
        self.br = tf.TransformBroadcaster()

        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.last_time = rospy.Time.now()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def cmd_vel_callback(self, msg):
        # print("running cmd vel calback")

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        # Translate linear_vel and angular_vel to your robot's protocol
        # e.g., "c100, 100". Please modify this part as needed

        left_wheel_vel, right_wheel_vel = self.diffdrive.compute(
            linear_vel, angular_vel
        )

        self.send_command(left_wheel_vel, right_wheel_vel)



    def send_command(self, left_command, right_command):
        command = "c{}, {}\n".format(left_command, right_command)
        command = f"c{left_command}, {right_command}\n"
        rospy.loginfo(command)
        self.ser.write(command.encode())

    def parse_sensor_data(self, data):
        # Assuming the data comes in as "w[left_velocity],[right_velocity]"
        parts = data.split(",")
        left_velocity = float(parts[0][1:])  # remove 'w' and convert to float
        right_velocity = float(parts[1])

        return left_velocity, right_velocity

    def publish_odometry(self):
        try:
            # Here, read your sensor data, e.g., "W100, 100", from the serial port
            sensor_data = self.ser.readline().strip().decode("utf-8")
            # print(sensor_data)
            if self.ser_init is not True:
                self.send_command(1, 1)
                self.ser_init = True
            left_velocity, right_velocity = self.parse_sensor_data(sensor_data)

            left_velocity = left_velocity / 1600
            right_velocity = right_velocity / 1600

            # compute odometry
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            # robot base's linear velocity is the average of the two wheels' velocities
            linear_velocity = (right_velocity + left_velocity) / 2.0
            # robot base's angular velocity is the difference in velocities of the two wheels / robot's width
            angular_velocity = (
                right_velocity - left_velocity
            ) / 0.1  # width of robot: 0.1m, modify if needed

            # compute change in x, y, and theta
            delta_x = linear_velocity * cos(self.th) * dt
            delta_y = linear_velocity * sin(self.th) * dt
            delta_th = angular_velocity * dt

            # update pose
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # create quaternion from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # create and publish odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.child_frame_id = "map"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = Quaternion(*odom_quat)

            odom.twist.twist.linear.x = linear_velocity
            odom.twist.twist.angular.z = angular_velocity

            # Publish TF transform
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0),
                odom_quat,
                current_time,
                "robot_footprint",
                "odom",
            )

        except:
            rospy.logwarn("hsa_bot_controller decipher UART warning ... ")
            pass

    def run(self):
        r = rospy.Rate(1000)  # 10 Hz
        while not rospy.is_shutdown():
            # Parse sensor_data and call publish_odometry
            self.publish_odometry()
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("hsa_bot_controller")
    controller = RobotController()
    controller.run()
