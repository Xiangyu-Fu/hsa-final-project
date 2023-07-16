#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import tf
import serial
import time
from math import sin, cos, pi

class RobotController:
    def __init__(self):
        self.serial_port = '/dev/ttyUSB1'  # Change to your serial port
        self.baud_rate = 115200  # Change to your baud rate
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

        self.ser_init = False

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = TransformBroadcaster()

        # new tf broadcaster
        self.br = tf.TransformBroadcaster()
        
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.last_time = rospy.Time.now()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        # Translate linear_vel and angular_vel to your robot's protocol
        # e.g., "c100, 100". Please modify this part as needed
        self.send_command(linear_vel, angular_vel)

    def send_command(self, left_command, right_command):
        command = "c{}, {}\n".format(left_command, right_command)
        print(command)
        self.ser.write(command.encode())

    def parse_sensor_data(self, data):
        # Assuming the data comes in as "w[left_velocity],[right_velocity]"
        parts = data.split(',')
        left_velocity = float(parts[0][1:])  # remove 'w' and convert to float
        right_velocity = float(parts[1])

        return left_velocity, right_velocity

    def publish_odometry(self):
        
        try:
            # Here, read your sensor data, e.g., "W100, 100", from the serial port
            sensor_data = self.ser.readline().strip().decode('utf-8')
            # print(sensor_data)
            if self.ser_init is not True:
                self.send_command(1, 1)
                self.ser_init = True
            left_velocity, right_velocity = self.parse_sensor_data(sensor_data)

            left_velocity = left_velocity / 1000
            right_velocity = right_velocity / 1000

            # compute odometry
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            # robot base's linear velocity is the average of the two wheels' velocities
            linear_velocity = (right_velocity + left_velocity) / 2.0
            # robot base's angular velocity is the difference in velocities of the two wheels / robot's width
            angular_velocity = (right_velocity - left_velocity) / 0.1  # width of robot: 0.1m, modify if needed

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

            # print(self.x, self.y)

            # self.odom_pub.publish(odom)

            # Publish TF transform
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0),
                odom_quat,
                current_time,
                "robot_footprint",
                "odom",
            )
               
        except:
            print("hsa_bot_controller decipher UART warning ... ")
            pass

    def run(self):
        r = rospy.Rate(1000)  # 10 Hz
        while not rospy.is_shutdown():
            # Parse sensor_data and call publish_odometry
            self.publish_odometry()
            r.sleep()


if __name__ == "__main__":
    rospy.init_node('hsa_bot_controller')
    controller = RobotController()
    controller.run()
