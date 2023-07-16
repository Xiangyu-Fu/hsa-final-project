#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf.broadcaster import TransformBroadcaster

class VelocityToOdom:

    def __init__(self):
        rospy.init_node('velocity_to_odom')

        # Broadcaster for "odom" to "base_footprint" and "base_link" to "robot_footprint"
        self.odom_broadcaster = TransformBroadcaster()
        self.base_link_broadcaster = TransformBroadcaster()

        # Publisher for "odom" topic
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        # Initialize robot's position and velocity
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscriber for "/key_vel" topic
        rospy.Subscriber("key_vel", Twist, self.key_vel_callback)

    def key_vel_callback(self, msg):
        # Assume a constant time step
        dt = 0.01

        # Compute odometry
        delta_x = msg.linear.x * dt
        delta_y = msg.linear.y * dt
        delta_theta = msg.angular.z * dt

        # Update robot's position
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create an odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "robot_footprints"

        # Set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Set the orientation (yaw only here)
        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Set the velocity
        odom.twist.twist = msg

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # # Also publish the transform over tf
        # self.odom_broadcaster.sendTransform(
        #     (self.x, self.y, 0.),
        #     tf.transformations.quaternion_from_euler(0, 0, self.theta),
        #     rospy.Time.now(),
        #     "robot_footprint",
        #     "odom"
        # )

        # # Publish the static transform from "base_link" to "robot_footprint"
        # self.base_link_broadcaster.sendTransform(
        #     (0, 0, 0),
        #     tf.transformations.quaternion_from_euler(0, 0, 0),
        #     rospy.Time.now(),
        #     "robot_footprint",
        #     "base_link"
        # )

    def run(self):
        # Use a constant rate loop
        r = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    try:
        node = VelocityToOdom()
        node.run()
    except rospy.ROSInterruptException:
        pass
