"""
This file contains the node of real robot to 
1) read from cmd_vel topic
2) use the package diff_drive_controller to control the robot
3) calculate the odometry of the robot
4) broadcast the odometry of the robot
"""

import rospy
from geometry_msgs.msg import Twist


def callback(data: Twist):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    return


def cmd_vel_listener():
    rospy.init_node('cmd_vel_listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()
