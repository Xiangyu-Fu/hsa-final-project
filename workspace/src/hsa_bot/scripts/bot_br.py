#!/usr/bin/env python

import rospy
import tf
from tf import transformations
import geometry_msgs.msg

def tf_broadcaster():
    rospy.init_node('tf_broadcaster')

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0) # 10Hz

    while not rospy.is_shutdown():
        # For /odom to /base_link
        br.sendTransform((0.0, 0.0, 0.0),
                         transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "/base_link",
                         "/odom")

        # For /base_link to /robot_footprint
        br.sendTransform((0.0, 0.0, 0.0),  # Just an example, adjust according to your robot's specification
                         transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "/robot_footprint",
                         "/base_link")

        rate.sleep()

if __name__ == '__main__':
    try:
        tf_broadcaster()
    except rospy.ROSInterruptException:
        pass
