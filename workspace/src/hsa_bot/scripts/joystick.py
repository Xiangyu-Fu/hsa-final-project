#!/usr/bin/env python3

import time
import rospy
from geometry_msgs.msg import Twist
from evdev import InputDevice, categorize, ecodes

key_map: dict = {
    "xBtn": 307,
    "yBtn": 308,
    "aBtn": 304,
    "bBtn": 305,
    "upBtn": 544,
    "downBtn": 545,
    "leftBtn": 546,
    "rightBtn": 547,
    "lBtn": 310,
    "rBtn": 311,
    "lTrig": 312,
    "rTrig": 313,
    "pressL": 317,
    "pressR": 318,
    "startBtn": 315,
    "selectBtn": 314,
    "lStickX": 0,
    "lStickY": 1,
    "rStickX": 2,
    "rStickY": 5,
    "dpadX": 16,
    "dpadY": 17,
}

def communication_with_evdev(pub) -> None:
    gamepad = InputDevice("/dev/input/event21")
    print(gamepad)
    for event in gamepad.read_loop():
        if event.code in key_map.values():
            print(f"event: {event.code} | value:{event.value} | type:{event.type})")
            
            # Map the controller input to linear and angular velocity
            twist = Twist()
            
            # Adjust these mappings according to your controller
            if event.code == key_map["lStickY"]:
                twist.linear.x = event.value / 32767.0  # normalize the value
            elif event.code == key_map["rStickX"]:
                twist.angular.z = event.value / 32767.0  # normalize the value
            
            print(f"twist: {twist}")
                
            pub.publish(twist)

def publish_to_cmd_vel() -> None:
    rospy.init_node('joystick_controller', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    
    try:
        communication_with_evdev(pub)
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    publish_to_cmd_vel()