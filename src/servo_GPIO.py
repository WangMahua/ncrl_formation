#!/usr/bin/env python3

import pigpio
import time
import rospy
from std_msgs.msg import Int32

servo_pin = 4  # GPIO pin number where the servo signal wire is connected

# Function to set the servo angle

fire = 0

def fire_cb(msg):
    global fire
    fire = msg.data
    if fire == 1:
        rospy.loginfo("Fire")
        pi.set_servo_pulsewidth(servo_pin, 1420)
        time.sleep(1)
        pi.set_servo_pulsewidth(servo_pin, 1050)

if __name__ == '__main__':
    rospy.init_node('gps_init_py')
    fire_sub = rospy.Subscriber('/fire', Int32, fire_cb)

    # Create a PWM object with a frequency of 50Hz
    rospy.spin()
