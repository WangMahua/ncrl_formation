import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32

servo_pin = 4  # GPIO pin number where the servo signal wire is connected

# Function to set the servo angle

fire = 0

def fire_cb(msg):
    global fire
    fire = msg
    pwm = GPIO.PWM(servo_pin, 1420)
    pwm.start(0)
    rospy.loginfo("This is an informational message.")

if __name__ == '__main__':
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servo_pin, GPIO.OUT)
    rospy.init_node('gps_init_py')
    fire_sub = rospy.Subscriber('/fire', Int32, fire_cb)

    # Create a PWM object with a frequency of 50Hz
    pwm = GPIO.PWM(servo_pin, 1050)
    pwm.start(0)

    rospy.spin()