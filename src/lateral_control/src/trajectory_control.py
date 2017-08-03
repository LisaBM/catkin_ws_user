#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32, Int16
from nav_msgs.msg import Odometry

error_list = [0]*5
rospy.init_node('trajectory_control', anonymous=True)

stop_start_pub = rospy.Publisher("/manual_control/stop_start",Int16, queue_size=1)
steering_pub = rospy.Publisher("/manual_control/steering",Int16, queue_size=1)
speed_pub = rospy.Publisher("/manual_control/speed",Int16, queue_size=1)

stop_start_pub.publish(0)
straight_angle = 60

#with open("zero_y.txt", 'r') as f:
#    zero_y = float(f.read(4))

def callback(odometry):
    global error_list
    Kp = 120 
    Ki = 0
    Kd = 0
    current_y = odometry.pose.pose.position.y
#    desired_y = zero_y + .5
    desired_y = 0.5
    error = desired_y - current_y
    error_list = error_list[1:]
    error_list.append(error)
    integral_error = sum(error_list)
    derivative_error = error_list[-1]-error_list[-2]
    steering_angle = straight_angle + Kp*error + Ki*integral_error + Kd*derivative_error
    if steering_angle > 180:
   	steering_angle = 180
    if steering_angle < 0:
        steering_angle = 0


    speed_pub.publish(-300)
    steering_pub.publish(steering_angle)

odom_sub = rospy.Subscriber("/odom",Odometry ,callback,  queue_size=100)




try:

#    r= rospy.Rate(10)
#    while not rospy.is_shutdown():
#   	r.sleep()
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()


