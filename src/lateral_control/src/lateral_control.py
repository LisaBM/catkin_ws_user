#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String, Float32, Int16


with open("zero_angle.txt",'r') as f:
    desired_heading = f.read(3)


rospy.init_node('image_converter', anonymous=True)


stop_start_pub = rospy.Publisher("/manual_control/stop_start",Int16, queue_size=1)
steering_pub = rospy.Publisher("/manual_control/steering",Int16, queue_size=1)
speed_pub = rospy.Publisher("/manual_control/speed",Int16, queue_size=1)



def callback(yaw):
    yaw = yaw.data
    Kp = -3
#    desired_heading = -97
    steering_angle = Kp*(float(desired_heading )- float(yaw)) + 120    
    print('error: ' +str(float(desired_heading)) + ' - ' + str(float(yaw)) )
    stop_start_pub.publish(0)
    speed_pub.publish(-150)
    steering_angle = int(round(steering_angle))
    steering_pub.publish(steering_angle)
    print('Desired heading: ' +str(desired_heading) + ', steering_angle: ' + str(steering_angle))
    
angle_sub = rospy.Subscriber("/model_car/yaw",Float32,callback, queue_size=1)


try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()
