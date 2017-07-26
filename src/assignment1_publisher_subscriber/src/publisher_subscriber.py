#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, String




def callback(yaw_value):
    message = 'I heard: ' + str(yaw_value)
    publisher.publish(message)

rospy.init_node("test_node")

rospy.Subscriber("/model_car/yaw", Float32, callback)
publisher = rospy.Publisher("/assignment1_publisher_subscriber", String)

rospy.spin()