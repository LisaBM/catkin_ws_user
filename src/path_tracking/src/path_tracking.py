#!/usr/bin/env python

import rospy
import numpy as np
import tf
from math import atan, pi, sin, cos
from std_msgs.msg import Int16
from fub_trajectory_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry



class PathTracker():
    def __init__(self):
        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=1, latch=True)
        self.steering_pub = rospy.Publisher("/manual_control/steering", Int16, queue_size=1, latch=True)
        self.stop_start_pub = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=1, latch=True)
        
        self.path_sub = rospy.Subscriber("/race_trajectory", Trajectory, self.path_callback, queue_size=1)
        self.gps_sub = rospy.Subscriber("/visual_gps/odom", Odometry, self.gps_callback, queue_size=10)

    def path_callback(self, path):
        #global flag
        self.path = path.trajectory
        self.path_sub.unregister()
        
    def gps_callback(self, odom):
        
        self.car_pos = np.array([ odom.pose.pose.position.x, odom.pose.pose.position.y ])
        #(__, __, self.car_yaw) = tf.transformations.euler_from_quaternion( [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w] )
        self.car_yaw =  odom.pose.pose.orientation.x

################
###   MAIN   ###
################

#flag = 0
rospy.init_node("path_tracking")

pt = PathTracker()

rospy.sleep(3)
#while flag ==  0:
    #print('Flag ist noch 0')
    #rospy.sleep(1)

print('Jetzt publishe ich')
pt.stop_start_pub.publish(0)
print(pt.steering_pub)
pt.steering_pub.publish(60)

pt.speed_pub.publish(-200)



while pt.path:
    next_point = np.array([ pt.path[0].pose.position.x, pt.path[0].pose.position.y ])
    tangent_normal = np.array([ -sin(pt.car_yaw), cos(pt.car_yaw) ])
    (__,__, tangent_yaw) = tf.transformations.euler_from_quaternion( [pt.path[0].pose.orientation.x, pt.path[0].pose.orientation.y, pt.path[0].pose.orientation.z, pt.path[0].pose.orientation.w] )

    Kp = -600
    Xa = .6
    tol = .5
    
    #if np.isnan(pt.car_pos):
    #   print( 'pt.car is NaN')
    e_now = np.dot( tangent_normal, next_point - pt.car_pos )
    #if np.isnan(pt.car_yaw):
    #    print( 'pt.car is NaN')
    e_future = e_now + Xa * sin( tangent_yaw - pt.car_yaw)
    print(pt.car_yaw)
    wheel_angle = Kp * (e_future)
    if not np.isnan(wheel_angle):
        steering_angle = int( -3.8 *wheel_angle + 60 )
    
        if steering_angle > 180:
            steering_angle = 180
        if steering_angle < 0:
            steering_angle = 0

        pt.steering_pub.publish(steering_angle)

        
    if np.linalg.norm( pt.car_pos -next_point ) < tol:
        print('deleted')
        del pt.path[0]


try:
    rospy.spin()
except KeyboardInterrupt:
    f.close()
