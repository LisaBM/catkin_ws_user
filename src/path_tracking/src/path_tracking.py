#!/usr/bin/env python

import rospy
import numpy as np
import tf
from math import atan, pi, sin, cos
from std_msgs.msg import Int16, Float64
from fub_trajectory_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry

look_ahead_index = 0
Kp = 33
Xa = 1.1
speed = -200
tol = .5
point_counter = 0


class PathTracker():
    def __init__(self):
        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=1, latch=True)
        self.steering_pub = rospy.Publisher("/manual_control/steering", Int16, queue_size=1, latch=True)
        self.stop_start_pub = rospy.Publisher("/manual_control/stop_start", Int16, queue_size=1, latch=True)
        self.error_pub = rospy.Publisher("/path_tracker/error", Float64, queue_size=1, latch=True)
        self.path_sub = rospy.Subscriber("/race_trajectory", Trajectory, self.path_callback, queue_size=1)
        self.gps_sub = rospy.Subscriber("/visual_gps/odom", Odometry, self.gps_callback, queue_size=10)

    def path_callback(self, path):
        self.path = path.trajectory
        self.path += self.path[:look_ahead_index]
        self.path_sub.unregister()

    def gps_callback(self, odom):

        self.car_pos = np.array([ odom.pose.pose.position.x, odom.pose.pose.position.y ])
        (__, __, self.car_yaw) = tf.transformations.euler_from_quaternion( [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w] )

################
###   MAIN   ###
################

rospy.init_node("path_tracking")

pt = PathTracker()

while not hasattr(pt, 'car_yaw'):
    print('Habe noch kein car_yaw')
    rospy.sleep(1)
while not hasattr(pt, 'path'):
    print('Habe keinen Pfad')
    rospy.sleep(1)

print('Jetzt publishe ich')
pt.stop_start_pub.publish(0)
pt.steering_pub.publish(60)
pt.speed_pub.publish(speed)



while len(pt.path)>look_ahead_index:
    next_point = np.array([ pt.path[0].pose.position.x, pt.path[0].pose.position.y ])
    (__,__, tangent_yaw) = tf.transformations.euler_from_quaternion( [pt.path[look_ahead_index].pose.orientation.x, pt.path[look_ahead_index].pose.orientation.y, pt.path[look_ahead_index].pose.orientation.z, pt.path[look_ahead_index].pose.orientation.w] )
    tangent_normal = np.array([ -sin(tangent_yaw), cos(tangent_yaw) ])
    e_now = np.dot( tangent_normal, next_point - pt.car_pos )
    e_future = Kp * (e_now + Xa * sin( tangent_yaw - pt.car_yaw))
    wheel_angle = (e_future)

    if not np.isnan(wheel_angle):
        steering_angle = int( -3.8 *wheel_angle + 60 )

        if steering_angle > 180:
            steering_angle = 180
        if steering_angle < 0:
            steering_angle = 0

        pt.steering_pub.publish(steering_angle)

    if np.linalg.norm( pt.car_pos -next_point ) < tol:
        point_counter += 1
        print('point counter = '+ str(point_counter))
        del pt.path[0]

    pt.error_pub.publish(e_future)

pt.speed_pub.publish(0)

try:
    rospy.spin()
except KeyboardInterrupt:
    f.close()
