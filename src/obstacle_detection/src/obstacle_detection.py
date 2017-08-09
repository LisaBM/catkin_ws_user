#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from math import isnan, atan, cos, sin, pi
from sklearn import linear_model
from std_msgs.msg import String, Float32, Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


desired_dist = 0.4
error_list = [0]*5
max_speed = -200


class obstacle_detector:
    def __init__(self):
        self.image_sub = rospy.Subscriber("/app/camera/depth/image",Image,self.callback, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom/pose/pose/position", Float32, self.odom_callback, queue_size = 1)

        self.stop_start_pub = rospy.Publisher("/manual_control/stop_start",Int16, queue_size=1)
        self.steering_pub = rospy.Publisher("/manual_control/steering",Int16, queue_size=1)
        self.speed_pub = rospy.Publisher("/manual_control/speed",Int16, queue_size=1)
        self.obstacle_pub = rospy.Publisher("/obstacle/distance2car", Float32, queue_size = 1)
        self.steering_pub.publish(60)
        self.stop_start_pub.publish(0)
        self.speed_pub.publish(-150)
	self.odom_x = 0
        

    def callback(self,data):
        global error_list
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data, "32FC1")


        # Obstacle Detection
        mid = [150, 320]
        dist_obstacle = np.inf
        for i in range( 0, mid[0], 2 ):
            for j in range( mid[1] - 50, mid[1] + 50, 2):
                if not np.isnan(image[i,j]):
                    dist_obstacle = min( dist_obstacle, image[i, j] )

        # PID-controler
        Kp = -800
        Ki = 0
        Kd = 0
        if dist_obstacle == np.inf:
            self.speed_pub.publish( max_speed )
            return
        error = dist_obstacle-desired_dist
        error_list = error_list[1:]
        error_list.append(error)

        integral_error = sum(error_list)
        derivative_error = error_list[-1]-error_list[-2]

        speed = Kp*error + Ki*integral_error + Kd*derivative_error
        speed = min ( 200, speed)
        speed = max (max_speed, speed)
        self.speed_pub.publish(speed)
        self.steering_pub.publish(60)
        self.obstacle_pub.publish(dist_obstacle)

    def odom_callback(self, odom):
        self.odom_x = odom.x


def main(args):
  rospy.init_node('obstacle_detector', anonymous=True)
  od = obstacle_detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)


