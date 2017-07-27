#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
from math import atan2
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
#from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub_gray = rospy.Publisher("/image_processing/img_gray",Image, queue_size=1)
    self.image_pub_bw = rospy.Publisher("/image_processing/img_bw", Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it gray
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 220
    ret,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    try:
      self.image_pub_bw.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
    except CvBridgeError as e:
      print(e)
   
    try:
      self.image_pub_gray.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
    except CvBridgeError as e:
      print(e)


    startingpoints = [[100, 100], [100,380], [200, 100], [200, 380], [ 360, 100], [360, 380]]
    positions = np.zeros((6, 2))
    for k, pk in enumerate(startingpoints):
      for i in range(100):
        for j in range(250):
          if thresh1[pk[0]+i,pk[1]+j] > 100:
            positions[k] = np.array([pk[0]+i, pk[1]+j ])
            break
        if positions[k][0] != 0:
	  break
#    print(positions)

    objectPoints = np.array([
    [0,0,0],
    [0,3,0],
    [3,0,0],
    [3,3,0],
    [6,0,0],
    [6,3,0]
    ], dtype = np.float)

    imagePoints = np.array(positions, dtype = np.float)

    cameraMatrix = np.array([
    [614.1699, 0       , 329.9491],
    [0       , 614.9002, 237.2788],
    [0       , 0       , 1       ]
    ], dtype = np.float)

    distCoeffs = np.array([ 0.1115, -0.1089, 0, 0 ], dtype = np.float)

    retval, rvec, tvec = cv2.solvePnP( objectPoints, imagePoints, cameraMatrix, distCoeffs)
#    print('rvec: ')
#    print(rvec)
#    print('tvec:')
#    print(tvec)
    rotationMatrix, __ = cv2.Rodrigues(rvec)
    print('Rotation matrix: ')
    print(rotationMatrix)
    tvec = np.reshape(tvec, 3)
    R0 = np.concatenate((np.transpose(rotationMatrix), np.zeros((1,3))))
    Rt1 = np.concatenate((np.dot(np.transpose(rotationMatrix),tvec), np.array([1])))
    Rt1 = np.reshape(Rt1, (4,1))
    inv_hom_transform = np.concatenate((R0,Rt1), axis = 1)
    print('inverse of the homogeneous transform: ')
    print(inv_hom_transform)
    R = rotationMatrix

    if np.linalg.det(inv_hom_transform)>1e-6:
      sy = np.sqrt(R[0,0]**2+R[1,0]**2)
      x = atan2(R[2,1], R[2,2])
      y = atan2(-R[2,0], sy)
      z = atan2(R[1,0], R[0,0])
      print("roll angle: "+str(x))
      print("pitch angle: "+str(y))
      print("yaw angle: "+str(z))

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
