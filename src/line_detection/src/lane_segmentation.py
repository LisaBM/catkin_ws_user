#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import pickle
import sys
import rospy
import cv2
import numpy as np
from sklearn import linear_model
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# from __future__ import print_function

flag = 0

class image_converter:
  def __init__(self):
    self.image_pub_rgb = rospy.Publisher("/image_processing/rgb_binary", Image, queue_size=1)
    self.image_pub_hsv = rospy.Publisher("/image_processing/hsv_binary", Image, queue_size=1)
    self.image_pub_yuv = rospy.Publisher("/image_processing/yub_binary", Image, queue_size=1)
    self.image_w_lines = rospy.Publisher("/image_processing/image_w_lines", Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    global flag
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #RGB
    b,g,r = cv2.split(cv_image) 
    b = np.array(b, dtype = float)
    g = np.array(g, dtype = float)
    r = np.array(r, dtype = float)
    intensity_brg = b + g + r
    intensity_brg = np.floor(intensity_brg/3)
    intensity_brg = np.array(intensity_brg, dtype = np.uint8)

    #bi_rgb
    bi_gray_max = 255
    bi_gray_min = 170
    _, rgb_binary = cv2.threshold(intensity_brg,
                              bi_gray_min,
                              bi_gray_max,
                              cv2.THRESH_BINARY)
    
    #HSV
    hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
    h,s,v = cv2.split(hsv)
    #bi_hsv
    bi_hsv_max = 255
    bi_hsv_min = 179
    _, hsv_binary = cv2.threshold(v,
                              bi_hsv_min,
                              bi_hsv_max,
                              cv2.THRESH_BINARY);


    #YUV
    yuv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2YUV);
    y,u,v = cv2.split(yuv)


    #bi_yuv
    bi_yuv_max = 255
    bi_yuv_min = 180
    _, yuv_binary = cv2.threshold(y,
                              bi_yuv_min,
                              bi_yuv_max,
                              cv2.THRESH_BINARY);


    #getting line equations from YUV
    xy1 = np.array([
       [i,j]
       for i in range(120,480)
       for j in range(320)
       if yuv_binary[i,j] == 255
       ])
    xy2 = np.array([
       [i,j]
       for i in range(120,480)
       for j in range(320, 640)
       if yuv_binary[i,j] == 255
       ])
    x1 = xy1[:,0]
    x1 = np.reshape(x1, (np.shape(x1)[0], 1))
    y1 = xy1[:,1]
    x2 = xy2[:,0]
    x2 = np.reshape(x2, (x2.shape[0],1))
    y2 = xy2[:,1]

    model_ransac1 = linear_model.RANSACRegressor(linear_model.LinearRegression())
    model_ransac1.fit(x1, y1)
    line1 = model_ransac1.predict([[0],[300]])
    model_ransac2 = linear_model.RANSACRegressor(linear_model.LinearRegression())
    model_ransac2.fit(x2, y2)
    line2 =  model_ransac2.predict([[320],[639]])
    print(line2)

    image = cv2.line(cv_image, tuple([0, int(line1[0])]), tuple(300, int(line1[1])]), 1)
    image = cv2.line(image, tuple([320, int(line2[0])]), tuple([639, int(line2[1])]), 1)



    try:
      self.image_pub_rgb.publish(self.bridge.cv2_to_imgmsg(rgb_binary, "mono8"))
      self.image_pub_hsv.publish(self.bridge.cv2_to_imgmsg(hsv_binary, "mono8"))
      self.image_pub_yuv.publish(self.bridge.cv2_to_imgmsg(yuv_binary, "mono8"))
      self.image_w_lines.publish(self.bridge.cv2_to_imgmsg(image, "8UC3"))

    except CvBridgeError as e:
      print(e)

    if flag == 0:
      with open("lane_picture", "w") as f:
        pickle.dump((rgb_binary, hsv_binary, yuv_binary), f)
        flag = 1

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
