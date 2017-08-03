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
    self.image_pub_s = rospy.Publisher("/image_processing/usb_cam_s", Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    global flag
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


#    #RGB
    b,g,r = cv2.split(cv_image) 
    b = np.array(b, dtype = float)
    g = np.array(g, dtype = float)
    r = np.array(r, dtype = float)
    intensity_brg = b + g + r
    intensity_brg = np.floor(intensity_brg/3)
    intensity_brg = np.array(intensity_brg, dtype = np.uint8)

    

    ##bi_rgb
    #bi_gray_max = 255
    #bi_gray_min = 170

    #_, rgb_binary = cv2.threshold(intensity_brg,
                              #bi_gray_min,
                              #bi_gray_max,
                              #cv2.THRESH_BINARY)
    
    #HSV
    hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
    h,s,v = cv2.split(hsv)
    s_blur = cv2.GaussianBlur(s,(5,5),0)

    #bi_hsv
    bi_s_max = 255
    bi_s_min = 100
    _, s_binary = cv2.threshold(s_blur,
                              bi_s_min,
                              bi_s_max,
                              cv2.THRESH_BINARY);

    starting_points = [[50,150], [50,300 ], [240, 150], [240, 300]]
#    color_minb  = np.zeros(4)+255
#    color_ming  = np.zeros(4)+255
#    color_minr  = np.zeros(4)+255
#    color_maxg  = np.zeros(4)
#    color_maxb  = np.zeros(4)
#    color_maxr  = np.zeros(4)

    color_array = []
    mean_array = np.zeros((4,3))
    std_array = np.zeros((4, 3))
    for k,spk in enumerate(starting_points):
      color_array.append([])
      for i in range(spk[0], spk[0]+100):
        for j in range(spk[1], spk[1]+100):
          if s_binary[i,j] == 255:
            color_array[k].append([r[i,j], g[i,j], b[i,j]])
            #color_minr[k] = min(color_minr[k], r[i,j])
            #color_ming[k] = min(color_ming[k], r[i,j])
            #color_minb[k] = min(color_minb[k], r[i,j])
            #color_maxr[k] = max(color_maxr[k], r[i,j])
            #color_maxg[k] = max(color_maxg[k], r[i,j])
            #color_maxb[k] = max(color_maxb[k], r[i,j])
      mean_array[k] = np.mean(color_array[k], axis = 0)
      std_array[k] = np.std(color_array[k], axis = 0)
    print(mean_array, std_array)

#    print('red color range = ')
#    print(color_minr, color_maxr)
#    print('green color range = ')
#    print(color_ming, color_maxg)
#    print('blue color range = ')
#    print(color_minb, color_maxb)


    ##YUV
    #yuv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2YUV);
    #y,u,v = cv2.split(yuv)


    ##bi_yuv
    #bi_yuv_max = 255

    #bi_yuv_min = 200
    #_, yuv_binary = cv2.threshold(y,
                              #bi_yuv_min,
                              #bi_yuv_max,
                              #cv2.THRESH_BINARY);


    ##getting line equations from YUV
    #xy1 = np.array([
       #[i,j]
       #for i in range(120,480)
       #for j in range(120, 320)
       #if yuv_binary[i,j] == 255
       #])
    #xy2 = np.array([
       #[i,j]
       #for i in range(120,480)
       #for j in range(320, 500)
       #if yuv_binary[i,j] == 255
       #])
    #x1 = xy1[:,0]
    #x1 = np.reshape(x1, (np.shape(x1)[0], 1))
    #y1 = xy1[:,1]

    #x2 = xy2[:,0]
    #x2 = np.reshape(x2, (x2.shape[0],1))
    #y2 = xy2[:,1]

    #model_ransac1 = linear_model.RANSACRegressor(linear_model.LinearRegression())
    #model_ransac1.fit(x1, y1)
    #line1 = model_ransac1.predict([[120],[479]])
    #model_ransac2 = linear_model.RANSACRegressor(linear_model.LinearRegression())
    #model_ransac2.fit(x2, y2)
    #line2 =  model_ransac2.predict([[120],[479]])
    #print(line2)

    #image = cv2.line(cv_image, tuple([int(line1[0]),120]), tuple([int(line1[1]), 479]), 1, thickness$
    #image = cv2.line(image, tuple([int(line2[0]), 120]), tuple([int(line2[1]), 479]), 1, thickness =$



    try:
      self.image_pub_s.publish(self.bridge.cv2_to_imgmsg(s_binary, "mono8"))



    except CvBridgeError as e:
      print(e)
#    if flag == 0:
#      with open("lane_picture", "w") as f:
#        pickle.dump((rgb_binary, hsv_binary, yuv_binary), f)
#        flag = 1

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


