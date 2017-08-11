#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import pickle
import sys
import rospy
import cv2
import numpy as np
from math import isnan, atan, cos, sin, pi, atan2
from sklearn import linear_model
from std_msgs.msg import String, Float32, Float64
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import tf

# from __future__ import print_function

xt_list = [[0]*2]*2
sigmat_list = [[[0]*2]*2]*2 
flag = 0
vorzeichen = 1


class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/usb_cam_s", Image, queue_size=1)
    self.gps_pub = rospy.Publisher("/visual_gps/odom", Odometry, queue_size = 10)

    self.bridge = CvBridge()

    self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.callback, queue_size=1)
    self.yaw_sub = rospy.Subscriber("/model_car/yaw", Float32, self.yaw_callback, queue_size=1) 
    #self.speed_sub = rospy.Subscriber("/model_car/twist/linear/x", Float64, self.speed_callback, queue_size=1)
    self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=1)

  def yaw_callback(self, yaw):
    self.yaw = yaw

  #def speed_callback(self, speed):
    #self.speed = speed/5.5-*0.031 # velocity in m/s
  def odom_callback(self, odom):
    self.odom_x = odom.pose.pose.position.x
    self.odom_y = odom.pose.pose.position.y
    (__, __, self.odom_yaw) = tf.transformations.euler_from_quaternion( [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w] )


  def callback(self,data):

    global flag
    global vorzeichen

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ### BUILD RGB ###
    b,g,r = cv2.split(cv_image)
    #r_blur = cv2.GaussianBlur(r,(3,3),0)
    #g_blur = cv2.GaussianBlur(g,(3,3),0)
    #b_blur = cv2.GaussianBlur(b,(3,3),0)
    #rgb_blur = cv2.GaussianBlur(cv_image, (3,3), 0)


    ### BUILD HSV ###
    #hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV);
    #_,s,_ = cv2.split(hsv)
    #s_blur = cv2.GaussianBlur(s,(5,5),0)

    ### S BINARY
    #bi_s_max = 255
    #bi_s_min = 140
    #_, s_binary = cv2.threshold(s_blur,
    #                          bi_s_min,
    #                          bi_s_max,
    #                          cv2.THRESH_BINARY);



    ### BUILD YUV ###
    yuv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2YUV);
    _,_,v = cv2.split(yuv)
    #v_blur = cv2.GaussianBlur(v,(3,3),0)


    #Testen, in welchen Quadraten die Ballons sitzen ( im Testbag)
    #hoehe = 180
    #breite = 455
    #image_lines = s_binary
    #image_lines= cv2.line(image_lines, tuple([ 0, hoehe ]), tuple([479, hoehe]), (255,250,250),2)
    #image_lines= cv2.line(image_lines, tuple([ 0, hoehe + 50 ]), tuple([479, hoehe + 50]), (255,250,250),2)
    #image_lines= cv2.line(image_lines, tuple([  breite,0 ]), tuple([breite, 479]), (255,250,250),2)
    #image_lines= cv2.line(image_lines, tuple([  breite + 50 ,0]), tuple([breite+ 50, 479]), (255,250,250),2)
    


    ## FINDING COLOR RANGES FOR THE DIFFERENT BALLOONS ###

    #starting_points = [[110, 455 ], [180, 455], [110, 350], [180, 350]]

    #color_array = [] # rot, lila, blau, gruen
    #Zeilen sind Farben:
    #mean_array = np.zeros((4, 4))
    #std_array = np.zeros((4, 4))
    
    #for k,spk in enumerate(starting_points):
    #  color_array.append([])
    #  for i in range( spk[0], spk[0] + 50 ):
    #    for j in range( spk[1], spk[1] + 50 ):
    #      if s_binary[i,j] == 255:
    #        color_array[k].append([ r[i,j], g[i,j], b[i,j], v[i,j] ])

            
     # #print('Fuer Ballon ' + str(k) + ' haben wir ' + str(len(color_array[k])) + ' Pixel')
     # mean_array[k] = np.mean(color_array[k], axis = 0)
     # std_array[k] = np.std(color_array[k], axis = 0)
    #print(mean_array, std_array)


    ## alte Farbranges:
    mean_array = np.array([
        [ 110,   36,   51 ,  172],
        [  60,   31,  175,  131 ],
        [  24,   27,  184,  110],
        [  19,   85,   32,   92]
        ])
    std_array = np.array([
        #[ 16,  11 ,  11,   6],   ###orig
        [40,   11,   11,   10],

        [ 38,  22,  76,   7],   ###orig
        #[ 38,  22,  40,   7],

        [ 11,  11,  80,   8],   ###orig
        #[ 11,  11,  40,   8],

        #[  9,  24,  13,   9]    ###orig
        [  20,  24,  13,   20]
        ])
        
    #### neue Farbranges:
    #mean_array = np.array([
        #[  97,   22,   28,  173],
        #[  55,   19,  133,  139],
        #[  25,   44,  105,  110],
        #[  23,   77,   10,  102]
        #])
    #std_array = np.array([
        #[ 40 ,  17, 17,  17],
        #[ 25,  15,  41,   5],
        #[  5,   6,  18,   5],
        #[  9,  24,   8,  10]
        #])


    ### define range of balloons in RGB
    
    #from old mean and std:
    #lower_rgb = np.array([[94, 25, 40], #r, g, b
                         #[22, 9 , 99],
                         #[13, 16, 104],
                         #[10, 61, 19]])
    #lower_rgb = np.fliplr(lower_rgb) # flip to b, g, r

    #upper_rgb = np.array([[126,47, 62], # r, g, b
                         #[98, 53, 251],
                         #[35, 38, 255],
                         #[28, 109, 45]])
    #upper_rgb = np.fliplr(upper_rgb) # flip to b, g, r

    #lower_v = np.array([[166],
                       #[124],
                       #[102],
                       #[83]])
    #upper_v = np.array([[178],
                       #[138],
                       #[118],
                       #[101]])

    ### from new mean and std:
    lower_rgb = mean_array[:, :3] - std_array[:, :3]
    lower_rgb = np.fliplr(lower_rgb) # flip to b, g, r

    upper_rgb = mean_array[:, :3] + std_array[:, :3]
    upper_rgb = np.fliplr(upper_rgb) # flip to b, g, r

    lower_v = mean_array[:, 3] - std_array[:, 3]
    upper_v = mean_array[:, 3] + std_array[:, 3]
    
    #cheat: take old range for red balloon
    lower_rgb[0, :] = [40, 25, 94]
    upper_rgb[0, :] = [62, 47, 126]
    lower_v[0] = 166
    upper_v[0] = 178


    #print(lower_rgb, upper_rgb, lower_v, upper_v)

    #image = cv_image
    #images = {}
    balloons_seen = [0]*4
    balloon_pos_im = [0]*4

    for k in range(4):
        mask_rgb = cv2.inRange( cv_image, lower_rgb[k], upper_rgb[k] )
        mask_v   = cv2.inRange( v       , lower_v[k],   upper_v[k]   )
        just_balloon = cv2.bitwise_and(mask_rgb, mask_v)
        #images[k] = cv2.bitwise_and(mask_rgb, mask_v)
        indices = np.nonzero(just_balloon)
        balloon_pos_im[k] = [ np.mean(indices[0]), np.mean(indices[1]) ]
        if not isnan(balloon_pos_im[k][0]): # check, if balloon appeared in picture
            #print(k,  len(indices[0]))
            balloons_seen[k] =1
        #    image = cv2.circle(image, tuple([ int(balloon_pos_im[k][1]) ,int( balloon_pos_im[k][0]) ]), 6, upper_rgb[k], 2)
        #print(balloon_pos_im[k])
#    print(balloons_seen)
#    print( 'gruener Ballon: ' + str(balloon_pos_im[3]))
    ### Testen, ob Farberkennung funktioniert:
    #max_length = [8, 40, 30, 10]
    #binary = r
    #balloon_pos_im = np.zeros((4,2))
    #for k in range(4): #Ballon
        #colored_points_k = []
        #for i in range(480): # Zeile
            #if len(colored_points_k) == max_length[k]:
                #break
            #for j in range(640): # Spalte
                #binary[i,j]=0
                #if len(colored_points_k) == max_length[k]:
                    #break
                #if mean_array[k][0] - std_array[k][0] <= r_blur[i,j] <= mean_array[k][0] + std_array[k][0]: # Rotwert
                    #if mean_array[k][1] - std_array[k][1] <= g_blur[i,j] <= mean_array[k][1] + std_array[k][1]: #Gruenwert
                        #if mean_array[k][2] - std_array[k][2] <= b_blur[i,j] <= mean_array[k][2] + std_array[k][2]: #Blauwert
                            #if mean_array[k][3] - std_array[k][3] <= v_blur[i,j] <= mean_array[k][3] + std_array[k][3]: #V-Wert (von YUV)
                                #binary[i,j] = 50+(k+1)*50
                                #colored_points_k.append([i,j])
        ##print('laenge von Liste, die zu Farbe k gehoert: ' + str(len(colored_points_k)))
        #balloon_pos_im[k] = np.mean(np.array(colored_points_k), axis = 0)


    balloon_pos_rw = [
        [3.57, -3    ],
        [2.33, -3    ],
        [3.57, -1.15 ],
        [2.33, -1.15 ]]

    #centers of the four balloons
    center_im = np.mean(
        np.array([ balloon_pos_im[k] for k in range(4) if balloons_seen[k] ]),
        axis=0
        )
    #image = cv2.circle(image, tuple([ int(center_im[1]) ,int( center_im[0])]), 3, (0,0,0), 2)
#    print('center im ', center_im)
    center_rw = np.mean(
        np.array([ balloon_pos_rw[k] for k in range(4) if balloons_seen[k] ]),
        axis=0)
#    print('center  rw ', center_rw)

    # image = cv2.line(v, tuple([ balloon_pos_im[0][1] , balloon_pos_im[0][0] ]), tuple([ balloon_pos_im[1][1] , balloon_pos_im[1][0] ]), (255,150,70), 2)
    # image = cv2.line(image, tuple([ balloon_pos_im[2][1] , balloon_pos_im[2][0] ]), tuple([ balloon_pos_im[3][1] , balloon_pos_im[3][0] ]), (255,150,70), 2)

    # calculate rotation matrix
    scaling = []
    rotation_angle = []

    for k in range(4):
        if balloons_seen[k]:
            scaling.append( np.sqrt(  ((balloon_pos_rw[k][0]-center_rw[0])**2+( balloon_pos_rw[k][1]-center_rw[1] )**2) / float(((balloon_pos_im[k][0]-center_im[0])**2+( balloon_pos_im[k][1]-center_im[1] )**2)) ) )

            atan_1 = atan2( (balloon_pos_rw[k][1] - center_rw[1] ), (balloon_pos_rw[k][0] - center_rw[0]) )
            #atan1 = atan( (balloon_pos_rw[k][1] - center_rw[1] )/ (balloon_pos_rw[k][0] - center_rw[0]) )
            #if balloon_pos_rw[k][0] - center_rw[0] <0:
            #    atan1 +=np.sign(balloon_pos_rw[k][1] - center_rw[1]) * pi
            atan_2 = atan2( (balloon_pos_im[k][1] - center_im[1]), (balloon_pos_im[k][0] - center_im[0]) )
            #atan2 = atan( (balloon_pos_im[k][1] - center_im[1])/ (balloon_pos_im[k][0] - center_im[0]) )
            #if balloon_pos_im[k][0] - center_im[0] <0:
            #    atan2 += pi * np.sign(balloon_pos_im[k][1] - center_im[1])

            #modulo 2*pi:

            rotation_angle.append( atan_1 - atan_2 )
            #if rotation_angle[-1] > 2*pi:
                #rotation_angle[-1] -= 2*pi
            #if rotation_angle[-1] < 0:
                #rotation_angle[-1] += 2*pi



    scaling_mean = np.mean( scaling[1:])
    counter = 0
    if rotation_angle:
        while np.max(rotation_angle) - np.min(rotation_angle) > pi:
            #print('alpha: ', rotation_angle)
            counter += 1
            rotation_angle[ np.argmax(rotation_angle) ] = rotation_angle[ np.argmax(rotation_angle) ] - 2*pi
            if counter > 3:
                balloons_seen = [0]*4
                break
    #print(balloons_seen)

        rotation_angle_mean = np.mean( rotation_angle[1:], axis = 0 )
    #print( 'angle: ' + str(rotation_angle_mean))
    #print( 'scaling: ' + str(scaling_mean))

        R = scaling_mean * np.array([
            [cos(rotation_angle_mean), -sin(rotation_angle_mean)],
            [sin(rotation_angle_mean),  cos(rotation_angle_mean)]
            ])

        current_pos_im = [226, 317]
        current_pos_rw = np.dot(R, current_pos_im - center_im) + center_rw

    
    #### Kalman-Filter ###
    #global sigmat_list
    #global xt_list
    #var_measurement = np.array([[0.19, 0], [0,0.46]])
    #sigmat_list[0] = sigmat_list[1]
    #xt_list[0] = xt_list[1] # das ist nun x_{t-1}, also x_t aus dem vorherigen Schritt
   

    ##Forecast
    #x_bar = xt_list[0]) + vel * np.array([ cos(yaw) , sin(yaw)]
    
    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)
      
#    if flag == 0:
#      with open("lane_picture", "w") as f:
#        pickle.dump((rgb_binary, hsv_binary, yuv_binary), f)
#        flag = 1
    
    odom1 = Odometry()
    odom1.header.frame_id  = 'map'
    odom1.header.seq = data.header.seq
    
    if sum(balloons_seen) >= 3:
        odom1.pose.pose.position.x = current_pos_rw[0]
        odom1.pose.pose.position.y = vorzeichen*current_pos_rw[1]

        [x,y] = np.dot( R, np.array([-1, 0]) )
        alpha = atan2( y, x )
        #if x  == 0:
            #alpha = pi * np.sign( y )
        #else:
            #alpha = atan( y / float(x) )
        #if x <0:
            #alpha += pi
        # modulo 2*pi:
        if alpha > 2*pi:
            alpha -= 2*pi
        if alpha  < 0:
            alpha += 2*pi
        #print(alpha)
        # if sum(balloons_seen) >= 3:
        odom1.pose.pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, alpha) )
        # odom.pose.pose.orientation = Quaternion([ alpha, 0,0,0])
    	self.last_gps_car_pos = current_pos_rw
    	self.last_gps_car_yaw = alpha
    	self.last_odom_pos = [ self.odom_x, self.odom_y ]
    	self.last_odom_yaw = self.odom_yaw
    	angle = self.last_gps_car_yaw - self.odom_yaw
    	self.rotation_matrix_odom2rw = [[cos(angle), -sin(angle)], [sin(angle),cos(angle)]]

    	self.gps_pub.publish( odom1 )

    	flag = 1
    elif flag == 1:
        #print('Hey')
        #self.last_gps_car_pos = self.last_gps_car_pos + time_delta * self.speed * np.array([ cos(self.car_yaw) , sin(self.car_yaw) ])
        delta_yaw = self.last_odom_yaw - self.odom_yaw
        delta_pos = np.dot(self.rotation_matrix_odom2rw, (np.array([self.odom_x, self.odom_y])-np.array(self.last_odom_pos)))
        [ odom1.pose.pose.position.x, odom1.pose.pose.position.y] =  self.last_gps_car_pos + delta_pos
        odom1.pose.pose.position.y *= vorzeichen
        odom1.pose.pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, self.last_gps_car_yaw + delta_yaw) )

        self.gps_pub.publish( odom1 )

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


