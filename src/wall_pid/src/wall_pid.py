#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from math import sqrt, sin, cos, tan
import numpy as np
from std_msgs.msg import String, Float32, Int16
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from sklearn.linear_model import RANSACRegressor




#with open("zero_angle.txt",'r') as f:
    #desired_heading = f.read(3)

#gewuenschter Abstand zur Wand:
delta = 1


error_list = [0]*5

def setCell(x,y):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled <0:
        return
    if int(np.floor(x_scaled)) <= occupancy_grid.info.width*3./4:
        occupancy_grid.data[int(np.floor(x_scaled)), int(np.floor(y_scaled)) ] = 100


def scanCallback(scan_msg):
    #global occupancy_grid
    
    #reset grid
    #occupancy_grid.data = np.zeros((occupancy_grid.info.width, occupancy_grid.info.height))

    #for i in range(len(scan_msg.ranges)):
    #    y = (scan_msg.ranges[i]*cos(scan_msg.angle_min+(scan_msg.angle_increment*i)))
    #    x = (scan_msg.ranges[i]*sin(scan_msg.angle_min+(scan_msg.angle_increment*i)))
    #    setCell(x, y)

    #data = occupancy_grid.data
    
    #occupancy_grid.data = list(occupancy_grid.data.flatten())
    #pub_grid.publish(occupancy_grid)
    ##Regression for points that correspond to wall
    #res = occupancy_grid.info.resolution
    #points = []
    #for i in range(data.shape[0]):
    #    for j in range(data.shape[1]):
    #        if data[i][j] == 100:
    #            points.append([i,j])
    #points = np.array(points)

    #x = points[:,0]
    #x = np.reshape(x, (np.shape(x)[0], 1))
    #y = points[:,1]
    #y = np.reshape(y, (np.shape(y)[0], 1))
    
    #reg = RANSACRegressor()
    #reg.fit(x,y)
    #reg.fit(y,x)
#    print('coeffs: ' + str(reg.estimator_.coef_))

    #b_alt = 333/2 - reg.predict( [[333/2   ]] )[0]
    #m_alt = -(333/2 - reg.predict( [[333/2+100 ]] )[0] - b_alt)/100.
    #b_neu = -b_alt/tan(m_alt)
    #b_meter = b_neu  * res    

#    print('b ist ' + str(b_meter))
#    print('m ist ' + str(m_alt))
    global error_list
    global delta
    Kp = -80 
    Ki = -5
    Kd = 0

#    error = b_meter/ sqrt(m_alt**2 + 1) - delta
    data_sorted = np.sort(scan_msg.ranges[30:120])[:10]
    
    d =data_sorted[0]
    print('d = '+str(d))
    error = d-delta
#    print(error)
    error_list = error_list[1:]
    error_list.append(error)
    
    integral_error = sum(error_list)
    derivative_error = error_list[-1]-error_list[-2]
    
    
    wheel_angle = Kp*error + Ki*integral_error + Kd*derivative_error # in degree
    
    steering_angle = -3.8 * wheel_angle + 60
    
    if steering_angle > 180:
        steering_angle = 180
    if steering_angle < 0:
        steering_angle = 0

    steering_pub.publish(steering_angle)




rospy.init_node('wall_pid', anonymous=True)

# init occupancy grid
#occupancy_grid = OccupancyGrid()
#occupancy_grid.header.frame_id = "laser"
#occupancy_grid.info.resolution = .03 # in m/cell

# width x height cells
#occupancy_grid.info.width = 450/3
#occupancy_grid.info.height = 450/3

# origin is shifted at half of cell size * resolution
#occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
#occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
#occupancy_grid.info.origin.position.z = 0
#occupancy_grid.info.origin.orientation.x = 0
#occupancy_grid.info.origin.orientation.y = 0
#occupancy_grid.info.origin.orientation.z = 0
#occupancy_grid.info.origin.orientation.w = 1


stop_start_pub = rospy.Publisher("/manual_control/stop_start",Int16, queue_size=1)
steering_pub = rospy.Publisher("/manual_control/steering",Int16, queue_size=1)
speed_pub = rospy.Publisher("/manual_control/speed",Int16, queue_size=1)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)

stop_start_pub.publish(0)
speed_pub.publish(-200)
rospy.sleep(5)
steering_pub.publish(60)
speed_pub.publish(-150)

rospy.Subscriber("/scan", LaserScan, scanCallback, queue_size=100)


try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()
