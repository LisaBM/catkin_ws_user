#!/usr/bin/env python

# --- imports ---
from math import sin, cos
import rospy
import numpy as np
import pickle
import cv2
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from sklearn.linear_model import RANSACRegressor


flag = 0 
# --- definitions ---

# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"
def setCell(x,y):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    #offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    #occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] =100
    occupancy_grid.data[int(np.floor(x_scaled)), int(np.floor(y_scaled)) ] = 100

def scanCallback(scan_msg):
    global occupancy_grid
    global flag
    
    #reset grid
    occupancy_grid.data = np.zeros((occupancy_grid.info.width, occupancy_grid.info.height))

    for i in range(len(scan_msg.ranges)):
        y = (scan_msg.ranges[i]*cos(scan_msg.angle_min+(scan_msg.angle_increment*i)))
        x = (scan_msg.ranges[i]*sin(scan_msg.angle_min+(scan_msg.angle_increment*i)))
        if x<333/2:
          setCell(x, y)

    #if flag == 10:
        #with open("laser_matrix", "w") as f:
            #pickle.dump(occupancy_grid.data, f)
    data = occupancy_grid.data

    # convert scan measurements into an occupancy grid    
    occupancy_grid.data = list(occupancy_grid.data.flatten())
    pub_grid.publish(occupancy_grid)
    flag +=1
    
    
    #Regression for points that correspond to wall
    res = occupancy_grid.info.resolution
    points = []
    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            if data[i][j] == 100:
                points.append([i,j])
    points = np.array(points)

    x = points[:,0]
    x = np.reshape(x, (np.shape(x)[0], 1))
    y = points[:,1]
    
    reg = RANSACRegressor()
    reg.fit(x,y)
    b = 333/2 - reg.predict( [[333/2   ]] )[0]
    m = (333/2 - reg.predict( [[333/2+100 ]] )[0] - b)/100.
    b_meter = b  * res    

    
    print('b in Meter: ' + str(b_meter))
    print('m: ' + str(m) + '\n\n')
    
    
    #bridge = CvBridge()
    #try:
      #cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    #except CvBridgeError as e:
      #print(e)
    
    
    #image = cv2.line(cv_image,
     #                tuple([ 10,  reg.predict( [[10 ]] )[0] ]),
      #               tuple([ 400, reg.predict( [[400]] )[0] ]),
       #              1)
    #image_pub_test.publish(self.bridge.cv2_to_imgmsg(image , "mono8"))

    
    
    
    
# --- main ---
rospy.init_node("scan_grid")

# init occupancy grid
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = .03 # in m/cell

# width x height cells
occupancy_grid.info.width = 1000/3
occupancy_grid.info.height = 1000/3

# origin is shifted at half of cell size * resolution
occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.z = 0
occupancy_grid.info.origin.orientation.x = 0
occupancy_grid.info.origin.orientation.y = 0
occupancy_grid.info.origin.orientation.z = 0
occupancy_grid.info.origin.orientation.w = 1


#image_pub_test = rospy.Publisher("/image_processing/image_pub_test", Image, queue_size=1)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)
rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=100)

rospy.spin()
