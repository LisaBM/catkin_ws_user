#!/usr/bin/env python

# --- imports ---
from math import sin, cos
import rospy
import numpy as np
import pickle
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from sklearn.linear_model import RANSACRegressor






rospy.init_node("steering_angle_test")

rospy.Subscriber("scan_grid", OccupancyGrid, gridCallback, queue_size=100)


def gridCallback(grid):
    
    points = []
    for i in range(grid.data.shape[0]):
        for j in range(grid.data.shape[1]):
            if grid.data[i][j] == 100:
                points.append([i,j])
    points = np.array(points)

    x = points[:,0]
    x = np.reshape(x, (np.shape(x)[0], 1))
    y = points[:,1]
    
    reg = RANSACRegressor()
    reg.fit(x,y)
    b = reg.predict([[333/2]])[0]
    m = reg.predict([[333/2+1]])[0] - b
    b_meter = b  * res