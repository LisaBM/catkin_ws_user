#!/usr/bin/env python

import rospy
import numpy as np
import tf
from math import atan, pi, atan2
from fub_trajectory_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion




rospy.init_node("race_trajectory")

publisher = rospy.Publisher("/race_trajectory", Trajectory, queue_size=1)


message = Trajectory()
message.trajectory = []
message.header.frame_id  = 'map'
#message.header.seq = data.header.seq


flag = 0
with open('/root/catkin_ws_user/src/path_tracking/src/race_trajectory.txt', 'r') as f:
#with open('/root/catkin_ws_user/src/path_tracking/src/sample_map_origin_map.txt', 'r') as f:
    reader = f.readlines()

for row in reader[18:]:
    row = row.split('\t')

    x = np.float64( row[1] )
    y = -np.float64( row[2].strip('\n') )

    point = TrajectoryPoint()

    ### Position
    #rescaling around z:
    z1 =  2.6
    z2 = -1.5
    scale_x = .88
    scale_y = .8
    x = scale_x * x + (1-scale_x) *  z1
    y = scale_y * y + (1-scale_y) *  z2
    point.pose.position.x = x
    point.pose.position.y = y
    #print(point.pose.position)
    ### Orientation (except for first point)
    if flag !=0 :
        p = message.trajectory[-1].pose.position #predecessor of current point
        #print(p)

        #if x - p.x  == 0:
        #    alpha = pi * np.sign(y - p.y )
        #else:
        #    alpha = atan( (y - p.y) / (x - p.x) )
        #if x - p.x <0:
        #    alpha += pi
        alpha = atan2( y- p.y, x-p.x )
        #print(alpha)
        # modulo 2*pi:
        if alpha > 2*pi:
            alpha -= 2*pi
        if alpha  < 0:
            alpha += 2*pi

        message.trajectory[-1].pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, alpha) )
    flag += 1
    
    ### append
    message.trajectory.append(point)

#Calculate orientation of first point
p = message.trajectory[-1].pose.position #last point on trajectory
#"current" point is last point on trajectory
p0 = message.trajectory[0].pose.position
#print(p, p0)
#if p0.x - p.x  == 0:
#    alpha = pi * np.sign(p0.y - p.y )
#else:
#    alpha = atan( (p0.y - p.y) / (p0.x - p.x) )
#if p0.x - p.x  <0:
#    alpha += pi
alpha = atan2(p0.y - p.y, p0.x - p.x )
# modulo 2*pi:
if alpha > 2*pi:
    alpha -= 2*pi
if alpha  < 0:
    alpha += 2*pi

message.trajectory[-1].pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, alpha) )



while True:
    #print('Ich publishe')
    publisher.publish(message)
    rospy.sleep(1)
#publisher.publish(message)

try:
    rospy.spin()
except KeyboardInterrupt:
    f.close()
