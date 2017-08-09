#!/usr/bin/env python

import rospy
import numpy as np
import tf
from math import atan, pi
from fub_trajectory_msgs.msg import Trajectory, TrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion




rospy.init_node("race_trajectory")

publisher = rospy.Publisher("/race_trajectory", Trajectory, queue_size=1)


message = Trajectory()
message.trajectory = []

flag = 0
with open('/root/catkin_ws_user/src/path_tracking/src/sample_map_origin_map.txt', 'r') as f:
    reader = f.readlines()

for row in reader[18:]:
    row = row.split('\t')
    
    x = np.float64( row[1] )
    y = -np.float64( row[2].strip('\n') )
    
    point = TrajectoryPoint()
    
    ### Position
    point.pose.position.x = x
    point.pose.position.y = y
    
    ### Orientation (except for first point)
    if flag !=0 :
        p = message.trajectory[-1].pose.position #predecessor of current point
        if x - p.x  == 0:
            alpha = pi * np.sign(y - p.y )
        else:
            alpha = atan( (y - p.y) / (x - p.x) )
        if x - p.x <0:
            alpha += pi

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
if p0.x - p.x  == 0:
    alpha = pi * np.sign(p0.y - p.y )
else:
    alpha = atan( (p0.y - p.y) / (p0.x - p.x) )
if p0.x - p.x  <0:
    alpha += pi

# modulo 2*pi:
if alpha > 2*pi:
    alpha -= 2*pi
if alpha  < 0:
    alpha += 2*pi

message.trajectory[-1].pose.orientation = Quaternion( *tf.transformations.quaternion_from_euler(0, 0, alpha) )



while True:
    print('Ich publishe')
    publisher.publish(message)
    rospy.sleep(1)
#publisher.publish(message)

try:
    rospy.spin()
except KeyboardInterrupt:
    f.close()
