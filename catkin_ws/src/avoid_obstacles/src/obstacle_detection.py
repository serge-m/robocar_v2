#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from geometry_msgs.msg import PointStamped, Point, Vector3, Vector3Stamped
import tf
import math
from visualization_msgs.msg import Marker

# callback has *argv for cases if we want to change number of sensors
def callback(*argv):    
    # activate listener for frame transformation
    t = tf.TransformListener()
    distance_array = []
    y_array = []
    for sensor in argv:        
        # wait a little bit for transfrormation from sensor frame to base frame
        t.waitForTransform("base", sensor.header.frame_id, rospy.Time(0),rospy.Duration(4.0))
        laser_point=PointStamped()
        laser_point.header.frame_id = sensor.header.frame_id
        laser_point.header.stamp = sensor.header.stamp
        laser_point.point.x=sensor.range
        laser_point.point.y=0
        laser_point.point.z=0
        # here goes transformation from sensor frame to base frame
        new_point = t.transformPoint("base", laser_point)
        distance_array.append(sensor.range)
        y_array.append(new_point.point.y)   
    min_dist = min(distance_array)
    # form a vector away from obstacle perpendicular to x-axis in base frame
    # magnitude is inversely proportional to distance to obstacle normalized by max_range
    # direction is opposite to y-coordinate of obstacle in base frame 
    vec_y = -math.copysign((argv[0].max_range-min_dist)/argv[0].max_range, y_array[distance_array.index(min_dist)])
    ao_heading = Vector3(0, vec_y, 1)
    
    # add header and create Vector3Stamped object
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base"
    ao_vector = Vector3Stamped(header, ao_heading)
    # publish vector for avoiding obstacles and minimal distance to it to topics 
    pub.publish(ao_vector)   

    # marker for vizualisation in RVIZ
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time()
    marker.ns = "/"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.points.append(Point(0, 0, 0.2))
    marker.points.append(Point(0, vec_y, 0.2))
    marker.scale.x = 0.02
    marker.scale.y = 0.05
    marker.scale.z = 0.1
    marker.color.a = 1.0 # Don't forget to set the alpha!
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    vis_pub.publish(marker)
    
    rate.sleep()               
       
    
def detect_and_publish():
    # 4 subscribers for 4 sensor scans
    left = message_filters.Subscriber("scan/left_ultra_sonic", Range)
    left_center = message_filters.Subscriber("scan/left_center_ultra_sonic", Range)
    right_center = message_filters.Subscriber("scan/right_center_ultra_sonic", Range)
    right = message_filters.Subscriber("scan/right_ultra_sonic", Range)
    # combine data from 4 sensors in one callback
    ts = message_filters.TimeSynchronizer([left, left_center, right_center, right], 1)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('obstacle_detection', anonymous=True)
    
    pub = rospy.Publisher('heading/avoid_obstacles', Vector3Stamped, queue_size=1)
    vis_pub = rospy.Publisher('/visualize/ao_vectors', Marker, queue_size=1)
    rate = rospy.Rate(50) 

    detect_and_publish()