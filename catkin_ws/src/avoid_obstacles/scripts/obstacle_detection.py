#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Header 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, Vector3, Vector3Stamped
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import tf
import math

# callback has *argv for cases if we want to change number of sensors
def callback(*argv):
    # vars for new vector
    x, y, distance_sum = 0, 0, 0
    for sensor in argv:
        # change all infinite distances to max value
        sensor.ranges = [sensor.range_max-0.01 if math.isinf(x) else x for x in sensor.ranges]
        # convert LaserScan to PointCloud2
        pc2_msg = lp.projectLaser(sensor, -1.0, 0x04)
        # activate listener for frame transformation
        t = tf.TransformListener()
        # wait a little bit for transfrormation from sensor frame to base frame
        t.waitForTransform("base", sensor.header.frame_id, rospy.Time(0),rospy.Duration(4.0))
                
        # convert PointCloud2 to a list of the individual points
        point_list = pc2.read_points_list(pc2_msg) 
        for point in point_list:
            # for transforming point into another frame we need to make a PointStamped object
            laser_point=PointStamped()
            laser_point.header.frame_id = sensor.header.frame_id
            laser_point.header.stamp = sensor.header.stamp
            laser_point.point.x=point.x
            laser_point.point.y=point.y
            laser_point.point.z=point.z
            # here goes transformation from sensor frame to base frame
            new_point = t.transformPoint("base", laser_point)
            # adding vector coordinates multiplied by weight (distance from that point)
            x += new_point.point.x*point.distances
            y += new_point.point.y*point.distances
            # sum up all distances, we need to divide our vector by this sum
            # than it will be correctly weighted
            distance_sum += point.distances
    # create Vector3 object a weighted vector that should avoid obstacles 
    ao_heading = Vector3(x/distance_sum, y/distance_sum, 1)
    # add header and create Vector3Stamped object
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base"
    ao_vector = Vector3Stamped(header, ao_heading)
    # publish vector for avoiding obstacles to topic
    pub.publish(ao_vector)
    rate.sleep()               
       
    
def detect_and_publish():
    # 4 subscribers for 4 sensor scans
    left = message_filters.Subscriber("scan/left_ultra_sonic", LaserScan)
    left_center = message_filters.Subscriber("scan/left_center_ultra_sonic", LaserScan)
    right_center = message_filters.Subscriber("scan/right_center_ultra_sonic", LaserScan)
    right = message_filters.Subscriber("scan/right_ultra_sonic", LaserScan)
    # combine +-simultaniously data from 4 sensors in one callback
    ts = message_filters.ApproximateTimeSynchronizer([left, left_center, right_center, right], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':

    rospy.init_node('obstacle_detection', anonymous=True)
    
    lp = lg.LaserProjection()
    pub = rospy.Publisher('heading/avoid_obstacles', Vector3Stamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    detect_and_publish()