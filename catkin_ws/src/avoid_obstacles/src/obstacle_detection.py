#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import Range
from geometry_msgs.msg import PointStamped, Vector3, Vector3Stamped
import tf

# callback has *argv for cases if we want to change number of sensors
def callback(*argv):
    # vars for new vector
    ao_x, ao_y = 0, 0
    distance_array = []
    for sensor in argv:
        # activate listener for frame transformation
        t = tf.TransformListener()
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
        # adding vector coordinates multiplied by weight (distance from that point)
        ao_x += new_point.point.x*sensor.range
        ao_y += new_point.point.y*sensor.range
        # add all distances to array, we need to divide our vector by this sum
        # than it will be correctly weighted
        distance_array.append(sensor.range)            
    # create Vector3 object a weighted vector that should avoid obstacles 
    ao_heading = Vector3(-ao_x/sum(distance_array), ao_y/sum(distance_array), 1)
    # add header and create Vector3Stamped object
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base"
    ao_vector = Vector3Stamped(header, ao_heading)
    # publish vector for avoiding obstacles and minimal distance to it to topics 
    pub.publish(ao_vector)   
    
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
    rate = rospy.Rate(50) 

    detect_and_publish()