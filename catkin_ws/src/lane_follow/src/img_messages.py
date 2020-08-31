from sensor_msgs.msg import CompressedImage
import cv2 
import numpy as np
import rospy

# (image_np: np.ndarray)
def compressed_img_message(image_np):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    retval, encoded = cv2.imencode('.jpg', image_np)
    if not retval:
        raise RuntimeError("Unable to encode image, {}".format(image_np))
    msg.data = np.array(encoded).tostring()
    return msg

# (ros_data) -> np.ndarray
def np_from_compressed_ros_msg(ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)