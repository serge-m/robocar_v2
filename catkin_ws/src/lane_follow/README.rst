Lane follow
=====================================

Module for lane following node.
The node is implemented using python3 and opencv. 

Refer to wiki page `Running-ROS-with-python3 <https://github.com/serge-m/robocar_v2/wiki/Running-ROS-with-python3>`_ 
to learn how to prepare the environment.


How to run
---------------------------------------------

.. code-block::

    rosrun lane_follow lf_node.py --camera-params $ROBOCAR_BASE/catkin_ws/src/robocar_config/camera_calibration.yaml


Development
--------------------------------------------

The code is slpit into two parts: ROS-related part and image processing logic. 
The idea is to keep image processing independent of ROS (messages, types, rospy etc). 
That allows reusing the code of the image processing in other projects and easier testing. 
Possibly the image processing will be extracted to a separate library.
Those two parts are interacting using simple python data structures (np.array, dict, etc.).




