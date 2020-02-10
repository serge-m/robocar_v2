===============================
robocar_v2
===============================

Self driving robo car. Target functionality:
* lane following (in progress)
* obstacles avoiding (planned)
* road signs recognition (planned)




How to build
===============================

Install ROS melodic: `Ubuntu install of ROS Melodic <http://wiki.ros.org/melodic/Installation/Ubuntu>`_ 

Activate standard environment: 

.. code-block:: 

    source /opt/ros/melodic/setup.bash

Clone the repo

.. code-block:: 

    git clone --recurse-submodules git@github.com:serge-m/robocar_v2.git ~/robocar_v2
    


``raspicam`` node is built for raspberry only. It won't work for a PC.
If building on PC, install dependencies for raspicam node (as described in raspicam's README):

create a file ``/etc/ros/rosdep/sources.list.d/30-ubiquity.list`` and add to it:

:: 

    yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml


Then run

:: 
    
    rosdep update
    cd ~/robocar_v2/catkin_ws
    rosdep install --from-paths src --ignore-src --rosdistro=melodic -y

Preparation for raspicam node done.

Building the workspace:

::

    cd ~/robocar_v2/catkin_ws
    catkin_make


Running camera node and viewer
===============================

::

    roslaunch raspicam_node camerav2_410x308_30fps.launch &
    rosrun rqt_image_view rqt_image_view


