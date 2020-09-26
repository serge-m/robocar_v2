# robocar_main

The module to launch the robocar in one of two modes: 
* with real hardware
* inside simulation


See corresponding launch files for the details.


## How to launch a robocar in simulation

    roslaunch robocar_main robocar.launch world_name:=$(rospack find robocar_world)/worlds/road.world 
    
or 
    
    roslaunch robocar_main robocar.launch simulation:=1
    
    
## How to launch a robocar with real hardware

    roslaunch robocar_main robocar.launch simulation:=0

