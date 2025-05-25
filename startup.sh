#!/bin/sh

#building and creating build,install and log directories.
colcon build


#sourcing setup file ie same as source install/setup.sh
. install/setup.sh

#launching gazebo
ros2 launch tusharbot_one launch_sim.launch.py 