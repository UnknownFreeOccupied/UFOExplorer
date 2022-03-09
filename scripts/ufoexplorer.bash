#!/bin/bash

# ROS
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
# Gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/px4/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/px4/Tools/sitl_gazebo/models:~/catkin_ws/src/rpl_uav/simulation/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/px4/build/px4_sitl_default/build_gazebo
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/px4:~/px4/Tools/sitl_gazebo

roscd ufoexplorer/data
# path="ufoexplorer"
# path="ufoexplorer_tsp"
# path="ufoexplorer_exponential"
# path="ufoexplorer_global_normalization"
path="ufoexplorer_test"
mkdir -p "${path}"

time_limit=1800
start_delay=10

# 'set -e' tells the shell to exit if any of the foreground command fails,
# i.e. exits with a non-zero status.
# set -eu

for map in "house"; do
    for max_vel in 2; do
        for i in {1..10}; do
            mkdir -p "${path}/${max_vel}/${map}"
            # trap 'kill -INT -$PID' INT
            trap 'echo caught interrupt and exiting;exit' INT
            timeout --foreground "${time_limit}"s roslaunch ufoexplorer ${map}.launch filename:="${path}/${max_vel}/${map}/${i}" map:="${map}" max_time:="${time_limit}" max_vel:="${max_vel}" start_delay:="${start_delay}"
            # PID=$!
            # wait $PID
            rosclean purge -y
            # sleep 10s
        done
    done
done

exit 0
