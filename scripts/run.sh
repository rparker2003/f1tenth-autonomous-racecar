#!/bin/bash

# Inside the Docker container, run the following commands
docker exec -it f1tenth_gym_ros_sim_1 /bin/bash -c \
"source /opt/ros/foxy/setup.bash && source install/local_setup.bash && colcon build && tmux"