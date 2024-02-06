#!/bin/bash

# Run the first command in the first tab
tmux send-keys 'ros2 launch f1tenth_gym_ros gym_bridge_launch.py' C-m

# Create another new tab (pane)
tmux new-window

# Select the second tab
tmux select-window -t :$(tmux list-windows | wc -l)

# Run the second command in the second tab
tmux send-keys 'ros2 run teleop_twist_keyboard teleop_twist_keyboard' C-m

# Create one more new tab (pane)
tmux new-window

# Select the third tab
tmux select-window -t :$(tmux list-windows | wc -l)

# Run the third command in the third tab
tmux send-keys 'cd src/gap_follow/scripts/' C-m