# F1TENTH Autonomous Racecar Setup Guide

**Disclaimer: Setting up and running this repository requires a solid understanding of Docker and ROS2.**

## Prerequisites
- Docker environment installed and configured
- ROS2 Environment set up properly
- Patience and persistence

## Setup

1. **Clone the repository:**
    ```bash
    git clone https://github.com/rparker2003/f1tenth-autonomous-racecar
    ```

2. **Adjust volume paths in `f1tenth-autonomous-racecar/f1tenth_gym_ros/docker-compose.yml`.**
    - Uncomment the volume paths.
    - Add your directory prefix to all of them.

3. **Navigate to the `f1tenth-autonomous-racecar/f1tenth_gym_ros` directory and start the Docker container:**
    ```bash
    docker-compose up
    ```

4. **Open another terminal and enter the Docker container:**
    ```bash
    docker exec -it f1tenth_gym_ros_sim_1 /bin/bash
    ```

5. **Inside the Docker container, source the necessary files and build the packages:**
    ```bash
    source /opt/ros/foxy/setup.bash
    source install/local_setup.bash
    colcon build
    ```

## Usage

Ensure you have at least two terminals available:

1. **Launch the simulator:**
    ```bash
    ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    ```

2. **In a separate terminal within the Docker container (you can use tmux), start the keyboard controls for the vehicle:**
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

3. **(Optional) Run the volume scripts:**
    ```bash
    cd src/volume_name/scripts/
    python3 script_name.py
    ```

## Accessing the VNC

After executing `docker-compose up`, a localhost VNC is available on port 8080. Access it through:
- `localhost:8080/vnc.html`
- `IP:8080/vnc.html` if running the container remotely.


## Volumes
These are the actual packages. AKA these are the files you will be running and writing to do stuff like automated driving and braking.


## Labs

In the `f1tenth-autonomous-racecar/labs` folder, there are 4 files. The first lab provides detailed instructions on setting up the original `f1tenth_gym_ros` package. If you encounter issues with setup, try Lab 1 first, as this repository is based on it.

## Scripts

The `f1tenth-autonomous-racecar/scripts` folder contains `run.sh` and `tmux.sh`:
- `run.sh` executes Setup steps 4 and 5.
- `tmux.sh` executes Usage steps 1, 2, and 3, with step 3 defaulting to `src/gap_follow/scripts`.

## Good luck! 🏎💨
