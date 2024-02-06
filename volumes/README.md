# VOLUMES

Welcome to my collection of packages tailored for my UNLV course.

## Safety Node
This volume ensures safety by halting when it detects a wall within a specific range directly ahead.

## Wall Follow
This volume adeptly tracks the left and right walls of the vehicle's path, aiming to maintain a centered position between them.

## Gap Follow
Utilizing lidar data, this volume identifies and navigates toward openings in front of the vehicle.

## Scan Matching
Precisely locates the vehicle by comparing successive lidar scans, ensuring accurate localization.

## Particle Filter
Derived from [this](https://github.com/mit-racecar/particle_filter) GitHub repository, with its README retained within its directory.

## Pure Pursuit
Harnessing the power of the particle filter, this volume optimizes race lines. Its operations include:
1. Gathering waypoints from a logger
2. Smoothing waypoints
3. Employing the particle filter for localization

Two versions of the pure pursuit package are available:
- Version 1: Basic waypoint tracking
- Version 2: Utilizes splines for precise path following, resulting in enhanced accuracy and faster lap times.
