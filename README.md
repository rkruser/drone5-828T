# drone5-828T
The final drone project for CMSC 828T (Fall 2017)

Packages we are using
---------------------
(List ROS packages here)


Todo:
-----
1. Be able to extract data (imu/odometry, images) from the drone in ROS
2. Figure out how to use drone control API
3. See if we can find a structure-from-motion ROS package
4. Figure out API to find AR tags in images
5. Write pseudo-code outline of our basic algorithm

Project Flow Outline
--------------------
1. Create a basic python class that can hover the drone, get all relevant data, and can command the drone.
2. Design a state machine to pilot the drone. For example, states = {takeoff, searchForWindow, moveToWindow, moveThroughWindow, findLastTag, moveToLastTag, hoverInFrontOfLastTag}. Implement this as a python class that interfaces with specific functions that implement the states.
3. Write specific functions:
   - Need to detect AR tags in images
   - Need to find the homography from the window to the drone using the tags
   - Need a way of generating trajectories given homography, tag locations, imu, etc. (At what level can we control the trajectory? Can we give the ROS package a sequence of points to go through, or only high level commands like "left, right, etc"?)
   - Maybe implement a Kalman filter or something similar (or use a package)
   - Maybe use a structure-from-motion package
   - Need to implement logic of control loop for each state
