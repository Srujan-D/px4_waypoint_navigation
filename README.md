# PX4 drone navigation

    There are two Python + ROS Melodic (rospy) nodes in /scripts.
    The commander node sends commands to to controller using a pub-sub model. 
    The commander takes NED coordinates from a waypoints.txt file, which is available in the /scripts, and sends these to the controller.
    The controller node subscribes to the imu and gps data, and publishes setpoints on topics which are subscribed by the drone.

    Right now, the drone operates at a constant altitude of 2 m, which is defined by 'takeoff_height'.
    
    Further Improvements I can think of:
    1. The controller can be improved by incorporating a PID for calculating the velocity.
    2. There are some redundant lines in controller, wrt the logic used. 
    Eg: The drone rotates (yaw angle is calculated in commander script), and then travels linearly in the x direction. Instead, this can be removed by giving both x and y linear velocities.
