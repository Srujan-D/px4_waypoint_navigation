# RoboconProbation
robocon probation

src.zip has two packages- my_services and robocon_practice

    my_services contains a custom interface (LidarScan.srv)
    robocon_practice has 5 scripts: 

    (1)mapping.py

    (2)lidar.py

    (3)server_lidar.py

    (4)client_map.py

    (5)client_faster.py

    (1) and (2) are still incomplete, and follow the pub-sub model.

    (3) and (4) are complete i think, and follow the client-service model.

    (3) and (5) also work together, much better than 3) and 4).

    In (5), I have given more freedom (as compared to that in (4)) to the line method in the Mapping class, which play a huge role in making the binary occupancy grid. So the exploration of (5) is much better than (4)
    
px4_task.zip itself is a package. 
    
    It has two ROS Melodic (rospy) nodes in /scripts.
    The commander node sends commands to to controller using a pub-sub model. 
    The commander takes NED coordinates from a waypoints.txt file, which is available in the /scripts, and sends these to the controller.
    The controller node subscribes to the imu and gps data, and publishes setpoints on topics which are subscribed by the drone.

    Right now, the drone operates at a constant altitude of 2 m, which is defined by 'takeoff_height'.
    I could not test the code properly, in terms of the target coordinates, as gazebo was lagging a lot.
    Also, I have referred to many codes available on the internet for building these scripts. Some were in CPP and few were in Python.
    
    Further Improvements I can think of:
    1. The controller can be improved by incorporating a PID for calculating the velocity.
    2. There are some redundant lines in controller, wrt the logic used. 
    Eg: The drone rotates (yaw angle is calculated in commander script), and then travels linearly in the x direction. Instead, this can be removed by giving both x and y linear velocities.
