# RoboconProbation
robocon probation

src has two packages- my_services and robocon_practice

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
