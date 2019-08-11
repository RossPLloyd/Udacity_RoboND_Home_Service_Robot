# Udacity_RoboND_Home_Service_Robot
Repository for the Home Service Robot Project I undertook as part of the Udacity Robotics Nanodegree

Custom C++ nodes were written to publish markers at defined goal locations (add_markers/src/add_markers_goal.cpp) as well as
a node that sends the same goal positions to the move base node (pick objects.cpp). The robot was to drive to the pickup point, whereupon the marker would disappear as if being picked up. There would be a 5 second pause, after which the robot would navigate to the dropoff point and ”drop off” the marker, i.e. the marker will appear at the dropoff point. All packages were launched with the home service.sh bash script, though the more basic testing of goal navigation behaviour was achieved with the pick objects.sh bash script.
