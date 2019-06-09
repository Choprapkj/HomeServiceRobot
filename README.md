# HomeServiceRobot
Project submission for the Robotics Software NanoDegree program. 
Instructions:
    1. Clone the repository to src folder of catkin workspace.
    2. Build the package with catkin_make and source the devel/setup.bash script
    3.  Run shell script home_service.sh
  The turtlebot will demonstrate a pickup and drop-off  within the environment.

# Summary

ROS move_base package is used to move the robot, which interfaces with the ROS Navigation stack. The gmapping package provides laser-based SLAM , as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map from laser and pose data collected by a mobile robot. The Navigation stack uses Dijkstra’s algorithm for path search.
      
# Screenshot:     
![alt text](https://github.com/Choprapkj/HomeServiceRobot/blob/master/screenshots/going%20to%20pickup%20location.PNG)
