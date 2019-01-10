# robot_slam
A ROS package that enables a robot in a V-REP scene to SLAM

To execute the queue of waypoints, run
```
rostopic pub /path_ready std_msgs/Empty -1
```

A demonstration video of the robot guide can be found <https://youtu.be/vzho2U02O7Y>.
To start the demo,
1. Run `./run_leading.bash`;
1. Set the pose estimate of the robot in rviz;
1. Start the simulation in V-REP.