# Airport Robot Gazebo

## 环境

1. Ubuntu 18.04
2. Ros Melodic
3. Gazebo 9.0

## 依赖包

`sudo apt update` \
`sudo apt install ros-melodic-ros-controllers` \
`sudo apt install ros-melodic-velodyne-simulator`

## 运行指令

`roslaunch airport_robot_gazebo airport_robot.launch`

## 主要话题

1. 控制器:&ensp;"/cmd_vel"
1. 惯导:&ensp;"/imu"
1. 激光雷达:&ensp;"/velodyne_points"