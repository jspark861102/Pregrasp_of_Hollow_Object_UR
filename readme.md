# Pregrasp of Hollow Object

## 1. Prerequisites
- [**Universal Robot ROS Driver for UR5**](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [**Robotiq Packages for 2F-85**](http://wiki.ros.org/robotiq)
- [**Robotiq URCAP for FT300**](https://robotiq.com/products/ft-300-force-torque-sensor)
- [**moveit**](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

## 2. Run
Run UR5 Ros Driver
'''
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=172.20.10.2 \ kinematics_config:=/home/jspark/catkin_ws/src/Pregrasp_Hollow_Object/calibration/ur5_calibration.yaml
'''

Run Moveit planner
'''
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
'''

Run 2F-85 gripper node
'''
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
'''

Run Pregrasp node
'''
python /home/jspark/catkin_ws/src/Pregrasp_Hollow_Object/script/Pregrasp_Hollow_Object.py
'''
