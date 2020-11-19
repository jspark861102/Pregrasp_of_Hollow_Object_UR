# Pregrasp of Hollow Object

## 1. Prerequisites
- [**Universal Robot ROS Driver for UR5**](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
    
    ```bash
    # PC IP : 172.20.10.1
    # UR IP : 172.20.10.2
    ```
- [**Robotiq Packages for 2F-85**](http://wiki.ros.org/robotiq)
    ```bash    
    #usb control
    sudo usermod -a -G dialout jspark
    dmesg | grep tty
    id -Gn #the output should print 'dialout'
    ```

- [**Robotiq URCAP for FT300**](https://robotiq.com/products/ft-300-force-torque-sensor)
- [**moveit**](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)

## 2. Run Pregrasp of Hollow Object

### 2.1 Gazebo Simulation
#### 2.1.1. Bringup the simulated robot in Gazebo
```bash
# Run UR5 Gazebo, set puased true for initial position
roslaunch ur_gazebo ur5.launch paused:=true
```

#### 2.1.2. Setup Moveit!
In simulation, delete "/scaled_pos_joint_traj_controller" in "/home/jspark/catkin_ws/src/fmauch_universal_robot/ur5_moveit_config/config/controller.yaml" before run the moveit node
```bash
# Run Moveit Planner
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true 	

# run UR5 Moveit Rviz
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

#### 2.1.3. Run Pregrasp node
```bash
# 4. Run Pregrasp Node
python /home/jspark/catkin_ws/src/Pregrasp_Hollow_Object/script/Pregrasp_Hollow_Object.py
```

### 2.2 UR5 experiment
For normal operation, four commands are needed that are run ur5_bringup, moveit planner, run 2F-85 node and Pregrasp node.

#### 2.2.1. Make UR5 calibration file
```bash
# Run Calibration_correction Node
roslaunch ur_calibration calibration_correction.launch \ robot_ip:=172.20.10.2 target_filename:="/home/jspark/catkin_ws/src/Pregrasp_Hollow_Object/calibration/ur5_calibration.yaml"
```

#### 2.2.2. Bringup the UR5 
```bash
# 1. Run UR5 Ros Driver
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=172.20.10.2 \ kinematics_config:=/home/jspark/catkin_ws/src/Pregrasp_Hollow_Object/calibration/ur5_calibration.yaml

# Run UR5 ROS Driver with limited joint range [-PI, PI] on all joints
roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=172.20.10.2 \ kinematics_config:=/home/jspark/catkin_ws/src/Pregrasp_Hollow_Object/calibration/ur5_calibration.yaml
```

#### 2.2.3. Setup Moveit!
```bash
# 2. Run Moveit Planner
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch

# Run Moveit Planner with limited joint
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch limited:=true

# run UR5 Moveit Rviz
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

#### 2.2.4. Setup 2F-85 gripper
```bash
# 3. Run 2F-85 Gripper node
rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

# Keyboard Control for 2F-85
rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

# Status Listener for 2F-85
rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py
```

#### 2.2.5. Call rosservice to run UR5 URcap program
```bash
# run UR5 with URcap
rosservice call /ur_hardware_interface/dashboard/play
```

#### 2.2.6. Run Pregrasp node
```bash
# 4. Run Pregrasp Node
python /home/jspark/catkin_ws/src/Pregrasp_Hollow_Object/script/Pregrasp_Hollow_Object.py
```

#### 2.2.7. To control UR5 with rqt controller
```bash
# Run Rqt Controller
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```


## 3. Troubleshooting
### 3.1. UR5 - ROS connection
The robot and the external PC have to be in the same network, ideally in a direct connection with each other to minimize network disturbances. In a external PC, static IP adress should be specified. 
In 'External Control' of Polyscope, hostname and port should be specified as jspark/50002.

### 3.2 moveit_commander 
#### 3.2.1. python version
moveit_commander should be installed with python2.7 as python version of ROS is python2.7

#### 3.2.2. PyAssimp import problem
PyAssimp import problem 
    
    1. install new version of PyAssimp (https://github.com/assimp/assimp) 

    2. cd .../port/PyAssimp
       python setup.py install

### 3.3 UR5 Gazebo (roslaunch ur_gazebo ur5.launch) 
#### 3.3.1. [ERROR] [1599703921.701884322, 0.349000000]: GazeboRosControlPlugin missing <legacyModeNS> while using DefaultRobotHWSim, defaults to true. This setting assumes you have an old package with an old implementation of DefaultRobotHWSim, where the robotNamespace is disregarded and absolute paths are used instead. If you do not want to fix this issue in an old package just set <legacyModeNS> to true.
  1. [**reference**](https://answers.ros.org/question/292444/gazebo_ros_control-plugin-gazeboroscontrolplugin-missing-legacymodens-defaultrobothwsim/)
  2. go to ur_discription/urdf/commom.gazebo.xacro
  3. add

    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    <legacyModeNS>true</legacyModeNS>

#### 3.3.2. [ERROR] [1599720862.238137424, 0.155000000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/shoulder_lift_joint.
1. [**reference**](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/886)

2. /home/jspark/catkin_ws/src/fmauch_universal_robot/ur_gazebo/controller/arm_controller_ur5.yaml 

    add
    ```
        /gazebo_ros_control:
        pid_gains:
            shoulder_pan_joint:
                p: 100.0
                i: 1.0
                d: 0.1
            shoulder_lift_joint:
                p: 100.0
                i: 0.01
                d: 10.0
            elbow_joint:
                p: 100.0
                i: 0.01
                d: 10.0
            wrist_1_joint:
                p: 100.0
                i: 0.01
                d: 10.0
            wrist_2_joint:
                p: 100.0
                i: 0.01
                d: 10.0
            wrist_3_joint:
                p: 100.0
                i: 0.01
                d: 10.0
    ```
  3. However, robot doesn't move correctly due to wrong controller gain. Therefore, you better ignore this error if there is no more problem.
  
### 3.4 moveit planning (roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true) 
[ WARN]     its child frame 'base_link' does not match the URDF frame 'world'

  1. [**reference**](https://github.com/ros-industrial/universal_robot/pull/284)
  2. /home/jspark/catkin_ws/src/fmauch_universal_robot/ur5_moveit_config/config/ur5.srdf    
    # from
    <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base_link" /> 
    # edit to
    <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="world" />
    
### 3.5 moveit_commander.Robotcommander() 
    terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
    중지됨 (core dumped)

1. [**reference**](https://github.com/ros-planning/moveit_commander/issues/15)
2. at the end of the code, add
    
    moveit_commander.os._exit(0)

3. Although error is occured, it can be ignored.


### 3.6 roslaunch ur5_moveit_planning_execution.launch limited:=true 
    [ERROR] [1600673900.953010467]: Action client not connected: /follow_joint_trajectory
1. [**reference**](https://github.com/ros-industrial/ur_modern_driver)
2. /home/jspark/catkin_ws/src/fmauch_universal_robot/ur5_moveit_config/config/controller.yaml

        # edit name
        name: "/scaled_pos_joint_traj_controller"
    
### 3.7 robotiq ros package install
    /home/jspark/catkin_ws/src/robotiq/robotiq_2f_gripper_control/nodes/Robotiq2FGripperRtuNode.py:65: SyntaxWarning: The publisher should be created with an explicit keyword argument 'queue_size'. Please see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers for more information.

1. [**reference**](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
2. add "queue_size=10" in Publisher declaration

### 3.8 gazebo initial pose edit
1. run the launch file as

        roslaunch ur_gazebo ur5.launch paused:=true

2. set initial pose in "ur5.launch"
        
        <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot -z 0.1 -unpause -J shoulder_lift_joint -1.57 -J elbow_joint 1.57 -J wrist_1_joint 3.14 -J wrist_2_joint -1.57" respawn="false" output="screen"/>