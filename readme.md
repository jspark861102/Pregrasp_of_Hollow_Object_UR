#note : gripper center can be set by put the offset to the jont btw wrist3 and ee_link
#note : but, object center should be calculated (slerp)
#note : grasping bushing with exact configruration is hard, so rotation radius is not exact, use of force sensor will help 

### topic explanation especially for status and result
## stats
topic: /execute_trajectory/status
msg: actionlib_msgs/GoalStatusArray 
#explanation: status only for group.execute()

topic: /move_group/status
msgs: actionlib_msgs/GoalStatusArray
#explanation: status only for group.go()

topic: /scaled_pos_joint_traj_controller/follow_joint_trajectory/status   
msg: actionlib_msgs/GoalStatusArray 
#explanation: there is no status information

topic: /ur_hardware_interface/set_mode/status   
msg: actionlib_msgs/GoalStatusArray 
#explanation: there is no status information

## result
topic: /execute_trajectory/result   
msgs: moveit_msgs/ExecuteTrajectoryActionResult
#explanation: only react for group.execute()

topic: /move_group/result  
result: moveit_msgs/ExecuteTrajectoryActionResult
#explanation: only react for group.go()

topic: /scaled_pos_joint_traj_controller/follow_joint_trajectory/result   
result: control_msgs/FollowJointTrajectoryActionResult
#explanation: react for both group.go or group.execute

topic: /ur_hardware_interface/set_mode/result   
msg: ur_dashboard_msgs/SetModeActionResult
explanation: no printed

## robot mode
topic: /ur_hardware_interface/robot_mode   
results: /ur_hardware_interface/robot_mode 
explanaation: #robot on/off/braking etc...

##moveit api for robot state
robot.get_current_state() #only joint values

##rosmsg show actionlib_msgs/GoalStatusArray
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
actionlib_msgs/GoalStatus[] status_list
  uint8 PENDING=0
  uint8 ACTIVE=1
  uint8 PREEMPTED=2
  uint8 SUCCEEDED=3
  uint8 ABORTED=4
  uint8 REJECTED=5
  uint8 PREEMPTING=6
  uint8 RECALLING=7
  uint8 RECALLED=8
  uint8 LOST=9
  actionlib_msgs/GoalID goal_id
    time stamp
    string id
  uint8 status
  string text

