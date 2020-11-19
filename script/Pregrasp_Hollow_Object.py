#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib_msgs.msg
import control_msgs.msg
import std_msgs.msg
import visualization_msgs.msg 
import math
import numpy
import tf
import copy
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

#=================================================## parameters ##===============================================#
## iteration ##
iter_pregrasp = 2

## vision parameters ##
vision_on = 1
object_orientation_from_vision = 0.0
object_position_from_vision = [0.0, 0.0]
vision_flag = 0
vision_update_succeed = 0
flag_iter = 5 #5 times

## object parameters ##
object_orientation = 0.0 #rotation by global z axis
object_length = 0.068
object_height = 0.034        

## tilt parameters ##
offset_L =  object_length*2/3*0.95
offset_h = -object_height*2/3   

theta_before_pick  = -math.pi/10      
desired_tilt_angle =  math.pi/2 - math.pi/10*1.0 #is is enough due to init grasping tilitng and sliding at point S

## tilt back parameters ##
offset_L_tilt_back = -0.015
offset_h_tilt_back = 0.015
theta_tilt_back    = -(desired_tilt_angle + theta_before_pick)

##pickup parameters ##
pick_down_xy = -0.006
pick_down_z =  -0.020

## sliding (to instert) parameters ##
down_z = -0.12
sliding_x =  0.04
sliding_y =  0.05
sliding_z = -0.01

## align parameters ##
down_z_align = -0.205

align_distance_x = 0.1
limit_force_align_x = 3.5
align_offset_x = -0.001

align_distance_y = 0.1
limit_force_align_y = -3.6
align_offset_y = -0.001

## robot arm parameters ##
velocity = 0.15
velocity_fast = 0.25

## gripper parameters ##
d_max = 0.085
position_of_girpper_before_pick = 50
position_of_gripper_for_pick = 230
position_of_gripper_for_tilt_back = 140    
position_of_gripper_for_sliding = 200    
velocity_of_gripper = 15
force_of_gripper = 1        

## environment parameters ##
base_height = 0.01 - 0.001 #base_plate = 0.01, spunge = 0.005, rubber = 0.001
z_force_limit = 40.0 
y_torque_limit = 20.0
fault_offset_z = 0.1

## control parameters initialization
wrench = geometry_msgs.msg.WrenchStamped()
move_result = 0
isfault = 0
#================================================================================================================#

def wrench_callback(msg):
    global wrench
    wrench = msg    

def move_result_callback(msg):
    global move_result
    move_result = msg.status.status

if vision_on == 1:
    def pose_vision_callback(msg):
        global object_position_from_vision
        global object_orientation_from_vision
        global vision_flag

        if msg.ns == 'obj':
            if msg.id == 1:
                print "============ I receive the object pose from vision"    
                if vision_update_succeed == 0:    
                    vision_flag = vision_flag + 1
            
                    object_position_from_vision = [msg.pose.position.x, msg.pose.position.y]
                    
                    quat_callback = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
                    euler_callback = tf.transformations.euler_from_quaternion(quat_callback, axes='sxyz')
                    if euler_callback[2] > math.pi/2:
                        object_orientation_from_vision = -math.pi + euler_callback[2]
                    elif euler_callback[2] < -math.pi/2:
                        object_orientation_from_vision = math.pi + euler_callback[2]
                    else:
                        object_orientation_from_vision = euler_callback[2]   

                    print "============ vision flag : %f" % vision_flag
                    print "============ object_position_from_vision" 
                    print object_position_from_vision
                    #print "============ euler_callback (rad)" 
                    #print euler_callback
                    print "============ object_orientation_from_vision (deg)" 
                    print object_orientation_from_vision*180/math.pi  

##### ROS #####
rospy.init_node('test_ur', anonymous=True)
pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
wrench_sub = rospy.Subscriber('/wrench', geometry_msgs.msg.WrenchStamped, wrench_callback)
move_result_sub = rospy.Subscriber('/scaled_pos_joint_traj_controller/follow_joint_trajectory/result', control_msgs.msg.FollowJointTrajectoryActionResult, move_result_callback)

if vision_on == 1:
    vision_call_publisher = rospy.Publisher('recog_on', std_msgs.msg.String, queue_size=1)
    pose_vision_sub = rospy.Subscriber('/det/pick_object/obj_marker', visualization_msgs.msg.Marker, pose_vision_callback)

##### moveit #####
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = robot.get_group_names()
group = moveit_commander.MoveGroupCommander(group_name[1])
group.set_max_velocity_scaling_factor(velocity)
##### tf #####
tf_listener = tf.TransformListener()

##############################################################################################
############################################# gripper API ####################################
#ee_link to opended finger tip : 0.198
#ee_link to closed finger tip : 0.198 + (0.1628-0.1493) = 0.2115
#command.rPR : 0=open, 255=close
#d: distance btw fingers
#L: increasing length when close the gripper
##############################################################################################
def gripper_d_to_rPR(d):
    d = d*1.0
    rPR = (d_max-d)/d_max*255
    return rPR

def gripper_rPR_to_d(rPR):
    rPR = rPR*1.0
    d = (255-rPR)/255*d_max
    return d

def gripper_d_to_L(d):
    d = d*1.0
    L = (d_max-d)/d_max*(0.1628-0.1493)
    return L

def gripper_rPR_to_L(rPR):
    rPR = rPR*1.0
    L = rPR/255*(0.1628-0.1493)
    return L

def gripper_active(pub):
    rospy.sleep(0.5)
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150					
    pub.publish(command)
    rospy.sleep(2)

def gripper_reset(pub):
    rospy.sleep(0.5)
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 0
    pub.publish(command)
    rospy.sleep(0.5)

def init_gripper():      
    print "============ gripper is initiation"
    gripper_reset(pub)
    gripper_active(pub)

def gripper_move(pub, pos, vel, force):  
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = vel
    command.rFR  = force						
    command.rPR  = pos
    pub.publish(command)
##############################################################################################

##############################################################################################
############################################# tf API #########################################
##############################################################################################
def get_tf(tf_from, tf_to):
    tf_listener.waitForTransform(tf_from, tf_to, rospy.Time(), rospy.Duration(4.0))
    now = rospy.Time.now()
    tf_listener.waitForTransform(tf_from, tf_to, now, rospy.Duration(4.0))
    (trans,rot) = tf_listener.lookupTransform(tf_from, tf_to, now)
    return trans, rot

def ori2eul(ori):
    quat = [0,0,0,0]
    quat[0] = ori.x
    quat[1] = ori.y
    quat[2] = ori.z
    quat[3] = ori.w
    eul = tf.transformations.euler_from_quaternion(quat, axes='rxyz')
    return eul
##############################################################################################

##############################################################################################
######################################### motion API #########################################
#group.set_joint_value_target(target_joint)
#group.set_pose_target(target_pose)
#group.go(wait=True)

#group.go(target_joint, wait=True)
#group.go(target_pose, wait=True)

#waypoints with pose
#plan3, fraction = group.compute_cartesian_path
#replan = group.retime_trajectory(robot.get_current_state(), plan3, vel)
#group.execute(replan, wait=True)
##############################################################################################
def go_joints_degree(joint_deg, vel, iswait):
    group.set_max_velocity_scaling_factor(vel)
    target_joint = [n*math.pi/180. for n in joint_deg]
    group.go(target_joint, wait=iswait)

def go_pose(pose, vel, iswait):
    group.set_max_velocity_scaling_factor(vel)
    group.go(pose, wait=iswait)

def execute_waypoints(waypoints, vel, iswait):
    print "============ execute waypoints"
    (plan3, fraction) = group.compute_cartesian_path(
                        waypoints,   # waypoints to follow
                        0.01,        # eef_step
                        0.0)         # jump_threshold

    replan= group.retime_trajectory(robot.get_current_state(), plan3, vel) #parameter that changes velocity    
    group.execute(replan, wait = iswait)
    
def go_to_init_position():
    print "============ go_to_init_position()"
    group.set_max_velocity_scaling_factor(velocity)
    init_joint = [n*math.pi/180. for n in [0., -90., 90., 180., -90., 0.]]
    group.go(init_joint, wait=True)
    group.stop()

def go_to_pick_position():
    print "============ go_to_pick_position()"
    group.set_max_velocity_scaling_factor(velocity_fast)
    #init_joint = [n*math.pi/180. for n in [0., -90., 90., 270., -90., -90.]]
    init_joint = [n*math.pi/180. for n in [0., -120., 124., 264., -90., -90.]]
    group.go(init_joint, wait=True)
    group.stop()

def go_to_pick2_position():
    print "============ go_to_pick_position()"
    group.set_max_velocity_scaling_factor(velocity_fast)
    #init_joint = [n*math.pi/180. for n in [0., -90., 90., 270., -90., -90.]]
    #init_joint = [n*math.pi/180. for n in [0., -120., 124., 264., -90., -90.]]
    init_joint = [n*math.pi/180. for n in [27.57, -117.53, 122.94, 263.56, -90.13, -62.73]]
    group.go(init_joint, wait=True)
    group.stop()

def go_to_drop_position():
    print "============ go_to_drop_position()"
    group.set_max_velocity_scaling_factor(velocity_fast)
    #init_joint = [n*math.pi/180. for n in [176., -93.75, 73.5, 288.3, -4.0, -162.4]]
    init_joint = [n*math.pi/180. for n in [182.07, -89.21, 80.48, 222.16, -7.38, -109.74]]
    group.go(init_joint, wait=True)
    group.stop()

def go_to_vision_position():
    print "============ go_to_vision_position()"
    group.set_max_velocity_scaling_factor(velocity_fast)
    init_joint = [n*math.pi/180. for n in [18.35, -117.42, 94.00, 292.45, -89.87, -71.79]]
    #init_joint = [n*math.pi/180. for n in [0.25, -109.5, 69.29, 309.43, -89.48, -89.81]]
    #init_joint = [n*math.pi/180. for n in [0.24, -108.55, 68.77, 308.99, -89.49, -104.28]]
    group.go(init_joint, wait=True)
    group.stop()

def go_to_align_position():
    print "============ go_to_align_position()"
    group.set_max_velocity_scaling_factor(velocity_fast)
    init_joint = [n*math.pi/180. for n in [-47.88, -94.91, 88.04, 276.94, -89.19, 44.44]]
    group.go(init_joint, wait=True)
    group.stop()
    
##############################################################################################
################################################# main #######################################
## tf note ## (rosrun rqt_tf_tree rqt_tf_tree)
## group.get_current_pose : from /world to /ee_link
## teach pendant pose     : from /base  to /tool0_controller
## tf of the tool0 and tool0_controler is the same
## tool0_controller, obtained from hardware, interface is more precise but, not included in group
## 
## moveit API ##
## group.get_current_pose()
## tf.transformations.euler_from_quaternion(moveit_rot)
##
## home position
## joint = [n*math.pi/180. for n in [0., -90., 90., 180., -90., 0.]]
## pose = [0.4744 (0.684), 0.1106, 0.6099], [0.00299, -0.000286, 0.00127, 0.9999]
##
## pick position
## init_joint = [n*math.pi/180. for n in [0., -90., 90., 270., -90., -90.]]
## init_joint = [n*math.pi/180. for n in [0., -120., 124., 264., -90., -90.]]

##############################################################################################    
if __name__ == '__main__': 
    try:    
        ##############################################################################################
        ## getting basic information ##
        print "============ Reference frame: %s" % group.get_planning_frame()
        print "============ end effector link: %s" % group.get_end_effector_link()
        

        for iter in range(0,iter_pregrasp):

            #=============================================## go to pick position ##==========================================#
            init_gripper()
            gripper_move(pub, position_of_girpper_before_pick, 150, 150) #====================================================
            rospy.sleep(0.1)
            go_to_pick_position() #===========================================================================================
            rospy.sleep(0.1)
            print "============ go_to_pick_position finished with status = %d" % move_result
            move_result = 0
            
            #print group.get_current_pose().pose   
            #================================================================================================================#
            go_to_vision_position() #===========================================================================================
            rospy.sleep(0.5) #wait for robot arm stabilization
            print "============ go_to_vision_position finished with status = %d" % move_result
            move_result = 0
            
            #===============================================## wait for vision ##============================================#
            if vision_on == 1:        
                go_to_vision_position() #===========================================================================================
                rospy.sleep(0.5) #wait for robot arm stabilization
                print "============ go_to_vision_position finished with status = %d" % move_result
                move_result = 0
                
                wait_count = 0
                while True:
                    if wait_count > 200:
                        print "============ vision call doesn't work"
                        break

                    if vision_flag < flag_iter: #5times call
                        vision_update_succeed = 0 
                        vision_call_publisher.publish("recog")
                        print "============ trying to call vision"

                    elif vision_flag == flag_iter:
                        print "============ vision update succeed!"
                        object_orientation = object_orientation_from_vision #rotation by global z axis
                        vision_update_succeed = 1 #after flag>3, will not update object pose
                        break

                    wait_count = wait_count + 1            
                    rospy.sleep(
1)

                go_to_pick2_position() #===========================================================================================
                rospy.sleep(0.1)
                print "============ go_to_pick2_position finished with status = %d" % move_result
                move_result = 0
            #================================================================================================================#        

            #==============================================## go to tilt position ##=========================================#
            if vision_flag == flag_iter:
                target_pose_to_object = group.get_current_pose().pose   
                ## position lower fingertip to hole of object before grasping, tune with the term of 'object_height/2' ##    
                target_pose_to_object.position.z = object_height*7/10 - base_height + gripper_rPR_to_d(position_of_girpper_before_pick)/2*math.sin(math.fabs(theta_before_pick))

                target_pose_to_object.position.x = object_position_from_vision[0] - 0.006*math.cos(object_orientation) #make some distance between one of finger and object
                target_pose_to_object.position.y = object_position_from_vision[1] - 0.006*math.sin(object_orientation) #make some distance between one of finger and object
                
                current_quat = [0,0,0,0]
                current_quat[0] = target_pose_to_object.orientation.x
                current_quat[1] = target_pose_to_object.orientation.y 
                current_quat[2] = target_pose_to_object.orientation.z
                current_quat[3] = target_pose_to_object.orientation.w        
                rot_quat = tf.transformations.quaternion_from_euler(-object_orientation, 0, theta_before_pick, axes='rxyz')
                
                target_pose_to_object_quat = tf.transformations.quaternion_multiply(current_quat, rot_quat)
                target_pose_to_object.orientation.x = target_pose_to_object_quat[0]
                target_pose_to_object.orientation.y = target_pose_to_object_quat[1]
                target_pose_to_object.orientation.z = target_pose_to_object_quat[2]
                target_pose_to_object.orientation.w = target_pose_to_object_quat[3]
                
                print "============ go to object"
                go_pose(target_pose_to_object, velocity_fast, True) #==================================================================
                rospy.sleep(0.1)
                print "============ go to object finished with status = %d" % move_result
                move_result = 0        

                print "============ pose at tilt position"
                print group.get_current_pose().pose     

            else:
                target_pose_to_object = group.get_current_pose().pose   
                ## position lower fingertip to hole of object before grasping, tune with the term of 'object_height/2' ##    
                target_pose_to_object.position.z = object_height*7/10 - base_height + gripper_rPR_to_d(position_of_girpper_before_pick)/2*math.sin(math.fabs(theta_before_pick))

                target_pose_to_object.position.x = target_pose_to_object.position.x + 0.10
                target_pose_to_object.position.y = target_pose_to_object.position.y + gripper_rPR_to_d(position_of_girpper_before_pick)/2*math.sin(object_orientation)

                current_quat = [0,0,0,0]
                current_quat[0] = target_pose_to_object.orientation.x
                current_quat[1] = target_pose_to_object.orientation.y 
                current_quat[2] = target_pose_to_object.orientation.z
                current_quat[3] = target_pose_to_object.orientation.w        
                rot_quat = tf.transformations.quaternion_from_euler(-object_orientation, 0, theta_before_pick, axes='rxyz')
                
                target_pose_to_object_quat = tf.transformations.quaternion_multiply(current_quat, rot_quat)
                target_pose_to_object.orientation.x = target_pose_to_object_quat[0]
                target_pose_to_object.orientation.y = target_pose_to_object_quat[1]
                target_pose_to_object.orientation.z = target_pose_to_object_quat[2]
                target_pose_to_object.orientation.w = target_pose_to_object_quat[3]
                
                print "============ go to object"
                go_pose(target_pose_to_object, velocity_fast, True) #==================================================================
                rospy.sleep(0.1)
                print "============ go to object finished with status = %d" % move_result
                move_result = 0
            #================================================================================================================#        
            
            #==============================================## grasp the object  ##===========================================#       
            init_force = wrench.wrench.force.z
            print "============ z force at initial state (init = %f)" % init_force    
            print "============ z force before grasping (init = %f)" % wrench.wrench.force.z    
            
            gripper_move(pub, position_of_gripper_for_pick, velocity_of_gripper, force_of_gripper) #==========================
            print "============ gripper_move()"                
            
            i = 0
            while i < 100: #for gripper, time (150/0.01=1.5sec) is used instead of 'move_result'           
                if wrench.wrench.force.z <= init_force - z_force_limit:
                    print "======================== grasping stop due to z force (current = %f)" % wrench.wrench.force.z   
                    isfault = 1  

                    offset_pose = group.get_current_pose().pose
                    offset_pose.position.z = offset_pose.position.z + fault_offset_z
                    go_pose(offset_pose, velocity, False) #================================================================
                    rospy.sleep(2)
                    break            
                rospy.sleep(0.01)
                i = i + 1
            rospy.sleep(0.1)
            #move_result = 0                 
            #================================================================================================================#        
            
            #==============================================## tilt w.r.t. pivot ##===========================================#        
            if isfault == 0:
                offset_x =  offset_L * math.cos(object_orientation) #view pivot point from current position
                offset_y =  offset_L * math.sin(object_orientation) #view pivot point from current position 
                offset_z =  offset_h #view pivot point from current position       
                #current_theta = math.atan2(-offset_z, -offset_x) #angle of r w.r.t. pivot
                current_theta = math.atan2(-offset_h, -offset_L) #angle of r w.r.t. pivot
                r = math.sqrt(offset_x*offset_x + offset_y*offset_y + offset_z*offset_z)
                
                current_pose = group.get_current_pose().pose
                pivot_x = current_pose.position.x + offset_x
                pivot_y = current_pose.position.y + offset_y
                pivot_z = current_pose.position.z + offset_z
                    
                target_pose_tilt = copy.deepcopy(current_pose)        
                current_quat = [0,0,0,0] #just initialisation
                current_quat[0] = target_pose_tilt.orientation.x
                current_quat[1] = target_pose_tilt.orientation.y
                current_quat[2] = target_pose_tilt.orientation.z
                current_quat[3] = target_pose_tilt.orientation.w        
                init_rpy = tf.transformations.euler_from_quaternion(current_quat, axes='rxyz')   #-1.57, 0, 1.57  
                print "============ pose before tilt"
                print group.get_current_pose().pose     

                del_theta = 0.0   
                waypoints = []
                current_pose = group.get_current_pose().pose           
                waypoints.append(copy.deepcopy(current_pose))  

                count = 0
                n = 100
                while count < n:
                    del_theta = del_theta + desired_tilt_angle/n
                    target_pose_tilt.position.x = pivot_x  + r * math.cos(current_theta - del_theta) * math.cos(object_orientation) 
                    target_pose_tilt.position.y = pivot_y  + r * math.cos(current_theta - del_theta) * math.sin(object_orientation) 
                    target_pose_tilt.position.z = pivot_z  + r * math.sin(current_theta - del_theta)   

                    target_pose_tilt_quat = tf.transformations.quaternion_from_euler(init_rpy[0], init_rpy[1], init_rpy[2] + del_theta, axes='rxyz')            
                    target_pose_tilt.orientation.x = target_pose_tilt_quat[0]
                    target_pose_tilt.orientation.y = target_pose_tilt_quat[1]
                    target_pose_tilt.orientation.z = target_pose_tilt_quat[2]
                    target_pose_tilt.orientation.w = target_pose_tilt_quat[3]

                    waypoints.append(copy.deepcopy(target_pose_tilt))   

                    count = count + 1

                del waypoints[:1]  
                execute_waypoints(waypoints, velocity, False) #================================================================                  
                
                #init_force = wrench.wrench.force.z
                print "============ z force before tilt (init = %f)" % wrench.wrench.force.z 
                while True:
                    if move_result == 3:
                        print "============ tilt finished with status = %d" % move_result                    
                        break
                    if wrench.wrench.force.z <= init_force  - z_force_limit:
                        group.stop()
                        print "======================== tilt stop due to z force (current = %f)" % wrench.wrench.force.z     
                        isfault = 1  

                        offset_pose = group.get_current_pose().pose
                        offset_pose.position.z = offset_pose.position.z + fault_offset_z
                        go_pose(offset_pose, velocity, False) #================================================================
                        rospy.sleep(0.5)
                        print "============ tilt execution stop with status = %d" % move_result
                        break
                move_result = 0
                    
                rospy.sleep(0.1)
                print "============ pose after tilt"
                print group.get_current_pose().pose  
            #================================================================================================================# 
                    
            #=========================================## tilt back w.r.t. fingertip ##=======================================#   
            if isfault == 0:     
                waypoints = []
                current_pose = group.get_current_pose().pose           
                waypoints.append(copy.deepcopy(current_pose))   

                target_pose_tilt = copy.deepcopy(current_pose)        
                current_quat = [0,0,0,0] #just initialisation
                current_quat[0] = target_pose_tilt.orientation.x
                current_quat[1] = target_pose_tilt.orientation.y
                current_quat[2] = target_pose_tilt.orientation.z
                current_quat[3] = target_pose_tilt.orientation.w        
                init_rpy = tf.transformations.euler_from_quaternion(current_quat, axes='rxyz')   #-1.57, 0, 1.57  
                print "============ pose before tilt back"
                print group.get_current_pose().pose     

                offset_x =  offset_L_tilt_back * math.cos(object_orientation) #view pivot point from current position
                offset_y =  offset_L_tilt_back * math.sin(object_orientation) #view pivot point from current position 
                offset_z =  offset_h_tilt_back 
                
                current_theta = math.atan2(-offset_z, -offset_x) #angle of r w.r.t. pivot
                #r = math.sqrt(offset_x*offset_x + offset_z*offset_z)
                r = math.sqrt(offset_x*offset_x + offset_y*offset_y + offset_z*offset_z)

                pivot_x = target_pose_tilt.position.x + offset_x
                pivot_y = target_pose_tilt.position.y + offset_y
                pivot_z = target_pose_tilt.position.z + offset_z

                count = 0
                n = 100
                del_theta = 0.0
                while count < n:
                    del_theta = del_theta + theta_tilt_back/n
                    target_pose_tilt.position.x = pivot_x + r * math.cos(current_theta - del_theta) * math.cos(object_orientation) 
                    target_pose_tilt.position.y = pivot_y + r * math.cos(current_theta - del_theta) * math.sin(object_orientation)
                    target_pose_tilt.position.z = pivot_z + r * math.sin(current_theta - del_theta)   

                    target_pose_tilt_quat = tf.transformations.quaternion_from_euler(init_rpy[0], init_rpy[1], init_rpy[2] + del_theta, axes='rxyz')            
                    target_pose_tilt.orientation.x = target_pose_tilt_quat[0]
                    target_pose_tilt.orientation.y = target_pose_tilt_quat[1]
                    target_pose_tilt.orientation.z = target_pose_tilt_quat[2]
                    target_pose_tilt.orientation.w = target_pose_tilt_quat[3]

                    waypoints.append(copy.deepcopy(target_pose_tilt))   

                    count = count + 1

                del waypoints[:1]            
                execute_waypoints(waypoints, velocity_fast, False) #================================================================       

                gripper_move(pub, position_of_gripper_for_tilt_back, velocity_of_gripper, force_of_gripper) #==================

                #init_force = wrench.wrench.force.z
                print "============ z force before tilt back (init = %f)" % wrench.wrench.force.z
                while True:
                    if move_result == 3:
                        print "============ tilt back finished with status = %d" % move_result
                        break
                    if wrench.wrench.force.z <= init_force  - z_force_limit:
                        group.stop()
                        print "======================== tilt back stop due to z force (current = %f)" % wrench.wrench.force.z     
                        isfault = 1  

                        offset_pose = group.get_current_pose().pose
                        offset_pose.position.z = offset_pose.position.z + fault_offset_z
                        go_pose(offset_pose, velocity, False) #================================================================
                        rospy.sleep(0.5)
                        print "============ tilt back execution stop with status = %d" % move_result
                        break
                rospy.sleep(0.1)
                move_result = 0
                print "============ pose after tilt back"
                print group.get_current_pose().pose  
            #================================================================================================================# 

            #==============================================## down for pick up ##============================================#
            if isfault ==0:
                gripper_move(pub, 190, velocity_of_gripper, force_of_gripper) #===================================================
                rospy.sleep(0.1)
                print "============ gripper_move() during down"

                target_pose_to_down = group.get_current_pose().pose   
                target_pose_to_down.position.x = target_pose_to_down.position.x + pick_down_xy*math.cos(object_orientation)
                target_pose_to_down.position.y = target_pose_to_down.position.y + pick_down_xy*math.sin(object_orientation)
                target_pose_to_down.position.z = target_pose_to_down.position.z + pick_down_z
                
                print "============ go down"
                go_pose(target_pose_to_down, velocity, False) #==================================================================        
                
                #init_force = wrench.wrench.force.z
                print "============ z force before down (init = %f)" % wrench.wrench.force.z
                while True:
                    if move_result == 3:
                        print "============ down finished with status = %d" % move_result
                        break
                    if wrench.wrench.force.z <= init_force  - z_force_limit:
                        group.stop()
                        print "======================== down stop due to z force (current = %f)" % wrench.wrench.force.z  
                        isfault = 1     

                        offset_pose = group.get_current_pose().pose
                        offset_pose.position.z = offset_pose.position.z + fault_offset_z
                        go_pose(offset_pose, velocity, False) #================================================================
                        rospy.sleep(0.5)
                        print "============ down execution stop with status = %d" % move_result
                        break
                rospy.sleep(0.1)
                move_result = 0
                print "============ pose after down"
                print group.get_current_pose().pose 
            #================================================================================================================# 

            #================================================## pick and up ##===============================================#
            if isfault == 0:
                gripper_move(pub, 250, velocity_of_gripper, 100) #===================================================
                rospy.sleep(1.0)
                print "============ gripper_move() for pick"

                target_pose_to_object = group.get_current_pose().pose 
                target_pose_to_object.position.z = target_pose_to_object.position.z + 0.1

                print "============ up"
                go_pose(target_pose_to_object, velocity_fast, True) #==================================================================
                rospy.sleep(0.1)
                print "============ up finished with status = %d" % move_result
                move_result = 0
            #================================================================================================================#         
            

            if iter == 0 or iter % 2 == 0:

                #=============================================## go to drop position ##==========================================#        
                if isfault ==0:
                    go_to_drop_position() #===========================================================================================
                    rospy.sleep(0.1)
                    print "============ go_to_drop_position finished with status = %d" % move_result
                    move_result = 0
                    #print group.get_current_pose().pose   
                #================================================================================================================#
                
                #=============================================## down to insert ##===============================================#
                if isfault ==0:
                    init_torque = wrench.wrench.torque.y
                    print "============ y torque at drop state (init = %f)" % init_torque    

                    waypoints = []
                    current_pose = group.get_current_pose().pose           
                    waypoints.append(copy.deepcopy(current_pose))   

                    print "============ pose before down to insert"
                    print group.get_current_pose().pose   

                    target_pose_tilt = copy.deepcopy(current_pose)        

                    count = 0
                    n = 100
                    while count < n:
                        target_pose_tilt.position.z = target_pose_tilt.position.z + down_z/n
                        waypoints.append(copy.deepcopy(target_pose_tilt))  
                        count = count + 1

                    del waypoints[:1]            
                    execute_waypoints(waypoints, velocity, False) #================================================================      

                    while True:
                        if move_result == 3:
                            print "============ down to insert finished with status = %d" % move_result
                            break
                        if wrench.wrench.torque.y >= init_torque  + y_torque_limit:
                            group.stop()
                            print "======================== down to insert stop due to y torque (current = %f)" % wrench.wrench.torque.y     
                            isfault = 1  

                            offset_pose = group.get_current_pose().pose
                            offset_pose.position.z = offset_pose.position.z + fault_offset_z
                            go_pose(offset_pose, velocity, False) #================================================================
                            rospy.sleep(0.5)
                            print "============ down to insert execution stop with status = %d" % move_result
                            break
                    rospy.sleep(0.1)
                    move_result = 0
                    print "============ pose after down to insert"
                    print group.get_current_pose().pose          
                #================================================================================================================#
                
                #=================================================## slide to insert ##==========================================#
                if isfault ==0:
                    gripper_move(pub, position_of_gripper_for_sliding, velocity_of_gripper, force_of_gripper) #===================
                    rospy.sleep(0.1)
                    print "============ pick to up"
                    
                    waypoints = []
                    current_pose = group.get_current_pose().pose           
                    waypoints.append(copy.deepcopy(current_pose))   

                    print "============ pose before slide to insert"
                    print group.get_current_pose().pose   

                    target_pose_tilt = copy.deepcopy(current_pose)        

                    count = 0
                    n = 100
                    while count < n:
                        target_pose_tilt.position.x = target_pose_tilt.position.x + sliding_x/n
                        target_pose_tilt.position.y = target_pose_tilt.position.y + sliding_y/n
                        target_pose_tilt.position.z = target_pose_tilt.position.z + sliding_z/n
                        waypoints.append(copy.deepcopy(target_pose_tilt))  
                        count = count + 1

                    del waypoints[:1]            
                    execute_waypoints(waypoints, velocity, False) #================================================================      

                    while True:
                        if move_result == 3:
                            print "============ slide to insert finished with status = %d" % move_result
                            break
                        if wrench.wrench.torque.y >= init_torque  + y_torque_limit:
                            group.stop()
                            print "======================== slide to insert stop due to y torque (current = %f)" % wrench.wrench.torque.y     
                            isfault = 1  

                            offset_pose = group.get_current_pose().pose
                            offset_pose.position.z = offset_pose.position.z + fault_offset_z
                            go_pose(offset_pose, velocity, False) #================================================================
                            rospy.sleep(0.5)
                            print "============ slide to insert execution stop with status = %d" % move_result
                            break
                    rospy.sleep(0.1)
                    move_result = 0
                    print "============ pose after slide to insert"
                    print group.get_current_pose().pose    
                #================================================================================================================#            


            elif iter % 2 == 1:

                #==============================================## go to align position ##========================================#
                if isfault ==0:
                    go_to_align_position() #===========================================================================================
                    rospy.sleep(0.1)
                    print "============ go_to_align_position finished with status = %d" % move_result
                    move_result = 0
                    #print group.get_current_pose().pose   
                #================================================================================================================#

                #==================================================## down to align ##===========================================#
                if isfault ==0:               
                    waypoints = []
                    current_pose = group.get_current_pose().pose           
                    waypoints.append(copy.deepcopy(current_pose))   

                    print "============ pose before down to align"
                    print group.get_current_pose().pose   

                    target_pose_tilt = copy.deepcopy(current_pose)        

                    count = 0
                    n = 100
                    while count < n:
                        target_pose_tilt.position.z = target_pose_tilt.position.z + down_z_align/n
                        waypoints.append(copy.deepcopy(target_pose_tilt))  
                        count = count + 1

                    del waypoints[:1]            
                    execute_waypoints(waypoints, velocity, True) #================================================================      
                    rospy.sleep(0.1)
                    move_result = 0
                    print "============ pose after down to align"
                    print group.get_current_pose().pose  
                #================================================================================================================#

                #=====================================================## align x ##===============================================#
                if isfault ==0:
                    init_force_align_x = wrench.wrench.force.x
                    print "============ x force at align (init = %f)" % init_force_align_x    

                    waypoints = []
                    current_pose = group.get_current_pose().pose           
                    waypoints.append(copy.deepcopy(current_pose))   

                    print "============ pose before align x"
                    print group.get_current_pose().pose   

                    target_pose_tilt = copy.deepcopy(current_pose)        

                    count = 0
                    n = 100
                    while count < n:
                        target_pose_tilt.position.x = target_pose_tilt.position.x + align_distance_x/n
                        waypoints.append(copy.deepcopy(target_pose_tilt))  
                        count = count + 1

                    del waypoints[:1]            
                    execute_waypoints(waypoints, velocity*0.1, False) #================================================================      

                    while True:
                        if move_result == 3:
                            print "============ align x finished without contact as status = %d" % move_result
                            break
                        if wrench.wrench.force.x >= init_force_align_x  + limit_force_align_x:
                            group.stop()
                            print "======================== align x stop with contact (current = %f)" % wrench.wrench.force.x     

                            offset_pose = group.get_current_pose().pose
                            offset_pose.position.x = offset_pose.position.x + align_offset_x
                            go_pose(offset_pose, velocity*0.1, False) #================================================================
                            rospy.sleep(0.5)
                            print "============ align x execution stop with status = %d" % move_result
                            break
                    rospy.sleep(0.1)
                    move_result = 0
                    print "============ pose after align x"
                    print group.get_current_pose().pose  
                #================================================================================================================#

                #=====================================================## align y ##===============================================#
                if isfault ==0:
                    init_force_align_y = wrench.wrench.force.y
                    print "============ y force at align (init = %f)" % init_force_align_y    

                    waypoints = []
                    current_pose = group.get_current_pose().pose           
                    waypoints.append(copy.deepcopy(current_pose))   

                    print "============ pose before align y"
                    print group.get_current_pose().pose   

                    target_pose_tilt = copy.deepcopy(current_pose)        

                    count = 0
                    n = 100
                    while count < n:
                        target_pose_tilt.position.y = target_pose_tilt.position.y + align_distance_y/n
                        waypoints.append(copy.deepcopy(target_pose_tilt))  
                        count = count + 1

                    del waypoints[:1]            
                    execute_waypoints(waypoints, velocity*0.1, False) #================================================================      

                    while True:
                        if move_result == 3:
                            print "============ align y finished without contact as status = %d" % move_result
                            break
                        if wrench.wrench.force.y <= init_force_align_y + limit_force_align_y:
                            group.stop()
                            print "======================== align y stop with contact (current = %f)" % wrench.wrench.force.y     

                            offset_pose = group.get_current_pose().pose
                            offset_pose.position.y = offset_pose.position.y + align_offset_y
                            go_pose(offset_pose, velocity*0.1, False) #================================================================
                            rospy.sleep(0.5)
                            print "============ align y execution stop with status = %d" % move_result
                            break
                    rospy.sleep(0.1)
                    move_result = 0
                    print "============ pose after align y"
                    print group.get_current_pose().pose  
                #================================================================================================================#
            
                #===========================================## release and passive align ##======================================#
                if isfault == 0:
                    gripper_move(pub, 190, velocity_of_gripper, force_of_gripper) #===================================================
                    rospy.sleep(1.0)
                    print "============ gripper_move() for release"

                    target_pose_to_object = group.get_current_pose().pose 
                    target_pose_to_object.position.z = target_pose_to_object.position.z - 0.015
                    
                    print "============ passive align"
                    go_pose(target_pose_to_object, velocity, True) #==================================================================
                    rospy.sleep(0.1)
                    print "============ passive align finished with status = %d" % move_result
                    move_result = 0

                    target_pose_to_object = group.get_current_pose().pose 
                    target_pose_to_object.position.x = target_pose_to_object.position.x + 0.002
                    target_pose_to_object.position.y = target_pose_to_object.position.y + 0.001

                    print "============ passive align2"
                    go_pose(target_pose_to_object, velocity, True) #==================================================================
                    rospy.sleep(0.1)
                    print "============ passive align2 finished with status = %d" % move_result
                    move_result = 0
                #================================================================================================================#
                
                #====================================================## up ##====================================================#
                if isfault == 0:
                    gripper_move(pub, 180, velocity_of_gripper, force_of_gripper) #===================================================
                    rospy.sleep(1.0)
                    print "============ gripper_move() for up"

                    target_pose_to_object = group.get_current_pose().pose          
                    target_pose_to_object.position.x = target_pose_to_object.position.x + 0.002           
                    target_pose_to_object.position.z = target_pose_to_object.position.z + 0.205

                    print "============ up"
                    go_pose(target_pose_to_object, velocity*0.5, True) #==================================================================
                    rospy.sleep(0.1)
                    print "============ up finished with status = %d" % move_result
                    move_result = 0
                #================================================================================================================#       
            
        
            if isfault == 1:
                break
            else:
                vision_flag = 0
                vision_update_succeed = 0
                move_result = 0
                isfault = 0
                object_orientation_from_vision = 0.0
                object_position_from_vision = [0.0, 0.0]
        
        ##############################################################################################     

        #=================================================## motion end ##===============================================#
        #group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        #================================================================================================================#               

    except rospy.ROSInterruptException: pass
