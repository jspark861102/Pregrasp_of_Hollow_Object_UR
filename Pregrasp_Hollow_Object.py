#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib_msgs.msg
import control_msgs.msg
import math
import numpy
import tf
import copy
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg

##### experiment prameters #####
velocity = 0.15
velocity_fast = 0.25
wrench = geometry_msgs.msg.WrenchStamped()
move_result = 0
d_max = 0.085
isfault = 0

def wrench_callback(msg):
    global wrench
    wrench = msg
    '''
    if wrench.wrench.force.z <= -5.:     
        print "============ stop due to z force = %f" % wrench.wrench.force.z       
        group.stop()
    '''

def move_result_callback(msg):
    global move_result
    move_result = msg.status.status

##### ROS #####
rospy.init_node('test_ur', anonymous=True)
pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
wrench_sub = rospy.Subscriber('/wrench', geometry_msgs.msg.WrenchStamped, wrench_callback)
move_result_sub = rospy.Subscriber('/scaled_pos_joint_traj_controller/follow_joint_trajectory/result', control_msgs.msg.FollowJointTrajectoryActionResult, move_result_callback)
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

#group.go(target_joint, wait=Ture)
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

def go_to_drop_position():
    print "============ go_to_drop_position()"
    group.set_max_velocity_scaling_factor(velocity_fast)
    init_joint = [n*math.pi/180. for n in [176., -93.75, 73.5, 288.3, -4.0, -162.4]]
    group.go(init_joint, wait=True)
    group.stop()

def down_to_insert():
    print "============ go_to_drop_position()"
    group.set_max_velocity_scaling_factor(velocity)
    init_joint = [n*math.pi/180. for n in [176., -98., 95., 272., -4., -85.]]
    group.go(init_joint, wait=True)
    group.stop()
    
def wait_for_vision():
    print "============ setting waypoints for wait_for_vision()"
    waypoints = []

def object_align():
    print "============ setting waypoints for obect_align()"
    waypoints = []

def pregrasp():
    print "============ setting waypoints for pregrasp()"
    waypoints = []

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
        #=================================================## parameters ##===============================================#
        ## object parameters ##
        object_length = 0.068
        object_height = 0.034
        object_orientation = -0.0*math.pi/7 #rotation by global z axis
        
        ## tilt parameters ## these parameters are tunning parameters!
        offset_L =  object_length*2/3*0.95
        offset_h = -object_height*2/3        
        offset_L_tilt_back = 0.000
        offset_h_tilt_back = 0.030
                
        desired_tilt_angle =  math.pi/2 - math.pi/10*1.0 #is is enough due to init grasping tilitng and sliding at point S
        theta_before_pick  = -math.pi/10
        theta_tilt_back    = -(desired_tilt_angle + theta_before_pick)
        
        ## gripper parameters ##
        position_of_girpper_before_pick = 50
        position_of_gripper_for_pick = 230
        position_of_gripper_for_tilt_back = 140        
        velocity_of_gripper = 15
        force_of_gripper = 1        

        ## environment parameters ##
        base_height = 0.01 - 0.001 #base_plate = 0.01, spunge = 0.005, rubber = 0.001
        z_force_limit = 40.0 #N there is issue =====================================================================================
        y_torque_limit = 20.0
        #================================================================================================================#

        ## getting basic information ##
        print "============ Reference frame: %s" % group.get_planning_frame()
        print "============ end effector link: %s" % group.get_end_effector_link()
        
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
                
        
        #==============================================## go to tilt position ##=========================================#
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
                offset_pose.position.z = offset_pose.position.z + 0.1
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
                    offset_pose.position.z = offset_pose.position.z + 0.10
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
            r = math.sqrt(offset_x*offset_x + offset_z*offset_z)

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
                    offset_pose.position.z = offset_pose.position.z + 0.10
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
            target_pose_to_down.position.x = target_pose_to_down.position.x - 0.025
            target_pose_to_down.position.z = target_pose_to_down.position.z - 0.012
            
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
                    offset_pose.position.z = offset_pose.position.z + 0.10
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
            gripper_move(pub, 250, velocity_of_gripper, 10) #===================================================
            rospy.sleep(0.5)
            print "============ gripper_move() for pick"

            target_pose_to_object = group.get_current_pose().pose 
            target_pose_to_object.position.z = target_pose_to_object.position.z + 0.1

            print "============ up"
            go_pose(target_pose_to_object, velocity_fast, True) #==================================================================
            rospy.sleep(0.1)
            print "============ up finished with status = %d" % move_result
            move_result = 0
        #================================================================================================================# 
        
        '''
        #==================================================## realese ##=================================================#
        gripper_move(pub, 0, velocity_of_gripper, force_of_gripper) #=================================
        rospy.sleep(1)
        print "============ realese"
        #================================================================================================================# 
        '''

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
                target_pose_tilt.position.z = target_pose_tilt.position.z - 0.120/n
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
                    #isfault = 1  

                    offset_pose = group.get_current_pose().pose
                    offset_pose.position.z = offset_pose.position.z + 0.10
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
            gripper_move(pub, 200, velocity_of_gripper, force_of_gripper) #===================================================
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
                target_pose_tilt.position.x = target_pose_tilt.position.x + 0.02/n
                target_pose_tilt.position.y = target_pose_tilt.position.y + 0.05/n
                target_pose_tilt.position.z = target_pose_tilt.position.z - 0.01/n
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
                    #isfault = 1  

                    offset_pose = group.get_current_pose().pose
                    offset_pose.position.z = offset_pose.position.z + 0.10
                    go_pose(offset_pose, velocity, False) #================================================================
                    rospy.sleep(0.5)
                    print "============ slide to insert execution stop with status = %d" % move_result
                    break
            rospy.sleep(0.1)
            move_result = 0
            print "============ pose after slide to insert"
            print group.get_current_pose().pose    
        #================================================================================================================#
        
        ##############################################################################################     

        #=================================================## motion end ##===============================================#
        #group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
        #================================================================================================================#               

    except rospy.ROSInterruptException: pass
