#!/usr/bin/env python

import rospy

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
            
rospy.init_node('test_ur', anonymous=True)
pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

def execute():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 50
    command.rFR  = 150
    pub.publish(command)   
    rospy.sleep(3)
    print(1)

    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 0       
    pub.publish(command)   
    rospy.sleep(3)
    print(0)

    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 50
    command.rFR  = 150
    pub.publish(command)   
    rospy.sleep(3)
    print(1)

    command.rPR  = 100
    pub.publish(command)   
    rospy.sleep(2)
    print(2)
    
    command.rSP  = 250
    command.rPR  = 255
    pub.publish(command)   
    rospy.sleep(1)
    print(3)

if __name__ == '__main__':
    try:
        rospy.sleep(1)
        execute()
    except rospy.ROSInterruptException: pass


            
          