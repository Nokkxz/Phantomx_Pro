#!/usr/bin/env python

import rospy
from phantomx_gazebo.gaitController import GaitPlanner
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from threading import Thread
import math


 
class ChassisController:

    def __init__(self, name_space='/phantomx/'):

        # robot speed
        self.Vx = 0.0
        self.Vw = 0.0

        # gazebo configuration
        self.name_space = name_space
        self.joints_feedback_name_list = None
        self.joints_feedback_angles_list = None
        self.joints_angles_set_dict = None

        # Joint state subscriber
        self.joints_state_subscriber = rospy.Subscriber(
            name_space + 'joint_states', JointState, self.joints_state_callback, queue_size=1) # subscribe topic; register callback function for joint state feedback.

        # wait for Gazebo initialization
        while not rospy.is_shutdown():
            if self.joints_feedback_name_list is not None:
                print(self.joints_feedback_name_list)
                break
            rospy.sleep(0.1)
            rospy.loginfo('Waiting for joint state message') 
        
        # register joint control publishers. give commands to the controllers.
        self.joint_angle_command_publisher_dict = {} 
        for joint_name in self.joints_feedback_name_list: 
            p = rospy.Publisher(
                name_space + joint_name + '_position_controller/command', Float64, queue_size=1)
            self.joint_angle_command_publisher_dict[joint_name] = p

        # gait controller
        self.gaitcontroller = GaitPlanner()

        self.ready_angle_dict = {}

        rospy.sleep(1)

    def set_walk_velocity(self, Vx, Vw):
        self.Vx = Vx
        self.Vw = Vw
        self.gaitcontroller.set_target_velocity([Vx,Vw])

    #joint info callback function.
    def joints_state_callback(self, msg): 
        if self.joints_feedback_name_list is None:
            self.joints_feedback_name_list = msg.name #register the name of joints. sequence matters. 
            print(msg)
        self.joints_feedback_angles_list = msg.position  #since the sequence of name list will not be changed, only position need to be updated.

    #get dict of feedback angles. The sequence of name SHOULD correspond to the sequence of feedback angles.
    def get_feedback_angles_dict(self):
        if self.joints_feedback_name_list is None:
            return None
        if self.joints_feedback_angles_list is None:
            return None
        return dict(zip(self.joints_feedback_name_list, self.joints_feedback_angles_list))

    def set_angles(self, set_angle_dict): # set angle for each joint and publish.
        for joint_name, angle in set_angle_dict.items():
            if joint_name not in self.joints_feedback_name_list:
                rospy.logerror('Invalid joint name "' + joint_name + '"')
                continue
            self.joint_angle_command_publisher_dict[joint_name].publish(angle)

    def update(self):
        self.gaitcontroller.update_plan()
        self.joints_angles_set_dict = self.gaitcontroller.get_plan_result()
        self.set_angles(self.joints_angles_set_dict)

    def chassis_set_speed_callback_fun(self,msg):
        self.set_walk_velocity(msg.linear.x,msg.angular.z)



# chassis controller node
if __name__ == '__main__':

    rospy.init_node('ChassisController')

    rospy.sleep(1)
    chassis = ChassisController()
    chassis.set_walk_velocity(0,0)
    loopController = rospy.Rate(100)

    a = rospy.Subscriber(
            "chassis_velocity_set", Twist, chassis.chassis_set_speed_callback_fun, queue_size=1)

    while (not rospy.is_shutdown()):
        chassis.update()
        loopController.sleep()

