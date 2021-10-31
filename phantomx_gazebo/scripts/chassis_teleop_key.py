#!/usr/bin/env python

import  os
import  sys
import  tty, termios
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('chassis_teleop_key')
    rospy.sleep(1)
    pub = rospy.Publisher(
        "chassis_velocity_set", Twist, queue_size=1)
    
    loopController = rospy.Rate(100)

    msg = Twist()

    while (not rospy.is_shutdown()):

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        
        try :
            tty.setraw( fd )
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


        if ch == 'w':
            msg.linear.x = 1.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif ch == 's':
            msg.linear.x = -1.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif ch == 'a':
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 1.0
        elif ch == 'd':
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = -1.0
        else:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        
        pub.publish(msg)
        loopController.sleep()
