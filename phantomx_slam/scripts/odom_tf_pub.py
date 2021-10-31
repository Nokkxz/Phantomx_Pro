#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import rospy

class Odom_tf_pub:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.trans = TransformStamped()

    def callback(self, msg):
        self.trans.header.stamp = rospy.Time.now()
        self.trans.header.frame_id = "odom"
        self.trans.child_frame_id = "phantomx_base_link"

        self.trans.transform.translation.x = msg.pose.pose.position.x
        self.trans.transform.translation.y = msg.pose.pose.position.y
        self.trans.transform.translation.z = msg.pose.pose.position.z

        self.trans.transform.rotation.x = msg.pose.pose.orientation.x
        self.trans.transform.rotation.y = msg.pose.pose.orientation.y
        self.trans.transform.rotation.z = msg.pose.pose.orientation.z
        self.trans.transform.rotation.w = msg.pose.pose.orientation.w

        self.br.sendTransform(self.trans)
        # print(self.trans)


if __name__ == '__main__':
    rospy.init_node("odom_tf_oub")
    odom_tf_pub = Odom_tf_pub()
    rospy.spin()