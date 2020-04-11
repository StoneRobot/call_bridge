#! /usr/bin/env python
#coding=utf-8

import rospy
from hirop_msgs.msg import ObjectArray
from hirop_msgs.msg import ObjectInfo

rospy.init_node("test_pickplace")
pub = rospy.Publisher('object_array', ObjectArray , queue_size=10)
rate = rospy.Rate(1)
pose = ObjectArray()
object = ObjectInfo()
object.pose.header.frame_id = "base_link"
object.pose.pose.position.x = 0.418
object.pose.pose.position.y = -0.65
object.pose.pose.position.z = 0.66
object.pose.pose.orientation.x = 0
object.pose.pose.orientation.y = 0
object.pose.pose.orientation.z = -0.706825
object.pose.pose.orientation.w = 0.707388
pose.objects.append(object)

rospy.set_param("/call_bridge/intent", 1)
rospy.set_param("call_bridge/target", 2)
print pose
while True:
    flag = rospy.get_param("/call_bridge/over", False)
    if flag ==  True:
        pub.publish(pose)
        rospy.loginfo("pub pose")
        rospy.set_param("/call_bridge/over", False)