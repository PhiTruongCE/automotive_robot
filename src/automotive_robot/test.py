#!/usr/bin/env python

import rospy
from automotive_robot.test_module import test_name

rospy.init_node("test",anonymous=True)
rospy.loginfo("Hello")
test_name()