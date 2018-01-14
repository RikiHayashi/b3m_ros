#!/usr/bin/env python
# coding:utf-8

import rospy
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':
    rospy.init_node("b3m_send")
    pub = rospy.Publisher("b3m_leg_setangle", Float64MultiArray, queue_size=1)
    pos1 = [0, 0, 0, 0, 0]
    pos2 = [0, -60, -80, 40, 0]
    pos = [pos1, pos2]
    while not rospy.is_shutdown():
        for p in pos:
            fma = Float64MultiArray()
            fma.data = p[:]
            pub.publish(fma)
            rospy.sleep(1)
