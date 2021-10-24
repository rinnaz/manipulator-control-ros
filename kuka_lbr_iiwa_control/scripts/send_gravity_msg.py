#!/usr/bin/env python3
import time
import rospy 
from rnrt_msgs.msg import JointGravity
import numpy as np
import math as m
from rosgraph_msgs.msg import Clock


def talker(qlist):
    rospy.init_node("pose_controller", anonymous=True)
    pub = rospy.Publisher("/kuka_lbr_iiwa_14_r820/tr_controller/gravity_input", 
                          JointGravity, 
                          queue_size=1)

    msg = JointGravity()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ""

    msg.name = ['joint_a' + str(x) for x in range(1, 8)]
    
    msg.gravity_torque = qlist
    
    time.sleep(0.5)

    pub.publish(msg)


# ============================== MAIN goes here ========================================


if __name__ == '__main__':
  
    qlist = [
                100.0,
                100.0,
                100.0,
                100.0,
                10.0,
                10.0,
                10.0
            ]

    try:
        talker(qlist)
    except rospy.ROSInterruptException:
        pass
