#!/usr/bin/env python3
import time
import rospy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math as m
from rosgraph_msgs.msg import Clock


def talker(qlist):
    rospy.init_node("pose_controller", anonymous=True)
    pub = rospy.Publisher("/kuka_lbr_iiwa_14_r820/tr_controller/command", 
                          JointTrajectory, 
                          queue_size=10)

    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_link"

    msg.joint_names = ['joint_a' + str(x) for x in range(1, 8)]
 
    tr_point = JointTrajectoryPoint(positions=qlist, 
                                    velocities=np.zeros(7), 
                                    accelerations=np.zeros(7), 
                                    time_from_start=rospy.Duration(3))
    msg.points.append(tr_point)
    time.sleep(0.5)

    pub.publish(msg)


# ============================== MAIN goes here ========================================


if __name__ == '__main__':
  
    qlist = [
                0.,
                -m.pi/4,
                0.,
                m.pi/4+0.1,
                0.,
                0.,
                0.
            ]

    try:
        talker(qlist)
    except rospy.ROSInterruptException:
        pass
