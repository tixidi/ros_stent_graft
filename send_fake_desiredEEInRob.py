#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('iiwa0_desiredEEInRob', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
				msg = Float64MultiArray()
				#msg.data = [-0.864252, -0.485083, 0.133277, 0.0865756, -0.000158412, -0.264671, -0.964339, 0.00806532, 0.503059, -0.833453, 0.228665, 0.15361]
				#msg.data = [-8.642523e-01, -4.850827e-01, 1.332769e-01, 8.657560e-02, -1.584119e-04, -2.646708e-01, -9.643388e-01, 8.065320e-03, 5.030586e-01, -8.334531e-01, 2.286655e-01, 1.536100e-01]
				msg.data = [-4.800266e-01, -8.635465e-01, 1.544729e-01, 1.432567e-01, 2.227581e-01, -2.903022e-01, -9.306468e-01, -7.883426e-02, 8.485006e-01, -4.123251e-01, 3.317147e-01, 5.948348e-02]
				rospy.loginfo(msg)
				pub.publish(msg)
				rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
