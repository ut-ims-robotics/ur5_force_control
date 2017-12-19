#!/usr/bin/env python
import rospy
from geometry_msgs.msg import WrenchStamped


class ForceReader:
    def __init__(self):
        rospy.Subscriber("/wrench", WrenchStamped,
                         self.wrench_callback)

    def wrench_callback(self, wrench_message):
        pass
        #rospy.loginfo("Wrench: {}".format(wrench_message))


if __name__ == '__main__':
    try:
        rospy.init_node("force_reading_test", anonymous=True)
        ForceReader()
        rate = rospy.Rate(30)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
