#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub1 = rospy.Publisher('messageIMUGPS', String, queue_size=10)
    rospy.init_node('node_IMUGPS')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "depth %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub1.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
