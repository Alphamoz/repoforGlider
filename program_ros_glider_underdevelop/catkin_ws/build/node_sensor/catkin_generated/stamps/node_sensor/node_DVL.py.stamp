#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from glider_msg.msg import dvlmsg

def talker():
    pub1 = rospy.Publisher('messageDVL', dvlmsg, queue_size=10)
    rospy.init_node('node_DVL')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        dvlMessage = dvlmsg()
        # msg1.header="Ini Message coba coba"

        dvlMessage.header = "!mantapjiwa"
        dvlMessage.errCode = "Ngga error bang"
        # dvlMessage.dataGood = [1,1,1,1]
        # dvlMessage.altitudeBeam = [0.1,0.2,0.3,0.4]
        # dvlMessage.bottomVelocityBeam = [0.0,0.0,0.0,0.0]
        # dvlMessage.velocityInst = [0.1,0.1,0.1]
        
        dvlMessage.rawData = "INI CONTOH RAW DATANYA (dummy)"
        
        # rospy.loginfo(msg1)
        pub1.publish(dvlMessage)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
