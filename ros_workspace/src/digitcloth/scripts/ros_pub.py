#!/usr/bin/env
# Build $ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8m -DCATKIN_BLACKLIST_PACKAGES="test_hd"
import rospy
from digitcloth.msg import Edge_msg


if __name__ == "__main__":
    try:
        pub = rospy.Publisher("/digitcloth/string", Edge_msg, queue_size=1)
        rospy.init_node("digitcloth_node", anonymous=True)
        rate = rospy.Rate(1)
        rospy.loginfo("digitcloth_node Started.")
        msg = Edge_msg()
        while not rospy.is_shutdown():
            msg.isEdge = True
            msg.p0 = 0
            msg.p1 = 1
            msg.p2 = 2
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
