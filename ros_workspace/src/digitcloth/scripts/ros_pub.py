#!/usr/bin/env
# Build $ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8m -DCATKIN_BLACKLIST_PACKAGES="test_hd"
import rospy
from std_msgs.msg import String


if __name__ == "__main__":
    try:
        pub = rospy.Publisher("/digitcloth/string", String, queue_size=1)
        rospy.init_node("digitcloth_node", anonymous=True)
        rate = rospy.Rate(1)
        rospy.loginfo("digitcloth_node Started.")
        while not rospy.is_shutdown():
            msg = String("Hello World")
            pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
