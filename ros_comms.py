import rospy
from digitcloth.msg import Edge_msg


def ros_talker():
    pub = rospy.Publisher("digitcloth_edge", Edge_msg)
    rospy.init_node("digitcloth_node", anonymous=True)
    rate = rospy.Rate(10)

    msg = Edge_msg()

    msg.isEdge = True
    msg.P0 = 0


if __name__ == "__main__":
    try:
        ros_talker()
    except rospy.ROSInterruptException:
        pass
