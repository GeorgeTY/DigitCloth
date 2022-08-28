import rospy
from edge_msg import Edge_msg


def ros_talker(isEdge, result):
    pub = rospy.Publisher("digitcloth_edge", Edge_msg, queue_size=10)
    rospy.init_node("digitcloth_edge_node", anonymous=True)
    rate = rospy.Rate(10)

    msg = Edge_msg()

    msg.isEdge = isEdge
    msg.p0 = result[0]
    msg.p1 = result[1]
    # msg.p2 = result[2]

    pub.publish(msg)


if __name__ == "__main__":
    while True:
        try:
            ros_talker(True, [11, 45, 14])
        except rospy.ROSInterruptException:
            pass
