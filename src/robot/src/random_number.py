#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Int32


def random_publisher():
    rospy.init_node("random_number_publisher", anonymous=True)
    pub = rospy.Publisher("random_number", Int32, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        random_number = Int32()
        random_number.data = random.randint(0, 100)
        pub.publish(random_number)
        rospy.loginfo("I published a random number: %s", random_number.data)
        rate.sleep()


if __name__ == "__main__":
    try:
        random_publisher()
    except rospy.ROSInterruptException:
        pass
