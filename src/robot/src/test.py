#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Int32


def random_publisher():
    # Create a new publisher. We specify the topic name, then type of message which is integer
    rospy.init_node("random_number_publisher", anonymous=True)
    pub = rospy.Publisher("random_number", Int32, queue_size=10)

    # Set the loop rate
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        # Generate a random number
        random_number = Int32()
        random_number.data = random.randint(0, 100)

        # Publish the message
        pub.publish(random_number)

        # Log info
        rospy.loginfo("I published a random number: %s", random_number.data)

        # Sleep for the remaining time to hit our 10hz publish rate
        rate.sleep()


if __name__ == "__main__":
    try:
        random_publisher()
    except rospy.ROSInterruptException:
        pass
