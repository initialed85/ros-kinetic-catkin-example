#!/usr/bin/env python

"""This script posts a contrived ExampleMessage to a topic"""

import random

import rospy
from example.msg import ExampleMessage

NODE_NAME = 'topic_publisher'
TOPIC_NAME = 'topic'


def run(first_name, last_name):
    rospy.init_node(NODE_NAME)

    pub = rospy.Publisher(TOPIC_NAME, ExampleMessage, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        data = ExampleMessage(
            first_name=first_name,
            last_name=last_name,
            age=random.randrange(1, 100),
            score=random.choice([420, 1337, 8008135])
        )

        rospy.loginfo(data)

        pub.publish(data)

        rate.sleep()


if __name__ == '__main__':
    try:
        run('Edward', 'Beech')
    except rospy.ROSInterruptException:
        pass
