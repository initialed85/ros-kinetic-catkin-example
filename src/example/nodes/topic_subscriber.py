#!/usr/bin/env python

"""This script subscribes to a topic and logs the ExampleMessage objects read"""

import rospy

from example.msg import ExampleMessage

NODE_NAME = 'topic_subscriber'
TOPIC_NAME = 'topic'


def topic_callback(data):
    rospy.loginfo('{} - {}'.format(
        rospy.get_caller_id(),
        data
    ))


def run():
    rospy.init_node(NODE_NAME)

    _ = rospy.Subscriber(TOPIC_NAME, ExampleMessage, topic_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
