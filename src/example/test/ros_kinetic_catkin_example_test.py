#!/usr/bin/env python

import unittest

import rospy
from example.msg import ExampleMessage
from example.srv import *

PKG = 'example'
NODE_NAME = 'everything_test'
TOPIC_NAME = 'topic'
SERVICE_NAME = 'service'

_first_name = 'Edward'
_last_name = 'Beech'


class TopicPublishSubscriberTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node(NODE_NAME)

    def test_topic_publisher(self):
        data = rospy.wait_for_message(TOPIC_NAME, ExampleMessage, timeout=2)

        self.assert_(data.first_name == _first_name)
        self.assert_(data.last_name == _last_name)
        self.assert_(data.age > 0)
        self.assert_(data.score in [420, 1337, 8008135])

    def test_service_hoster(self):
        # wait for a message (it's a prerequisite for the service passing)
        _ = rospy.wait_for_message(TOPIC_NAME, ExampleMessage, timeout=2)

        rospy.wait_for_service(SERVICE_NAME, 1)

        service = rospy.ServiceProxy(SERVICE_NAME, ExampleService)

        data = service(_first_name, _last_name)

        self.assert_(data.first_name == _first_name)
        self.assert_(data.last_name == _last_name)
        self.assert_(data.age > 0)
        self.assert_(data.score in [420, 1337, 8008135])


class TopicPublishSubscriberSuite(unittest.TestSuite):
    def __init__(self, *args, **kwargs):
        super(TopicPublishSubscriberSuite).__init__(*args, **kwargs)


if __name__ == '__main__':
    import rostest

    rostest.rosrun(
        PKG,
        NODE_NAME,
        TopicPublishSubscriberTest
    )