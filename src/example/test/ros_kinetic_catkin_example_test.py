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
_min_age = 0
_permitted_scores = [420, 1337, 8008135]


class TopicPublishSubscriberTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node(NODE_NAME)

    # note the TM4J test case number (ide_tnnnn); if you copy and paste this code, be sure to update or remove this
    def test_topic_publisher_ide_t2857(self):
        # arrange

        # act
        data = rospy.wait_for_message(TOPIC_NAME, ExampleMessage, timeout=3)

        # assert
        self.assertEqual(_first_name, data.first_name, 'data.first_name was not {}'.format(_first_name))
        self.assertEqual(_last_name, data.last_name, 'data.last_name was not {}'.format(_last_name))
        self.assertLess(_min_age, data.age, 'data.age was not greater than {}'.format(_min_age))
        self.assertIn(data.score, _permitted_scores, 'data.scores was not in {}'.format(_permitted_scores))

    # note the TM4J test case number (ide_tnnnn); if you copy and paste this code, be sure to update or remove this
    def test_service_hoster_ide_t2858(self):
        # arrange
        _ = rospy.wait_for_message(TOPIC_NAME, ExampleMessage, timeout=3)

        rospy.wait_for_service(SERVICE_NAME, 1)

        service_method = rospy.ServiceProxy(SERVICE_NAME, ExampleService)

        # act
        data = service_method(_first_name, _last_name)

        # assert
        self.assertEqual(_first_name, data.first_name, 'data.first_name was not {}'.format(_first_name))
        self.assertEqual(_last_name, data.last_name, 'data.last_name was not {}'.format(_last_name))
        self.assertLess(_min_age, data.age, 'data.age was not greater than {}'.format(_min_age))


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
