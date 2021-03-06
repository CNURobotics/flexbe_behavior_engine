#!/usr/bin/env python
import sys
import os
import unittest
import zlib
import rospy

from flexbe_onboard.flexbe_onboard import FlexbeOnboard
from flexbe_core.proxy import ProxySubscriberCached

from flexbe_msgs.msg import BehaviorSelection, BEStatus, BehaviorLog, BehaviorModification


class TestOnboard(unittest.TestCase):

    def __init__(self, name):
        super(TestOnboard, self).__init__(name)
        self.sub = ProxySubscriberCached({
            'flexbe/status': BEStatus,
            'flexbe/log': BehaviorLog
        })
        self.rate = rospy.Rate(100)
        # make sure that behaviors can be imported
        data_folder = os.path.dirname(os.path.realpath(__file__))
        sys.path.insert(0, data_folder)
        # run onboard and add custom test behaviors to onboard lib
        self.onboard = FlexbeOnboard()
        self.lib = self.onboard._behavior_lib
        self.lib._add_behavior_manifests(data_folder)

    def assertStatus(self, expected, timeout):
        """ Assert that the expected onboard status is received before the timeout. """
        for i in range(int(timeout*100)):
            self.rate.sleep()
            if self.sub.has_msg('flexbe/status'):
                break
        else:
            raise AssertionError('Did not receive a status as required.')
        msg = self.sub.get_last_msg('flexbe/status')
        self.sub.remove_last_msg('flexbe/status')
        self.assertEqual(msg.code, expected)
        return msg

    def test_onboard_behaviors(self):
        behavior_pub = rospy.Publisher('flexbe/start_behavior', BehaviorSelection, queue_size=1)
        rospy.sleep(0.5)  # wait for publisher

        # wait for the initial status message
        self.assertStatus(BEStatus.READY, 1)

        # send simple behavior request without checksum
        be_id, _ = self.lib.find_behavior("Test Behavior Log")
        request = BehaviorSelection()
        request.behavior_id = be_id
        request.autonomy_level = 255
        behavior_pub.publish(request)
        self.assertStatus(BEStatus.ERROR, 2)

        # send valid simple behavior request
        with open(self.lib.get_sourcecode_filepath(be_id)) as f:
            request.behavior_checksum = zlib.adler32(f.read())
        behavior_pub.publish(request)
        self.assertStatus(BEStatus.STARTED, 1)
        self.assertStatus(BEStatus.FINISHED, 3)
        self.assertIn('data', self.sub.get_last_msg('flexbe/log').text)

        # send valid complex behavior request
        be_id, _ = self.lib.find_behavior("Test Behavior Complex")
        request = BehaviorSelection()
        request.behavior_id = be_id
        request.autonomy_level = 255
        request.arg_keys = ['param']
        request.arg_values = ['value_2']
        request.input_keys = ['data']
        request.input_values = ['2']
        with open(self.lib.get_sourcecode_filepath(be_id)) as f:
            content = f.read()
        modifications = [('flexbe_INVALID', 'flexbe_core'), ('raise ValueError("TODO: Remove!")', '')]
        for replace, by in modifications:
            index = content.index(replace)
            request.modifications.append(BehaviorModification(index, index + len(replace), by))
        for replace, by in modifications:
            content = content.replace(replace, by)
        request.behavior_checksum = zlib.adler32(content)
        behavior_pub.publish(request)
        self.assertStatus(BEStatus.STARTED, 1)
        result = self.assertStatus(BEStatus.FINISHED, 3)
        self.assertEqual(result.args[0], 'finished')
        self.assertIn('value_2', self.sub.get_last_msg('flexbe/log').text)

        # send the same behavior with different parameters
        request.arg_keys = ['param', 'invalid']
        request.arg_values = ['value_1', 'should be ignored']
        request.input_keys = []
        request.input_values = []
        behavior_pub.publish(request)
        self.assertStatus(BEStatus.STARTED, 1)
        result = self.assertStatus(BEStatus.FINISHED, 3)
        self.assertEqual(result.args[0], 'failed')
        self.assertIn('value_1', self.sub.get_last_msg('flexbe/log').text)


if __name__ == '__main__':
    rospy.init_node('test_flexbe_onboard')
    import rostest
    rostest.rosrun('flexbe_onboard', 'test_flexbe_onboard', TestOnboard)
