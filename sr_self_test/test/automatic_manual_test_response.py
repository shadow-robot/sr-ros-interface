#! /usr/bin/env python

import rospy
from sr_robot_msgs.srv import ManualSelfTest, ManualSelfTestResponse

class AutomaticResponse(object):
    """
    """

    def __init__(self, ):
        """
        """
        self.srv_ = rospy.Service("/sr_self_test_test/manual_self_tests", ManualSelfTest, self.srv_cb_)

    def srv_cb_(self, req):
        rospy.logerr(req.message)

        return ManualSelfTestResponse(True, "Test")

if __name__ == '__main__':
    rospy.init_node("automatic_response")
    ar = AutomaticResponse()
    rospy.spin()


