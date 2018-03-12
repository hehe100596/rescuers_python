#!/usr/bin/env python

# Edited by xbacov04

import os
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ProjectorHelper():

    def __init__(self, proj_id):

        self.proj_id = proj_id
        self.calibrated = None
        self.calibrated_cb = None
        self.calibrating = False

        self.proj_ns = "/art/" + proj_id + "/projector/"

        self.calib_sub = rospy.Subscriber(self.proj_ns + "calibrated", Bool, self.calib_cb, queue_size=10)
        self.srv_calibrate = rospy.ServiceProxy(self.proj_ns + "calibrate", Trigger)

    def wait_until_available(self):

        self.srv_calibrate.wait_for_service()

        while self.calibrated is None:

            rospy.sleep(0.1)

    def calibrate(self, calibrated_cb=None):

        if self.calibrating:
            return False

        if self.is_calibrated():

            if calibrated_cb is not None:
                calibrated_cb(self)
            return True

        self.calibrated_cb = calibrated_cb
        self.calibrating = True

        try:
            ret = self.srv_calibrate()
        except rospy.ServiceException:
            self.calibrating = False
            self.calibrated_cb = None
            return False

        return ret.success

    def is_calibrated(self):

        return self.calibrated

    def calib_cb(self, msg):

        self.calibrated = msg.data
        self.calibrating = False
        if self.calibrated_cb is not None:
            self.calibrated_cb(self)
            self.calibrated_cb = None

    def chessboard(self):

        ret = self.srv_calibrate()
        os.system('rostopic pub --once /art/interface/projected_gui/app/projectors_calibrated std_msgs/Bool "data: true"')
