#!/usr/bin/env python

# Edited by xbacov04

import sys
import signal
import rospy
from PyQt5 import QtWidgets, QtCore
from xbacov04.gui import SceneViewer


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtWidgets.QApplication.quit()


def main(args):

    rospy.init_node('rescuers_gui_projector', anonymous=True)
    rospy.sleep(1)

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtWidgets.QApplication(sys.argv)

    ps = SceneViewer()

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
