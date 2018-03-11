#!/usr/bin/env python

import sys
import signal
import rospy
from PyQt5 import QtWidgets, QtCore
import rospkg

from xbacov04.gui import UICoreRos


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtWidgets.QApplication.quit()


def main(args):

    rospy.init_node('rescuers_gui', anonymous=True, log_level=rospy.DEBUG)

    signal.signal(signal.SIGINT, sigint_handler)

    app = QtWidgets.QApplication(sys.argv)

    rospack = rospkg.RosPack()

    translator = QtCore.QTranslator()
    translator.load(rospy.get_param('~locale', 'cs_CZ') + '.qm', rospack.get_path('xbacov04') + '/lang')
    app.installTranslator(translator)

    ui = UICoreRos()

    dbg = rospy.get_param('~show_scene', False)
    if dbg:
        ui.debug_view()

    timer = QtCore.QTimer()
    timer.start(500)
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.

    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main(sys.argv)
    except KeyboardInterrupt:
        print("Shutting down")
