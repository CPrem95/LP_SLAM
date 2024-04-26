import sys

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox
)
from PyQt5.QtCore import QThread, pyqtSignal, QObject

from PyQt5.uic import loadUi
from ui.range_disp import Ui_MainWindow

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import Float32MultiArray
from rclpy.timer import Rate
import time

rot_thresh = 5000 # Range threshold for rotation [mm]

class UISubscriber(Node, QMainWindow, Ui_MainWindow):

    def __init__(self):
        self.anc_ini = [0,0,0,0,0,0] # Anchor position initialisation [x1, y1, x2, y2, x3, y3]

        super().__init__('UI_subscriber')
        self.subscription = self.create_subscription(
            Int16,
            '/ranges/value1',
            self.listener_callback_1,
            qos_profile=self.set_qos_profile()
            )
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Int16,
            '/ranges/value2',
            self.listener_callback_2,
            qos_profile=self.set_qos_profile()
            )
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Int16,
            '/ranges/value3',
            self.listener_callback_3,
            qos_profile=self.set_qos_profile()
            )
        self.subscription  # prevent unused variable warning

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'anchor_positions',
            self.listener_callback_anc_ini,
            qos_profile=self.set_qos_profile()
            )
        self.subscription  # prevent unused variable warning

    # Anchor position initialisation callback
    def listener_callback_anc_ini(self, msg):
        self.anc_ini = msg.data

    def listener_callback_1(self, msg):
        val = msg.data
        win.lcdNumber.display(val)
        if self.anc_ini[0] != 0:
            win.label.setStyleSheet("background-color: rgb(0, 171, 50);") # green
        elif 0 < val < rot_thresh:
            win.label.setStyleSheet("background-color: rgb(255, 255, 0);") # yellow
        elif val > rot_thresh:
            win.label.setStyleSheet("background-color: rgb(246, 97, 81);") # red
    
    def listener_callback_2(self, msg):
        val = msg.data
        win.lcdNumber_2.display(val)
        if self.anc_ini[2] != 0:
            win.label_2.setStyleSheet("background-color: rgb(0, 171, 50);") # green
        elif 0 < val < rot_thresh:
            win.label_2.setStyleSheet("background-color: rgb(255, 255, 0);") # yellow
        elif val > rot_thresh:
            win.label_2.setStyleSheet("background-color: rgb(246, 97, 81);") # red

    def listener_callback_3(self, msg):
        val = msg.data
        win.lcdNumber_3.display(val)
        if self.anc_ini[4] != 0:
            win.label_3.setStyleSheet("background-color: rgb(0, 171, 50);") # green
        elif 0 < val < rot_thresh:
            win.label_3.setStyleSheet("background-color: rgb(255, 255, 0);") # yellow
        elif val > rot_thresh:
            win.label_3.setStyleSheet("background-color: rgb(246, 97, 81);") # red

    def set_qos_profile(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        return qos_profile

class Worker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def run(self):
        """Long-running task."""
        rclpy.init(args=None)
        uwb_subscriber = UISubscriber()

        # rclpy.spin(uwb_subscriber)
        while 1:
            rclpy.spin_once(uwb_subscriber)
            time.sleep(0.1)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        uwb_subscriber.destroy_node()
        rclpy.shutdown()

        self.finished.emit()

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.thread = QThread()
        self.worker = Worker()
        self.worker.moveToThread(self.thread)
        
        self.thread.started.connect(self.worker.run)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.thread.finished.connect(self.thread.deleteLater)
        # self.worker.progress.connect(self.reportProgress)

        self.thread.start()

app = QApplication(sys.argv)
win = Window()

def main():
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()