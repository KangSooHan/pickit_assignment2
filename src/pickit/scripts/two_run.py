#!/usr/bin/env python3
import sys
import math
import rospy
import PyKDL
from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread

import two_gui

from visualization_msgs.msg import MarkerArray, Marker
from pickit.msg import Select, PMarker, PPcd

class Main(two_gui.mainWindow):
	def __init__(self, parent=None):
		super().__init__(parent)
		self.pub1 = rospy.Publisher("select", Select, queue_size=1)
		self.pub2 = rospy.Publisher("marker_show", PMarker, queue_size=1)
		self.pub3 = rospy.Publisher("pcd_show", PPcd, queue_size=1)
		self.msg1 = Select()
		self.msg2 = PMarker()
		self.msg3 = PPcd()
		self.msg1.data = ["NaN", "NaN"]

		self.obj1List.itemClicked.connect(self.click1)
		self.obj2List.itemClicked.connect(self.click2)
		self.cbMarker.stateChanged.connect(self.click3)
		self.cbScene.stateChanged.connect(self.click4)
		self.cbEdge.stateChanged.connect(self.click4)

		self.show()

	def callback(self, item_list):
		self.obj1List.clear()
		for item in item_list:
			self.obj1List.addItem(item)
		self.item_list = item_list

	def click1(self, data):
		self.obj2List.clear()
		for item in self.item_list:
			if item != data.text():
				self.obj2List.addItem(item)
		self.msg1.data[0] = data.text()

		if "NaN" not in self.msg1.data:
			self.pub1.publish(self.msg1)

	def click2(self, data):
		self.msg1.data[1] = data.text()
		if "NaN" not in self.msg1.data:
			self.pub1.publish(self.msg1)

	def click3(self):
		self.msg2.data = self.cbMarker.isChecked()
		self.pub2.publish(self.msg2)

	def click4(self):
		self.msg3.scene = self.cbScene.isChecked()
		self.msg3.edge = self.cbEdge.isChecked()
		self.pub3.publish(self.msg3)

class CallBack:
	def __init__(self, window):
		self.item_list = []
		self.window = window

	def callback(self, data):
		self.item_list = [] 
		for marker in data.markers:
			if marker.type==9:
				self.item_list.append(marker.text)	

		self.window.callback(self.item_list)


if __name__ == "__main__":
	import sys
	rospy.init_node("pickit_assignment")

	app = QApplication(sys.argv)
	main = Main()
	cb = CallBack(main)
	rospy.Subscriber("marker_array", MarkerArray, cb.callback)
	sys.exit(app.exec_())
