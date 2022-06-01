from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer

class mainWindow(QWidget):
	def __init__(self, parent=None):
		super(mainWindow, self).__init__(parent)
		self.initUI()

	def initUI(self):
		Layout = QGridLayout()

		OBJ1Label = QLabel("Select OBJ1")
		Layout.addWidget(OBJ1Label, 0, 0)
		OBJ2Label = QLabel("Select OBJ2")
		Layout.addWidget(OBJ2Label, 0, 1)

		self.obj1List = QListWidget()
		self.obj2List = QListWidget()

		Layout.addWidget(self.obj1List, 1, 0)
		Layout.addWidget(self.obj2List, 1, 1)

		CheckBoxLayout = QHBoxLayout()
		self.cbScene = QCheckBox("Scene")
		self.cbEdge = QCheckBox("Edge")
		self.cbMarker = QCheckBox("Marker")
		CheckBoxLayout.addWidget(self.cbScene)
		CheckBoxLayout.addWidget(self.cbEdge)
		CheckBoxLayout.addWidget(self.cbMarker)

		Layout.addLayout(CheckBoxLayout, 2, 0, 1, 3)

		self.setLayout(Layout)

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    mainWindow_ = mainWindow()
    mainWindow_.show()
    sys.exit(app.exec())
