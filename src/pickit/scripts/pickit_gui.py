from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer

class mainWindow(QWidget):
    def __init__(self, parent=None):
        super(mainWindow, self).__init__(parent)
        self.elements = ["x", "y", "z", "R", "P", "Y"]
        self.initUI()

    def initUI(self):
        def UI(self, name):
            # Make Coords Layout
            Layout = QGridLayout()

            def CreateDSpinBox(self, name, values):
                minimum, maximum, singlestep = values
                setattr(self, name+"Box", QDoubleSpinBox())
                getattr(self, name+"Box").setMinimum(minimum)
                getattr(self, name+"Box").setMaximum(maximum)
                getattr(self, name+"Box").setValue(0)
                getattr(self, name+"Box").setSingleStep(singlestep)

            xyzCLabel = QLabel(name+" xyz")
            Layout.addWidget(xyzCLabel, 0, 0)
            rpyCLabel = QLabel(name+" rpy")
            Layout.addWidget(rpyCLabel, 2, 0)

            for idx, e in enumerate(self.elements):
                if idx>2:
                    values = (-3.14, 3.14, 0.1)
                else:
                    values = (-5, 5, 0.1)
                subname = name + e
                CreateDSpinBox(self, subname, values)
                Layout.addWidget(getattr(self, subname+"Box"), idx//3*2+1, idx%3)

            if name=="Base":
                setattr(self, name+"Button", QPushButton("Create"))
            elif name=="Object":
                setattr(self, name+"Button", QPushButton("Transform"))
            elif name=="TCP":
                setattr(self, name+"Button", QPushButton("Change TCP"))
            getattr(self, name+"Button").setDefault(True)
            Layout.addWidget(getattr(self, name+"Button"), 4, 2)

            return Layout

        self.setWindowTitle("Pickit Qt UI")
        self.resize(320, 240)

        # Make Main Layout
        Layout = QVBoxLayout()

        BaseLayout = UI(self, "Base")
        Layout.addLayout(BaseLayout)
        Layout.addStretch(1)

        ObjectLayout = UI(self, "Object")
        Layout.addLayout(ObjectLayout)
        Layout.addStretch(1)

        TCPLayout = UI(self, "TCP")
        Layout.addLayout(TCPLayout)
        Layout.addStretch(1)

        self.moveButton = QPushButton("Move")
        Layout.addWidget(self.moveButton)

        self.setLayout(Layout)

if __name__ == "__main__":
    import sys
    app = QApplication(sys.argv)
    mainWindow_ = mainWindow()
    mainWindow_.show()
    sys.exit(app.exec())
