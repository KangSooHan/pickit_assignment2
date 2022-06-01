#!/usr/bin/env python3
import sys
import math
import rospy
import PyKDL
from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QThread

import pickit_gui

from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Bool
from geometry_msgs.msg import Point, Vector3, Quaternion

elements = ["x", "y", "z", "R", "P", "Y"]
frame_name = ["Base", "Flange", "TCP", "Camera", "Object"]

class Main(pickit_gui.mainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)

        # Button clicked Action
        self.BaseButton.clicked.connect(self.create)
        self.ObjectButton.clicked.connect(self.transform)
        self.moveButton.clicked.connect(self.move)
        self.TCPButton.clicked.connect(self.change)

        # Ros Publisher -> publish marker_array topic :: MarkerArray msg
        self.marker_pub = rospy.Publisher("marker_array", MarkerArray, queue_size=1)

        # Init
        self.markers = MarkerArray()
        self.frame_list = None

        # For XYZ axes Arrow
        self.align_y = PyKDL.Rotation.RPY(0, 0, math.pi/2)
        self.align_z = PyKDL.Rotation.RPY(0, -math.pi/2, 0)

        self.show()

    def create(self):
        '''
            Action when clicked create button
            Purpose : create given frames and calculate object pos
            There are given 6 param
                (x, y, z, R, P, Y) -> DSpinBox type
            Read SpinBox & Make base frame
            Save transformation matrix given
            Calculate object pos
        '''
        self.frame_list = list()
        for e in elements:
            globals()[e] = getattr(self, "Base"+e+"Box").value()

        # Base
        vec = PyKDL.Vector(x, y, z)
        rot = PyKDL.Rotation.RPY(R,P,Y)
        base_frame = PyKDL.Frame(rot, vec)
        self.frame_list.append(base_frame)

        # Base2Flange
        vec = PyKDL.Vector(0.036, 1.124, 1.0095)
        rot = PyKDL.Rotation.Quaternion(0.68706, 0.70876, 0.14008, -0.077337)
        b2f_frame = PyKDL.Frame(rot, vec)
        self.frame_list.append(b2f_frame)

        # Flange2TCP
        vec = PyKDL.Vector(0, 0, 0.5)
        rot = PyKDL.Rotation.Quaternion(0, 0, 0, 1)
        f2t_frame = PyKDL.Frame(rot, vec)
        self.frame_list.append(f2t_frame)

        # Flange2Camera
        vec = PyKDL.Vector(0.103, -0.062, 0.152 )
        rot = PyKDL.Rotation.Quaternion(0.059, 0.056, 0.701, 0.708)
        f2c_frame = PyKDL.Frame(rot, vec)
        self.frame_list.append(f2c_frame)

        # Camera2Object
        vec = PyKDL.Vector(0.05585, 0.25085, 1.2182)
        rot = PyKDL.Rotation.Quaternion(0.92704, -0.30428, 0.21370, -0.04183)
        c2o_frame = PyKDL.Frame(rot, vec)
        self.frame_list.append(c2o_frame)

        # Object
        object_frame = base_frame * b2f_frame * f2c_frame * c2o_frame
        self.frame_list.append(object_frame)

        # Visualize
        self.visualize()

    def transform(self):
        '''
            Action when clicked transform button
            Purpose : change object pos
            There are given 6 param
                (x, y, z, R, P, Y) -> DSpinBox type
            Read SpinBox & transform object 6dpos
            Calculate new object pos and save
        '''

        for e in elements:
            globals()[e] = getattr(self, "Object"+e+"Box").value()
        vec = PyKDL.Vector(x, y, z)
        rot = PyKDL.Rotation.RPY(R,P,Y)
        transform_frame = PyKDL.Frame(rot, vec)
        self.frame_list[-1] = self.frame_list[-1] * transform_frame
        self.visualize()

    def move(self):
        '''
            Action when clicked move button
            Purpose : move flange to allow TCP to reach object grasp pos
            Read object pos and calculate object grasp pos(z+0.3 & same x axes & opposite z axes)
            Change base2flange transformation matrix & move flange
            Make object grasp point & TCP same
        '''

        vec = PyKDL.Vector(0, 0, 0.3)
        rot = PyKDL.Rotation.RPY(math.pi, 0, 0)
        o2t_frame = PyKDL.Frame(rot, vec)

        b, b2f, f2t, f2c, c2o, o = self.frame_list
        
        target_frame = o * o2t_frame
        new_b2f = b.Inverse() * target_frame * f2t.Inverse()
        self.frame_list[1] = new_b2f

        self.visualize()

    def change(self):
        '''
            Action when clicked change button
            Purpose : change flange2TCP transformation matrix (new TCP)
            There are given 6 param
                (x, y, z, R, P, Y) -> DSpinBox type
            Change flange2tcp transformation to given transformation matrix & move TCP
        '''

        for e in elements:
            globals()[e] = getattr(self, "TCP"+e+"Box").value()
        vec = PyKDL.Vector(x, y, z)
        rot = PyKDL.Rotation.RPY(R,P,Y)
        self.frame_list[2] = PyKDL.Frame(rot, vec)

        self.visualize()



    def visualize(self):
        '''
            Visualize to RVIZ function
        '''
        self.markers = MarkerArray()

        clear_msg = Marker()
        clear_msg.header.frame_id = "map"
        clear_msg.action = Marker.DELETEALL
        clear_msg.header.stamp = rospy.Time()
        self.markers.markers.append(clear_msg)

        if self.frame_list == None:
            return

        b, b2f, f2t, f2c, c2o, o = self.frame_list

        base = b
        flange = b*b2f
        tcp = flange * f2t
        cam  = flange * f2c
        obj = o

        self.markers = MarkerArray()

        ids = 0

        for i in range(3):
            line= Marker()
            line.header.frame_id = "map"
            line.ns = f"line_{i}"
            line.id = ids
            ids += 1

            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.color = ColorRGBA(0, 0, 0, 1)
            line.scale.x = 0.05

            bp, fp, tp, cp = base.p, flange.p, tcp.p, cam.p

            if i==0:
                line.points.append(Point(bp[0], bp[1], bp[2]))
                line.points.append(Point(fp[0], fp[1], fp[2]))
            elif i==1:
                line.points.append(Point(fp[0], fp[1], fp[2]))
                line.points.append(Point(tp[0], tp[1], tp[2]))
            else:
                line.points.append(Point(fp[0], fp[1], fp[2]))
                line.points.append(Point(cp[0], cp[1], cp[2]))

            self.markers.markers.append(line)

        for name, frame in zip(frame_name, [base, flange, tcp, cam, obj]):
            vec = frame.p
            rot = frame.M

            point = Marker()
            point.header.frame_id = "map"
            point.ns = f"{name}_p"
            point.id = ids
            ids += 1

            point.type = Marker.TEXT_VIEW_FACING
            point.action = Marker.ADD
            point.text = name
            point.color = ColorRGBA(1, 1, 1, 1)
            point.scale = Vector3(0.05, 0.05, 0.05)
            point.pose.position = Point(vec.x(), vec.y(), vec.z())
            self.markers.markers.append(point)

            for axis in ["x", "y", "z"]:
                arrow = Marker()
                arrow.header.frame_id = "map"
                arrow.ns = f"{name}_axis_{axis}"
                arrow.id = ids
                ids += 1

                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD

                if axis=="x":
                    arrow.color = ColorRGBA(1, 0, 0, 1)
                    arot = rot
                elif axis=="y":
                    arrow.color = ColorRGBA(0, 1, 0, 1)
                    arot = rot * self.align_y
                else:
                    arrow.color = ColorRGBA(0, 0, 1, 1)
                    arot = rot * self.align_z
                arrow.scale = Vector3(0.1, 0.02, 0.02)

                qx, qy, qz, qw = list(PyKDL.Rotation.GetQuaternion(arot))

                arrow.pose.position = Point(vec.x(), vec.y(), vec.z())
                arrow.pose.orientation = Quaternion(qx, qy, qz, qw)
                self.markers.markers.append(arrow)

            self.marker_pub.publish(self.markers)


if __name__ == "__main__":
    import sys
    rospy.init_node("pickit_assignment")
    app = QApplication(sys.argv)
    main = Main()
    sys.exit(app.exec_())
