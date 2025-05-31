# -*- coding: utf-8 -*-

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QFrame, QLabel, QPushButton, QComboBox, QWidget
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import QMetaObject, QRect
from matplotlib.figure import Figure
import json
import os

class MatplotlibCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = fig.add_subplot(111)
        super(MatplotlibCanvas, self).__init__(fig)
        self.setParent(parent)

class Ui_Form(QWidget):
    def __init__(self,node):
        super(Ui_Form, self).__init__()
        path = os.environ["BRIAN_JSON"]
        self.node = node
        with open(path) as f:
            data = json.load(f)
            self.duration = data["duration"]
            self.monitors = data["pub_monitors"]
            self.variable_info = data["variable_info"]
        self.setupUi(self)
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(800, 500)
        Form.setWindowTitle("Brian Monitor Viewer")

        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(245, 245, 245))
        Form.setPalette(palette)

        self.main_layout = QFrame(Form)
        self.main_layout.setGeometry(QRect(10, 10, 780, 480))
        self.main_layout.setFrameShape(QFrame.StyledPanel)
        self.main_layout.setFrameShadow(QFrame.Raised)

        self.plot = MatplotlibCanvas(parent=self.main_layout, width=5, height=4)
        self.plot.setGeometry(QRect(10, 10, 500, 460))
        self.plot.setObjectName("plot")

        self.right_panel = QFrame(self.main_layout)
        self.right_panel.setGeometry(QRect(520, 10, 250, 460))
        self.right_panel.setFrameShape(QFrame.StyledPanel)
        self.right_panel.setFrameShadow(QFrame.Raised)

        self.comboBox = QComboBox(self.right_panel)
        self.comboBox.setGeometry(QRect(20, 30, 210, 30))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.setFont(QFont("Arial", 10))
        self.comboBox.addItems([monit["name"] for monit in self.monitors])  

        self.label_input = QLabel(self.right_panel)
        self.label_input.setGeometry(QRect(20, 80, 210, 20))
        self.label_input.setText("Durée à afficher (ms) :")
        self.label_input.setFont(QFont("Arial", 10))

        from PyQt5.QtWidgets import QLineEdit
        self.textInput = QLineEdit(self.right_panel)
        self.textInput.setGeometry(QRect(20, 110, 210, 30))
        self.textInput.setPlaceholderText("Ex: 0")
        self.textInput.setFont(QFont("Arial", 10))

        self.showButton = QPushButton(self.right_panel)
        self.showButton.setGeometry(QRect(20, 160, 210, 40))
        self.showButton.setText("Show")
        self.showButton.setFont(QFont("Arial", 11, QFont.Bold))
        self.showButton.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; border-radius: 5px; }"
                                    "QPushButton:hover { background-color: #45a049; }")


        QMetaObject.connectSlotsByName(Form)
