# -*- coding: utf-8 -*-

from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QFrame, QLabel, QPushButton, QComboBox, QWidget, QVBoxLayout
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import QMetaObject, QRect
from matplotlib.figure import Figure

class MatplotlibCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.ax = fig.add_subplot(111)
        super(MatplotlibCanvas, self).__init__(fig)
        self.setParent(parent)

class Ui_Form(QWidget):
    def __init__(self,node):
        super(Ui_Form, self).__init__()
        self.node = node

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
        self.toolbar_container = QWidget(self.main_layout)
        self.toolbar_container.setGeometry(QRect(10, 0, 500, 30))  # Position sous le plot
        self.toolbar_layout = QVBoxLayout(self.toolbar_container)
        self.toolbar_layout.setContentsMargins(0, 0, 0, 0)
        self.toolbar = NavigationToolbar(self.plot, self)
        self.toolbar_layout.addWidget(self.toolbar)

        self.right_panel = QFrame(self.main_layout)
        self.right_panel.setGeometry(QRect(520, 10, 250, 460))
        self.right_panel.setFrameShape(QFrame.StyledPanel)
        self.right_panel.setFrameShadow(QFrame.Raised)

        self.comboBox = QComboBox(self.right_panel)
        self.comboBox.setGeometry(QRect(20, 30, 210, 30))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.setFont(QFont("Futura", 10))

        self.label_input = QLabel(self.right_panel)
        self.label_input.setGeometry(QRect(20, 80, 210, 20))
        self.label_input.setText("Index :")
        self.label_input.setFont(QFont("Futura", 10))

        from PyQt5.QtWidgets import QLineEdit
        self.textInput = QLineEdit(self.right_panel)
        self.textInput.setGeometry(QRect(20, 110, 210, 30))
        self.textInput.setPlaceholderText("Ex: 0")
        self.textInput.setFont(QFont("Futura", 10))

        self.plusButton = QPushButton(self.right_panel)
        self.plusButton.setGeometry(QRect(20, 160, 100, 40))
        self.plusButton.setText("+")
        self.plusButton.setFont(QFont("Futura", 20, QFont.Bold))
        self.plusButton.setStyleSheet("QPushButton { background-color: #2196F3; color: white; border-radius: 5px; }"
                                    "QPushButton:hover { background-color: #0b7dda; }")
        
        self.minusButton = QPushButton(self.right_panel)
        self.minusButton.setGeometry(QRect(130, 160, 100, 40))
        self.minusButton.setText("-")
        self.minusButton.setFont(QFont("Futura", 20, QFont.Bold))
        self.minusButton.setStyleSheet("QPushButton { background-color: #f44336; color: white; border-radius: 5px; }"
                                    "QPushButton:hover { background-color: #da190b; }")
        

        QMetaObject.connectSlotsByName(Form)
