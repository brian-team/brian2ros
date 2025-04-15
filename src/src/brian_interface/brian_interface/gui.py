# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'designerZvQqzG.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PyQt5.QtCore import QCoreApplication, QMetaObject, QRect, Qt
from PyQt5.QtGui import QPalette, QColor, QFont, QBrush, QCursor
from PyQt5.QtWidgets import QFrame, QProgressBar, QCheckBox, QLabel, QTextBrowser, QTabWidget, QPushButton, QComboBox, QTableWidgetItem, QTableWidget
from rqt_plot import plot_widget
from rqt_plot import data_plot
import json
import os


class Ui_Form(object):
    def __init__(self, node):
        super(Ui_Form, self).__init__()
        path = os.environ["BRIAN_JSON"]
        self.node = node
        with open(path) as f:
            data = json.load(f)
            self.duration = data["duration"]
            self.monitors = data["pub_monitors"]
            self.variable_info = data["variable_info"]

    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName("Form")
        Form.resize(1280, 720)
        self.frame = QFrame(Form)
        self.frame.setObjectName("frame")
        self.frame.setGeometry(QRect(1000, 10, 251, 691))
        palette = QPalette()
        brush = QBrush(QColor(0, 0, 0, 255))
        brush.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.WindowText, brush)
        brush1 = QBrush(QColor(255, 255, 255, 255))
        brush1.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Button, brush1)
        palette.setBrush(QPalette.Active, QPalette.Light, brush1)
        palette.setBrush(QPalette.Active, QPalette.Midlight, brush1)
        brush2 = QBrush(QColor(127, 127, 127, 255))
        brush2.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Dark, brush2)
        brush3 = QBrush(QColor(170, 170, 170, 255))
        brush3.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Mid, brush3)
        palette.setBrush(QPalette.Active, QPalette.Text, brush)
        palette.setBrush(QPalette.Active, QPalette.BrightText, brush1)
        palette.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette.setBrush(QPalette.Active, QPalette.Base, brush1)
        palette.setBrush(QPalette.Active, QPalette.Window, brush1)
        palette.setBrush(QPalette.Active, QPalette.Shadow, brush)
        palette.setBrush(QPalette.Active, QPalette.AlternateBase, brush1)
        brush4 = QBrush(QColor(255, 255, 220, 255))
        brush4.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.ToolTipBase, brush4)
        palette.setBrush(QPalette.Active, QPalette.ToolTipText, brush)
        brush5 = QBrush(QColor(0, 0, 0, 128))
        brush5.setStyle(Qt.SolidPattern)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.Active, QPalette.PlaceholderText, brush5)
        # endif
        palette.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette.setBrush(QPalette.Inactive, QPalette.Button, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.Light, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.Midlight, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.Dark, brush2)
        palette.setBrush(QPalette.Inactive, QPalette.Mid, brush3)
        palette.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette.setBrush(QPalette.Inactive, QPalette.BrightText, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette.setBrush(QPalette.Inactive, QPalette.Base, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.Window, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.Shadow, brush)
        palette.setBrush(QPalette.Inactive, QPalette.AlternateBase, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.ToolTipBase, brush4)
        palette.setBrush(QPalette.Inactive, QPalette.ToolTipText, brush)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.Inactive, QPalette.PlaceholderText, brush5)
        # endif
        palette.setBrush(QPalette.Disabled, QPalette.WindowText, brush2)
        palette.setBrush(QPalette.Disabled, QPalette.Button, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Light, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Midlight, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Dark, brush2)
        palette.setBrush(QPalette.Disabled, QPalette.Mid, brush3)
        palette.setBrush(QPalette.Disabled, QPalette.Text, brush2)
        palette.setBrush(QPalette.Disabled, QPalette.BrightText, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.ButtonText, brush2)
        palette.setBrush(QPalette.Disabled, QPalette.Base, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Window, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Shadow, brush)
        palette.setBrush(QPalette.Disabled, QPalette.AlternateBase, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.ToolTipBase, brush4)
        palette.setBrush(QPalette.Disabled, QPalette.ToolTipText, brush)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.Disabled, QPalette.PlaceholderText, brush5)
        # endif
        self.frame.setPalette(palette)
        self.frame.setFrameShape(QFrame.Box)
        self.frame.setFrameShadow(QFrame.Raised)
        self.frame.setLineWidth(5)

        self.line = QFrame(self.frame)
        self.line.setObjectName("line")
        self.line.setGeometry(QRect(10, 290, 241, 20))
        self.line.setLineWidth(5)
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)

        # =====================#
        # Create Start Button #
        # =====================#

        # Create Start Button
        self.Start_Button = QPushButton(self.frame)
        self.Start_Button.setObjectName("Start_Button")
        self.Start_Button.setGeometry(QRect(40, 194, 171, 31))

        # Set Start Button Color
        palette1 = QPalette()
        brush6 = QBrush(QColor(87, 227, 137, 255))
        brush6.setStyle(Qt.SolidPattern)
        palette1.setBrush(QPalette.Active, QPalette.Button, brush6)
        palette1.setBrush(QPalette.Inactive, QPalette.Button, brush6)
        palette1.setBrush(QPalette.Disabled, QPalette.Button, brush6)
        self.Start_Button.setPalette(palette1)

        # Set Start Button Font
        font = QFont()
        font.setBold(True)
        font.setWeight(75)
        font.setStrikeOut(False)
        self.Start_Button.setFont(font)
        self.Start_Button.setCursor(QCursor(Qt.PointingHandCursor))
        self.Start_Button.setAutoFillBackground(True)
        self.Start_Button.setAutoRepeatDelay(300)

        # ===================#
        # Create End Button #
        # ===================#

        # Create End Button
        self.End_Button = QPushButton(self.frame)
        self.End_Button.setObjectName("End_Button")
        self.End_Button.setGeometry(QRect(40, 240, 171, 31))

        # Set End Button Color
        palette2 = QPalette()
        brush7 = QBrush(QColor(246, 97, 81, 255))
        brush7.setStyle(Qt.SolidPattern)
        palette2.setBrush(QPalette.Active, QPalette.Button, brush7)
        palette2.setBrush(QPalette.Inactive, QPalette.Button, brush7)
        palette2.setBrush(QPalette.Disabled, QPalette.Button, brush7)
        self.End_Button.setPalette(palette2)

        # Set End Button Font
        font1 = QFont()
        font1.setBold(True)
        font1.setWeight(75)
        self.End_Button.setFont(font1)
        self.End_Button.setCursor(QCursor(Qt.PointingHandCursor))

        # ==================#
        # Create map_box   #
        # ==================#

        self.map_box = QComboBox(self.frame)
        self.map_box.setObjectName("map_box")
        self.map_box.setGeometry(QRect(40, 80, 161, 25))

        # =====================#
        # Create Progress Bar #
        # =====================#

        self.progressBar = QProgressBar(self.frame)
        self.progressBar.setObjectName("progressBar")
        self.progressBar.setGeometry(QRect(20, 320, 211, 31))
        self.progressBar.setValue(0)

        # ====================#
        # Create loop button #
        # ====================#

        self.loop_button = QCheckBox(self.frame)
        self.loop_button.setObjectName("loop_button")
        self.loop_button.setGeometry(QRect(40, 140, 181, 23))
        self.loop_button.setChecked(True)

        # ====================#
        # Create map label   #
        # ====================#

        self.map_label = QLabel(self.frame)
        self.map_label.setObjectName("map_label")
        self.map_label.setGeometry(QRect(40, 60, 67, 17))

        # ====================#
        # Create Frame 2     #
        # ====================#

        self.frame_2 = QFrame(self.frame)
        self.frame_2.setObjectName("frame_2")
        self.frame_2.setGeometry(QRect(160, 0, 95, 51))
        self.frame_2.setFrameShape(QFrame.Box)
        self.frame_2.setFrameShadow(QFrame.Raised)
        self.frame_2.setLineWidth(5)

        # =====================#
        # Create Text Browser #
        # =====================#

        self.textBrowser_2 = QTextBrowser(self.frame_2)
        self.textBrowser_2.setObjectName("textBrowser_2")
        self.textBrowser_2.setGeometry(QRect(10, 10, 75, 31))

        # =======================#
        # Create Restart Button #
        # =======================#

        # Create Restart Button
        self.restart = QPushButton(self.frame)
        self.restart.setObjectName("restart")
        self.restart.setGeometry(QRect(40, 470, 171, 31))

        # Set Restart Button Color
        brush_r = QBrush(QColor(98, 160, 234, 255))
        palette4 = QPalette()
        palette4.setBrush(QPalette.Active, QPalette.Button, brush_r)
        palette4.setBrush(QPalette.Inactive, QPalette.Button, brush_r)
        palette4.setBrush(QPalette.Disabled, QPalette.Button, brush_r)
        self.restart.setPalette(palette4)

        # Set Restart Button Font
        self.restart.setFont(font1)
        self.restart.setCursor(QCursor(Qt.PointingHandCursor))

        self.restart.hide()

        # =============================#
        # Create Restart Brian Button #
        # =============================#

        # Create Restart Brian Button
        self.restart_brian = QPushButton(self.frame)
        self.restart_brian.setObjectName("restart_brian")
        self.restart_brian.setGeometry(QRect(40, 540, 171, 31))

        # Set Restart Brian Button Color
        palette5 = QPalette()
        palette5.setBrush(QPalette.Active, QPalette.Button, brush_r)
        palette5.setBrush(QPalette.Inactive, QPalette.Button, brush_r)
        palette5.setBrush(QPalette.Disabled, QPalette.Button, brush_r)
        self.restart_brian.setPalette(palette5)

        # Set Restart Brian Button Font
        self.restart_brian.setFont(font1)
        self.restart_brian.setCursor(QCursor(Qt.PointingHandCursor))

        self.restart_brian.hide()

        # ===================#
        # Create Tab Widget #
        # ===================#

        self.tabWidget = QTabWidget(Form)
        self.tabWidget.setObjectName("tabWidget")
        self.tabWidget.setGeometry(QRect(30, 30, 911, 651))

        palette6 = QPalette()
        palette6.setBrush(QPalette.Active, QPalette.WindowText, brush)
        brush9 = QBrush(QColor(222, 221, 218, 255))
        brush9.setStyle(Qt.SolidPattern)
        palette6.setBrush(QPalette.Active, QPalette.Button, brush9)
        palette6.setBrush(QPalette.Active, QPalette.Light, brush1)
        brush10 = QBrush(QColor(238, 238, 236, 255))
        brush10.setStyle(Qt.SolidPattern)
        palette6.setBrush(QPalette.Active, QPalette.Midlight, brush10)
        brush11 = QBrush(QColor(111, 110, 109, 255))
        brush11.setStyle(Qt.SolidPattern)
        palette6.setBrush(QPalette.Active, QPalette.Dark, brush11)
        brush12 = QBrush(QColor(148, 147, 145, 255))
        brush12.setStyle(Qt.SolidPattern)
        palette6.setBrush(QPalette.Active, QPalette.Mid, brush12)
        palette6.setBrush(QPalette.Active, QPalette.Text, brush)
        palette6.setBrush(QPalette.Active, QPalette.BrightText, brush1)
        palette6.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette6.setBrush(QPalette.Active, QPalette.Base, brush1)
        palette6.setBrush(QPalette.Active, QPalette.Window, brush9)
        palette6.setBrush(QPalette.Active, QPalette.Shadow, brush)
        palette6.setBrush(QPalette.Active, QPalette.AlternateBase, brush10)
        palette6.setBrush(QPalette.Active, QPalette.ToolTipBase, brush4)
        palette6.setBrush(QPalette.Active, QPalette.ToolTipText, brush)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette6.setBrush(QPalette.Active, QPalette.PlaceholderText, brush5)
        # endif
        palette6.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette6.setBrush(QPalette.Inactive, QPalette.Button, brush9)
        palette6.setBrush(QPalette.Inactive, QPalette.Light, brush1)
        palette6.setBrush(QPalette.Inactive, QPalette.Midlight, brush10)
        palette6.setBrush(QPalette.Inactive, QPalette.Dark, brush11)
        palette6.setBrush(QPalette.Inactive, QPalette.Mid, brush12)
        palette6.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette6.setBrush(QPalette.Inactive, QPalette.BrightText, brush1)
        palette6.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette6.setBrush(QPalette.Inactive, QPalette.Base, brush1)
        palette6.setBrush(QPalette.Inactive, QPalette.Window, brush9)
        palette6.setBrush(QPalette.Inactive, QPalette.Shadow, brush)
        palette6.setBrush(QPalette.Inactive, QPalette.AlternateBase, brush10)
        palette6.setBrush(QPalette.Inactive, QPalette.ToolTipBase, brush4)
        palette6.setBrush(QPalette.Inactive, QPalette.ToolTipText, brush)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette6.setBrush(QPalette.Inactive, QPalette.PlaceholderText, brush5)
        # endif
        palette6.setBrush(QPalette.Disabled, QPalette.WindowText, brush11)
        palette6.setBrush(QPalette.Disabled, QPalette.Button, brush9)
        palette6.setBrush(QPalette.Disabled, QPalette.Light, brush1)
        palette6.setBrush(QPalette.Disabled, QPalette.Midlight, brush10)
        palette6.setBrush(QPalette.Disabled, QPalette.Dark, brush11)
        palette6.setBrush(QPalette.Disabled, QPalette.Mid, brush12)
        palette6.setBrush(QPalette.Disabled, QPalette.Text, brush11)
        palette6.setBrush(QPalette.Disabled, QPalette.BrightText, brush1)
        palette6.setBrush(QPalette.Disabled, QPalette.ButtonText, brush11)
        palette6.setBrush(QPalette.Disabled, QPalette.Base, brush9)
        palette6.setBrush(QPalette.Disabled, QPalette.Window, brush9)
        palette6.setBrush(QPalette.Disabled, QPalette.Shadow, brush)
        palette6.setBrush(QPalette.Disabled, QPalette.AlternateBase, brush9)
        palette6.setBrush(QPalette.Disabled, QPalette.ToolTipBase, brush4)
        palette6.setBrush(QPalette.Disabled, QPalette.ToolTipText, brush)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette6.setBrush(QPalette.Disabled, QPalette.PlaceholderText, brush5)
        # endif
        self.tabWidget.setPalette(palette6)
        self.tabWidget.setCursor(QCursor(Qt.CrossCursor))
        self.tabWidget.setToolTipDuration(0)
        self.tabWidget.setElideMode(Qt.ElideNone)

        # ====================#
        # Create Plot Widget #
        # ====================#

        self.rqt_plot = plot_widget.PlotWidget(node=self.node)
        self._data_plot = data_plot.DataPlot(self.rqt_plot)
        self._data_plot.set_autoscale(x=False)
        self._data_plot.set_autoscale(
            y=data_plot.DataPlot.SCALE_EXTEND | data_plot.DataPlot.SCALE_VISIBLE
        )
        self._data_plot.set_xlim([0, 10.0])
        print(f"\n\n TOPIC LIST {self.rqt_plot._node.get_topic_names_and_types()}\n\n")
        self.rqt_plot.switch_data_plot_widget(self._data_plot)
        self.tabWidget.addTab(self.rqt_plot, "Plot")

        # ====================#
        # Create Table Widget #
        # ====================#
        
        self.tableWidget = QTableWidget(self.tabWidget)
        self.tableWidget.setObjectName("tableWidget")
        self.tableWidget.setRowCount(len(self.variable_info))
        self.tableWidget.setColumnCount(3)
        
        for row,var in enumerate(self.variable_info):
            
            name = QTableWidgetItem(var["name"])   
            name.setFlags(name.flags() & ~Qt.ItemIsEditable)  # Désactive l'édition
            self.tableWidget.setItem(row, 0, name)
            
            value = QTableWidgetItem(None)
            self.tableWidget.setItem(row, 1, value)
            
            dim = QTableWidgetItem(var["dimension"])
            dim.setFlags(dim.flags() & ~Qt.ItemIsEditable)
            self.tableWidget.setItem(row, 2, dim)
            
        self.tabWidget.addTab(self.tableWidget, "Table")
        self.tableWidget.setHorizontalHeaderLabels(["Name", "Value", "Dimension"])
        self.tableWidget.horizontalHeader().setSectionResizeMode(0, 1)
        self.tableWidget.horizontalHeader().setSectionResizeMode(1, 1)
        self.tableWidget.setColumnWidth(0, 200)

        
        # ====================#

        self.tabWidget.raise_()
        self.frame.raise_()
        # if QT_CONFIG(shortcut)
        self.map_label.setBuddy(self.map_box)
        # endif // QT_CONFIG(shortcut)

        self.retranslateUi(Form)

        self.tabWidget.setCurrentIndex(0)

        QMetaObject.connectSlotsByName(Form)

    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", "Form", None))
        self.Start_Button.setText(QCoreApplication.translate("Form", "Start", None))
        self.End_Button.setText(QCoreApplication.translate("Form", "End", None))
        folder = os.path.dirname(os.path.realpath(__file__)).split("/src")[0] + "/src/src/turtlebot3_gz/launch"

        file = []
        if os.path.exists(folder):

            for name in os.listdir(folder):
                path = os.path.join(folder, name)
                if os.path.isfile(path):
                    if (
                        name != "spawn_turtlebot3.launch.py"
                        and name != "robot_state_publisher.launch.py"
                    ):
                        file.append(name)
                    print("File : ", name)
            for i in range(len(file)):
                self.map_box.addItem("")
                self.map_box.setItemText(
                    i, QCoreApplication.translate("Form", file[i].split(".")[0], None)
                )

        self.loop_button.setText(
            QCoreApplication.translate("Form", "With Gazebo", None)
        )
        self.map_label.setText(QCoreApplication.translate("Form", "Map", None))
        self.textBrowser_2.setHtml(
            QCoreApplication.translate(
                "Form",
                '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">\n'
                '<html><head><meta name="qrichtext" content="1" /><style type="text/css">\n'
                "p, li { white-space: pre-wrap; }\n"
                "</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
                '<p align="center" style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:12pt; font-weight:600;">00 : 00</span></p></body></html>',
                None,
            )
        )
        self.restart.setText(QCoreApplication.translate("Form", "Restart", None))
        self.restart_brian.setText(
            QCoreApplication.translate("Form", "Restart Brian", None)
        )

    # retranslateUi
