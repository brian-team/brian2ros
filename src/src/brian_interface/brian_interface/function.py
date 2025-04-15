# Import the necessary libraries
import sys
import rclpy
import numpy as np
import subprocess
import signal
import time
import os
import json

# Import the necessary ROS libraries and files
from rqt_plot.rosplot import ROSData
from qt_gui.plugin import Plugin
from .gui import Ui_Form
from rosgraph_msgs.msg import Clock

# Import the necessary PyQt5 libraries
import pyqtgraph as pg
from PyQt5.QtCore import pyqtSignal, QTimer, QRect, QCoreApplication, QSize, Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QApplication, QWidget


class MyPlugin(Plugin):
    progress_sim = pyqtSignal(int)
    time = pyqtSignal(str)

    def __init__(self, context):
        self.context = context
        super(MyPlugin, self).__init__(context)

        # Initialize the variables
        self.sim_launch = False
        self.wait_time = 0
        self._rosdata = {}
        self.data = {}
        self.t = 0
        self.sub = None
        self.with_gazebo = False
        self.start_time = time.time()
        self.start_user = False
        self.sub_main = None

        # Load the JSON file
        path = os.environ["BRIAN_JSON"]
        with open(path) as f:
            data = json.load(f)
            self.duration = data["duration"]
            self.monitors = data["pub_monitors"]
            self.variable_info = data["variable_info"]

        self.setObjectName("MyPlugin")

        # Create the widget and initialize the UI
        self._widget = QWidget()
        self.ui = Ui_Form(self.context.node)
        self.ui.setupUi(self._widget)

        # Connect signals to slots
        self.ui.End_Button.clicked.connect(self.close_plugin)
        self.ui.Start_Button.clicked.connect(self.start_sim)
        self.ui.restart.clicked.connect(self.restart_gazebo)
        self.ui.restart_brian.clicked.connect(self.restart_brian)
        self.ui.loop_button.stateChanged.connect(self.on_loop_button_state_changed)

        self.time.connect(self.ui.textBrowser_2.setText)
        self.progress_sim.connect(self.ui.progressBar.setValue)

        # Add the widget to the context
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        context.add_widget(self._widget)

        # Initialize ROS node without rclpy.init()
        self.node = context.node

        # Create the different plot widgets
        self.plot_widgets = {}
        self._curves = {}
        for monitor in self.monitors:
            if monitor["type"] == "Float64":
                if monitor["name"].split("_")[0] == "spikemonitor":
                    monitor_tab = QWidget()
                    plot_widget = pg.PlotWidget(monitor_tab)
                    plot_widget.setGeometry(QRect(20, 20, 851, 581))
                    self.ui.tabWidget.addTab(monitor_tab, monitor["name"])
                    symbol = "o"
                    symbolPen = pg.mkPen(QColor(Qt.green))

                    plot = plot_widget.plot(
                        [],
                        [],
                        name=monitor["name"],
                        symbol=symbol,
                        symbolPen=symbolPen,
                        symbolSize=10,
                        pen=None,
                    )
                    self.plot_widgets[monitor["name"]] = plot_widget
                    self._curves[monitor["name"]] = plot
                elif monitor["name"].split("_")[0] == "ratemonitor":
                    monitor_tab = QWidget()
                    plot_widget = pg.PlotWidget(monitor_tab)
                    plot_widget.setGeometry(QRect(20, 20, 851, 581))
                    self.ui.tabWidget.addTab(monitor_tab, monitor["name"])

                    symbolPen = pg.mkPen(QColor(Qt.green))

                    plot = plot_widget.plot(
                        [],
                        [],
                        name=monitor["name"],
                        pen=symbolPen,
                        symbol=None,
                        symbolPen=None,
                        symbolSize=4,
                    )

                    self.plot_widgets[monitor["name"]] = plot_widget
                    self._curves[monitor["name"]] = plot
                else:
                    raise Exception("Unknown monitor type")

        # Launch the function once at the beginning to avoid the first choice to be lost
        self.on_loop_button_state_changed(0)

        # Start a timer to update the plot data
        self._first_timestamp = None
        self.timer = QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plot_data)

    def on_loop_button_state_changed(self, state):
        # Check if the loop button is checked
        if self.ui.loop_button.isChecked():
            self.with_gazebo = True
        else:
            self.with_gazebo = False

    def update_plot_data(self):
        # Update the plot data
        for monitor in self.monitors:
            if monitor["type"] == "Float64":
                if monitor["name"] not in self._rosdata:
                    self.data[monitor["name"] + "_x"] = []
                    self.data[monitor["name"] + "_y"] = []
                    ros_data = ROSData(
                        node=self.context.node,
                        topic="/" + monitor["name"] + "/data",
                        start_time=self.wait_time,
                    )
                    if ros_data.error is None:
                        self._rosdata[monitor["name"]] = ros_data

                if monitor["name"] in self._rosdata and monitor["name"] in self._curves:
                    data_x, data_y = self._rosdata[monitor["name"]].next()
                    if not len(data_y):
                        continue
                    if self._first_timestamp is None:
                        self._first_timestamp = data_x[0]
                    old_x, old_y = self._curves[monitor["name"]].getData()
                    if old_x is None:
                        old_x = np.array([])
                    if old_y is None:
                        old_y = np.array([])
                    new_x = np.append(old_x, np.array(data_x) - self._first_timestamp)
                    new_y = np.append(old_y, data_y)

                    self._curves[monitor["name"]].setData(x=new_x, y=new_y)

    def time_callback(self, msg):
        if self.with_gazebo:
            self.t = msg.clock.sec
        else:
            self.t += time.time() - self.start_time
        t_m = self.t // 60
        t_s = self.t % 60

        if t_s < 10:
            str_t_s = "0" + str(t_s)
        else:
            str_t_s = str(t_s)

        if t_m < 10:
            str_t_m = "0" + str(t_m)
        else:
            str_t_m = str(t_m)

        self.time.emit(
            QCoreApplication.translate(
                "Form",
                '<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">\n'
                '<html><head><meta name="qrichtext" content="1" /><style type="text/css">\n'
                "p, li { white-space: pre-wrap; }\n"
                "</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
                '<p align="center" style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:12pt; font-weight:600;">'
                + str_t_m
                + " : "
                + str_t_s
                + "</span></p></body></html>",
                None,
            )
        )

        if self.sim_launch:
            ti = self.t - self.wait_time
            p_t = np.clip(int(ti / self.duration * 100), 0, 100)
            self.progress_sim.emit(p_t)
            if p_t == 100:
                self.sim_launch = False
                self.wait_time = self.t
                self.progress_sim.emit(0)

        else:
            self.wait_time = self.t

    def start_main(self):
        
        modifier = ""
        for row,var in enumerate(self.variable_info):
            if self.ui.tableWidget.item(row, 1).text() != "":
                modifier += "neurongroup." + var["name"] + "=" + self.ui.tableWidget.item(row, 1).text() + ","
                print("\033[35m ➤ New value for : ", var["name"], " = ", self.ui.tableWidget.item(row, 1).text(), "\033[0m")
        if modifier != "":
            self.sub_main = subprocess.Popen(
                [
                    os.environ["COLCON_PREFIX_PATH"]
                    + "/brian_project/lib/brian_project/main",
                    modifier,
                ],
                shell=False,
                env=os.environ,
            )
        else:
            self.sub_main = subprocess.Popen(
                [
                    os.environ["COLCON_PREFIX_PATH"]
                    + "/brian_project/lib/brian_project/main",
                ],
                shell=False,
                env=os.environ,
            )
        time.sleep(1)

    def stop_main(self):
        if self.sub_main:
            try:
                self.sub_main.terminate()
                self.sub_main.wait()  # Ensure the process is terminated
            except Exception as e:
                print(f"❌ \033[31m  Failed to kill Main : {e} \033[0m")
            finally:
                self.sub_main = None
            time.sleep(1)

    def start_gazebo(self):
        self.sub = subprocess.Popen(
            [
                "/bin/bash",
                "-c",
                "source " 
                + os.path.dirname(os.path.realpath(__file__)).split("/src")[0]
                + "/src/install/local_setup.bash && ros2 launch turtlebot3_gz "
                + self.ui.map_box.currentText()
                + ".launch.py",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            shell=False,
            preexec_fn=os.setsid,
        )
        # Wait for Gazebo to start
        time.sleep(5)
        if self.sub.poll() is not None:
            raise Exception(
                "❌ \033[31m Gazebo failed to start, error code: " + str(self.sub.returncode)
            )

    def stop_gazebo(self):
        if self.sub:
            try:
                print("\033[34m Killing Gazebo... \033[0m")
                os.killpg(
                    os.getpgid(self.sub.pid), signal.SIGTERM
                )  # Send the signal to the process group
                self.sub.wait()  # Ensure the process is terminated
                print("\033[32m \u2713 Gazebo killed \033[0m")
            except Exception as e:
                print(f"❌ \033[31m  Failed to kill Gazebo: {e}  \033[0m")
            finally:
                self.sub = None

            time.sleep(1)

    def start_sim(self):
        if self.start_user:
            print("❌ \033[31m Start already launch. Use Restart or Restart Brian instead \033[0m")
        else:
            if self.with_gazebo:
                # Start Gazebo
                print(f"\U0001F680 \033[34m Start Gazebo and Simulation ... \033[0m")

                self.start_gazebo()
                self.start_main()

                self.subscription_time = self.node.create_subscription(
                    Clock,
                    "/clock",
                    self.time_callback,
                    qos_profile=rclpy.qos.qos_profile_sensor_data,
                )
                print(f"\033[32m \u2713 Gazebo and Simulation started \033[0m")
            else:
                # Start real robot
                print("\U0001F680 \033[34m Start Robot Simulation ... \033[0m")

                self.start_main()
                self.timer_robot = QTimer()
                self.timer_robot.setInterval(1000)
                self.timer_robot.timeout.connect(self.time_callback)
            
            # Start the timer to know when the simulation begins
            self.sim_launch = True

            # Start the timer to update the plot
            self.timer.start()
            if self.with_gazebo:
                self.ui.restart.show()
            self.ui.restart_brian.show()

            # Avoid to start the simulation twice
            self.start_user = True



    def restart_gazebo(self):
        print("\033[34m Restarting Gazebo and Simulation ... \033[0m")
        self.stop_gazebo()
        self.stop_main()
        
        self.start_gazebo()
        self.start_main()
        print("\033[32m \u2713 Gazebo and Simulation restarted \033[0m")

        self.sim_launch = True
    def restart_brian(self):
        print("\033[34m Restarting Simulation ... \033[0m")
        self.stop_main()
        self.start_main()

        print("\033[32m \u2713 Simulation restarted \033[0m")

        self.sim_launch = True

    def close_plugin(self):

        print("\033[34m Closing the Simulation ... \033[0m")
        if self.with_gazebo:
            # Close gazebo
            self.stop_gazebo()
        elif "CYCLONEDDS_URI" in os.environ:  
            # Close real robot
            del os.environ["CYCLONEDDS_URI"]
        if self.sub_main is not None:
            print("\033[34m Killing Main ... \033[0m")
            self.stop_main()
        # Close the plugin
        print("\033[32m \u2713 Killing Node ... \033[0m")
        self.context.node.destroy_node()
        rclpy.shutdown()
        print("\033[32m \u2713 Simulation closed \033[0m")

        # Stop the program
        sys.stderr = open(os.devnull, "w")
        sys.exit(0)
        sys.stderr = sys.__stderr__  



if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyPlugin(None)  # Pass None as context when testing outside RQT
    window._widget.show()
    sys.stderr = open(os.devnull, "w")

    sys.exit(app.exec_())
    sys.stderr = sys.__stderr__  
