# Import the necessary libraries
import sys
import rclpy
import numpy as np
import subprocess
import signal
import time
import os
import json
from pathlib import Path
import shutil

# Import the necessary ROS libraries and files
from rqt_plot.rosplot import ROSData
from qt_gui.plugin import Plugin
from .gui import Ui_Form
from .gui_show_result import Ui_Form as Ui_Form_Result
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

# Import the necessary PyQt5 libraries
import pyqtgraph as pg
from PyQt5.QtCore import pyqtSignal, QTimer, QRect, QCoreApplication, QSize, Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QApplication, QWidget

RESULT_FOLDER = os.path.dirname(os.path.realpath(__file__)).split("/src")[0] + "/src/src/brian_project/results"
DEBUG_FOLDER = os.path.dirname(os.path.realpath(__file__)).split("/src")[0] + "/src/src/brian_project/debug"

class B2R_UI_Plugin(Plugin):
    progress_sim = pyqtSignal(int)
    time = pyqtSignal(str)

    def __init__(self, context):
        self.context = context
        super(B2R_UI_Plugin, self).__init__(context)

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
        self.ui_results = None
        self.current_plot = []
        # Load the JSON file
        path = os.environ["BRIAN_JSON"]
        with open(path) as f:
            data = json.load(f)
            self.duration = data["duration"]
            self.monitors = data["pub_monitors"]
            self.variable_info = data["variable_info"]

        self.setObjectName("B2R_UI_Plugin")

        # Create the widget and initialize the UI
        self._widget = QWidget()
        self.ui = Ui_Form(self.context.node)
        self.ui_results = Ui_Form_Result(self.context.node)
        self.ui.setupUi(self._widget)

        # Connect signals to slots
        self.ui.End_Button.clicked.connect(self.close_plugin)
        self.ui.Start_Button.clicked.connect(self.start_sim)
        self.ui.restart.clicked.connect(self.restart_gazebo)
        self.ui.restart_brian.clicked.connect(self.restart_brian)
        self.ui.loop_button.stateChanged.connect(self.on_loop_button_state_changed)
        self.ui.show_results.clicked.connect(self.show_results)
        self.ui.stop_brian.clicked.connect(self.publish_stop_brian)

        self.ui_results.plusButton.clicked.connect(self.add_display)
        self.ui_results.minusButton.clicked.connect(self.remove_last_display)

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
        self.create_plot_widget()

        # Launch the function once at the beginning to avoid the first choice to be lost
        self.on_loop_button_state_changed(0)

        # Start a timer to update the plot data
        self._first_timestamp = None
        self.timer = QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.update_plot_data)

        self.publisher_control = self.node.create_publisher(String, "brian_control", 10)

    
##=====================================##
# Controll functions with the main file #
##=====================================##

    def publish_stop_brian(self):
        # Publish a message to stop the simulation
        msg = String()
        msg.data = "stop"
        self.publisher_control.publish(msg)
        print("\033[34m Stopping the simulation ... \033[0m")

    def publish_shutdown_brian(self):
        # Publish a message to shutdown the simulation
        msg = String()
        msg.data = "shutdown"
        self.publisher_control.publish(msg)
        print("\033[34m Shutting down the simulation ... \033[0m")


##======================##
# Show results functions #
##======================##

    # Function to empty the results folder 
    def empty_file(self, file_path):
        file = Path(file_path)
        for element in file.iterdir():
            if element.is_file() or element.is_symlink():
                element.unlink()
            elif element.is_dir():
                shutil.rmtree(element)

    # Check if the folder is empty
    def is_file_empty(self, file_path):
        return not any(Path(file_path).iterdir())

    def show_results(self):
        if not self.is_file_empty(RESULT_FOLDER):
            print("\033[34m Opening results folder ... \033[0m")
            self.monitors_name = self.find_file_name()
            self.ui_results.comboBox.clear()
            self.ui_results.comboBox.addItems(
                [name for name in self.monitors_name.keys()]
            )
            self.ui_results.plot.ax.clear()
            self.ui_results.show()
        else:
            print("\033[31m ❌ Results folder is empty, please start a new simulation or wait for the end of the current one \033[0m")

    ##=====
    # Function in the show results window
    ##=====
    

        
    def find_file_name(self):
        # Find all files in the results folder that match with any monitor type
        # Ne pas changer le nom des monitors sinon il ne les trouve plus
        folder = Path(RESULT_FOLDER)
        files_name = [f.name for f in folder.iterdir() if f.is_file()]

        looking_for = ["statemonitor", "spikemonitor", "ratemonitor"]

        unmatch_x = []
        unmatch_y = []

        res = {}
        for name in files_name:
            for look in looking_for:

                if look in name.split("_") and "t" in name.split("_") and "dynamic" in name.split("_"):
                    unmatch_x.append([name, os.path.join(folder, name)])

                if look in name.split("_") and "t" not in name.split("_") and "dynamic" in name.split("_"):
                    unmatch_y.append([name, os.path.join(folder, name),look])

        for x in unmatch_x:
            for y in unmatch_y:

                if x[0].split("_dynamic_array_")[-1].split("_")[:-2] == y[0].split("_dynamic_array_")[-1].split("_")[:-2]:
                    res["_".join(y[0].split("_")[:-1])] = {"x": x[1], "y": y[1], "type": y[2]}

        return res
            
    def get_value(self, fname_x, fname_y, monitor_type, new_indice):

        with open(fname_x, "rb") as f_x:
            data_x = np.fromfile(f_x, dtype=np.float64)

        with open(fname_y, "rb") as f_y:
            if monitor_type == "spikemonitor":
                # For spikemonitor, we read the data as int32
                data_y = np.fromfile(f_y, dtype=np.int32)
                resh = False  # Spikemonitor data is not reshaped
            elif monitor_type == "ratemonitor":
                # For ratemonitor, we read the data as float64
                data_y = np.fromfile(f_y, dtype=np.float64)
                resh = False  # Ratemonitor data is not reshaped
            elif monitor_type == "statemonitor":
                # For statemonitor, we read the data as float64
                data_y = np.fromfile(f_y, dtype=np.float64)
                resh = True  # Statemonitor data is reshaped to 2D
            else:
                raise ValueError(f"Unknown monitor type: {monitor_type}")
            
        if resh:
            resh_value = len(data_y) // len(data_x)
            data_y = data_y.reshape(-1, resh_value).T

        if new_indice is not None:
            return data_x, data_y[new_indice]
        else:
            return data_x, data_y

    def plot_array_2d(self, x, y, monitor_type):
        if monitor_type == "spikemonitor":
            self.ui_results.plot.ax.plot(x, y, '.')
        elif monitor_type == "ratemonitor":
            self.ui_results.plot.ax.plot(x, y, marker='o', linestyle='-', color='g')
        elif monitor_type == "statemonitor":
            self.ui_results.plot.ax.plot(x, y, '-', linewidth=1.)
        self.ui_results.plot.ax.set_title("Simulation Results")
        self.ui_results.plot.ax.set_xlabel("Time")
        self.ui_results.plot.ax.set_ylabel("Value")
        self.ui_results.plot.figure.tight_layout()
        self.ui_results.plot.draw()
        
    # Function to add a new plot, called when the user clicks on the show button
    def add_display(self):

        # Check if plot is already displayed
        new_display = self.ui_results.comboBox.currentText()
        new_indice = self.ui_results.textInput.text()
        for plot in self.current_plot:
            if plot["name"] == new_display and plot["index"] == int(new_indice):
                print("\033[31m ❌ This plot is already displayed \033[0m")
                return
        
        name_x = self.monitors_name[new_display]["x"]
        name_y = self.monitors_name[new_display]["y"]
        monitor_type = self.monitors_name[new_display]["type"]

        try:
            if monitor_type == "spikemonitor":
                # For spikemonitor, we don't need an index
                new_indice = None
            else:
                new_indice = int(new_indice)
        except ValueError:
            print("\033[31m ❌ Invalid index, please enter a valid integer \033[0m")
            return
        
        x,y = self.get_value(name_x, name_y, monitor_type, new_indice)

        self.plot_array_2d(x,y,monitor_type)
        self.current_plot.append({"name": new_display, "index": new_indice})

    def remove_last_display(self):
        if len(self.current_plot) == 0:
            print("\033[31m ❌ No plot to remove \033[0m")
            return
        last_plot = self.current_plot.pop()
        print(f"\033[34m Removing plot: {last_plot['name']} at index {last_plot['index']} \033[0m")
        self.ui_results.plot.ax.clear()
        for plot in self.current_plot:
            name = plot["name"]
            if self.monitors_name[name]["type"] == "spikemonitor":
                x,y = self.get_value(self.monitors_name[name]["x"],self.monitors_name[name]["y"], self.monitors_name[name]["type"], None)
            else:
                x,y = self.get_value(self.monitors_name[name]["x"], self.monitors_name[name]["y"], self.monitors_name[name]["type"], plot["index"])
            self.plot_array_2d(x, y, self.monitors_name[name]["type"])
        self.ui_results.plot.figure.tight_layout()
        self.ui_results.plot.draw()

##===========================##
# Interface control functions #
##===========================##

    def create_plot_widget(self):
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
                        symbolSize=4,
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
        
        if not self.is_file_empty(RESULT_FOLDER):
            self.empty_file(RESULT_FOLDER)
        if not self.is_file_empty(DEBUG_FOLDER):
            self.empty_file(DEBUG_FOLDER)

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

    def stop_main(self):
        if self.sub_main:
            self.publish_stop_brian()
            time.sleep(1)  # Wait for the process to respond
            try:
                self.publish_shutdown_brian()
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
        #time.sleep(5)
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
        for n in self.plot_widgets:
            self.ui.tabWidget.removeTab(-1)
        self.create_plot_widget()  # Recreate the plot widgets
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
    window = B2R_UI_Plugin(None)  # Pass None as context when testing outside RQT
    window._widget.show()
    sys.stderr = open(os.devnull, "w")

    sys.exit(app.exec_())
    sys.stderr = sys.__stderr__  
