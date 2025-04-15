from collections import namedtuple

# import some common tool
import numpy as np
import json

# import some brian2 tools
from brian2.devices.device import (
    all_devices,
    get_device,
)
from brian2.devices.cpp_standalone import device
import os
from brian2.utils.logger import std_silent
from brian2.utils.filetools import in_directory
from brian2.core.preferences import prefs, BrianPreference
from brian2 import Function, second
from brian2.core.variables import ArrayVariable, VariableView
from brian2 import SpikeMonitor, StateMonitor, PopulationRateMonitor, NeuronGroup

# Define the header files for the ROS messages
HEADER_FILES = {
    "LaserScan": "sensor_msgs/msg/laser_scan.hpp",
    "Image": "sensor_msgs/msg/image.hpp",
    "Imu": "sensor_msgs/msg/imu.hpp",
    "NavSatFix": "sensor_msgs/msg/nav_sat_fix.hpp",
    "Odometry": "nav_msgs/msg/odometry.hpp",
    "Path": "nav_msgs/msg/path.hpp",
    "Twist": "geometry_msgs/msg/twist.hpp",
    "PoseStamped": "geometry_msgs/msg/pose_stamped.hpp",
    "PoseWithCovarianceStamped": "geometry_msgs/msg/pose_with_covariance_stamped.hpp",
    "TransformStamped": "tf2_msgs/msg/transform_stamped.hpp",
    "JointState": "sensor_msgs/msg/joint_state.hpp",
    "BatteryState": "sensor_msgs/msg/battery_state.hpp",
    # Additional messages from sensor_msgs
    "Range": "sensor_msgs/msg/range.hpp",
    "FluidPressure": "sensor_msgs/msg/fluid_pressure.hpp",
    "Temperature": "sensor_msgs/msg/temperature.hpp",
    "RelativeHumidity": "sensor_msgs/msg/relative_humidity.hpp",
    "MagneticField": "sensor_msgs/msg/magnetic_field.hpp",
    "CameraInfo": "sensor_msgs/msg/camera_info.hpp",
    "ChannelFloat32": "sensor_msgs/msg/channel_float32.hpp",
    "MultiEchoLaserScan": "sensor_msgs/msg/multi_echo_laser_scan.hpp",
    # Additional messages from geometry_msgs
    "Point": "geometry_msgs/msg/point.hpp",
    "Quaternion": "geometry_msgs/msg/quaternion.hpp",
    "Vector3": "geometry_msgs/msg/vector3.hpp",
    "Pose": "geometry_msgs/msg/pose.hpp",
    "PoseArray": "geometry_msgs/msg/pose_array.hpp",
    "PoseWithCovariance": "geometry_msgs/msg/pose_with_covariance.hpp",
    "TwistStamped": "geometry_msgs/msg/twist_stamped.hpp",
    "TwistWithCovariance": "geometry_msgs/msg/twist_with_covariance.hpp",
    "TwistWithCovarianceStamped": "geometry_msgs/msg/twist_with_covariance_stamped.hpp",
    "Vector3Stamped": "geometry_msgs/msg/vector3_stamped.hpp",
    "Wrench": "geometry_msgs/msg/wrench.hpp",
    # Additional messages from nav_msgs
    "MapMetaData": "nav_msgs/msg/map_meta_data.hpp",
    "OccupancyGrid": "nav_msgs/msg/occupancy_grid.hpp",
    "GridCells": "nav_msgs/msg/grid_cells.hpp",
    # Additional messages from std_msgs
    "Bool": "std_msgs/msg/bool.hpp",
    "Byte": "std_msgs/msg/byte.hpp",
    "ColorRGBA": "std_msgs/msg/color_rgba.hpp",
    "Float32": "std_msgs/msg/float32.hpp",
    "Float64": "std_msgs/msg/float64.hpp",
    "Int8": "std_msgs/msg/int8.hpp",
    "Int16": "std_msgs/msg/int16.hpp",
    "Int32": "std_msgs/msg/int32.hpp",
    "Int64": "std_msgs/msg/int64.hpp",
    "String": "std_msgs/msg/string.hpp",
    "UInt8": "std_msgs/msg/u_int8.hpp",
    "UInt16": "std_msgs/msg/u_int16.hpp",
    "UInt32": "std_msgs/msg/u_int32.hpp",
    "UInt64": "std_msgs/msg/u_int64.hpp",
    # Additional messages from action_msgs
    "GoalInfo": "action_msgs/msg/goal_info.hpp",
    "GoalStatus": "action_msgs/msg/goal_status.hpp",
    "GoalStatusArray": "action_msgs/msg/goal_status_array.hpp",
    # Additional messages from diagnostic_msgs
    "DiagnosticArray": "diagnostic_msgs/msg/diagnostic_array.hpp",
    "DiagnosticStatus": "diagnostic_msgs/msg/diagnostic_status.hpp",
    "KeyValue": "diagnostic_msgs/msg/key_value.hpp",
    # Additional messages from tf2_msgs
    "TFMessage": "tf2_msgs/msg/tf_message.hpp",
}

prefs.register_preferences(
    "devices.ros_standalone",
    prefbasedoc="""
    Preferences for the ROS standalone device.
    """,
    cyclonedds=BrianPreference(
        default=False,
        docs="""
        Whether to use CycloneDDS for the ROS communication.
        """,
    ),
    network_interface=BrianPreference(
        default="",
        validator=lambda x: isinstance(x, str),
        docs="""
        The network interface to use for the ROS communication.
        """,
    ),
    list_address_ip=BrianPreference(
        default=[""],
        validator=lambda x: isinstance(x, list) and all(isinstance(i, str) for i in x),
        docs="""
        The list of IP addresses to use for the ROS communication.
        """,
    ),
    interface=BrianPreference(
        default=True,
        docs="""
        Whether to use the Brian2ROS interface.
        """,
    ),
)


class Publisher:
    """Create and configure a ROS publisher.

        Parameters
        ----------
        name : str,
            Name of the publisher.
        topic : str,
            Name of the ROS topic to publish to.
        topic_type : str,
            Path of the ROS topic to publish to.
        input : dict,
            Should be : {"name": value, "name": value,...}
            List of input variables to publish.
            Ex : {"linear.x": 0.26, "angular.z": 1.82}
        rate : int, optional
            Rate at which to publish messages (default is 500 Hz).
        reset_values : dict, optional
            Should be : {"name": value, "name": value,...}
            List of reset values for the input variables.
            Ex : {"linear.x": 0., "angular.z": 0.}
        header : str, optional
            Path of the header file to include (default is None).
            It has to be add if the topic is not in the HEADER_FILES list.
        """
    def __init__(self, topic, topic_type, input, rate=500, reset_values={}, name=None, header=None):
        
        self.name = name
        self.header = header
        self.topic = topic
        self.topic_type = topic_type
        self.input = input
        self.rate = rate
        self.reset_values = reset_values   
             
class TwistPublisher(Publisher):
    
    def __init__(self, input, rate=500, reset_values={}, name=None, header=None):
        super().__init__(topic="cmd_vel", topic_type="geometry_msgs/msg/Twist", input=input, rate=rate, reset_values=reset_values, name=name, header=header)
       

class Subscriber(Function):
    """
    Create and configure a ROS subscriber.

    Parameters
    ----------
    name : str
        Name of the subscriber.
    topic : str,
        Name of the ROS topic to subscribe to.
    topic_type : str,
        Path of the ROS topic to subscribe to.
    output : dict,
        Should be : {"name": value, "name": value,...}
        List of output variables to subscribe.
        Ex : {"ranges": [0,...]} or {"ranges": None} or {"ranges": []} or {"ranges": [None]} or {"ranges": 0}
    header : str, optional
        Path of the header file to include (default is None).
        It has to be add if the topic is not in the HEADER_FILES list.
    Returns
    -------
    Function
        A function that returns the value of the subscribed topic.
    """
    def __init__(self, name, topic, topic_type, output, header=None):
        super().__init__(self._pose_obj, arg_units=(second, 1, 1), return_unit=1)

        self.name = name
        self.topic = topic
        self.topic_type = topic_type
        self.output = output
        self.header = header
        self.outs = []
        self.var_time_name = "_array_" + self.name + "_time_" + self.name
        
        self.create_code()

    def _pose_obj(self, t, index):
        pass
    
    def create_code(self):
        
        # Create the function to add to Brian's code.
        code = (
            """double """
            + self.name
            + """(double t,int index,int var_index){
                //std::cout << "t = " << t << "var_time = " <<  brian::"""
            + self.var_time_name
            + """[0] << std::endl;

                while(t > brian::"""
            + self.var_time_name
            + """[0]+ 0.2){

                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                std::vector<double> result = {
                """
        )

        # Loop through the output values to verify the format.
        for i, (out_name, out_value) in enumerate(self.output.items()):
                
            # Add the Brian name of the output to the function.
            code += """brian::_array_""" + self.name + """_var_""" + out_name + """[index]"""

            # Add a comma if it is not the last output.
            if i != len(self.output) - 1:
                code += ","

        code += """
                };
                return result[var_index];  
                }"""

        
        self.implementations.add_implementation(
            "cpp", code, compiler_kwds={"headers": ["<thread>", "<chrono>"]}
        )
 
class LaserScanSubscriber(Subscriber):
    
    def __init__(self, name, output, header=None):
        super().__init__(name=name, topic="LaserScan", topic_type="sensor_msgs/msg/LaserScan", output=output, header=header)
    
   
    
class ROSStandaloneDevice(device.CPPStandaloneDevice):
    def __init__(self):
        super().__init__()

        self.headers += ['"brianros.h"', '"std_msgs/msg/float64.hpp"']

        # Initialize the ROSStandaloneDevice by extending the CPPStandaloneDevice from Brian2.
        self.templater = self.code_object_class().templater.derive("brian2ros")

        # Derive a new templater specific to ROS from the existing code object templater.
        self.code_object_class().templater = self.templater



        # Initialize a list to hold the monitor configurations.
        # This list is a global template which can be used in all the project file
        self.templater.env.globals["pub_monitors"] = []
        
        # Initialize a list to hold the variable name for the modifier.
        self.templater.env.globals["variable_info"] = []
        self.templater.env.globals["subscribers"] = []
        self.templater.env.globals["publishers"] = []

        # Initialize a variable which will be used to store the duration of the simulation.
        # Then it will be shared with the json file to be used in the RQT interface.
        self.duration = 0
        
        # Initialize a variable to store the number of Twist/LaserScan messages for the name of the publisher/subscriber.
        self.twist_number = 0 
        self.pub_number = 0
        self.laser_number = 0

        self.file_path = os.path.dirname(os.path.abspath(__file__))

    def build(
        self,
        directory="output",
        compile=True,
        run=True,
        debug=False,
        clean=False,
        with_output=True,
        additional_source_files=None,
        run_args=None,
        direct_call=True,
        **kwds,
    ):
        #! OVERRIDE the build method from the CPPStandaloneDevice class. !#
        # Build the project by compiling the source files.

        # Turn directory into absolute path
        directory = os.path.abspath(directory)
        super().build(
            directory=directory,
            compile=compile,
            run=run,
            debug=debug,
            clean=clean,
            with_output=with_output,
            additional_source_files=additional_source_files,
            run_args=run_args,
            direct_call=direct_call,
            **kwds,
        )

    def network_run(
        self,
        net,
        duration,
        report=None,
        report_period=...,
        namespace=None,
        profile=None,
        level=0,
        **kwds,
    ):
        #! OVERRIDE the network_run method from the CPPStandaloneDevice class. !#
        # Store the duration of the simulation.
        self.duration = duration
        super().network_run(
            net,
            duration,
            report,
            report_period,
            namespace,
            profile,
            level=level + 1,
            **kwds,
        )

    
        
    def add_publisher(
        self,
        *args,
        **kwargs
    ):
        for arg in args:
            
            if not isinstance(arg, TwistPublisher) and not isinstance(arg, Publisher):
                raise ValueError("The argument should be a TwistPublisher or Publisher object.")
            
            # Check if the header file is provided.
            if arg.header is not None:
                header_path = arg.header
            else:
                header_path = HEADER_FILES[arg.topic_type.split("/")[2]]

            # Check if the name is provided.
            if arg.name is None and isinstance(arg, TwistPublisher):
                name = "twist_" + str(self.twist_number)
                self.twist_number += 1
            elif arg.name is None and isinstance(arg, Publisher):
                name = "publisher_" + str(self.pub_number)
                self.pub_number += 1
            else:
                name = arg.name
                
            # List of the reset values
            reset_value = []

            # List of the input values
            inputs = []

            # Loop through the input values to verify the format.
            for input_name, input_value in arg.input.items():
                try:
                    # Check if the input value is a Brian object.
                    if isinstance(input_value, VariableView):
                        inputs.append(
                            {
                                "name": input_name,
                                "value": "brian::"
                                + self.get_array_name(input_value.variable)
                                + "[0]",
                            }
                        )
                    else:
                        inputs.append({"name": input_name, "value": input_value})
                except Exception as e:
                    error_message = "Wrong output format, accept type : dict with name and value.\n error : {}".format(
                        e
                    )
                    raise RuntimeError(error_message)

                # Check if the reset value is provided.
                reset = {"name": input_name, "value": 0.0}
                for key, value in arg.reset_values.items():
                    if key == input_name:
                        reset = {"name": key, "value": value}
                        break
                reset_value.append(reset)

            # Add the publisher configuration to the list.
            self.templater.env.globals["publishers"].append(
                {
                    "name": name,
                    "topic": arg.topic,
                    "topic_type": "::".join(arg.topic_type.split("/")),
                    "topic_include": header_path,
                    "rate": arg.rate,
                    "input": inputs,
                    "reset_value": reset_value,
                }
            )

        
    def add_subscriber(
        self,
        *args,
        **kwargs
    ):

        for arg in args:
            
            # Check if the header file is provided.
            if arg.header is not None:
                header_path = arg.header
            else:
                header_path = HEADER_FILES[arg.topic_type.split("/")[2]]
            
            
            # Create a function for Brian to be able to have the same time as the ROS time.
            Owner = namedtuple("Owner", ["name"])
            group = Owner(name=arg.name)

            # Create a variable to store the time of the ROS topic.
            var_time = ArrayVariable("time_" + arg.name, size=1, owner=group, device=get_device())

            # # Create a variable to store the time of the ROS topic.
            self.add_array(var_time)
            self.init_with_zeros(var_time, var_time.dtype)
            self.fill_with_array(var_time, np.array([-np.inf]))
            
            try:
                for out_name, out_value in arg.output.items():    
                    # Check if the output value is a integer.
                    if isinstance(out_value, int):
                        out_value = [out_value]
                    elif isinstance(out_value, list):
                        if all(i is None for i in out_value): 
                            out_value = [None]
                    # Check if the output value is empty.
                    elif out_value is None:
                        out_value = [None]

                    # Create a temporary Brian variable to find the name for the function.
                    var_tmp = ArrayVariable(
                        "var_" + out_name,
                        size=len(out_value),
                        owner=group,
                        device=get_device(),
                    )
                    self.add_array(var_tmp)
                    self.init_with_zeros(var_tmp, var_tmp.dtype)

                    arg.outs.append(
                        {
                            "name": out_name,
                            "index": [str(o) for o in out_value],
                            "var": self.get_array_name(var_tmp),
                        }
                    )
            except Exception as e:
                error_message = "Wrong output format, accept type : dict with name and value.\n error : {}".format(
                    e
                )
                raise RuntimeError(error_message)

            # Add the subscriber configuration to the list.
            self.templater.env.globals["subscribers"].append(
                {
                    "name": arg.name,
                    "topic_type": "::".join(arg.topic_type.split("/")),
                    "topic_include": header_path,
                    "var_time": self.get_array_name(var_time),
                    "out": arg.outs,
                }
            )

    def generate_makefile(
        self, writer, compiler, compiler_flags, linker_flags, nb_threads, debug
    ):
        #! OVERRIDE the generate_makefile method from the CPPStandaloneDevice class. !#
        # This function is override to generate a CMakeLists.txt file instead of a Makefile.
        cmakefile = self.templater.CMakeLists(
            None,
            None,
            source_files=" ".join(sorted(writer.source_files)),
            compiler_flags=compiler_flags,
            cyclonedds=prefs.devices.ros_standalone.cyclonedds,
        )
        writer.write("CMakeLists.txt", cmakefile)

        if prefs.devices.ros_standalone.cyclonedds:
            cyclonedds = self.templater.cyclone_profile(
                None,
                None,
                network_interface=prefs.devices.ros_standalone.network_interface,
                list_address_ip=prefs.devices.ros_standalone.list_address_ip,
            )
            writer.write("cyclone_profile.xml", cyclonedds)

        pck_file = self.templater.package(None, None)
        writer.write("package.xml", pck_file)
        
    def detect(self):
        subset = set()

        spike_monitors_nb = 0
        state_monitors_nb = 0
        population_rate_monitors_nb = 0

        for codeobj in self.code_objects.values():
              # Add the SpikeMonitor to pub_monitors
            if isinstance(codeobj.owner, SpikeMonitor):
                spike_monitors_nb += 1
                self.templater.env.globals["pub_monitors"].append(
                    {
                        "name": codeobj.owner.name, 
                        "type": "Float64"
                    }
                )
                

            # Add the StateMonitor to pub_monitors
            if isinstance(codeobj.owner, StateMonitor):
                state_monitors_nb += 1
                for var in codeobj.owner.recorded_variables:
                    self.templater.env.globals["pub_monitors"].append(
                        {
                            "name": codeobj.owner.name + "_" + var,
                            "type": "Float64MultiArray"
                        }
                    )
                

            # Add the PopulationRateMonitor to pub_monitors
            if isinstance(codeobj.owner, PopulationRateMonitor):
                population_rate_monitors_nb += 1
                self.templater.env.globals["pub_monitors"].append(
                    {
                        "name": codeobj.owner.name, 
                        "type": "Float64"
                    }
                )

            # Add the NeuronGroup variable to the modifier
            if isinstance(codeobj.owner, NeuronGroup):
                # We only take variable from stateupdater
                if codeobj.name == codeobj.owner.name + "_stateupdater_codeobject":
                    for var in codeobj.variables.values():
                        # We only take variable that are not read only and in ArrayVariable format
                        if isinstance(var, ArrayVariable) and not var.read_only:                            
                            self.templater.env.globals["variable_info"].append(
                                {
                                    "name": var.name, 
                                    "type": var.dtype_str, 
                                    "dimension": str(var.dim)
                                }
                            )
                            
            for var_name, var in codeobj.variables.items():
                if isinstance(var, Subscriber): 
                    subset.add(var)
        self.add_subscriber(*subset)
        print(f"\033[35m➤ Spikemonitor detected : {spike_monitors_nb}\033[0m")
        print(f"\033[35m➤ Statemonitor detected : {state_monitors_nb}\033[0m")
        print(f"\033[35m➤ PopulationRateMonitor detected : {population_rate_monitors_nb}\033[0m")
     
    def generate_objects_source(
        self,
        writer,
        arange_arrays,
        synapses,
        static_array_specs,
        networks,
        timed_arrays,
    ):
        self.detect()

        arr_tmp = self.code_object_class().templater.objects(
            None,
            None,
            array_specs=self.arrays,
            dynamic_array_specs=self.dynamic_arrays,
            dynamic_array_2d_specs=self.dynamic_arrays_2d,
            zero_arrays=self.zero_arrays,
            arange_arrays=arange_arrays,
            synapses=synapses,
            clocks=self.clocks,
            static_array_specs=static_array_specs,
            networks=networks,
            get_array_filename=self.get_array_filename,
            get_array_name=self.get_array_name,
            profiled_codeobjects=self.profiled_codeobjects,
            code_objects=list(self.code_objects.values()),
            timed_arrays=timed_arrays,
        )
        writer.write("objects.*", arr_tmp)
        
    def generate_main_source(self, writer):
        #! OVERRIDE the generate_main_source method from the CPPStandaloneDevice class. !#
        # This function is override to generate the main.cpp file with the ROS publisher and subscriber configurations.
        # It also generates the brianros.h file and implements the monitoring configurations adapted to ROS.
        super().generate_main_source(writer)
        # Generate the brianros.h file with the ROS publisher and subscriber configurations.
        brianros_tmp = self.templater.brianros(
            None,
            None,
            code_lines=self.code_lines,
            code_objects=list(self.code_objects.values()),
            report_func=self.report_func,
            dt=float(self.defaultclock.dt),
        )

        writer.write("brianros.h", brianros_tmp)

        # Generate the main.bash file.
        main_bash = self.templater.main_sh(
            None, None, 
            cyclonedds=prefs.devices.ros_standalone.cyclonedds, 
            interface=prefs.devices.ros_standalone.interface,
            path=self.file_path.split("/src")[0]
        )
        writer.write("main.bash", main_bash)
        
         
    def compile_source(self, directory, compiler, debug, clean):
        #! OVERRIDE the compile_source method from the CPPStandaloneDevice class. !#
        # This function is override to compile the project with the ROS libraries
        
        
        with in_directory(directory):
            with std_silent(debug):
                os.makedirs(os.path.join(directory, "json"), exist_ok=True)
                with open(
                    os.path.join(directory, "json/b_control.txt"), "w"
                ) as f:
                    data_for_rqt = {
                        "publisher": self.templater.env.globals["publishers"],
                        "subscriber": self.templater.env.globals["subscribers"],
                        "duration": float(self.duration),
                        "pub_monitors": self.templater.env.globals["pub_monitors"],
                        "variable_info": self.templater.env.globals["variable_info"],
                    }

                    json.dump(data_for_rqt, f)
                
                # Send variable information to the RQT interface for modification at each simulation
                with open(
                    os.path.join(directory, "json/mod_variable.txt"), "w"
                ) as f:
                    data_for_rqt = {
                        "variable_info": self.templater.env.globals["variable_info"],
                    }

                    json.dump(data_for_rqt, f)
                xc = os.system(
                    'cd ' + self.file_path + '/../../ && MAKEFLAGS="-j1 -l1" colcon build --executor sequential --packages-up-to brian_project'
                )

                if xc != 0:
                    error_message = (
                        "Projects compilation failed (error " "code: %u)."
                    ) % xc

                    raise RuntimeError(error_message)

    def run(self, directory, results_directory, with_output, run_args):
        #! OVERRIDE the run method from the CPPStandaloneDevice class. !#
        # This function is override to run the project with the ROS libraries.
        # It launch main.bash file which contains the ROS launch command.
        # It also generates the json file to be used in the RQT interface.
        # The json file contains the publisher, subscriber, duration and monitor configurations.
        with in_directory(directory):
            with std_silent(with_output):
                xm = os.system(
                    'bash -c "source ' + self.file_path + '/../../install/local_setup.bash; ros2 run brian_project main.bash"'
                )

                #
                if xm != 0:
                    error_message = ("Projects run failed (error " "code: %u).") % xm
                    raise RuntimeError(error_message)

                self.project_dir = (
                    self.file_path + "/../../install/brian_project/lib/brian_project"
                )
                self.has_been_run = True


ros_standalone_device = ROSStandaloneDevice()
all_devices["ros_standalone"] = ros_standalone_device

    