from collections import namedtuple, Counter

# import some common tool
import numpy as np
import json
import tempfile
from distutils import ccompiler
# import some brian2 tools
from brian2.devices.device import (
    all_devices,
    get_device,
)
from brian2.devices.cpp_standalone import device
import os
from brian2.utils.logger import std_silent, get_logger
from brian2.utils.filetools import in_directory, ensure_directory
from brian2.core.preferences import prefs, BrianPreference
from brian2 import Function, second
from brian2.core.variables import ArrayVariable, VariableView
from brian2 import SpikeMonitor, StateMonitor, PopulationRateMonitor, NeuronGroup
from brian2.codegen.cpp_prefs import get_compiler_and_args

logger = get_logger(__name__)

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
    "UInt8MultiArray": "std_msgs/msg/u_int8_multi_array.hpp",
    "UInt16MultiArray": "std_msgs/msg/u_int16_multi_array.hpp",
    "UInt32MultiArray": "std_msgs/msg/u_int32_multi_array.hpp",
    "UInt64MultiArray": "std_msgs/msg/u_int64_multi_array.hpp",
    "Float32MultiArray": "std_msgs/msg/float32_multi_array.hpp",
    "Float64MultiArray": "std_msgs/msg/float64_multi_array.hpp",
    "Int8MultiArray": "std_msgs/msg/int8_multi_array.hpp",
    "Int16MultiArray": "std_msgs/msg/int16_multi_array.hpp",
    "Int32MultiArray": "std_msgs/msg/int32_multi_array.hpp",
    "Int64MultiArray": "std_msgs/msg/int64_multi_array.hpp",
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
    buffer_multiplier=BrianPreference(
        default=10,
        docs="""
        The multiplier for the circular buffer size. 
        circular_buffer_size = buffer_multiplier * buffer_size.
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
        super().__init__(self._pose_obj, arg_units=(second, 1), return_unit=1)

        self.name = name
        self.topic = topic
        self.topic_type = topic_type
        self.output = output
        self.header = header
        self.outs = []
        self.var_time_name = "_array_" + self.name + "_time_" + self.name
        
        self.create_code()

    def _pose_obj(self, t, var_index):
        pass
    
    def create_code(self):
        
        # Create the function to add to Brian's code.
        code = (
            """double """
            + self.name
            + """(double t,int var_index){""")
        
        
        for i, (out_name, out_value) in enumerate(self.output.items()):
        
            code += """
            if (var_index == """ + str(i) + """) {
            static int tail_""" + out_name + """ = 0;
            static int previous_frame_id_""" + out_name + """ = 0;

            tail_""" + out_name + """ = (static_cast<int>(std::round(t / brian::_array_defaultclock_dt[0]))) % """ + str(len(out_value) * prefs.devices.ros_standalone.buffer_multiplier) + """;
            int nb_""" + out_name + """_while = 0;
            //std::cout << "tail_""" + out_name + """ : " << tail_""" + out_name + """ << std::endl;
            while(brian::_array_""" + self.name + """_frame_id_""" + out_name + """[tail_""" + out_name + """] < previous_frame_id_""" + out_name + """ || brian::_array_""" + self.name + """_frame_id_""" + out_name + """[tail_""" + out_name + """] == 0){ 
                std::this_thread::sleep_for(std::chrono::duration<double>(brian::_array_defaultclock_dt[0]));
                if(Network::_globally_stopped){
                    return 0;
                    }
            }

            previous_frame_id_""" + out_name + """ = brian::_array_""" + self.name + """_frame_id_""" + out_name + """[tail_""" + out_name + """];
            return brian::_array_""" + self.name + """_var_""" + out_name + """[tail_""" + out_name + """];
            }
            """

        code += """}"""
        
        self.implementations.add_implementation(
            "cpp", code, compiler_kwds={"headers": ["<thread>", "<chrono>"]}
        )
 
class LaserScanSubscriber(Subscriber):
    
    def __init__(self, name, output, header=None):
        super().__init__(name=name, topic="LaserScan", topic_type="sensor_msgs/msg/LaserScan", output=output, header=header)
    
class ROSStandaloneDevice(device.CPPStandaloneDevice):
    def __init__(self):
        super().__init__()

        self.headers += ['"brianros.h"', '"std_msgs/msg/float64.hpp"', '"brian_project/msg/float_state_monitor.hpp"']

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
        results_directory="results",
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
       
        """
        Build the project

        TODO: more details

        Parameters
        ----------
        directory : str, optional
            The output directory to write the project to, any existing files
            will be overwritten. If the given directory name is ``None``, then
            a temporary directory will be used (used in the test suite to avoid
            problems when running several tests in parallel). Defaults to
            ``'output'``.
        compile : bool, optional
            Whether or not to attempt to compile the project. Defaults to
            ``True``.
        run : bool, optional
            Whether or not to attempt to run the built project if it
            successfully builds. Defaults to ``True``.
        debug : bool, optional
            Whether to compile in debug mode. Defaults to ``False``.
        with_output : bool, optional
            Whether or not to show the ``stdout`` of the built program when run.
            Output will be shown in case of compilation or runtime error.
            Defaults to ``True``.
        clean : bool, optional
            Whether or not to clean the project before building. Defaults to
            ``False``.
        additional_source_files : list of str, optional
            A list of additional ``.cpp`` files to include in the build.
        direct_call : bool, optional
            Whether this function was called directly. Is used internally to
            distinguish an automatic build due to the ``build_on_run`` option
            from a manual ``device.build`` call.
        """
        if self.build_on_run and direct_call:
            raise RuntimeError(
                "You used set_device with build_on_run=True "
                "(the default option), which will automatically "
                "build the simulation at the first encountered "
                "run call - do not call device.build manually "
                "in this case. If you want to call it manually, "
                "e.g. because you have multiple run calls, use "
                "set_device with build_on_run=False."
            )
        if self.has_been_run:
            raise RuntimeError(
                "The network has already been built and run "
                "before. To build several simulations in "
                'the same script, call "device.reinit()" '
                'and "device.activate()". Note that you '
                "will have to set build options (e.g. the "
                "directory) and defaultclock.dt again."
            )
        renames = {
            "project_dir": "directory",
            "compile_project": "compile",
            "run_project": "run",
        }
        if len(kwds):
            msg = ""
            for kwd in kwds:
                if kwd in renames:
                    msg += (
                        f"Keyword argument '{kwd}' has been renamed to "
                        f"'{renames[kwd]}'. "
                    )
                else:
                    msg += f"Unknown keyword argument '{kwd}'. "
            raise TypeError(msg)

        if additional_source_files is None:
            additional_source_files = []
        if run_args is None:
            run_args = []
        if directory is None:
            directory = tempfile.mkdtemp(prefix="brian_standalone_")
        self.project_dir = directory
        ensure_directory(directory)
        if os.path.isabs(results_directory):
            raise TypeError(
                "The 'results_directory' argument needs to be a relative path but was "
                f"'{results_directory}'."
            )
        # Translate path to absolute path which ends with /
        self.results_dir = os.path.join(
            os.path.abspath(os.path.join(directory, results_directory)), ""
        )

        # Determine compiler flags and directories
        compiler, default_extra_compile_args = get_compiler_and_args()
        extra_compile_args = self.extra_compile_args + default_extra_compile_args
        extra_link_args = self.extra_link_args + prefs["codegen.cpp.extra_link_args"]

        codeobj_define_macros = [
            macro
            for codeobj in self.code_objects.values()
            for macro in codeobj.compiler_kwds.get("define_macros", [])
        ]
        define_macros = (
            self.define_macros
            + prefs["codegen.cpp.define_macros"]
            + codeobj_define_macros
        )

        codeobj_include_dirs = [
            include_dir
            for codeobj in self.code_objects.values()
            for include_dir in codeobj.compiler_kwds.get("include_dirs", [])
        ]
        include_dirs = (
            self.include_dirs + prefs["codegen.cpp.include_dirs"] + codeobj_include_dirs
        )

        codeobj_library_dirs = [
            library_dir
            for codeobj in self.code_objects.values()
            for library_dir in codeobj.compiler_kwds.get("library_dirs", [])
        ]
        library_dirs = (
            self.library_dirs + prefs["codegen.cpp.library_dirs"] + codeobj_library_dirs
        )

        codeobj_runtime_dirs = [
            runtime_dir
            for codeobj in self.code_objects.values()
            for runtime_dir in codeobj.compiler_kwds.get("runtime_library_dirs", [])
        ]
        runtime_library_dirs = (
            self.runtime_library_dirs
            + prefs["codegen.cpp.runtime_library_dirs"]
            + codeobj_runtime_dirs
        )

        codeobj_libraries = [
            library
            for codeobj in self.code_objects.values()
            for library in codeobj.compiler_kwds.get("libraries", [])
        ]
        libraries = self.libraries + prefs["codegen.cpp.libraries"] + codeobj_libraries

        compiler_obj = ccompiler.new_compiler(compiler=compiler)

        # Distutils does not use the shell, so it does not need to quote filenames/paths
        # Since we include the compiler flags in the makefile, we need to quote them
        include_dirs = [f'"{include_dir}"' for include_dir in include_dirs]
        library_dirs = [f'"{library_dir}"' for library_dir in library_dirs]
        runtime_library_dirs = [
            f'"{runtime_dir}"' for runtime_dir in runtime_library_dirs
        ]
        
        # define_macros = [
        #         f"{k}={v}" if isinstance(item, tuple) else item
        #         for item in define_macros
        #         for k, v in [item] if isinstance(item, tuple)
        #     ] + [
        #         item for item in define_macros if isinstance(item, str)
        #     ]
        

        compiler_flags = (
            ccompiler.gen_preprocess_options(define_macros, include_dirs)
            + extra_compile_args
        )
        for item in compiler_flags:
            if item == '-std=c++11':
                compiler_flags.remove(item)

        linker_flags = (
            ccompiler.gen_lib_options(
                compiler_obj,
                library_dirs=library_dirs,
                runtime_library_dirs=runtime_library_dirs,
                libraries=libraries,
            )
            + extra_link_args
        )

        codeobj_source_files = [
            source_file
            for codeobj in self.code_objects.values()
            for source_file in codeobj.compiler_kwds.get("sources", [])
        ]
        additional_source_files += codeobj_source_files

        for d in ["code_objects", "results", "static_arrays"]:
            ensure_directory(os.path.join(directory, d))

        self.writer = device.CPPWriter(directory)

        # Get the number of threads if specified in an openmp context
        nb_threads = prefs.devices.cpp_standalone.openmp_threads
        # If the number is negative, we need to throw an error
        if nb_threads < 0:
            raise ValueError("The number of OpenMP threads can not be negative !")

        logger.diagnostic(
            "Writing C++ standalone project to directory "
            f"'{os.path.normpath(directory)}'."
        )

        self.check_openmp_compatible(nb_threads)

        self.write_static_arrays(directory)

        # Check that all names are globally unique
        names = [obj.name for net in self.networks for obj in net.sorted_objects]
        non_unique_names = [name for name, count in Counter(names).items() if count > 1]
        if len(non_unique_names):
            formatted_names = ", ".join(f"'{name}'" for name in non_unique_names)
            raise ValueError(
                "All objects need to have unique names in "
                "standalone mode, the following name(s) were used "
                f"more than once: {formatted_names}"
            )

        self.generate_objects_source(
            self.writer,
            self.arange_arrays,
            self.synapses,
            self.static_array_specs,
            self.networks,
            self.timed_arrays,
        )
        self.generate_main_source(self.writer)
        self.generate_codeobj_source(self.writer)
        self.generate_network_source(self.writer, compiler)
        self.generate_synapses_classes_source(self.writer)
        self.generate_run_source(self.writer)
        self.copy_source_files(self.writer, directory)

        self.writer.source_files.update(additional_source_files)

        self.generate_makefile(
            self.writer,
            compiler,
            compiler_flags=" ".join(compiler_flags),
            linker_flags=" ".join(linker_flags),
            include_dirs=" ".join(include_dirs),
            nb_threads=nb_threads,
            debug=debug,
        )

        if compile:
            self.compile_source(directory, compiler, debug, clean)
            if run:
                self.run(directory, results_directory, with_output, run_args)
        time_measurements = {
            "'make clean'": self.timers["compile"]["clean"],
            "'make'": self.timers["compile"]["make"],
            "running 'main'": self.timers["run_binary"],
        }
        logged_times = [
            f"{task}: {measurement:.2f}s"
            for task, measurement in time_measurements.items()
            if measurement is not None
        ]
        logger.debug(f"Time measurements: {', '.join(logged_times)}")


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
            var_time = ArrayVariable(
                "time_" + arg.name, 
                size=1, 
                owner=group, 
                device=get_device()
            )

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
                        size=len(out_value) * prefs.devices.ros_standalone.buffer_multiplier,
                        owner=group,
                        device=get_device(),
                    )
                    self.add_array(var_tmp)
                    self.init_with_zeros(var_tmp, var_tmp.dtype)

                    # Create a Brian variable buffer to find the name for the function.
                    var_frame_id = ArrayVariable(
                        "frame_id_" + out_name,
                        size=len(out_value) * prefs.devices.ros_standalone.buffer_multiplier,
                        owner=group,
                        device=get_device(),
                    )
                    self.add_array(var_frame_id)
                    self.init_with_zeros(var_frame_id, var_frame_id.dtype)
                    arg.outs.append(
                        {
                            "name": out_name,
                            "index": [str(o) for o in out_value],
                            "var": self.get_array_name(var_tmp),
                            "buffer_size": len(out_value) * prefs.devices.ros_standalone.buffer_multiplier,
                            "frame_id": self.get_array_name(var_frame_id),
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
                    "topic": arg.topic,
                    "topic_type": "::".join(arg.topic_type.split("/")),
                    "topic_include": header_path,
                    "var_time": self.get_array_name(var_time),
                    "out": arg.outs,
                }
            )

    def generate_makefile(
        self, writer, compiler, compiler_flags, linker_flags, include_dirs, nb_threads, debug
    ):
        #! OVERRIDE the generate_makefile method from the CPPStandaloneDevice class. !#
        # This function is override to generate a CMakeLists.txt file instead of a Makefile

        cmakefile = self.templater.CMakeLists(
            None,
            None,
            source_files=" ".join(sorted(writer.source_files)),
            compiler_flags=compiler_flags,
            linker_flags=linker_flags,
            include_dirs=include_dirs,
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
    
    def publish_monitors(self, monitors):

        spike_monitors_nb = 0
        state_monitors_nb = 0
        population_rate_monitors_nb = 0

        for monitor in monitors:
            if isinstance(monitor, SpikeMonitor):
                spike_monitors_nb += 1
                self.templater.env.globals["pub_monitors"].append(
                    {
                        "name": monitor.name, 
                        "type": "Float64",
                    }
                )
            elif isinstance(monitor, StateMonitor):
                state_monitors_nb += 1
                for var in monitor.recorded_variables:
                    self.templater.env.globals["pub_monitors"].append(
                        {
                            "name": monitor.name + "_" + var,
                            "type": "FloatStateMonitor",
                        }
                    )
            elif isinstance(monitor, PopulationRateMonitor):
                population_rate_monitors_nb += 1
                self.templater.env.globals["pub_monitors"].append(
                    {
                        "name": monitor.name, 
                        "type": "Float64",
                    }
                )
            else:
                raise RuntimeError(
                    f"Unknown monitor type: {type(monitor)}"
                )
        print(f"\033[35m➤ Spikemonitor detected : {spike_monitors_nb}\033[0m")
        print(f"\033[35m➤ Statemonitor detected : {state_monitors_nb}\033[0m")
        print(f"\033[35m➤ PopulationRateMonitor detected : {population_rate_monitors_nb}\033[0m")

    def detect(self):

        subset = set()
        for codeobj in self.code_objects.values():

            #---------------------------------#
            #! Part of the variable modifier !#
            #---------------------------------#
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
            #-----------------------------------#
            #! Part of the Subscriber creation !#
            #-----------------------------------#  
            for var_name, var in codeobj.variables.items():
                if isinstance(var, Subscriber): 
                    subset.add(var)
        self.add_subscriber(*subset)

     
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
            interface=prefs.devices.ros_standalone.interface,
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

                # Add the msg directory to the brian_project package
                xmsg = os.system(
                    'cd ' + self.file_path + '/../../ && mkdir -p src/brian_project/msg && cp -r ' + self.file_path + '/templates/msg/* src/brian_project/msg'
                )
                if xmsg != 0:
                    error_message = (
                        "Error in construction of msg directory (error " "code: %u)."
                    ) % xmsg

                    raise RuntimeError(error_message)
                
                xdebug = os.system(
                    'cd ' + self.file_path + '/../../ && mkdir -p src/brian_project/debug'
                )
                if xdebug != 0:
                    error_message = (
                        "Error in construction of debug directory (error " "code: %u)."
                    ) % xdebug

                    raise RuntimeError(error_message)
                
                nb_cpu = os.cpu_count() or 1
                print(f"\033[35m➤ Number of CPU detected : {str(nb_cpu)}\033[0m")

                load_limit = max(1, int(nb_cpu * 0.8))
                
                xc = os.system(
                    'cd ' + self.file_path + '/../../ && MAKEFLAGS="-j' + str(nb_cpu) 
                    + ' -l' + str(load_limit)
                    + '" colcon build --packages-up-to brian_project ' #--event-handlers console_direct+ ' 
                    + '--cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache' 
                    + ' --packages-skip my_audio_listener --packages-ignore my_audio_listener' # DOIT ETRE ENLEVER LORSQUE LE PACKAGE SERA SUPPRIMER
                    + (' --packages-skip turtlebot3_gz brian_interface --packages-ignore turtlebot3_gz brian_interface' if not prefs.devices.ros_standalone.interface else '')
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

    