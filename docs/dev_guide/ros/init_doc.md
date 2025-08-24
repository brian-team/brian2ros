# `__init__.py`

When the simulation starts, the `__init__.py` file is executed.  
Its purpose is to **initialize the environment** and **create all the objects required** to start the simulation.  

The first step is to create the core {class}`Subscriber` and {class}`Publisher` objects,  
which handle communication between Brian2 and ROS.

---

## Publisher

```{class} Publisher
A container for all publisher-related information.  
It does not perform publishing itself but stores metadata such as topic, headers, and format.  
The data is later used by {class}`ROSStandaloneDevice`.

When {meth}`ROSStandaloneDevice.add_publisher` is called, new publishers are created and registered.
```

```{method} Publisher.add_publisher()
Validate and register a publisher.

Steps:

1. **Check type**  
   Ensure the object is an instance of {class}`Publisher`.

2. **Validate header**  
   If no header is given, search in {data}`HEADER_FILES`.

3. **Assign a name**  
   If no name is provided, generate a default name.

4. **Process input information**  
   - Validate the input format  
   - Add metadata to the internal dictionary (`templater`):  
     - `name`: identifier of the input variable  
     - `value`: Brian2 internal variable name  

5. **Check reset**  
   If a reset condition is provided, validate it.

6. **Append to templater**  
   Store validated information for code generation.
```

---

## Subscriber

```{class} Subscriber
Handles all subscriber-related information and generates C++ bridge code for Brian2 ↔ ROS communication.
```

```{method} Subscriber.__init__()
Stores subscriber data and automatically calls {meth}`Subscriber.create_code`.
```

```{method} Subscriber.create_code()
Generate a C++ script that:

- Synchronizes **Brian2 simulation time** with **ROS time**.  
- Adds the subscriber’s output variables into Brian2 simulation code.
```

```{method} Subscriber.add_subscriber()
Validate and register a subscriber.

Steps:

1. **Validate header**  
   If no header is provided, search {data}`HEADER_FILES`.

2. **Time variable**  
   Create an internal variable to store ROS topic timestamps.

3. **Process output information**  
   - Detect the type of `output_value`.  
   - Map this type to a Brian2-compatible variable.  
   - Retrieve the Brian2 variable name.  
   - Update the `outs` attribute.

4. **Append to templater**  
   Store processed subscriber data for code generation.
```

---

## HEADER_FILES

```{data} HEADER_FILES
A dictionary mapping **ROS message types** to their corresponding `.hpp` headers.

- Purpose: avoid manually searching for header files when known.  
- If missing, the header must be provided manually.
```

---

## Preferences

Brian2ROS provides preferences similar to Brian2’s `cpp_standalone`.  
These can be set at the beginning of the Python script:

- **`cyclonedds`** *(bool)*  
  Use CycloneDDS for communication instead of FastDDS (default in ROS 2 Jazzy).  

- **`network_interface`** *(str)*  
  Specify the network interface (TODO: clarify).  

- **`list_address_ip`** *(list[str])*  
  List of IP addresses for communication (mainly used with CycloneDDS).  

- **`interface`** *(bool)*  
  Enable/disable the GUI interface.  
  If disabled, Gazebo and interface packages are not loaded.  

- **`buffer_multiplier`** *(int, default=10)*  
  Multiplier for the circular buffer size.  
  Effective size = `base_buffer × buffer_multiplier`.

---

## Specialized Classes

```{class} LaserScanSubscriber
Subclass of {class}`Subscriber` configured for TurtleBot LaserScan messages (topic: `scan`).
```

```{class} TwistPublisher
Subclass of {class}`Publisher` configured for TurtleBot Twist messages (topic: `cmd_vel`).
```

These are convenience classes to reduce boilerplate.

---

## ROSStandaloneDevice

```{class} ROSStandaloneDevice
Extends {class}`CPPStandalone` to bridge Brian2 with ROS.  
Handles publishers, subscribers, and ROS-specific build configuration.
```

```{method} ROSStandaloneDevice.__init__()
Initialization steps:

- Extend `CPPStandalone.headers` with:  
  - `brianros.h`  
  - `float64.hpp` and `float_state_monitor.hpp`
- Define environment variables for publishers, subscribers, monitors, and variable modifiers.  
  These are later exported to `.json` files.
```

```{method} ROSStandaloneDevice.build()
Override from `CPPStandalone`. Handles Brian2ROS-specific build steps.
```

```{method} ROSStandaloneDevice.network_run()
Override to retrieve simulation time.
```

```{method} ROSStandaloneDevice.generate_makefile()
Generate a `CMakeLists.txt` instead of a Makefile.  
If `cyclonedds` is enabled, also create `cyclone_profile.xml`.
```

```{method} ROSStandaloneDevice.publish_monitors()
Register selected monitors in `pub_monitor`.  
Print the number of monitors initialized.
```

```{method} ROSStandaloneDevice.detect()
- Detect modifiable variables and store them in `variable_info`.  
- Collect all subscribers and call {meth}`Subscriber.add_subscriber`.
```

```{method} ROSStandaloneDevice.generate_objects_source()
Call {meth}`ROSStandaloneDevice.detect` before generating Brian2 code objects.
```

```{method} ROSStandaloneDevice.generate_main_source()
Generate `brianros.h` and `main.bash`.
```

```{method} ROSStandaloneDevice.compile_source()
- Populate JSON files:  
  - `b_control.txt` (publishers, subscribers, duration, monitors, variables).  
- Add `msg/` to the project.  
- Run `colcon build`.  
  - If `interface=False`, skip `turtlebot3_gz` and `brian_interface`.
```

```{method} ROSStandaloneDevice.run()
Launch the generated `main.bash` file.
```

---

## Data for the Interface

The following environment variables are exported for the GUI interface:

- **publisher**  
  Filled by {meth}`Publisher.add_publisher`.  
  Contains: name, topic, type, header path, refresh rate, input values, reset policy.

- **subscriber**  
  Filled by {meth}`Subscriber.add_subscriber`.  
  Contains: name, topic, type, header path, time variable, output values.

- **duration**  
  Total simulation duration.

- **pub_monitors**  
  Filled by {meth}`ROSStandaloneDevice.publish_monitors`.  
  Contains monitor name and type:  
  - `StateMonitor` → FloatStateMonitor  
  - `SpikeMonitor` → Float64  
  - `PopulationRateMonitor` → Float64  

- **variable_info**  
  Used for variable modifiers.  
  Stores name, type, and dimensions of Brian2 variables that:  
  - belong to `stateupdater`  
  - are `ArrayVariable`  
  - are not `read_only`
