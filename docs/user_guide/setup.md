# How to use ROS in Brian

## Setup

To enable ROS integration in your Brian simulation, you need to configure Brian to use **Brian2ROS**. At the beginning of your script, include the following line:

```python
set_device("ros_standalone", directory="src/src/brian_project", debug=True)
```

**Parameters:**
- `directory`: Path where the Brian2ROS files will be installed.
- `debug`: *(Optional)* Enables debug messages. Defaults to `False`.

Additional options are available to customize how Brian2ROS behaves:

### Interface

By default, an interface window is displayed when launching a simulation. If you want to **disable the interface** and start the simulation directly (useful for embedded or headless execution), add the following line:

```python
prefs.devices.ros_standalone.interface = False
```

This is particularly helpful when running code on embedded systems or in automated pipelines.

### CycloneDDS Integration

Brian2ROS supports integration with **CycloneDDS** for communication with a physical robot. To enable this, add the following lines:

```python
prefs.devices.ros_standalone.cyclonedds = True
prefs.devices.ros_standalone.network_interface = 'wlxf07959eb0bcf'
prefs.devices.ros_standalone.list_address_ip = ['10.42.0.1', '10.42.0.103']
```

> ⚠️ Make sure your robot is configured with the **same IP setup and network interface**.  
> This setup is especially useful when running the simulation inside a **Docker** container.



## Publisher 

In ROS 2, a **publisher** is a node component responsible for sending messages to a specific topic. It periodically generates and broadcasts data (e.g., sensor values, simulation outputs, control commands) that can be received by other nodes subscribing to that topic.

In the context of **Brian2ROS**, setting up a publisher is straightforward. You can define a publisher by simply creating an instance of the {class}`.Publisher` class.

### Example: Basic Publisher

```python
motor_commands = Publisher(
    name="wheel",
    topic="cmd_vel",
    topic_type="geometry_msgs/msg/Twist",
    input={"linear.x": 0.26, "angular.z": 1.82},
    rate=500,
    reset_values={"linear.x": 0.0, "angular.z": 0.0},
    header=header
)
```

**Parameters:**
- `name`: Internal name of the publisher.
- `topic`: ROS topic to publish to.
- `topic_type`: Full ROS message type.
- `input`: Dictionary mapping message fields to fixed values or Brian variables.
- `rate`: Publication frequency in Hz.
- `reset_values`: Values to reset the message to, if needed.
- `header`: Optional header information.


### Example: Using a TwistPublisher

If you're publishing velocity commands using the `geometry_msgs/msg/Twist` message type, you can use the simplified {class}`.TwistPublisher` class:

```python
motor_commands = TwistPublisher(
    rate=200,
    input={"linear.x": neuron.velocity, "angular.z": neuron.angular},
)
```

Finally, register the publisher with the ROS device:

```python
get_device().add_publisher(motor_commands)
```

This will start publishing data to the ROS network according to the specified rate and input values.


## Subscriber

In ROS 2, a **subscriber** is a node component that listens to a specific topic and receives messages published by other nodes. When a message is received, a **callback function** is triggered to process the data — for example, to display it, control a simulation, or log the information.

As with publishers, creating a {class}`.Subscriber` with **Brian2ROS** is simple and intuitive. Just add the following line to your code:

### Example: Basic Subscriber

```python
range_sensor = Subscriber(
    name="range",
    topic="LaserScan",
    topic_type="sensor_msgs/msg/LaserScan",
    output={"ranges": [0]},
    header=header
)
```

**Parameters:**
- `name`: Internal name of the subscriber.
- `topic`: The name of the ROS topic to subscribe to.
- `topic_type`: Full ROS message type.
- `output`: Dictionary mapping fields from the ROS message to Brian variables.
- `header`: Optional header information


### Example: LaserScanSubscriber

When subscribing specifically to a LaserScan message, you can use the simplified {class}`.LaserScanSubscriber` class:

```python
range_sensor = LaserScanSubscriber(
    name="range",
    output={"ranges": 0}
)
```

This allows your Brian simulation to receive real-time sensor data and use it directly within your model.


