# How the interface works 

Brian2ROS features an interactive interface designed to facilitate both control and information exchange between the spiking neural network and the robotic system during simulation.
It provides plots for publishers and subscribers, visualizations of user-selected monitors, and tools for variable modification.



```{image} ../_static/user_guide/interface/interface_1.png
```

## Details 

### Plot ROS
In this section, you can monitor the evolution of any ROS variable of your choice in real time. To do so, simply enter the full path to the desired ROS topic. For example: "/cmd_vel/linear/x".

The interface supports any valid ROS topic path, and updates are displayed dynamically as the simulation runs. This makes it especially useful for monitoring sensor data, command signals, or internal control variables.

Make sure the topic you want to observe is currently being published in your ROS environment. If the variable is not updating, check that the corresponding node is running and actively publishing data.
```{image} ../_static/user_guide/interface/interface_4.png
```

### Plot Monitor
To facilitate the observation of the Brian simulation results, a SpikeMonitor, StateMonitor, or RateMonitor is included in the Python code. A dedicated tab is added to the graphical window, allowing direct visualization of the results without any user intervention.
```{image} ../_static/user_guide/interface/interface_5.png
```

### Button
```{image} ../_static/user_guide/interface/interface_3.png
```

**"Start" Button:**  
Launches the simulation (`Brian2ROS`) as well as Gazebo, if selected by the user.

**"End" Button:**  
Completely stops the simulation and closes the interface along with Gazebo (if it was started).

**Dialog Box:**  
Allows the user to choose the environment (terrain) that Gazebo will use for the simulation.  
After clicking the "Start" button, the environment can be changed using the "Restart" button.  
Users can also add custom terrains.

**Clock:**  
Displays the simulation time. Gazebo time may differ from real time, depending on the computing power of the machine being used.

**"Restart Brian" Button:**  
Restarts the Brian simulation.

**"Restart" Button:**  
Restarts both the Brian simulation and the Gazebo environment.  
This button is only available if Gazebo was selected by the user.

### Variable modificator

A dedicated section is available to easily modify certain variables before starting a new simulation. You can adjust parameters that influence the system’s behavior which allows you to test different scenarios without having to edit the code. Simply change the values in this section and restart the simulation to see how the system responds.

It’s a quick and easy way to experiment with different configurations and better understand the impact of each variable.
```{image} ../_static/user_guide/interface/interface_2.png
```