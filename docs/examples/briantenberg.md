# Briantenberg

This is an implementation of Braitenberg robot described in the following book :

Valentino Braitenberg, Vehicles: Experiments in Synthetic Psychology, 1st MIT Press
pbk. ed., 1986.

## Overview

A Braitenberg vehicle is a simple reactive agent that demonstrates how basic sensor-to-motor connections can lead to seemingly intelligent behavior.

In its basic form, the vehicle consists of:

- Sensors (e.g., light, distance)

- Motors (e.g., for wheels or movement)

- Direct connections between sensors and motors

The behavior of the vehicle depends on how the sensors are connected to the motors. For example:

- A sensor on the left connected to the right motor will cause the vehicle to turn toward the stimulus (e.g., light), simulating attraction.

- Reversing the connection or inverting the signal can simulate avoidance.

## Code 

```python
import os
import matplotlib.pyplot as plt
from brian2 import *
from brian2ros import *

set_device("ros_standalone", directory="src/src/brian_project", debug=True)

f = 2 * Hz
tau = 20 * ms
A = 150 / second
sig = 0.5
f = 2 * Hz
N = 90
taum = 30 * ms

list_angle = np.concatenate(
    (
        np.linspace(360 - (N // 2), 359, N // 2, dtype=int),
        np.linspace(0, N // 2, N // 2, dtype=int),
    )
)

sub = LaserScanSubscriber(
    name='sub',
    output={"ranges": list_angle}
)

eq_s = """
dv/dt = -(v/tau) + (40/x_o)*Hz : 1
x_o = sub(t, i, 0) : 1
"""

sensor = NeuronGroup(
    N, eq_s, threshold="v>1", reset="v=0", method="euler", name="sensor"
)

eq_c = """
dv/dt = -v/tau : 1
motor = x*v : 1 (constant over dt)
x = (-1)**i: 1
"""

control = NeuronGroup(2, eq_c, method="exact", name="control")

S = Synapses(sensor, control, "w : 1", on_pre="v_post += w", name="S_1")
S.connect(j="i//45")
S.w = 0.2

motor_commands = TwistPublisher(
    rate=200,
    input={"linear.x": 0.26, "angular.z": control.motor},
    reset_values={"linear.x": 0.0, "angular.z": 0.0}
)
get_device().add_publisher(motor_commands)  

S_sensor = SpikeMonitor(sensor)

M = StateMonitor(sensor, variables=True, record=True)

P = PopulationRateMonitor(sensor)
run(100000 * ms)
```