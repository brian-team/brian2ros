from brian2 import *
from brian2ros import *
import numpy as np
import matplotlib.pyplot as plt

set_device("ros_standalone", directory="src/src/brian_project", debug=True)
defaultclock.dt = (1 / 48_000) * second

#prefs.devices.ros_standalone.interface = False
prefs.devices.ros_standalone.buffer_multiplier = 10
audio = Subscriber(
    name="audio",
    topic="audio_sin",
    topic_type="turtleaudio/msg/StereoAudioBlock",
    output={"left": np.int16(np.linspace(0,1023,1024)), "right": np.int16(np.linspace(0,1023,1024))},
    header="turtleaudio/msg/stereo_audio_block.hpp",
)

eq = Equations('''
x = audio(t,i) : 1 (constant over dt)
''')

group = NeuronGroup(2,eq,name="ears",method="euler",threshold="x>10",reset="",dt=defaultclock.dt)

state_x = StateMonitor(group, "x", record=True)


run(10 * second, report="text", report_period=1 * second, profile=True)


