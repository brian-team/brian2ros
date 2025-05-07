from brian2 import *
from brian2ros import *
import numpy as np
import matplotlib.pyplot as plt

set_device("ros_standalone", directory="src/src/brian_project", debug=True)
defaultclock.dt = (1 / 48_000) * second

#prefs.devices.ros_standalone.interface = False

audio = Subscriber(
    name="audio",
    topic="audio_data",
    topic_type="turtleaudio/msg/StereoAudioBlock",
    output={"left": np.int64(np.linspace(0,1023,1024)), "right": np.int64(np.linspace(0,1023,1024))},
    header="turtleaudio/msg/stereo_audio_block.hpp",
)

eq = Equations('''
x = audio(t,i) : 1
''')

group = NeuronGroup(2,eq,name="ears",method="euler",threshold="x>10",reset="")

state_x = StateMonitor(group, "x", record=True)


run(10000 * second, report="text", report_period=0.1 * second)


