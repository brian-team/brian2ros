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
    output={"left": np.int64(np.linspace(0,255,256)), "right": np.int64(np.linspace(0,255,256))},
    header="turtleaudio/msg/stereo_audio_block.hpp",
)

eq = Equations('''
sample : integer (shared)
x = audio(t,0,sample) * i + audio(t,1,sample) * (1 - i) : 1
''')

group = NeuronGroup(2,eq,name="ears",method="euler",threshold="x>10",reset="")
group.run_regularly('sample = (sample + 1) % 256', dt=(1 / 48_000) * second)

state_x = StateMonitor(group, "x", record=True)
#state_sample = StateMonitor(group, "sample", record=True)

run(10000 * second, report="text", report_period=0.1 * second)


