from brian2 import *
from brian2ros import *
import numpy as np
import matplotlib.pyplot as plt

set_device("ros_standalone", directory="src/src/brian_project", debug=True)
defaultclock.dt = (1 / 48_000) * second

prefs.devices.ros_standalone.interface = False

audio = Subscriber(
    name="audio",
    topic="audio_data",
    topic_type="std_msgs/msg/Float64MultiArray",
    output={"left": np.int64(np.linspace(0,255,256)), "right": np.int64(np.linspace(0,255,256))}
)

eq = Equations('''
x = audio(t,0,sample) * i + audio(t,1,sample) * (1 - i) : 1
sample : 1
''')

group = NeuronGroup(2,eq,name="ears",method="euler",threshold="x>10",reset="")
group.run_regularly('sample = (sample + 1) % 256', dt=(1 / 48_000) * second)

state_x = StateMonitor(group, "x", record=True)
state_sample = StateMonitor(group, "sample", record=True)

run(5 * second)

plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(state_x.t / second, state_x[0], label="left")
plt.plot(state_x.t / second, state_x[1], label="right")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(state_sample.t / second, state_sample[0], label="left")
plt.plot(state_sample.t / second, state_sample[1], label="right")
plt.xlabel("Time (s)")
plt.ylabel("Sample")
plt.legend()
plt.tight_layout()
plt.show()
