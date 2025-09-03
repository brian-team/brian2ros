from brian2 import *
from brian2ros import *
import matplotlib.pyplot as plt
import numpy as np
from scipy.special import softmax

prefs.devices.ros_standalone.buffer_multiplier = 7
set_device("ros_standalone", directory="src/src/brian_project", debug=True)
defaultclock.dt = 1 / 48_000 * second

# Create a ROS Subscriber for audio input
audio = Subscriber(
    name="audio",
    topic="audio_data",
    topic_type="turtleaudio/msg/StereoAudioBlock",
    output={"left": np.int16(np.linspace(0,1023,1024)), "right": np.int16(np.linspace(0,1023,1024))},
    header="turtleaudio/msg/stereo_audio_block.hpp",
)

# EARS PARAMETERS

tau_ear = 0.5*ms
lam = 0.7  

# Thresholds
a = 1
p = 1.5
tau_thresh = 5*ms
#min_thresh = 3000

# Filter parameters
f_min = 100 * Hz
f_max = 2500 * Hz  
nb_band = 16
BW = 0.2
fs = 48000 * Hz 

# EARS

eqs_ears = '''
f_center : Hz
xn = audio(t, i // nb_band) : 1 (constant over dt)

w0 : 1 (constant)
alpha : 1 (constant)
a0 : 1 (constant)
a1 : 1 (constant)
a2 : 1 (constant)
b0 : 1 (constant)
b1 : 1 (constant)
b2 : 1 (constant) 


yn = b0 * xn + b1 * xn1 + b2 * xn2 - a1 * yn1 - a2 * yn2 : 1 (constant over dt)

yn1 : 1
yn2 : 1

xn2 : 1 
xn1 : 1 

dx/dt = 3*clip(yn - x, 0, inf)**(1/3)/tau_ear : 1 (unless refractory)
dthresh/dt = (a * clip(x, 0, inf) - thresh) / tau_thresh : 1

beta : 1 (constant)
min_thresh : 1 (linked)
'''


reset = '''
x = 0
thresh = p * thresh + lam * x
'''

ears = NeuronGroup(2 * nb_band, eqs_ears, threshold='x>thresh and x>min_thresh * beta',reset=reset,
                    name='ears', method='euler',refractory=0.2*ms)
ears.f_center = ' f_min * (f_max / f_min) ** ((i%nb_band) / (nb_band - 1))'  # Logarithmic spacing of frequencies
ears.w0 = '(2 * pi * f_center) / fs'  # Angular frequency
ears.alpha = 'sin(w0) * sinh((log(2)/2) * BW * (w0 / sin(w0)))'  # Filter coefficient
ears.a0 = '1 + alpha'  # Normalization factor
ears.b0 = '(sin(w0) / 2) / a0'  # Coefficient for current input
ears.b1 = '0'  # Coefficient for previous input
ears.b2 = '(-sin(w0) / 2) / a0'  # Coefficient for input two steps back
ears.a1 = '(-2 * cos(w0)) / a0'  # Coefficient for previous output
ears.a2 = '(1 - alpha) / a0'  # Coefficient for output two steps back
ears.run_regularly('xn2 = xn1 ; xn1 = xn ; yn2 = yn1 ; yn1 = yn', dt=defaultclock.dt, when='end')

ears.thresh = np.ones(2 * nb_band) * 32767  # Initialize thresholds

eqs_thresh = '''
dnoiselevel/dt = (clip(x_in, 0, inf) - noiselevel) / (50*ms) : 1
x_in : 1 (linked)
'''
adaptive_thresh = NeuronGroup(2 * nb_band, eqs_thresh, method='euler', name='adaptive_thresh')
adaptive_thresh.x_in = linked_var(ears, 'x')

ears.min_thresh = linked_var(adaptive_thresh, 'noiselevel')
ears.beta = 2 

ears_state = StateMonitor(ears, 'yn', record=True)
ears_spikes = SpikeMonitor(ears)

get_device().publish_monitors([ears_spikes, ears_state])
run(8 * second)