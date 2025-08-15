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

sound_speed = 343.0*metre/second
distance_between_ears = 0.2*metre
sigma_ear = .1
tau_ear = 0.5*ms
max_delay = distance_between_ears / sound_speed
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

# NEURONS PARAMETERS
num_neurons = 30
tau = 0.4*ms

# RADAR PARAMETERS
tau_radar = 5 * ms
num_radar = 15 # NOT USED

# DIRECTION PARAMETERS
num_direction = 2 # Left and Right
tau_direction = 5 * ms  
tau_target = 100 * ms
max_vel = 1.82 
alpha_vel = 0.2 # acceleration factor
a_front = 0.05 # Determines the front
min_front = num_neurons//2 - np.floor(num_neurons*a_front/2)
max_front = (num_neurons//2 + np.floor(num_neurons*a_front/2)) + 1

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

ears.thresh = np.ones(2 * nb_band)  # Initialize thresholds

eqs_thresh = '''
dnoiselevel/dt = (clip(x_in, 0, inf) - noiselevel) / (50*ms) : 1
x_in : 1 (linked)
'''
adaptive_thresh = NeuronGroup(2 * nb_band, eqs_thresh, method='euler', name='adaptive_thresh')
adaptive_thresh.x_in = linked_var(ears, 'x')

ears.min_thresh = linked_var(adaptive_thresh, 'noiselevel')
ears.beta = 2 

# NEURONS

eqs_neurons = '''
dv/dt = -v / tau : 1
'''
neurons = NeuronGroup(num_neurons * nb_band, eqs_neurons, threshold='v>1',
                       reset='v=0', name='neurons', method='euler')
# Connect ears to neurons
synapses = Synapses(ears, neurons, on_pre='v += .6')
synapses.connect('i % nb_band == j // num_neurons')  

synapses.delay['i//nb_band==1'] = '(1.0*(j%num_neurons))/(num_neurons-1)*1.1*max_delay'
synapses.delay['i//nb_band==0'] = '(1.0*(num_neurons-(j%num_neurons)-1))/(num_neurons-1)*1.1*max_delay'

# RADAR

eqs_radar = '''
dv/dt = -v / tau_radar: 1
'''
radar = NeuronGroup(num_neurons, eqs_radar, threshold='v>1', method='euler', name='radar', reset='v=0', dt=5*defaultclock.dt)
radar_synapses = Synapses(neurons, radar, on_pre='v += 0.65')
radar_synapses.connect(condition='j == i%num_neurons')

wta = Synapses(radar, radar, on_pre='v -= 0.65')  
wta.connect(condition='i != j')

# DIRECTION DETECTION

eqs_direction = '''
dv/dt = -v / tau_direction : 1
dvel/dt = (v - vel) / tau_target : 1
'''

direction = NeuronGroup(num_direction, eqs_direction, method='euler', name='direction', dt=5*defaultclock.dt)
direction_synapses = Synapses(radar, direction, on_pre='v += alpha_vel * ((abs(i-((num_neurons-1)/2))/((num_neurons-1)/2))-a_front)')
direction_synapses.connect(i=np.arange(0,max_front,dtype=int), j=0)
direction_synapses.connect(i=np.arange(min_front,num_neurons,dtype=int), j=1)

# COMMAND MOTOR

eqs_rad = '''
vel_left : 1 (linked)
vel_right : 1 (linked)
veldiff = clip(vel_left - vel_right, -max_vel, max_vel) : 1 (constant over dt)
'''
radian = NeuronGroup(1, eqs_rad)
radian.vel_left = linked_var(direction, 'vel', [0])
radian.vel_right = linked_var(direction, 'vel', [1]) 

wheel = TwistPublisher(
    name="wheel",
    input={"angular.z": radian.veldiff},
)
get_device().add_publisher(wheel) 

s_yn = StateMonitor(ears, 'yn', record=True)
#s_yn1 = StateMonitor(ears, 'yn1', record=True)
#s_yn2 = StateMonitor(ears, 'yn2', record=True)
s_xn = StateMonitor(ears, 'xn', record=True)   
#s_xn1 = StateMonitor(ears, 'xn1', record=True)
#s_xn2 = StateMonitor(ears, 'xn2', record=True)
#son = StateMonitor(ears, 'x', record=True)
#thresh = StateMonitor(ears, 'thresh', record=True)
s_noiselevel = StateMonitor(adaptive_thresh, 'noiselevel', record=True)

#neurons_state = StateMonitor(neurons, 'v', record=True)
#radar_state = StateMonitor(radar, 'v', record=True)

#dir = StateMonitor(direction, 'v', record=True)
dir_vel = StateMonitor(direction, 'vel', record=True)

radian_state = StateMonitor(radian, 'veldiff', record=True)
ears_spikes = SpikeMonitor(ears)
spikes = SpikeMonitor(neurons)
radar_spikes = SpikeMonitor(radar)

neurons_rate = PopulationRateMonitor(neurons)
#combi = SpikeMonitor(combination)

# Ne peut pas etre utilise en un seul state monitor
#combi_state = StateMonitor(combination, 'v', record=True)


get_device().publish_monitors([dir_vel, s_xn, radian_state, s_noiselevel, spikes, radar_spikes])
run(60 * second)
