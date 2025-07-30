from brian2 import *
from brian2tools import *
from matplotlib import pyplot as plt
import numpy as np

set_device("cpp_standalone")
defaultclock.dt = 1 / 48_000 * second

# Sound
audio = TimedArray((32767.0 / 2) * sin(np.arange(0,10,1/48000) * 2 * np.pi * 440), dt=defaultclock.dt) # 440 Hz sine wave

# EARS PARAMETERS
sound_speed = 343.0*metre/second
distance_between_ears = 0.25*metre
sigma_ear = .1
tau_ear = 0.5*ms
max_delay = distance_between_ears / sound_speed
lam = 0.7  
angular_speed = 0.5 * pi / second

# Thresholds
a = 1
p = 1.5
tau_thresh = 5*ms
min_thresh = 50000

# Filter parameters
f_min = 100 * Hz
f_max = 2500 * Hz  
nb_band = 16
BW = 0.2
fs = 48000 * Hz 

# NEURONS PARAMETERS
num_neurons = 30
tau = 0.2*ms

# RADAR PARAMETERS
tau_radar = 2 * ms
num_radar = 15

# DIRECTION PARAMETERS
num_direction = 2 # Left and Right
tau_direction = 5 * ms  
tau_target = 50 * ms
max_vel = 1.82 
alpha_vel = 3 # acceleration factor
a_front = 0.2 # Determines the front
min_front = num_neurons//2 - np.floor(num_neurons*a_front/2)
max_front = (num_neurons//2 + np.floor(num_neurons*a_front/2)) + 1

# EARS

eqs_ears = '''
f_center : Hz
xn = 3* clip(audio(t -delay), 0, inf)**(1.0/3.0) : 1 
delay = distance * sin(theta) : second
distance : second
dtheta/dt = angular_speed : radian
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

dx/dt = (yn - x)/tau_ear : 1 (unless refractory)
dthresh/dt = (a * clip(x, 0, inf) - thresh) / tau_thresh : 1
'''


reset = '''
x = 0
thresh = p * thresh + lam * x
'''

ears = NeuronGroup(2 * nb_band, eqs_ears, threshold='x>thresh and x>min_thresh',reset=reset,
                    name='ears', method='euler',refractory=0.2*ms)
ears.distance = concatenate((ones(nb_band) * -0.5 * max_delay, ones(nb_band) * 0.5 * max_delay)) * second
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

ears.thresh = np.ones(2 * nb_band) * min_thresh  # Initialize thresholds

# NEURONS

eqs_neurons = '''
dv/dt = -v / tau : 1
'''
neurons = NeuronGroup(num_neurons * nb_band, eqs_neurons, threshold='v>1',
                       reset='v=0', name='neurons', method='euler')
# Connect ears to neurons
synapses = Synapses(ears, neurons, on_pre='v += .6')
synapses.connect('i % nb_band == j // num_neurons')  

synapses.delay['i//nb_band==0'] = '(1.0*(j%num_neurons))/(num_neurons-1)*1.1*max_delay'
synapses.delay['i//nb_band==1'] = '(1.0*(num_neurons-(j%num_neurons)-1))/(num_neurons-1)*1.1*max_delay'

# MONITOR
ears_state = StateMonitor(ears, 'x', record=True)
neurons_state = StateMonitor(neurons, 'v', record=True)

ears_spikes = SpikeMonitor(ears)
neurons_spikes = SpikeMonitor(neurons)

run(5*second)
# PLOT RESULTS
plt.figure(figsize=(12, 6))
plt.subplot(2, 1, 1)
plt.plot(ears_spikes.t / ms, ears_spikes.i, '.k')
plt.title('Ears Spikes')
plt.xlabel('Time (ms)')
plt.ylabel('Neuron Index')
plt.subplot(2, 1, 2)
plt.plot(neurons_spikes.t / ms, neurons_spikes.i, '.r')
plt.title('Neurons Spikes')
plt.xlabel('Time (ms)')
plt.ylabel('Neuron Index')
plt.tight_layout()
plt.show()