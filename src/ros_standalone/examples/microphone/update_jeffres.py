from brian2 import *
from brian2 import ms
from scipy.io import wavfile as wav
import matplotlib.pyplot as plt
import numpy as np

sample_rate = 48 * kHz
defaultclock.dt = 1 / sample_rate

# Ears parameters
tau_ear = 0.5*ms

# Coincidence detectors
num_neurons = 30
tau = 0.2*ms

# Thresholds
a = 1
p = 3
tau_thresh = 5*ms

# Sin wave input parameters
amplitude = 32  # Amplitude for 16-bit audio
phase_shift = 0.5 * np.pi  # Phase shift for the sound
chrono = 100
max_delay = 0.2/343 * second
eqs_ears = '''
dx/dt = (input - x)/tau_ear : 1
input = 3*clip(sound, 0, inf)**(1/3) : 1
sound = sound_left * (1 - i) + sound_right * i : 1
sound_left = amplitude * sin(2 * pi * frequency * (t + left_delay)) : 1
sound_right = amplitude * sin(2 * pi * frequency * (t + right_delay)) : 1
right_delay = 0.5 * max_delay * sin(phase_shift) : second
left_delay = -0.5 * max_delay * sin(phase_shift) : second


dthresh/dt = (a * clip(x, 0, inf) - thresh) / tau_thresh : 1
frequency : Hz
'''
reset = '''
x = 0
thresh = p * thresh
'''
ears = NeuronGroup(2, eqs_ears, threshold='x>thresh and x>0.05',reset=reset,
                    name='ears', method='euler')
ears.thresh = [1, 1]
ears.frequency = 440*Hz  # Initial frequency for the sound input  
# @network_operation(dt=chrono*ms)
# def change_freq():
#     ears.frequency += 0 * Hz # Increment frequency for demonstration


eqs_neurons = '''
dv/dt = -v / tau : 1
'''
neurons = NeuronGroup(num_neurons, eqs_neurons, threshold='v>1',
                       reset='v=0.2', name='neurons', method='euler',refractory=10*ms)
# Connect ears to neurons
synapses = Synapses(ears, neurons, on_pre='v += .65')
synapses.connect()

synapses.delay['i==0'] = '(1.0*j)/(num_neurons-1)*1.1*max_delay'
synapses.delay['i==1'] = '(1.0*(num_neurons-j-1))/(num_neurons-1)*1.1*max_delay'

wta = Synapses(neurons, neurons, on_pre='v -= 0.65')  
wta.connect(condition='i != j') 


res = SpikeMonitor(ears)
spikes = SpikeMonitor(neurons)
x_ears = StateMonitor(ears, 'x', record=True)
th_ears = StateMonitor(ears, 'thresh', record=True)
run(1* second, report='text')

# Plotting the results
plt.figure(figsize=(18, 16))

plt.subplot(4, 1, 1)
plt.plot((res.t[res.i == 1] / ms)%chrono, (res.t[res.i == 1] / ms) // chrono, '.k')
plt.ylabel('Trial')

plt.subplot(4, 2, 1)
plt.plot((res.t[res.i == 0] / ms)%chrono, (res.t[res.i == 0] / ms) // chrono, '.r')
plt.title('Spike Times of Ears')
plt.xlabel('Time (ms)')

plt.subplot(4, 1, 2)
plt.plot(x_ears.t / ms, x_ears.x[1], label='Right Ear', color='blue')
plt.plot(th_ears.t / ms, th_ears.thresh[1], label='Right Ear Threshold', color='red')
plt.title('Ears State Variable x')
plt.xlabel('Time (ms)')
plt.ylabel('x value')
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(x_ears.t / ms, x_ears.x[0], label='Left Ear', color='blue')
plt.plot(th_ears.t / ms, th_ears.thresh[0], label='Left Ear Threshold', color='red')
plt.title('Ears Threshold')
plt.xlabel('Time (ms)')
plt.ylabel('Threshold value')
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(spikes.t / ms, spikes.i, '.k')
plt.title('Spike Times of Neurons')
plt.xlabel('Time (ms)')
plt.ylabel('Neuron Index')
plt.xlim(0, 1000)
plt.ylim(-1, num_neurons)

plt.tight_layout()
plt.show()


