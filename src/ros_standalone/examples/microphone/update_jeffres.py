from brian2 import *
from brian2 import ms, NeuronGroup, Synapses, SpikeMonitor, StateMonitor, NetworkOperation, run, defaultclock
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
p = 1.5
tau_thresh = 5*ms
lam = 0.5
# Sin wave input parameters
amplitude = 32  # Amplitude for 16-bit audio
chrono = 50
max_delay = 0.2/343 * second
eqs_ears = '''
dx/dt = (input - x)/tau_ear : 1 (unless refractory)
input = 3*clip(sound, 0, inf)**(1/3) : 1
sound = sound_left * (1 - i) + sound_right * i : 1
sound_left = amplitude * sin(2 * pi * frequency * (t + left_delay)) : 1
sound_right = amplitude * sin(2 * pi * frequency * (t + right_delay)) : 1
right_delay = 0.5 * max_delay * sin(phase_shift) : second
left_delay = -0.5 * max_delay * sin(phase_shift) : second


dthresh/dt = (a * clip(x, 0, inf) - thresh) / tau_thresh : 1
frequency : Hz
rec_freq : Hz
phase_shift : 1
'''
reset = '''
thresh = p * thresh + lam * (x - thresh)
x = 0
'''
ears = NeuronGroup(2, eqs_ears, threshold='x>thresh and x>0.05',reset=reset,
                    name='ears', method='euler', refractory=0.5*ms)
ears.thresh = [1, 1]
ears.frequency = 720*Hz  # Initial frequency for the sound input  
ears.rec_freq = ears.frequency  # Record the frequency for monitoring
# @network_operation(dt=chrono*ms)
# def change_freq():
#     ears.frequency *= 1.2 # Increment frequency for demonstration

@network_operation(dt=chrono*10*ms)
def change_phase():
    ears.phase_shift += 0.1 * np.pi  # Increment phase shift for demonstration
eqs_neurons = '''
dv/dt = -v / tau : 1
'''
neurons = NeuronGroup(num_neurons, eqs_neurons, threshold='v>1',
                       reset='v=0', name='neurons', method='euler',refractory=10*ms)
# Connect ears to neurons
synapses = Synapses(ears, neurons, on_pre='v += .65')
synapses.connect()

synapses.delay['i==0'] = '(1.0*j)/(num_neurons-1)*1.1*max_delay'
synapses.delay['i==1'] = '(1.0*(num_neurons-j-1))/(num_neurons-1)*1.1*max_delay'

#wta = Synapses(neurons, neurons, on_pre='v -= 0.65')  
#wta.connect(condition='i != j') 

tau_radar = 1 * ms  # Time constant for the radar

eqs_radar = '''
dv/dt = -v / tau_radar : 1
'''


radar = NeuronGroup(num_neurons, eqs_radar, method='euler', name='radar')
radar_synapses = Synapses(neurons, radar, on_pre='v += 100')
radar_synapses.connect(condition='j == i')


res = SpikeMonitor(ears)
spikes = SpikeMonitor(neurons)
x_ears = StateMonitor(ears, 'x', record=True)
state_rad = StateMonitor(radar, 'v', record=True)
th_ears = StateMonitor(ears, 'thresh', record=True)
run(5* second, report='text')

tot = np.sum(state_rad.v, axis=0)
deg = np.linspace(0, 180, num_neurons)
deg_coeff = state_rad.v.T*deg
#direction = np.mean(deg_coeff, axis=1)
direction = []
for i in range (len(deg_coeff)):
    dir_tmp = []
    for j in range(num_neurons):
        dir_tmp.append(deg_coeff[i][j] / (tot[i]+1e-99))  
    direction.append(dir_tmp)
dir_final = np.sum(np.array(direction), axis=1)
#direction = (deg_coeff.T.dot(1/(tot + 1e-99)))
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
plt.plot(state_rad.t / ms, dir_final, label='Radar Position', color='green')
plt.title('Spike Times of Neurons')
plt.xlabel('Time (ms)')
plt.ylabel('Neuron Index')

plt.tight_layout()
plt.show()


