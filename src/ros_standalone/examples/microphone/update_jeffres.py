from brian2 import *
from brian2 import ms
from scipy.io import wavfile as wav
import matplotlib.pyplot as plt
import numpy as np

sample_rate = 48 * kHz
defaultclock.dt = 1 / sample_rate

# Ears parameters
tau_ear = 0.5*ms

# Thresholds
a = 1
p = 2
tau_thresh = 2*ms

# Sin wave input parameters
amplitude = 32  # Amplitude for 16-bit audio
phase_shift = 0.5 * np.pi  # Phase shift for the sound
chrono = 100
max_delay = 0.2/343 * second
eqs_ears = '''
dx/dt = (sound - x)/tau_ear : 1

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
                    name='ears', method='euler', refractory=0.5*ms)
ears.thresh = [1, 1]
ears.frequency = 100*Hz  # Initial frequency for the sound input  
@network_operation(dt=chrono*ms)
def change_freq():
    ears.frequency = 440 * Hz # Increment frequency for demonstration
res = SpikeMonitor(ears)
x_ears = StateMonitor(ears, 'x', record=True)
th_ears = StateMonitor(ears, 'thresh', record=True)
run(1* second, report='text')

# Plotting the results
plt.figure(figsize=(12, 10))
plt.subplot(4, 1, 1)
plt.plot((res.t[res.i == 1] / ms)%chrono, (res.t[res.i == 1] / ms) // chrono, '.k')
plt.title('Spike Times of Ears')
plt.xlabel('Time (ms)')
plt.ylabel('Trial')
plt.subplot(4, 1, 2)
plt.plot((res.t[res.i == 0] / ms)%chrono, (res.t[res.i == 0] / ms) // chrono, '.r')
plt.title('Spike Times of Ears')
plt.xlabel('Time (ms)')
plt.ylabel('Trial')
plt.subplot(4, 1, 3)
plt.plot(x_ears.t / ms, x_ears.x[1], label='Right Ear', color='blue')
plt.plot(th_ears.t / ms, th_ears.thresh[1], label='Right Ear Threshold', color='red')
plt.title('Ears State Variable x')
plt.xlabel('Time (ms)')
plt.ylabel('x value')
plt.legend()
plt.subplot(4, 1, 4)
plt.plot(x_ears.t / ms, x_ears.x[0], label='Left Ear', color='blue')
plt.plot(th_ears.t / ms, th_ears.thresh[0], label='Left Ear Threshold', color='red')
plt.title('Ears Threshold')
plt.xlabel('Time (ms)')
plt.ylabel('Threshold value')
plt.legend()
plt.tight_layout()
plt.show()


