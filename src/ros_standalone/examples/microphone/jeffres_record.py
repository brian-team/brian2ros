from brian2 import *
from brian2ros import *
from scipy.io import wavfile as wav

set_device('ros_standalone',directory="src/src/brian_project", debug=True)

sample_rate = 48 * kHz
buffer_size = 128
defaultclock.dt = 1 / sample_rate

# Ears parameters
sigma_ear = .1
tau_ear = 0.5*ms
max_delay = 0.62*ms 

# Coincidence detectors
num_neurons = 30
tau = 0.2*ms

# Thresholds
a = 0.1
p = 2.7
tau_thresh = 2*ms

sample_rate, stereo_data = wav.read("src/ros_standalone/examples/microphone/500Hz.wav")

sound_left = TimedArray(stereo_data[:, 0], dt=defaultclock.dt)
sound_right = TimedArray(stereo_data[:, 1], dt=defaultclock.dt)

# Ears and sound motion around the head (constant angular speed)
eqs_ears = '''
dx/dt = (sound - x)/tau_ear : 1 
sound = sound_left(sample) * (1 - i) + sound_right(sample) * i : 1
sample = t - 5*second * (t/second//5) : second
dthresh/dt = (a * x - thresh) / tau_thresh : 1
'''

reset = '''
x = 0
thresh = p * thresh
'''
ears = NeuronGroup(2, eqs_ears, threshold='x>thresh',reset=reset,
                    name='ears', method='euler')
ears.thresh = [1, 1]

eqs_neurons = '''
dv/dt = -v / tau : 1
'''
neurons = NeuronGroup(num_neurons, eqs_neurons, threshold='v>1',
                      reset='v = 0', name='neurons', method='euler')

synapses = Synapses(ears, neurons, on_pre='v += .65')
synapses.connect()

synapses.delay['i==0'] = '(1.0*j)/(num_neurons-1)*1.1*max_delay'
synapses.delay['i==1'] = '(1.0*(num_neurons-j-1))/(num_neurons-1)*1.1*max_delay'

spikes = SpikeMonitor(neurons)
son = StateMonitor(ears, 'x', record=True)
th = StateMonitor(ears, 'thresh', record=True)
run(10000*second)
