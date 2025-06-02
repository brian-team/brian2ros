from brian2 import *
from brian2ros import *

set_device("ros_standalone", directory="src/src/brian_project", debug=True)
defaultclock.dt = 1 / 48_000 * second

# Create a ROS Subscriber for audio input
audio = Subscriber(
    name="audio",
    topic="audio_sin",
    topic_type="turtleaudio/msg/StereoAudioBlock",
    output={"left": np.int16(np.linspace(0,1023,1024)), "right": np.int16(np.linspace(0,1023,1024))},
    header="turtleaudio/msg/stereo_audio_block.hpp",
)

# Ears parameters
sound_speed = 343.0*metre/second
distance_between_ears = 0.2*metre
sigma_ear = .1
tau_ear = 0.5*ms
max_delay = distance_between_ears / sound_speed

# Coincidence detectors
num_neurons = 30
tau = 0.2*ms

# Thresholds
a = 0.1
p = 2.7
tau_thresh = 2*ms

# Ears and sound motion around the head (constant angular speed)
eqs_ears = '''
sound = audio(t,i) : 1 (constant over dt)
dx/dt = (sound - x)/tau_ear : 1 
dthresh/dt = (a * clip(x, 0, inf) - thresh) / tau_thresh : 1
'''

reset = '''
x = 0
thresh = p * thresh
'''

ears = NeuronGroup(2, eqs_ears, threshold='x>thresh and x>0.05',reset=reset,
                    name='ears', method='euler')

ears.thresh = [1, 1]
eqs_neurons = '''
dv/dt = -v / tau : 1
'''
neurons = NeuronGroup(num_neurons, eqs_neurons, threshold='v>1',
                       reset='v=0', name='neurons', method='euler')
# Connect ears to neurons
synapses = Synapses(ears, neurons, on_pre='v += .65')
synapses.connect()

synapses.delay['i==0'] = '(1.0*j)/(num_neurons-1)*1.1*max_delay'
synapses.delay['i==1'] = '(1.0*(num_neurons-j-1))/(num_neurons-1)*1.1*max_delay'

spikes = SpikeMonitor(neurons)
son = StateMonitor(ears, 'x', record=True)
spikes_thresh = StateMonitor(ears, 'thresh', record=True)

get_device().display_monitors([spikes, son, spikes_thresh])
run(10 * second, report="text", report_period=1 * second)