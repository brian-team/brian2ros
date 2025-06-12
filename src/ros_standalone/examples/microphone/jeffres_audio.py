from brian2 import *
from brian2ros import *

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

# Ears parameters
sound_speed = 343.0*metre/second
distance_between_ears = 0.22*metre
sigma_ear = .1
tau_ear = 0.5*ms
max_delay = distance_between_ears / sound_speed

# Coincidence detectors
num_neurons = 30
tau = 0.2*ms

# Thresholds
a = 0
p = 2.8
tau_thresh = 20*ms

# Ears and sound motion around the head (constant angular speed)
eqs_ears = '''
sound = audio(t,i) : 1 (constant over dt)
dx/dt = (sound - x)/tau_ear : 1 (unless refractory)
dthresh/dt = (a * clip(x, 0, inf) - thresh) / tau_thresh : 1
'''

reset = '''
x = 0
thresh = p * thresh
'''

ears = NeuronGroup(2, eqs_ears, threshold='x>thresh and x>0.05',reset=reset,
                    name='ears', method='euler',refractory=0.5*ms)

ears.thresh = [1, 1]
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

ears_spikes = SpikeMonitor(ears)
spikes = SpikeMonitor(neurons)
# Ne peut pas etre utilise en un seul state monitor
sound = StateMonitor(ears, 'sound', record=True)
thresh = StateMonitor(ears, 'thresh', record=True)
son = StateMonitor(ears, 'x', record=True)


get_device().publish_monitors([ears_spikes, spikes, sound, thresh, son])
run(8 * second, report="text", report_period=1 * second)