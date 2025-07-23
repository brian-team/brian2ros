from brian2 import *
from scipy.io import wavfile as wav
defaultclock.dt = 1 / 48_000 * second

set_device('cpp_standalone',)
# Sound
sample_rate, stereo_data = wav.read("500Hz.wav")

# convert int16 to float32 range [-1, 1]
stereo_data = stereo_data.astype(np.float32)
stereo_data /= np.iinfo(np.int16).max  # Normalise entre -1 et 1

# standardize to mean=0, std=1 (like np.random.randn)
stereo_data -= np.mean(stereo_data, axis=0)
stereo_data /= np.std(stereo_data, axis=0)

# Create TimedArrays
sound_left = TimedArray(stereo_data[:, 0], dt=defaultclock.dt)
sound_right = TimedArray(stereo_data[:, 1], dt=defaultclock.dt)
#=== Find the interaural distance ===#
sound_speed = 300*metre/second

# === Determine the interaural distance ===
# Ears and sound motion around the head (constant angular speed)
max_delay = 0.35*ms #interaural_distance / sound_speed
print("Maximum interaural delay: %s" % max_delay)
angular_speed = 2 * pi / second # 1 turn/second
tau_ear = 1*ms
sigma_ear = .1
eqs_ears = '''
dx/dt = (sound-x)/tau_ear : 1 
sound = sound_left(t) * (1 - i) + sound_right(t) * i : 1
'''
ears = NeuronGroup(2, eqs_ears, threshold='x>1', reset='x = 0',
                   refractory=2.5*ms, name='ears', method='euler')
# Coincidence detectors
num_neurons = 30
tau = 1*ms
sigma = .1
eqs_neurons = '''
dv/dt = -v / tau + sigma * (2 / tau)**.5 * xi : 1
'''
neurons = NeuronGroup(num_neurons, eqs_neurons, threshold='v>1',
                      reset='v = 0', name='neurons', method='euler')

synapses = Synapses(ears, neurons, on_pre='v += .5')
synapses.connect()

synapses.delay['i==0'] = '(1.0*j)/(num_neurons-1)*1.1*max_delay'
synapses.delay['i==1'] = '(1.0*(num_neurons-j-1))/(num_neurons-1)*1.1*max_delay'

spikes = SpikeMonitor(neurons)

run(10000*ms)

# Plot the results
i, t = spikes.it
plot(t/ms, i, '.')
xlabel('Time (ms)')
ylabel('Neuron index')
xlim(0, 1000)
show()