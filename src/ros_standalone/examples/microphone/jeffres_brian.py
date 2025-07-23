from brian2 import *

set_device("cpp_standalone")
defaultclock.dt = 1 / 48_000 * second

# Sound
audio = TimedArray(10 * randn(50000), dt=defaultclock.dt) # white noise

# EARS PARAMETERS

sound_speed = 343.0*metre/second
distance_between_ears = 0.22*metre
sigma_ear = .1
tau_ear = 0.5*ms
max_delay = distance_between_ears / sound_speed
lam = 0.7  
angular_speed = 2 * pi / second

# Thresholds
a = 1
p = 1.5
tau_thresh = 5*ms

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
tau_radar = 25 * ms
num_radar = 15

# DIRECTION PARAMETERS
num_direction = 2 # Left and Right
tau_direction = 2 * ms  
tau_target = 20 * ms
max_vel = 1.82 
alpha_vel = 2 # acceleration factor
a_front = 0.2 # Determines the front
min_front = num_radar//2 - np.floor(num_radar*a_front/2)
max_front = (num_radar//2 + np.floor(num_radar*a_front/2)) + 1

# EARS

eqs_ears = '''
f_center : Hz (constant)
xn = audio(t -delay) : 1 
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

ears = NeuronGroup(2 * nb_band, eqs_ears, threshold='x>thresh and x>5000',reset=reset,
                    name='ears', method='euler',refractory=0.5*ms)
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

ears.thresh = np.ones(2 * nb_band) * 5000  # Initialize thresholds

# NEURONS

eqs_neurons = '''
dv/dt = -v / tau : 1
'''
neurons = NeuronGroup(num_neurons * nb_band, eqs_neurons, threshold='v>1',
                       reset='v=0', name='neurons', method='euler',refractory=10*ms)
# Connect ears to neurons
synapses = Synapses(ears, neurons, on_pre='v += .65')
synapses.connect('i % nb_band == j // num_neurons')  

synapses.delay['i//nb_band==0'] = '(1.0*j)/(num_neurons-1)*1.1*max_delay'
synapses.delay['i//nb_band==1'] = '(1.0*(num_neurons-j-1))/(num_neurons-1)*1.1*max_delay'
 
# RADAR

eqs_radar = '''
dv/dt = -v / tau_radar: 1
'''

radar = NeuronGroup(num_radar * nb_band, eqs_radar, threshold='v>1', method='euler', name='radar', reset='v=0', dt=0.1*ms)
radar_synapses = Synapses(neurons, radar, on_pre='v += 0.65')
radar_synapses.connect(condition='j == i//(num_neurons//num_radar)')

# COMBINATION

eqs_combi = '''
dv/dt = -v / tau_radar : 1
'''
combination = NeuronGroup(num_radar, eqs_combi, threshold='v>1', reset='v=0', method='euler', name='combination', dt=0.1*ms)
combination_synapses = Synapses(radar, combination, on_pre='v += 0.65')
combination_synapses.connect('j == i // nb_band') 

# DIRECTION DETECTION

eqs_direction = '''
dv/dt = clip(-v, -max_vel, max_vel) / tau_direction : 1
dvel/dt = (v - vel) / tau_target : 1
'''

direction = NeuronGroup(num_direction, eqs_direction, method='euler', name='direction', dt=0.1*ms)
direction_synapses = Synapses(combination, direction, on_pre='v += alpha_vel * ((abs(i-((num_radar-1)/2))/((num_radar-1)/2))-a_front)')
direction_synapses.connect(i=np.arange(0,max_front,dtype=int), j=0)
direction_synapses.connect(i=np.arange(min_front,num_radar,dtype=int), j=1)

# COMMAND MOTOR

eqs_rad = '''
vel_left : 1 (linked)
vel_right : 1 (linked)
veldiff = vel_left - vel_right : 1 (constant over dt)
'''
radian = NeuronGroup(1, eqs_rad)
radian.vel_left = linked_var(direction, 'vel', [0])
radian.vel_right = linked_var(direction, 'vel', [1]) 

# MONITORS

ears_spikes = SpikeMonitor(ears)
neuron_spikes = SpikeMonitor(neurons)
radar_spikes = SpikeMonitor(radar)
combi_spikes = SpikeMonitor(combination)

ears_state = StateMonitor(ears, ['yn', 'yn1', 'yn2', 'xn', 'xn1', 'xn2', 'x', 'thresh'], record=True)
radar_state = StateMonitor(radar, 'v', record=True)
combi_state = StateMonitor(combination, 'v', record=True)
dire_state = StateMonitor(direction, ['v', 'vel'], record=True)
radian_state = StateMonitor(radian, 'veldiff', record=True)

run(10*second, report='text', report_period=1*second, profile=True)


