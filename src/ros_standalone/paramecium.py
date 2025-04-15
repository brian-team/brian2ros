"""
Full model
"""

from brian2 import *
from brian2.units.allunits import umetre3
from brian2.units.constants import faraday_constant as F, gas_constant as R
from brian2ros import *

from matplotlib.animation import FuncAnimation

set_device("ros_standalone", directory="src/src/brian_project", debug=True)

#prefs.devices.ros_standalone.cyclonedds = True
#prefs.devices.ros_standalone.network_interface = 'wlxf07959eb0bcf'
#prefs.devices.ros_standalone.list_address_ip = ['10.42.0.1', '10.42.0.103']
#prefs.devices.ros_standalone.interface = False

cell_length = 120.0
cell_width = 35.0


def cell_border(theta=0.0):
    """
    Returns cell coordinates in um, centered on 0.
    The cell points towards the right.
    """
    # Shape coordinates
    asymetry = 0.15
    xcell = np.linspace(-cell_length * 0.5, cell_length * 0.5, 150)
    ycell = (
        0.5
        * cell_width
        * (
            (1 - 4 * xcell**2 / cell_length**2) ** 0.5
            - asymetry * np.sin(np.pi * 2 * xcell / cell_length)
        )
    )
    ycell = np.hstack([ycell, -ycell[::-1]])
    xcell = np.hstack([xcell, xcell[::-1]])

    xshape = xcell * np.cos(theta) - ycell * np.sin(theta)
    yshape = xcell * np.sin(theta) + ycell * np.cos(theta)

    return xshape, yshape


def get_cell_outline(x, y, scale, theta=0.0):
    xcell, ycell = cell_border(theta=theta)  # pointing up
    xcell = np.hstack([xcell, [xcell[0]]]) / cell_length
    ycell = np.hstack([ycell, [ycell[0]]]) / cell_length

    return x + scale * xcell, y + scale * ycell


constants = {
    "C": 275.0 * pfarad,
    "Cai0_cilia": 100.0 * nmolar,
    "DV": R * (273 + 20.0) * kelvin / F,  # 20 °C
    "EK": -48.0 * mvolt,
    "EL": -23.411751 * mvolt,
    "F": F,
    "Jpumpmax_cilia": 1.14231898 * khertz,
    "K_electromotor": 1.4 * umolar,
    "R": R,
    "Re": 82.21786662 * Mohm,
    "VCa_cilia": 5.235069 * mvolt,
    "V_IK": 0.33134 * mvolt,
    "a_IK": 100.0 * usecond,
    "alpha_cilia": 2.52799157 * hertz,
    "angle_min": 0.761927814,
    "angle_span": -4.093696582,
    "b_IK": 2.973515 * msecond,
    "gCa_cilia": 1.0 * uamp,
    "gKCa_cilia": 27.8 * namp,
    "gL": 11.8 * nsiemens,
    "g_IK": 2.31783779 * namp,
    "kCa_cilia": 4.659191 * mvolt,
    "k_IK": 3.230705 * mvolt,
    "nCaM_Ca_cilia": 4.223339529,
    "nCaM_KCa_cilia": 1.830536013,
    "n_electromotor": 5.592777655,
    "n_electromotor_coupling": 2,
    "omega_max": 8 * pi / second,
    "omega_min": 2 * pi / second,
    "pKCa": 3.196528772,
    "pKKCa": 7.367400475,
    "pK_electromotor": 2.632978236,
    "p_ICa": 2.0,
    "taue": 0.540317 * msecond,
    "taum_Ca_cilia": 1.419777 * msecond,
    "theta_max": 90 * pi / 180,
    "theta_min": 13 * pi / 180,
    "v_cilia": 1700.0 * umetre3,
    "v_minus": -0.001 * metre * second**-1,
    "v_plus": 0.001 * metre * second**-1,
    "v0": 3,
    "a_max": 1.86,
    "a_min": -1.86,
}

range = LaserScanSubscriber(
    name= "range",
    output={"ranges": 0}
)

equations = Equations(
    """
    dv/dt = (IL+ICa_cilia+IK+IKCa_cilia+I)/C : volt
    d = int(range(t,0,0) <= 0.5) : 1
    I = 10*nA*d : amp
    dx/dt = velocity*cos(orientation) : meter
    dy/dt = velocity*sin(orientation) : meter
    IL = gL*(EL-v) : amp
    IV_K = (EK-v)/DV : 1
    IV_Ca = 1/exprel(2*v/DV) : 1
    ## IK (rectifier)
    IK = g_IK * n_IK**2 * IV_K : amp
    ninf_IK = 1/(1+exp((V_IK-v)/k_IK)) : 1
    dn_IK / dt = (ninf_IK - n_IK)/tau_IK: 1
    tau_IK = a_IK + b_IK/(cosh((v-V_IK)/(2*k_IK))) : second
    
    ### ICa_cilia (depolarization-activated)
    ICa_cilia = gCa_cilia*m_Ca_cilia**2*h_Ca_cilia*IV_Ca : amp
    dm_Ca_cilia/dt = (minf_Ca_cilia-m_Ca_cilia)/taum_Ca_cilia : 1
    minf_Ca_cilia = 1/(1+exp((VCa_cilia-v)/kCa_cilia)) : 1
    h_Ca_cilia = 1/(1+exp(nCaM_Ca_cilia*(pCa-pKCa))) : 1
    
    ### IK(Ca)
    IKCa_cilia = gKCa_cilia / (1+ exp(-nCaM_KCa_cilia*(pCa-pKKCa)))* IV_K : amp # gKCa is the amplitude at 10 uM
    
    #### Calcium dynamics ####
    dpCa/dt = ICa_cilia/(2*F*Cai0_cilia*v_cilia)*exp(-pCa) + alpha_cilia*(exp(-pCa)-1)- Jpump_cilia : 1
    Jpump_cilia = Jpumpmax_cilia / (1+exp(pCa)) : 1/second
    Cai_cilia = Cai0_cilia*exp(pCa) : mM
    
    ### Electromotor coupling
    velocity = (-v_plus + 2*v_plus/(1+(Cai_cilia/K_electromotor)**2))*200 : meter/second (constant over dt)
    escape_orientation : radian
    dorientation/dt = ((orientation - escape_orientation + pi) % (2*pi) - pi)/(50*ms): radian
    angular = clip((direction - 0.5)*int(t < t_start + t_turn)*v0,a_min,a_max) : 1 (constant over dt)
    direction : 1
    t_start : second
    t_turn : second
    """,
    **constants,
)
width, height = 2, 2

neuron = NeuronGroup(
    1,
    equations,
    method="euler",
    threshold="velocity > 0*meter/second",
    refractory="velocity > 0*meter/second",
    reset="direction=rand(); t_turn = rand()*100*ms + 500*ms; t_start = t",
)

neuron.v = constants["EL"]
neuron.orientation = -np.pi / 4
neuron.escape_orientation = -np.pi / 4

motor_commands = TwistPublisher(
    rate=200,
    input={"linear.x": neuron.velocity, "angular.z": neuron.angular},
)

get_device().add_publisher(motor_commands)  
S_sensor = SpikeMonitor(neuron)
pops = PopulationRateMonitor(neuron)
M = StateMonitor(neuron, "v", record=True, dt=1 * ms)

trigger_time = 500 * ms
I0 = 5 * nA

runtime = 1e5 * second
run(runtime, report="text", report_period=10 * second)



