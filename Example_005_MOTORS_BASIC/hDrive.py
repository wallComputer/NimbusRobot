import time
import math
from nrpd import *
from lib import motorHandler
from machine import Pin

"""

Holonomic Drive.

Given ((v, θ, ø),t), the robot moves at:
    1. With velocity v, 
    2. At angle θ, 
    3. Turning around its center at angle ø,
    4. For a duration of t milliseconds.
                     +Y
                     ▲ π/2
                     ┆     ↗ v
                     ┆   ↗
          π          ┆ ↗ θ       0
        -X◀--------- ⊙ ---------▶+X 
         -π          ┆ ø⤴       0 
                     ┆
                     ┆        
                -π/2 ▼
                    -Y

        v is applied velocity magnitude from [0,65535]
        θ is from twist angle from [0,2π) with +X as 0˚
        ø turn angle from [0,π] and (-π,π], effectively (-π,π]. Same as θ, ø references +X as 0˚
            In the above image, even though ø is placed next to -Y,
            it does not mean ø references -Y for 0˚ . It's places so due to space constraints.
        t is applied time of (v, θ, ø) in milliseconds. Default is  2500
        
        if θ is out of its range or None, only turn angle and velocity is used.
        if ø is out of its range or None, only θ value and velocity is used. Trivially, this is also true when ø=0
        if both θ and ø are out of their ranges or None, velocity is ignored.
        
In experience, mixing θ and ø results in unpredicted movement. So its better to use only one at a time. 
"""

LTParams = {
    'motorID': 0,
    'MPINS': [MLTIN1, MLTIN2],
    'motorFreq': 75,
    'sm_no': -1,
}
LBParams = {
    'motorID': 1,
    'MPINS': [MLBIN1, MLBIN2],
    'motorFreq': 75,
    'sm_no': -1,
}
RTParams = {
    'motorID': 2,
    'MPINS': [MRTIN1, MRTIN2],
    'motorFreq': 75,
}
RBParams = {
    'motorID': 3,
    'MPINS': [MRBIN1, MRBIN2],
    'motorFreq': 75,
}
# time.sleep(10)
mc = motorHandler.nimbusMotorController(MOTOR_SWITCH_PIN,
                                        MLTParams=LTParams,
                                        MLBParams=LBParams,
                                        MRTParams=RTParams,
                                        MRBParams=RBParams)
btn = Pin(BUTTON_PIN, Pin.IN)


def btnIRQ(pin):
    global mc
    mc.MSP(state=1 - mc.MSP())


btn.irq(trigger=Pin.IRQ_FALLING, handler=btnIRQ)

U#
# theta = 0
# while True:
#     mc.MVA(theta=theta, vel=65535/2)
#     theta += math.pi/60
#     time.sleep(0.08)
#     if theta > math.pi *2:
#         break
