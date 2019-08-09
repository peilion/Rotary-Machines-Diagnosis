import numpy as np

class Blender(object):
    sn = 'PR31101'
    power = 200  # kW
    teeth_num = np.array([28,69])
    motor_blade = 5
    blender_blade = 5

    d_bpfi = 7.646
    d_bpfo = 5.354
    d_bsf = 5.497
    d_ftf = 0.41187

    nd_bpfi = 4.9
    nd_bpfo = 3.1
    nd_bsf = 2.1
    nd_ftf = 0.39

    bd_bpfi = 13.8
    bd_bpfo = 11.2
    bd_bsf = 4.76
    bd_ftf = 0.45

    opeartion_modes = {
        0:'blow',
        1:'mix'
    }

    def __init__(self, motor_rpm=None, blade_rpm=None):
        if motor_rpm is None:
            self.motor_rpm = {'blow': 990, 'mix': 1486}
        if blade_rpm is None:
            self.blade_rpm = {'blow': 80, 'mix': 120}
