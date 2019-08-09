import numpy as np

class Compressor(object):
    sn = 'C-501'
    power = 580  # kW
    teeth_num = np.array([28,69,167])
    motor_blade = 5
    compressor_blade = 5

    d_bpfi = 5.81
    d_bpfo = 4.19
    d_bsf = 2.99
    d_ftf = 0.42

    nd_bpfi = 5.29
    nd_bpfo = 3.71
    nd_bsf = 2.75
    nd_ftf = 0.41

    def __init__(self, motor_rpm=1491, blade_rpm=10384):
        self.motor_rpm = motor_rpm
        self.blade_rpm = blade_rpm
