# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Compressor_NonDriven_Axial
import scipy.io as sio
from simulators import *


def chlorinecompressor_compressor_nondriven_end_axial_diagnosis(xdata: ndarray,
                                                                fs: int, R: ndarray, th: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = CP_Compressor_NonDriven_Axial(x=x, y=x, r=R[1],
                                                wd_threshold=th[0:3],
                                                rb_threshold=th[3:6],
                                                harmonic_threshold=th[6:16],
                                                subharmonic_threshold=th[16:21])
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.hstack((0, mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.sub_harmonics, \
           mp_instance.sub_har_num, \
           mp_instance.harmonic_number, \
           mp_instance.x.ow_amp, \
           np.hstack((0, mp_instance.x_vel.harmonics_index)), \
           mp_instance.x.ow_index, \
           mp_instance.x_vel.sub_harmonics_index, \
           {'bearing': mp_instance.wd_threshold,
            'rubbing': mp_instance.rb_threshold
            }


if __name__ == '__main__':
    data = np.loadtxt('Chlorine.csv', delimiter=',', usecols=(14))  # m/s2

    res = chlorinecompressor_compressor_nondriven_end_axial_diagnosis(xdata=1000 * data,  # mm/s2
                                                                      fs=25600,
                                                                      R=np.array([1491.0, 10384.0]),
                                                                      th=np.array([
                                                                          10, 20, 30,
                                                                          10, 20, 30,
                                                                          10, 10, 10, 10, 10,
                                                                          10, 10, 10, 10, 10,
                                                                          10, 10, 10, 10, 10,
                                                                      ]))
