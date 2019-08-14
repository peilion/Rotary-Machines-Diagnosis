# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Gearbox_Output_Vertical
import scipy.io as sio
from simulators import *


def chlorinecompressor_gearbox_outputshaft_vertical(xdata: ndarray, teeth_num: ndarray,
                                                    fs: int, R: ndarray, th: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = CP_Gearbox_Output_Vertical(x=x, y=x, r=R[1], teeth_num=teeth_num,
                                             gf_threshold=th[0:3],
                                             ma_threshold=th[3:6],
                                             wd_threshold=th[6:9],
                                             al_threshold=th[9:12],
                                             bl_threshold=th[12:15],
                                             kurtosis_threshold=th[15],
                                             harmonic_threshold=th[16:26],
                                             )
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_lp.freq, mp_instance.x_lp.spec)), \
           np.vstack((mp_instance.x_hp.freq, mp_instance.x_hp.spec)), \
           mp_instance.ma_indicator, \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_lp.harmonics)), \
           mp_instance.x_lp.ow_amp, \
           mp_instance.gf_indicator, \
           mp_instance.x.kurtosis, \
           mp_instance.harmonic_number, \
           np.hstack((mp_instance.x_lp.half_fr_indexes, mp_instance.x_lp.harmonics_index)), \
           mp_instance.x_lp.ow_index, \
           mp_instance.x_hp.sideband_indexes, \
           {'gear': mp_instance.gf_threshold,
            'misalignment': mp_instance.ma_threshold,
            'bearing': mp_instance.wd_threshold,
            'atype_loosen': mp_instance.al_threshold,
            'btype_loosen': mp_instance.bl_threshold
            }


if __name__ == '__main__':
    data = np.loadtxt('Chlorine.csv', delimiter=',', usecols=(8))  # m/s2

    res = chlorinecompressor_gearbox_outputshaft_vertical(xdata=data,
                                                          fs=25600,
                                                          R=np.array([1491.0, 10384.0]),
                                                          teeth_num=np.array([167, 28, 69]),
                                                          th=np.array([
                                                              10, 20, 30,
                                                              10, 20, 30,
                                                              10, 20, 30,
                                                              10, 20, 30,
                                                              10, 20, 30,
                                                              6.0,
                                                              10, 10, 10, 10, 10, 10, 10, 10, 10, 10
                                                          ])
                                                          )
