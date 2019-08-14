# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Compressor_NonDriven_Horizontal
from simulators import *


def chlorinecompressor_compressor_nondriven_end_horizontal_diagnosis(xdata: ndarray, ydata: ndarray, pressure: ndarray,
                                                                     fs: int, R: ndarray,
                                                                     th: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    y = VibrationSignal(data=ydata, fs=fs, type=2)
    mp_instance = CP_Compressor_NonDriven_Horizontal(x=x, y=y, r=R[1], pressure=pressure,
                                                     ib_threshold=th[0:3],
                                                     wd_threshold=th[3:6],
                                                     sg_threshold=th[6:9],
                                                     pres_threshold=th[9:12],
                                                     rb_threshold=th[12:15],
                                                     thd_threshold=th[15],
                                                     pd_threshold=th[16],
                                                     harmonic_threshold=th[17:27],
                                                     subharmonic_threshold=th[27:32],
                                                     )
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.hstack((mp_instance.x_vel.sub_harmonics[0], mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.sub_harmonics, \
           mp_instance.lt_fr_amp.max(), \
           abs(mp_instance.phase_diff) / np.pi * 180, \
           mp_instance.x_vel.thd, \
           mp_instance.harmonic_number, \
           mp_instance.x.ow_amp, \
           mp_instance.pres_std, \
           mp_instance.sub_har_num, \
           mp_instance.ib_indicator, \
           np.hstack((mp_instance.x_vel.sub_harmonics_index[0], mp_instance.x_vel.harmonics_index)), \
           mp_instance.x.ow_index, \
           mp_instance.x_vel.sub_harmonics_index, \
           mp_instance.sg_index, \
           {'unbalance': mp_instance.ib_threshold,
            'bearing': mp_instance.wd_threshold,
            'surge': mp_instance.sg_threshold,
            'rubbing': mp_instance.rb_threshold
            }


if __name__ == '__main__':
    data = np.loadtxt('Chlorine.csv', delimiter=',', usecols=(12, 13))  # m/s2

    res = chlorinecompressor_compressor_nondriven_end_horizontal_diagnosis(xdata=1000 * data[:, 1],  # mm/s2
                                                                           ydata=1000 * data[:, 0],
                                                                           fs=25600,
                                                                           R=np.array([1491.0, 10384.0]),
                                                                           pressure=np.ones(20),
                                                                           th=np.array([
                                                                               10, 20, 30,
                                                                               10, 20, 30,
                                                                               10, 20, 30,
                                                                               10, 20, 30,
                                                                               10, 20, 30,
                                                                               1, 10 / 180 * np.pi,
                                                                               10, 10, 10, 10, 10,
                                                                               10, 10, 10, 10, 10,
                                                                               10, 10, 10, 10, 10,
                                                                           ]))
