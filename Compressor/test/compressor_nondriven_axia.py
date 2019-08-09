# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Compressor_NonDriven_Axial
import scipy.io as sio
from simulators import *


def chlorinecompressor_compressor_nondriven_end_axial_diagnosis(xdata: ndarray,
                                                                fs: int, R: float, rb_threshold: ndarray,
                                                                wd_threshold: ndarray,
                                                                subharmonic_threshold: ndarray,
                                                                harmonic_threshold: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = CP_Compressor_NonDriven_Axial(x=x, y=x, r=R,
                                                harmonic_threshold=harmonic_threshold,
                                                rb_threshold=rb_threshold,
                                                subharmonic_threshold=subharmonic_threshold,
                                                wd_threshold=wd_threshold, )
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.hstack((0, mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.sub_harmonics, \
           mp_instance.harmonic_number, \
           mp_instance.sub_har_num, \
           mp_instance.x.ow_amp, \
           mp_instance.x.ow_index, \
           mp_instance.x_vel.sub_harmonics_index, \
           np.hstack((0, mp_instance.x_vel.harmonics_index))


if __name__ == '__main__':
    for index, item in enumerate((oil_whirl(AMP=100, FN=10384 / 60.0),
                                  rubbing(AMP=350, FN=10384 / 60.0),)):
        res = chlorinecompressor_compressor_nondriven_end_axial_diagnosis(xdata=item[0],
                                                                          fs=51200,
                                                                          R=10384.0,
                                                                          harmonic_threshold=0.01 * np.ones(
                                                                              (10,)),
                                                                          rb_threshold=np.array([4, 10, 13]),
                                                                          subharmonic_threshold=0.01 * np.ones(
                                                                              (5,)),
                                                                          wd_threshold=np.array(
                                                                              [75, 125, 175]))
        assert (res[0][index * 3] == 1) & (res[0].sum() == 1)
