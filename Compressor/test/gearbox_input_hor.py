# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Gearbox_Input_Horizontal
from simulators import *

def chlorinecompressor_gearbox_inputshaft_horizontal(xdata: ndarray, teeth_num: ndarray,
                                                     fs: int, R: float,
                                                     kurtosis_threshold: float,
                                                     ma_threshold: ndarray, gf_threshold: ndarray,
                                                     wd_threshold: ndarray):

    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = CP_Gearbox_Input_Horizontal(x=x, y=x, r=R, wd_threshold=wd_threshold, gf_threshold=gf_threshold,
                                              ma_threshold=ma_threshold,
                                              teeth_num=teeth_num,
                                              kurtosis_threshold=kurtosis_threshold)
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x_hp.freq, mp_instance.x_hp.spec)), \
           np.hstack((0,mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.ow_amp, \
           mp_instance.gf_indicator, \
           mp_instance.x_hp.kurtosis, \
           mp_instance.ma_indicator, \
           np.hstack((0, mp_instance.x_vel.harmonics_index)), \
           mp_instance.x_vel.ow_index, \
           mp_instance.x_hp.sideband_indexes


if __name__ == '__main__':
    for index, item in enumerate((misalignment(AMP=350, FN=1491 / 60.0),
                                  oil_whirl(AMP=100, FN=1491 / 60.0),
                                  gear(AMP=300, FN=1491 / 60.0, MESH_RATIO=167))):
        res = chlorinecompressor_gearbox_inputshaft_horizontal(xdata=item[0],
                                                               fs=51200,
                                                               R=1491.0,
                                                               kurtosis_threshold=6.0,
                                                               ma_threshold=np.array([0.71, 4.5, 11.2]),
                                                               gf_threshold=np.array([400, 600, 800]),
                                                               teeth_num=np.array([28, 69, 167]),
                                                               wd_threshold=np.array([0.71, 4.5, 11.2]))
        assert (res[0][index*3] == 1) &  (res[0].sum() == 1)