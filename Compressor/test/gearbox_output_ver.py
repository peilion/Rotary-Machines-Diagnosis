# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Gearbox_Output_Vertical
import scipy.io as sio
from simulators import *


def chlorinecompressor_gearbox_outputshaft_vertical(xdata: ndarray, teeth_num: ndarray,
                                                    fs: int, R: float,
                                                    kurtosis_threshold: float,
                                                    ma_threshold: ndarray, gf_threshold: ndarray,
                                                    wd_threshold: ndarray,
                                                    al_threshold: ndarray, bl_threshold: ndarray,
                                                    harmonic_threshold: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = CP_Gearbox_Output_Vertical(x=x, y=x, r=R, wd_threshold=wd_threshold, gf_threshold=gf_threshold,
                                             ma_threshold=ma_threshold,
                                             al_threshold=al_threshold,
                                             bl_threshold=bl_threshold, harmonic_threshold=harmonic_threshold,
                                             teeth_num=teeth_num,
                                             kurtosis_threshold=kurtosis_threshold)
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           mp_instance.ma_indicator, \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_lp.harmonics)), \
           mp_instance.x_lp.ow_amp, \
           mp_instance.gf_indicator, \
           mp_instance.x.kurtosis, \
           mp_instance.harmonic_number, \
           np.hstack((mp_instance.x_lp.half_fr_indexes, mp_instance.x_lp.harmonics_index)), \
           mp_instance.x_lp.ow_index, \
           mp_instance.x_hp.sideband_indexes,


if __name__ == '__main__':
    for index, item in enumerate((misalignment(AMP=350, FN=10384 / 60.0),
                                  oil_whirl(AMP=100, FN=10384 / 60.0),
                                  gear(AMP=300, FN=10384 / 6.964 / 60.0, MESH_RATIO=167),
                                  a_loose(AMP=300, FN=10384 / 60.0),
                                  b_loose(AMP=100, FN=10384 / 60.0))):
        res = chlorinecompressor_gearbox_outputshaft_vertical(xdata=item[0],
                                                              fs=51200,
                                                              R=10384.0,
                                                              al_threshold=np.array([200, 400, 600]),
                                                              bl_threshold=np.array([75, 125, 175]),
                                                              harmonic_threshold=0.71 * np.ones((10,)),
                                                              kurtosis_threshold=6.0,
                                                              ma_threshold=np.array([150, 200, 250]),
                                                              gf_threshold=np.array([400, 600, 800]),
                                                              teeth_num=np.array([28, 69, 167]),
                                                              wd_threshold=np.array([75, 125, 175]))
        assert (res[0][index * 3] == 1 )& (res[0].sum() == 1)