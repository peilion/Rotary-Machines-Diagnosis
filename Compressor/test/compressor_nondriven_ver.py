# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Compressor_NonDriven_Vertical
import scipy.io as sio
from simulators import *


def chlorinecompressor_compressor_nondriven_end_vertical_diagnosis(xdata: ndarray, ydata: ndarray, pressure: ndarray,
                                                                   fs: int, R: float, rb_threshold: ndarray,
                                                                   thd_threshold: float, pd_threshold: float,
                                                                   wd_threshold: ndarray,
                                                                   ib_threshold: ndarray,
                                                                   sg_threshold: ndarray,
                                                                   subharmonic_threshold: ndarray,
                                                                   al_threshold: ndarray, bl_threshold: ndarray,
                                                                   harmonic_threshold: ndarray, pres_threshold):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    y = VibrationSignal(data=ydata, fs=fs, type=2)
    mp_instance = CP_Compressor_NonDriven_Vertical(x=x, y=y, r=R, pressure=pressure,
                                                   ib_threshold=ib_threshold,
                                                   al_threshold=al_threshold, bl_threshold=bl_threshold,
                                                   harmonic_threshold=harmonic_threshold, pd_threshold=pd_threshold,
                                                   thd_threshold=thd_threshold,
                                                   rb_threshold=rb_threshold, sg_threshold=sg_threshold,
                                                   subharmonic_threshold=subharmonic_threshold,
                                                   wd_threshold=wd_threshold, pres_threshold=pres_threshold, )
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.sub_harmonics, \
           mp_instance.lt_fr_amp, \
           mp_instance.phase_diff, \
           mp_instance.x_vel.thd, \
           mp_instance.ib_indicator, \
           mp_instance.harmonic_number, \
           mp_instance.x.ow_amp, \
           mp_instance.pres_std, \
           mp_instance.sub_har_num, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           mp_instance.x.ow_index, \
           mp_instance.x_vel.sub_harmonics_index, \
           mp_instance.sg_index


if __name__ == '__main__':
    for index, item in enumerate((unbalance(AMP=350, FN=10384 / 60.0),
                                  oil_whirl(AMP=100, FN=10384 / 60.0),
                                  rubbing(AMP=350, FN=10384 / 60.0),
                                  a_loose(AMP=400, FN=10384 / 60.0),
                                  b_loose(AMP=400, FN=10384 / 60.0),
                                  surge(AMP=300, FN=10384 / 60.0))):
        res = chlorinecompressor_compressor_nondriven_end_vertical_diagnosis(xdata=item[0],
                                                                             ydata=item[1],
                                                                             fs=51200,
                                                                             R=10384.0,
                                                                             pd_threshold=10 / 180 * np.pi,
                                                                             al_threshold=np.array([0.3, 4.5, 11.2]),
                                                                             bl_threshold=np.array([0.3, 2.2, 5.1]),
                                                                             harmonic_threshold=0.01 * np.ones((10,)),
                                                                             ib_threshold=np.array([0.1, 0.3, 0.5]),
                                                                             thd_threshold=1,
                                                                             pressure=np.ones((100,)),
                                                                             pres_threshold=[10, 20, 30],
                                                                             rb_threshold=np.array([4, 10, 13]),
                                                                             sg_threshold=np.array([100, 200, 300]),
                                                                             subharmonic_threshold=0.01 * np.ones((5,)),
                                                                             wd_threshold=np.array([75, 125, 175]))
        assert (res[0][index * 3] == 1) & (res[0].sum() == 1)
