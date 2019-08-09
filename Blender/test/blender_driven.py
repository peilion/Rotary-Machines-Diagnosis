# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Blender.measure_points import BD_Blender_Driven
from simulators import *


def blender_diagnosis(xdata: ndarray, fs: int, R: dict, bearing_ratio: ndarray, cur: float,
                      kurtosis_threshold: float, bw_threshold: ndarray, pd_threshold: ndarray,
                      rb_threshold: ndarray, al_threshold: ndarray, bl_threshold: ndarray,
                      ib_threshold: ndarray, subharmonic_threshold: ndarray, thd_threshold: float,
                      harmonic_threshold: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = BD_Blender_Driven(x=x, y=x, r=R,
                                    al_threshold=al_threshold,
                                    bl_threshold=bl_threshold, harmonic_threshold=harmonic_threshold,
                                    kurtosis_threshold=kurtosis_threshold, bearing_ratio=bearing_ratio,
                                    bw_threshold=bw_threshold, cur=cur, ib_threshold=ib_threshold,
                                    pd_threshold=pd_threshold, rb_threshold=rb_threshold,
                                    subharmonic_threshold=subharmonic_threshold, thd_threshold=thd_threshold)
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.sub_harmonics, \
           mp_instance.x_env.bearing_amp, \
           mp_instance.x_vel.thd, \
           mp_instance.ib_indicator, \
           mp_instance.harmonic_number, \
           mp_instance.sub_har_num, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           np.reshape(mp_instance.x_env.bearing_index, (3, 4)), \
           mp_instance.x_vel.sub_harmonics_index,


if __name__ == '__main__':
    for index, item in enumerate((unbalance(AMP=300, FN=120 / 60.0),
                                  rolling_bearing(AMP=100, FN=120 / 60.0, bearing_ratios=[5.81, 4.19, 2.99, 0.42]),
                                  rubbing(AMP=300, FN=120 / 60.0),
                                  a_loose(AMP=300, FN=120 / 60.0),
                                  b_loose(AMP=100, FN=120 / 60.0),)):
        res = blender_diagnosis(xdata=item[0],
                                fs=51200,
                                R={0: 80, 1: 120},
                                al_threshold=np.array([[20, 40, 60], [20, 40, 60]]),
                                bl_threshold=np.array([[14, 26, 38], [14, 26, 38]]),
                                bw_threshold=np.array([[1, 2, 3], [1, 2, 3]]),
                                ib_threshold=np.array([[20, 40, 60], [20, 40, 60]]),
                                pd_threshold=10 / 180 * np.pi,
                                rb_threshold=np.array([[7, 10, 13], [7, 10, 13]]),
                                subharmonic_threshold=np.vstack(
                                    (0.71 * np.ones((5,)), 0.71 * np.ones((5,)))),
                                harmonic_threshold=np.vstack(
                                    (0.35 * np.ones((10,)), 0.35 * np.ones((10,)))),
                                kurtosis_threshold=6.0,
                                bearing_ratio=np.array([5.81, 4.19, 2.99, 0.42]),
                                cur=400,
                                thd_threshold=1)

        if index == 0 or index == 3 :
            assert (res[0][0] == 1) & (res[0][9] == 1) & (res[0].sum() == 2)
        else:
            assert (res[0][index * 3] == 1) & (res[0].sum() == 1)