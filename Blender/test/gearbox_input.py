# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Blender.measure_points import BD_Gearbox_Input
from simulators import *


def chlorinecompressor_gearbox_inputshaft(xdata: ndarray, teeth_num: ndarray,
                                          fs: int, R: dict, bearing_ratio: ndarray, cur: float,
                                          kurtosis_threshold: float, bw_threshold: ndarray,
                                          ma_threshold: ndarray, gf_threshold: ndarray,
                                          al_threshold: ndarray, bl_threshold: ndarray,
                                          harmonic_threshold: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = BD_Gearbox_Input(x=x, y=x, r=R, gf_threshold=gf_threshold,
                                   ma_threshold=ma_threshold,
                                   al_threshold=al_threshold,
                                   bl_threshold=bl_threshold, harmonic_threshold=harmonic_threshold,
                                   teeth_num=teeth_num,
                                   kurtosis_threshold=kurtosis_threshold, bearing_ratio=bearing_ratio,
                                   bw_threshold=bw_threshold, cur=cur)
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_env.bearing_amp, \
           mp_instance.gf_indicator, \
           mp_instance.x.kurtosis, \
           mp_instance.harmonic_number, \
           mp_instance.ma_indicator, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           np.reshape(mp_instance.x_env.bearing_index, (3, 4)), \
           mp_instance.x.sideband_indexes, mp_instance.sideband_energies


if __name__ == '__main__':
    for index, item in enumerate((misalignment(AMP=400, FN=1486 / 60.0),
                                  rolling_bearing(AMP=100, FN=1486 / 60.0, bearing_ratios=[5.81, 4.19, 2.99, 0.42]),
                                  gear(AMP=300, FN=1486 / 60.0, MESH_RATIO=28),
                                  a_loose(AMP=300, FN=1486 / 60.0),
                                  b_loose(AMP=100, FN=1486 / 60.0),)):
        res = chlorinecompressor_gearbox_inputshaft(xdata=item[0],
                                                    fs=51200,
                                                    R={0: 990, 1: 1486},
                                                    al_threshold=np.array([[1.2, 4.5, 11.2], [1.2, 4.5, 11.2]]),
                                                    bl_threshold=np.array([[0.3, 2.2, 5.1], [0.3, 2.2, 5.1]]),
                                                    bw_threshold=np.array([[1, 2, 3], [1, 2, 3]]),
                                                    ma_threshold=np.array([[0.9, 4.5, 11.2], [0.9, 4.5, 11.2]]),
                                                    gf_threshold=np.array([[400, 600, 800], [400, 600, 800]]),
                                                    harmonic_threshold=np.vstack(
                                                        (0.35 * np.ones((10,)), 0.35 * np.ones((10,)))),
                                                    kurtosis_threshold=6.0,
                                                    teeth_num=np.array([28, 69]),
                                                    bearing_ratio=np.array([5.81, 4.19, 2.99, 0.42]),
                                                    cur=400)

        assert (res[0][index * 3] == 1) & (res[0].sum() == 1)
