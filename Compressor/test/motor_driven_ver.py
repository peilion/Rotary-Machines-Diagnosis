# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Motor_Driven_Vertical
import scipy.io as sio
from simulators import unbalance, misalignment, a_loose, b_loose, rolling_bearing


def chlorinecompressor_motor_driven_end_vertical_diagnosis(xdata: ndarray, ydata: ndarray,
                                                           fs: int, R: float, bearing_ratio: ndarray,
                                                           thd_threshold: float, pd_threshold: float,
                                                           kurtosis_threshold: float,
                                                           ib_threshold: ndarray, ma_threshold: ndarray,
                                                           bw_threshold: ndarray,
                                                           al_threshold: ndarray, bl_threshold: ndarray,
                                                           harmonic_threshold: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    y = VibrationSignal(data=ydata, fs=fs, type=2)
    mp_instance = CP_Motor_Driven_Vertical(x=x, y=y, r=R, bearing_ratio=bearing_ratio,
                                           ib_threshold=ib_threshold, ma_threshold=ma_threshold,
                                           bw_threshold=bw_threshold, al_threshold=al_threshold,
                                           bl_threshold=bl_threshold, harmonic_threshold=harmonic_threshold,
                                           pd_threshold=pd_threshold,
                                           thd_threshold=thd_threshold, kurtosis_threshold=kurtosis_threshold)
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.vstack((mp_instance.x_env.freq, mp_instance.x_env.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_env.bearing_amp, \
           mp_instance.phase_diff, \
           mp_instance.x_vel.thd, \
           mp_instance.x.kurtosis, \
           mp_instance.harmonic_number, \
           mp_instance.ma_indicator, \
           mp_instance.ib_indicator, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           np.reshape(mp_instance.x_env.bearing_index, (3, 4))


if __name__ == '__main__':
    for index, item in enumerate((unbalance(AMP=300, FN=1491 / 60.0),
                                  misalignment(AMP=350, FN=1491 / 60.0),
                                  rolling_bearing(AMP=100, FN=1491 / 60.0, bearing_ratios=[5.81, 4.19, 2.99, 0.42]),
                                  a_loose(AMP=300, FN=1491 / 60.0),
                                  b_loose(AMP=100, FN=1491 / 60.0),)):
        res = chlorinecompressor_motor_driven_end_vertical_diagnosis(xdata=item[0],
                                                                     ydata=item[1],
                                                                     fs=51200,
                                                                     R=1491,
                                                                     bearing_ratio=np.array([5.81, 4.19, 2.99, 0.42]),
                                                                     pd_threshold=10 / 180 * np.pi,
                                                                     al_threshold=np.array([0.71, 4.5, 11.2]),
                                                                     bl_threshold=np.array([0.3, 2.2, 5.1]),
                                                                     bw_threshold=np.array([1, 2, 3]),
                                                                     harmonic_threshold=0.35 * np.ones((10,)),
                                                                     ib_threshold=np.array([0.71, 4.5, 11.2]),
                                                                     kurtosis_threshold=6.0,
                                                                     ma_threshold=np.array([0.71, 4.5, 11.2]),
                                                                     thd_threshold=1)
        assert (res[0][index * 3] == 1) & (res[0].sum() == 1)
