# encoding: utf-8
from numpy import ndarray
from base import VibrationSignal
from Blender.measure_points import BD_Motor_Driven_Vertical
from simulators import *


def blender_motor_driven_end_vertical_diagnosis(xdata: ndarray, ydata: ndarray,
                                                fs: int, R: float, bearing_ratio: ndarray, cur: float, th1: ndarray,
                                                th2: ndarray):
    th = th1 if (cur > 150) else th2

    x = VibrationSignal(data=xdata, fs=fs, type=2)
    y = VibrationSignal(data=ydata, fs=fs, type=2)

    mp_instance = BD_Motor_Driven_Vertical(x=x, y=y, r=R, bearing_ratio=bearing_ratio,
                                           ib_threshold=th[0:3],
                                           ma_threshold=th[3:6],
                                           bw_threshold=th[6:9],
                                           al_threshold=th[9:12],
                                           bl_threshold=th[12:15],
                                           thd_threshold=th[15],
                                           pd_threshold=th[16],
                                           kurtosis_threshold=th[17],
                                           harmonic_threshold=th[18:28])

    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.vstack((mp_instance.x_env.freq, mp_instance.x_env.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_env.bearing_amp, \
           abs(mp_instance.phase_diff) / np.pi * 180, \
           mp_instance.x_vel.thd, \
           mp_instance.ma_indicator, \
           mp_instance.ib_indicator, \
           mp_instance.x.kurtosis, \
           mp_instance.harmonic_number, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           np.reshape(mp_instance.x_env.bearing_index, (3, 4)), \
           {'unbalance': mp_instance.ib_threshold,
            'misalignment': mp_instance.ma_threshold,
            'bearing': mp_instance.bw_threshold,
            'atype_loosen': mp_instance.al_threshold,
            'btype_loosen': mp_instance.bl_threshold
            }


if __name__ == '__main__':
    data = np.loadtxt('Prepolymerizer.csv', delimiter=',', usecols=(1, 2))  # m/s2

    res = blender_motor_driven_end_vertical_diagnosis(xdata=1000 * data[:, 0],  # mm/s2
                                                      ydata=1000 * data[:, 1],
                                                      fs=25600, cur=200,
                                                      R=1486,
                                                      bearing_ratio=np.array([0.42, 2.99, 4.19, 5.81]),
                                                      th1=np.array([
                                                          10, 20, 30,
                                                          10, 20, 30,
                                                          4, 6, 10,
                                                          10, 20, 30,
                                                          10, 20, 30,
                                                          1, 10 / 180 * np.pi, 6.0,
                                                          10, 10, 10, 10, 10, 10, 10, 10, 10, 10
                                                      ]),
                                                      th2=np.array([
                                                          10, 20, 30,
                                                          10, 20, 30,
                                                          4, 6, 10,
                                                          10, 20, 30,
                                                          10, 20, 30,
                                                          1, 10 / 180 * np.pi, 6.0,
                                                          10, 10, 10, 10, 10, 10, 10, 10, 10, 10
                                                      ]),
                                                      )

