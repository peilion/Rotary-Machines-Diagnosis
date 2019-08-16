# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Blender.measure_points import BD_Motor_NonDriven


def prepolymerization_kettle_motor_nondriven_end(xdata: ndarray, th1: ndarray, th2: ndarray,
                                                 fs: int, R: float, bearing_ratio: ndarray, cur: ndarray):
    cur_mean = cur.mean()
    th = th1 if (cur_mean > 150) else th2

    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = BD_Motor_NonDriven(x=x, y=x, r=R, bearing_ratio=bearing_ratio,
                                     ib_threshold=th[0:3],
                                     bw_threshold=th[3:6],
                                     al_threshold=th[6:9],
                                     bl_threshold=th[9:12],
                                     thd_threshold=th[12],
                                     pd_threshold=th[13] / 180 * np.pi,
                                     kurtosis_threshold=th[14],
                                     harmonic_threshold=th[15:25],
                                     )
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.vstack((mp_instance.x_env.freq, mp_instance.x_env.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_env.bearing_amp, \
           abs(mp_instance.phase_diff) / np.pi * 180, \
           mp_instance.x_vel.thd, \
           mp_instance.x.kurtosis, \
           mp_instance.harmonic_number, \
           mp_instance.ib_indicator, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           np.transpose(np.reshape(mp_instance.x_env.bearing_index, (4, 3))), \
           {'unbalance': mp_instance.ib_threshold,
            'bearing': mp_instance.bw_threshold,
            'atype_loosen': mp_instance.al_threshold,
            'btype_loosen': mp_instance.bl_threshold
            }


if __name__ == '__main__':
    data = np.loadtxt('Prepolymerizer.csv', delimiter=',', usecols=(3))  # m/s2

    res = prepolymerization_kettle_motor_nondriven_end(xdata=1000 * data,  # mm/s2
                                                       fs=25600, cur=200 * np.ones(10),
                                                       R=1486,
                                                       bearing_ratio=np.array([0.39, 2.1, 3.1, 4.9]),
                                                       th1=np.array([
                                                           10, 20, 30,
                                                           4, 6, 10,
                                                           10, 20, 30,
                                                           10, 20, 30,
                                                           1, 10, 6.0,
                                                           10, 10, 10, 10, 10, 10, 10, 10, 10, 10
                                                       ]),
                                                       th2=np.array([
                                                           10, 20, 30,
                                                           4, 6, 10,
                                                           10, 20, 30,
                                                           10, 20, 30,
                                                           1, 10, 6.0,
                                                           10, 10, 10, 10, 10, 10, 10, 10, 10, 10
                                                       ]),
                                                       )
    # import matplotlib.pyplot as plt
    #
    # plt.plot(res[1][0, :], res[1][1, :], )
    # plt.xlim(0, 250)
    # plt.xlabel('Frequency Hz')
    # plt.ylabel('Amplitude mm/s')
    # # plt.axvline(x=res[1][0,:][res[11][1]], color='#000000', linewidth=0.3)
    # for item in res[11].flatten():
    #     plt.axvline(x=res[3][0, :][item], color='#000000', linewidth=0.3)
    # plt.show()
    #
    # np.savetxt('tmp_value.csv', res[12] * 0.05, delimiter=',')
    # np.savetxt('tmp_index.csv', np.transpose(np.reshape(res[5], (4, 3))), delimiter=',')
