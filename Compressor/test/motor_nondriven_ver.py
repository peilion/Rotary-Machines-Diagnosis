# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Compressor.measure_points import CP_Motor_NonDriven_Vertical


def chlorinecompressor_motor_nondriven_end_vertical_diagnosis(xdata: ndarray, ydata: ndarray,
                                                              fs: int, R: ndarray, bearing_ratio: ndarray, th: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    y = VibrationSignal(data=ydata, fs=fs, type=2)
    mp_instance = CP_Motor_NonDriven_Vertical(x=x, y=y, r=R[0], bearing_ratio=bearing_ratio,
                                              ib_threshold=th[0:3],
                                              bw_threshold=th[3:6],
                                              al_threshold=th[6:9],
                                              bl_threshold=th[9:12],
                                              thd_threshold=th[12], pd_threshold=th[13] / 180.0 * np.pi,
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
    data = np.loadtxt('Chlorine.csv', delimiter=',', usecols=(3, 4))  # m/s2

    res = chlorinecompressor_motor_nondriven_end_vertical_diagnosis(xdata=1000 * data[:, 0],  # mm/s2
                                                                    ydata=1000 * data[:, 1],
                                                                    fs=25600,
                                                                    R=np.array([1491.0, 10384.0]),
                                                                    bearing_ratio=np.array([0.41, 2.75, 3.71, 5.29]),
                                                                    th=np.array([
                                                                        10, 20, 30,
                                                                        4, 6, 10,
                                                                        10, 20, 30,
                                                                        10, 20, 30,
                                                                        1, 10, 6.0,
                                                                        10, 10, 10, 10, 10, 10, 10, 10, 10, 10
                                                                    ]))

    # import matplotlib.pyplot as plt
    #
    # plt.plot(res[3][0,:], res[3][1,:],)
    # plt.xlim(0, 500)
    # plt.xlabel('Frequency Hz')
    # plt.ylabel('Amplitude mm/s2')
    # # plt.axvline(x=res[1][0,:][res[11][2]], color='#000000', linewidth=0.3)
    # for item in res[12].flatten():
    #     plt.axvline(x=res[3][0,:][item], color='#000000', linewidth=0.3)
    # plt.show()
