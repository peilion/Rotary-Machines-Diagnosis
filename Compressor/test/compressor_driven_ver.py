# encoding: utf-8
from numpy import ndarray
from base import VibrationSignal
from Compressor.measure_points import CP_Compressor_Driven_Vertical
import numpy as np


def chlorinecompressor_compressor_driven_end_vertical_diagnosis(xdata: ndarray, ydata: ndarray, pressure: ndarray,
                                                                fs: int, R: ndarray, th: ndarray):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    y = VibrationSignal(data=ydata, fs=fs, type=2)
    mp_instance = CP_Compressor_Driven_Vertical(x=x, y=y, r=R[1], pressure=pressure,
                                                ib_threshold=th[0:3],
                                                ma_threshold=th[3:6],
                                                wd_threshold=th[6:9],
                                                al_threshold=th[9:12],
                                                bl_threshold=th[12:15],
                                                sg_threshold=th[15:18],
                                                pres_threshold=th[18:21],
                                                rb_threshold=th[21:24],
                                                thd_threshold=th[24],
                                                pd_threshold=th[25] / 180 * np.pi,
                                                harmonic_threshold=th[26:36],
                                                subharmonic_threshold=th[36:41],
                                                )
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.sub_harmonics, \
           mp_instance.lt_fr_amp.max(), \
           abs(mp_instance.phase_diff) / np.pi * 180, \
           mp_instance.x_vel.thd, \
           mp_instance.ma_indicator, \
           mp_instance.ib_indicator, \
           mp_instance.harmonic_number, \
           mp_instance.x.ow_amp, \
           mp_instance.pres_std, \
           mp_instance.sub_har_num, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           mp_instance.x.ow_index, \
           mp_instance.x_vel.sub_harmonics_index, \
           mp_instance.sg_index, \
           {'unbalance': mp_instance.ib_threshold,
            'misalignment': mp_instance.ma_threshold,
            'bearing': mp_instance.wd_threshold,
            'atype_loosen': mp_instance.al_threshold,
            'btype_loosen': mp_instance.bl_threshold,
            'surge': mp_instance.sg_threshold,
            'rubbing': mp_instance.rb_threshold
            }


if __name__ == '__main__':
    data = np.loadtxt('Chlorine.csv', delimiter=',', usecols=(10, 11))  # m/s2

    res = chlorinecompressor_compressor_driven_end_vertical_diagnosis(xdata=1000 * data[:, 0],  # mm/s2
                                                                      ydata=1000 * data[:, 1],
                                                                      fs=25600,
                                                                      R=np.array([1491.0, 10384.0]),
                                                                      pressure=np.ones(20),
                                                                      th=np.array([
                                                                          10, 20, 30,
                                                                          200, 300, 400,
                                                                          200, 300, 400,
                                                                          10, 20, 30,
                                                                          10, 20, 30,
                                                                          200, 300, 400,
                                                                          10, 20, 30,
                                                                          10, 20, 30,
                                                                          1, 10,
                                                                          10, 10, 10, 10, 10,
                                                                          10, 10, 10, 10, 10,
                                                                          10, 10, 10, 10, 10,
                                                                      ]))

    # import matplotlib.pyplot as plt
    #
    # plt.plot(res[2][0,:], res[2][1,:],)
    # plt.xlim(0, 2000)
    # plt.xlabel('Frequency Hz')
    # plt.ylabel('Amplitude mm/s2')
    # # plt.axvline(x=res[1][0,:][res[15]], color='#000000', linewidth=0.3)
    # # for item in res[17].flatten():
    # #     plt.axvline(x=res[1][0,:][item], color='#000000', linewidth=0.3)
    # # for item in res[16].flatten():
    # #     plt.axvline(x=res[1][0,:][item], color='#000000', linewidth=0.3)
    # plt.show()
    #
    # np.savetxt('tmp.csv', res[11]*0.05,delimiter=',')