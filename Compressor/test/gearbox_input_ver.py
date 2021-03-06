# encoding: utf-8
from numpy import ndarray
from base import VibrationSignal
from Compressor.measure_points import CP_Gearbox_Input_Vertical
import numpy as np


def chlorinecompressor_gearbox_inputshaft_vertical(xdata: ndarray, teeth_num: ndarray,
                                                   fs: int, R: ndarray, th: ndarray, ):
    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = CP_Gearbox_Input_Vertical(x=x, y=x, r=R[0], teeth_num=teeth_num,
                                            gf_threshold=th[0:3],
                                            ma_threshold=th[3:6],
                                            wd_threshold=th[6:9],
                                            al_threshold=th[9:12],
                                            bl_threshold=th[12:15],
                                            kurtosis_threshold=th[15],
                                            harmonic_threshold=th[16:26], )

    mp_instance.diagnosis()
    # return mp_instance
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x_hp.freq, mp_instance.x_hp.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.ow_amp, \
           mp_instance.gf_indicator, \
           mp_instance.x_hp.kurtosis, \
           mp_instance.harmonic_number, \
           mp_instance.ma_indicator, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           mp_instance.x_vel.ow_index, \
           mp_instance.x_hp.sideband_indexes, \
           {'gear': mp_instance.gf_threshold,
            'misalignment': mp_instance.ma_threshold,
            'bearing': mp_instance.wd_threshold,
            'atype_loosen': mp_instance.al_threshold,
            'btype_loosen': mp_instance.bl_threshold
            }


if __name__ == '__main__':
    """    
    th = np.array([
        10, 20, 30,
        10, 20, 30,
        10, 20, 30,
        10, 20, 30,
        10, 20, 30,
        6.0,
        10, 10, 10, 10, 10, 10, 10, 10, 10, 10
    ])
    """
    data = np.loadtxt('Chlorine.csv', delimiter=',', usecols=(5))  # m/s2

    res = chlorinecompressor_gearbox_inputshaft_vertical(xdata=data*1000,
                                                         fs=25600,
                                                         R=np.array([1491.0, 10384.0]),
                                                         teeth_num=np.array([167, 28, 69]),
                                                         th=np.array([
                                                             10, 20, 30,
                                                             10, 20, 30,
                                                             -1, -0.5, -0.25,
                                                             10, 20, 30,
                                                             10, 20, 30,
                                                             6.0,
                                                             10, 10, 10, 10, 10, 10, 10, 10, 10, 10
                                                         ])
                                                         )

    # import matplotlib.pyplot as plt
    #
    # plt.plot(res[2][0,:], res[2][1,:],)
    # # plt.xlim(0, 250)
    # plt.xlabel('Frequency Hz')
    # plt.ylabel('Amplitude mm/s2')
    # # plt.axvline(x=res[1][0,:][res[9][1]], color='#000000', linewidth=0.3)
    # for item in res[11].flatten():
    #     plt.axvline(x=res[1][0,:][item], color='#000000', linewidth=0.3)
    # plt.show()
    #
    # np.savetxt('tmp.csv', res[11] * 0.05, delimiter=',')