# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Blender.measure_points import BD_Blender_Driven


def blender_diagnosis(xdata: ndarray, fs: int, R: float, bearing_ratio: ndarray,
                      cur: float, th1: ndarray, th2: ndarray, teeth_num: ndarray):
    th = th1 if (cur > 150) else th2

    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = BD_Blender_Driven(x=x, y=x, r=R, bearing_ratio=bearing_ratio,
                                    teeth_num=teeth_num,
                                    ib_threshold=th[0:3],
                                    bw_threshold=th[3:6],
                                    al_threshold=th[6:9],
                                    bl_threshold=th[9:12],
                                    rb_threshold=th[12:15],
                                    kurtosis_threshold=th[15],
                                    thd_threshold=th[16],
                                    pd_threshold=th[17] / 180 * np.pi,
                                    harmonic_threshold=th[18:28],
                                    subharmonic_threshold=th[28:33]
                                    )
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_vel.freq, mp_instance.x_vel.spec)), \
           np.vstack((mp_instance.x.freq, mp_instance.x.spec)), \
           np.vstack((mp_instance.x_env.freq, mp_instance.x_env.spec)), \
           np.hstack((mp_instance.bl_indicator, mp_instance.x_vel.harmonics)), \
           mp_instance.x_vel.sub_harmonics, \
           mp_instance.x_env.bearing_amp, \
           mp_instance.x_vel.thd, \
           mp_instance.ib_indicator, \
           mp_instance.harmonic_number, \
           mp_instance.sub_har_num, \
           np.hstack((mp_instance.x_vel.half_fr_indexes, mp_instance.x_vel.harmonics_index)), \
           np.transpose(np.reshape(mp_instance.x_env.bearing_index,(4,3)))  , \
           mp_instance.x_vel.sub_harmonics_index, \
           {'unbalance': mp_instance.ib_threshold,
            'bearing': mp_instance.bw_threshold,
            'atype_loosen': mp_instance.al_threshold,
            'btype_loosen': mp_instance.bl_threshold,
            'rubbing': mp_instance.rb_threshold
            }


if __name__ == '__main__':
    data = np.loadtxt('Prepolymerizer.csv', delimiter=',', usecols=(6))  # m/s2

    res = blender_diagnosis(xdata=1000 * data,  # mm/s2
                            fs=25600, cur=200,
                            R=1486,
                            bearing_ratio=np.array([0.45, 4.76, 11.2, 13.8]),
                            teeth_num=np.array([28, 69, 12.5]),
                            th1=np.array([
                                10, 20, 30,
                                200, 300, 400,
                                10, 20, 30,
                                10, 20, 30,
                                10, 20, 30,
                                6.0, 1, 10,
                                10, 10, 10, 10, 10,
                                10, 10, 10, 10, 10,
                                10, 10, 10, 10, 10,
                            ]),
                            th2=np.array([
                                10, 20, 30,
                                4, 6, 10,
                                10, 20, 30,
                                10, 20, 30,
                                10, 20, 30,
                                6.0, 1, 10 ,
                                10, 10, 10, 10, 10,
                                10, 10, 10, 10, 10,
                                10, 10, 10, 10, 10,
                            ]),
                            )

    import matplotlib.pyplot as plt

    plt.plot(res[3][0,:], res[3][1,:],)
    plt.xlim(0, 150)
    plt.xlabel('Frequency Hz')
    plt.ylabel('Amplitude mm/s2')
    # plt.axvline(x=res[1][0,:][res[11][2]], color='#000000', linewidth=0.3)
    # for item in res[11].flatten():
    #     plt.axvline(x=res[1][0,:][item], color='#000000', linewidth=0.3)

    for item in res[12].flatten():
        plt.axvline(x=res[1][0,:][item], color='#000000', linewidth=0.3)

    plt.show()

    np.savetxt('tmp.csv', res[12]*0.05,delimiter=',')

    np.savetxt('tmp_value.csv', res[12] * 0.05, delimiter=',')
    np.savetxt('tmp_index.csv', np.transpose(np.reshape(res[6],(4,3))) , delimiter=',')
