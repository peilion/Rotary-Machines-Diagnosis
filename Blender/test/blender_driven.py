# encoding: utf-8
from numpy import ndarray
import numpy as np
from base import VibrationSignal
from Blender.measure_points import BD_Blender_Driven
from simulators import *


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
                                    pd_threshold=th[17],
                                    harmonic_threshold=th[18:28],
                                    subharmonic_threshold=th[28:33]
                                    )
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
                            bearing_ratio=np.array([0.42, 2.99, 4.19, 5.81]),
                            teeth_num=np.array([28, 69, 12.5]),
                            th1=np.array([
                                10, 20, 30,
                                4, 6, 10,
                                10, 20, 30,
                                10, 20, 30,
                                10, 20, 30,
                                6.0, 1, 10 / 180 * np.pi,
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
                                6.0, 1, 10 / 180 * np.pi,
                                10, 10, 10, 10, 10,
                                10, 10, 10, 10, 10,
                                10, 10, 10, 10, 10,
                            ]),
                            )
