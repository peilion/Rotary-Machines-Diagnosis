# encoding: utf-8
from numpy import ndarray
from base import VibrationSignal
from Compressor.measure_points import CP_Gearbox_Inner_Ring
from simulators import *

def chlorinecompressor_gearbox_inner_ring(xdata: ndarray, teeth_num: ndarray,
                                          fs: int, R: float,
                                          kurtosis_threshold: float,
                                          gf_threshold: ndarray):

    x = VibrationSignal(data=xdata, fs=fs, type=2)
    mp_instance = CP_Gearbox_Inner_Ring(x=x, y=x, r=R, gf_threshold=gf_threshold,
                                        teeth_num=teeth_num,
                                        kurtosis_threshold=kurtosis_threshold)
    mp_instance.diagnosis()
    return mp_instance.fault_num, \
           np.vstack((mp_instance.x_hp.freq, mp_instance.x_hp.spec)), \
           mp_instance.gf_indicator, \
           mp_instance.x_hp.kurtosis, \
           mp_instance.x_hp.sideband_indexes


if __name__ == '__main__':
    x,y = gear(AMP=300, FN=1491 / 60.0, MESH_RATIO=167)
    res = chlorinecompressor_gearbox_inner_ring(xdata=x,
                                                fs=51200,
                                                R=1491.0,
                                                kurtosis_threshold=6.0,
                                                gf_threshold=np.array([400, 600, 800]),
                                                teeth_num=np.array([28, 69, 167]),
                                                )
    assert (res[0][0] == 1) & (res[0].sum() == 1)