from main import VibrationSignal
import numpy as np
import scipy.io as sio
from scipy.fftpack import fft,ifft,fftshift,ifftshift
import matplotlib.pyplot as plt


def real_signal():
    xdata = sio.loadmat('RMAM_50Hz_Load_5.mat').get('data')[int(4e5):int(4e5) + 20480 * 20, 1]  # 20s signal from vib_x
    ydata = sio.loadmat('RMAM_50Hz_Load_5.mat').get('data')[int(4e5):int(4e5) + 20480 * 20, 2]  # 20s signal from vib_x
    x = VibrationSignal(xdata,fs=20480)
    y = VibrationSignal(ydata,fs=20480)
    # x.acc2vel_fd()
    # x.visualize('v_data')
    x.acc2vel_td(detrend_type='poly')
    x.visualize('v_data')
    x.visualize('data')

def sin_signal():
    t = np.linspace(0,1,4096,endpoint=False)
    vel = 2 * np.pi * 50 * np.cos(2 * np.pi * 50 * t)
    acc = -(2 * np.pi * 50)**2. * np.sin(2 * np.pi * 50 * t)

    vel = VibrationSignal(vel, fs=4096)
    acc = VibrationSignal(acc, fs=4096)

    vel.visualize('data')
    acc.visualize('data')
    # acc.acc2vel_fd()
    # acc.visualize('v_data')
    # acc.acc2vel_td(detrend_type='poly')
    acc.acc2vel_td(detrend_type='poly')
    acc.visualize('v_data')

if __name__ == "__main__":

    # sin_signal()
    sin_signal()
