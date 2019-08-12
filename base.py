# encoding: utf-8

import matplotlib.pyplot as plt
import numpy as np
from numpy import ndarray
from scipy import signal
from scipy.integrate import cumtrapz
from scipy.signal import detrend
from scipy import stats


class VibrationSignal:
    type_mapper = {
        0: 'Displacement',
        1: 'Velocity',
        2: 'Acceleration',
        3: 'Envelope'
    }

    def __init__(self, data: ndarray, fs: int, type=2, isdetrend=False):
        """

        :param data:
        :param fs:
        :param type: 0:位移 1:速度 2:加速度 3:加速度包络
        :param isdetrend:
        """
        if not isdetrend:  # 默认未消除趋势
            self.data = self.linear_detrend(data)
        else:
            self.data = data
        self.sampling_rate = fs
        self.time_vector = np.linspace(0.0, len(self.data) / self.sampling_rate, len(self.data))
        self.N = len(data)
        self.nyq = 1 / 2 * self.sampling_rate
        self.type = type
        self._kurtosis = None

    # def acc2vel_fd(self):
    #     data = self.data
    #     N = len(data)
    #     df = 1.0/(N*1/self.sampling_rate)
    #     nyq = 1/(2*1/self.sampling_rate)
    #     iomega_array = 1j*2*np.pi*np.linspace(-nyq,nyq,int(2*nyq/df))
    #     A = fftshift(fft(data))
    #     A = A * iomega_array
    #     A = ifftshift(A)
    #     self.v_data = np.real(ifft(A))
    #
    # def acc2vel_wj(self):
    #     data = self.data
    #     dt = self.time_vector[1] - self.time_vector[0]
    #     baseline = np.mean(data)
    #     datafft = np.fft.fft(data - baseline)
    #     datafreq = np.fft.fftfreq(len(data), dt)
    #     datafreq = [(i + 1) * datafreq[1] for i in range(len(datafft))]
    #     self.v_data = np.real(np.fft.ifft(datafft / (2. * np.pi * np.abs(datafreq)))) + baseline

    def to_velocity(self, detrend_type='poly'):
        data = cumtrapz(self.data, self.time_vector)
        detrend_meth = getattr(self, detrend_type + '_detrend')
        return VibrationSignal(detrend_meth(data), fs=self.sampling_rate, type=self.type - 1)

    def to_filted_signal(self, filter_type: str, co_frequency):
        b, a = signal.butter(8, co_frequency, filter_type)
        data = signal.filtfilt(b, a, self.data)
        return VibrationSignal(data=data, fs=self.sampling_rate, type=self.type)

    def to_envelope(self):
        return VibrationSignal(data=np.abs(signal.hilbert(self.data)), fs=self.sampling_rate, type=self.type + 1)

    def visualize(self, *args):
        """
        :param args: 'data','v_data' 中的一个或多个
        """
        for item in args:
            data = getattr(self, item)
            if data is not None:
                plt.plot(data)
                plt.show()

    def compute_harmonics(self, fr: float, upper: int, tolerance=0.5):
        assert self.spec is not None, '需先计算频谱'
        spec = self.spec
        freq = self.freq
        df = freq[1] - freq[0]

        nfr_indexes = [i * fr for i in range(1, upper)]
        harmonics = []
        harmonics_index = []

        for nfr_index in nfr_indexes:
            upper_search = np.rint((nfr_index + tolerance) / df).astype(np.int)
            lower_search = np.rint((nfr_index - tolerance) / df).astype(np.int)

            nth_harmonic_index = lower_search + np.argmax(spec[lower_search:upper_search])
            nth_harmonic = spec[nth_harmonic_index]

            harmonics_index.append(nth_harmonic_index)
            harmonics.append(nth_harmonic)

        self.harmonics = np.array(harmonics)
        self.harmonics_index = np.array(harmonics_index)

        self.thd = np.sqrt((self.harmonics[1:] ** 2).sum()) / self.harmonics[0]

    def compute_sub_harmonic(self, fr: float, upper=10,tolerance = 0.5):
        assert self.spec is not None, '需先计算频谱'
        spec = self.spec
        freq = self.freq
        df = freq[1] - freq[0]

        subhar_frequencies = [i * fr / 2 for i in range(1, upper, 2)]
        harmonics = []
        harmonics_index = []

        for subhar_frequency in subhar_frequencies:
            upper_search = np.rint((subhar_frequency + tolerance) / df).astype(np.int)
            lower_search = np.rint((subhar_frequency - tolerance) / df).astype(np.int)

            nth_harmonic_index = lower_search + np.argmax(spec[lower_search:upper_search])
            nth_harmonic = spec[nth_harmonic_index]

            harmonics_index.append(nth_harmonic_index)
            harmonics.append(nth_harmonic)

        self.sub_harmonics = np.array(harmonics)
        self.sub_harmonics_index = np.array(harmonics_index)

    def get_band_energy(self, fr: float, band_range: tuple):
        assert self.spec is not None, '需先计算频谱'
        assert len(band_range) == 2, '频带设置错误'
        df = self.freq[1] - self.freq[0]
        upper_search = np.rint(band_range[1] * fr / df).astype(np.int)
        lower_search = np.rint(band_range[0] * fr / df).astype(np.int)
        search_range = self.spec[lower_search:upper_search]
        return lower_search + np.argmax(search_range), \
               np.max(search_range)

    def compute_spectrum(self):
        spec = np.fft.fft(self.data)[0:int(self.N / 2)] / self.N
        spec[1:] = 2 * spec[1:]
        self.spec = np.abs(spec)
        self.freq = np.fft.fftfreq(self.N, 1.0 / self.sampling_rate)[0:int(self.N / 2)]

    def compute_bearing_frequency(self, bpfi, bpfo, bsf, ftf, fr, upper=3, tolerance=0.5):
        assert self.spec is not None, '需先计算频谱'
        spec = self.spec
        freq = self.freq
        df = freq[1] - freq[0]

        bearing_index = []
        bearing_amp = []
        for component in [bpfi, bpfo, bsf, ftf]:
            for i in range(1, upper + 1):
                upper_search = np.rint((component * fr * i + tolerance) / df).astype(np.int)
                lower_search = np.rint((component * fr * i - tolerance) / df).astype(np.int)

                tmp_index = lower_search + np.argmax(spec[lower_search:upper_search])
                tmp_amp = spec[tmp_index]

                bearing_index.append(tmp_index)
                bearing_amp.append(tmp_amp)
        self.bearing_index = np.array(bearing_index)
        self.bearing_amp = np.array(bearing_amp)

    def compute_half_harmonic(self, fr, tolerance=0.5):
        spec = self.spec
        freq = self.freq
        df = freq[1] - freq[0]

        half_fr_indexes = fr / 2

        upper_search = np.rint((half_fr_indexes + tolerance) / df).astype(np.int)
        lower_search = np.rint((half_fr_indexes - tolerance) / df).astype(np.int)

        self.half_fr_indexes = lower_search + np.argmax(spec[lower_search:upper_search])
        self.half_fr_amp = spec[self.half_fr_indexes]

    def compute_mesh_frequency(self, fr, mesh_ratio, sideband_order=6, upper_order=3, tolerance=0.5):
        spec = self.spec
        freq = self.freq
        df = freq[1] - freq[0]
        mesh_frequencies = [mesh_ratio * fr * i for i in range(1, upper_order + 1)]
        self.sideband_amps = []
        self.sideband_indexes = []
        for mesh_frequency in mesh_frequencies:
            for i in range(-sideband_order, sideband_order + 1):
                frequecy = mesh_frequency + i * fr
                upper_search = np.rint((frequecy + tolerance) / df).astype(np.int)
                lower_search = np.rint((frequecy - tolerance) / df).astype(np.int)
                sideband_index = lower_search + np.argmax(spec[lower_search:upper_search])
                sideband_amp = spec[sideband_index]
                self.sideband_indexes.append(sideband_index)
                self.sideband_amps.append(sideband_amp)
        self.sideband_amps = np.reshape(self.sideband_amps, (upper_order, sideband_order * 2 + 1))
        self.sideband_indexes = np.reshape(self.sideband_indexes, (upper_order, sideband_order * 2 + 1))

    def compute_oilwhirl_frequency(self, fr):
        spec = self.spec
        freq = self.freq
        df = freq[1] - freq[0]

        ow_frequency_lower = fr * 0.45
        ow_frequency_upper = fr * 0.48

        lower_search = np.rint(ow_frequency_lower / df).astype(np.int)
        upper_search = np.rint(ow_frequency_upper / df).astype(np.int)

        self.ow_index = lower_search + np.argmax(spec[lower_search:upper_search])

        self.ow_amp = spec[self.ow_index]

    @staticmethod
    def linear_detrend(data):
        return detrend(data, type='linear')

    @staticmethod
    def const_detrend(data):
        return detrend(data, type='const')

    @staticmethod
    def poly_detrend(data):
        x = np.arange(len(data))
        fit = np.polyval(np.polyfit(x, data, deg=3), x)
        data -= fit
        return data

    @staticmethod
    def diff_detrend(data):
        return np.diff(data)

    @property
    def kurtosis(self):
        if self._kurtosis is None:
            self._kurtosis = stats.kurtosis(self.data)
        return self._kurtosis

    def __repr__(self):
        return "{0} Signal with a size of {1}, and the sampling rate is {2}.".format(self.type_mapper[type],
                                                                                     len(self.data),
                                                                                     self.sampling_rate)


class MeasurePoint:
    fault_num_mapper = {
        0: [0, 0, 0],
        1: [1, 0, 0],
        2: [0, 1, 0],
        3: [0, 0, 1]
    }

    equip = None
    x = None
    y = None
    # should be specified in sub classes
    require_phase_diff = True

    def __init__(self, x: VibrationSignal, y: VibrationSignal, r: float):
        self.x = x
        self.y = y
        self.r = r
        self.fr = r / 60.0
        self._phase_diff = None

    @property
    def phase_diff(self):
        if self._phase_diff is None:
            trimed_x = self.x.data[:self.x.sampling_rate]  # 只取前一秒的加速度数据进行互相关计算,考虑计算量以及积分后的相位移动
            trimed_y = self.y.data[:self.y.sampling_rate]
            t = np.linspace(0.0, ((len(trimed_x) - 1) / self.x.sampling_rate), len(trimed_x))
            cross_correlate = np.correlate(trimed_x, trimed_y, "full")
            dt = np.linspace(-t[-1], t[-1], (2 * len(trimed_x)) - 1)
            time_shift = dt[cross_correlate.argmax()]
            self._phase_diff = ((2.0 * np.pi) * ((time_shift / (1.0 / self.fr)) % 1.0)) - np.pi
        return self._phase_diff

    def diagnosis(self):
        pass


    def compute_fault_num(self):
        self.fault_num = []

        for item in type(self).__bases__:
            if item.__module__ == 'mixin':
                 self.fault_num +=  self.fault_num_mapper[getattr(self,item.fault_num_name)]
        self.fault_num = np.array(self.fault_num)
