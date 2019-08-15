from base import VibrationSignal
import numpy as np
from numpy import ndarray

class UnbalanceMixin:
    ib_threshold = None
    pd_threshold = None
    thd_threshold = None
    fault_num_name = 'ib_level'

    def unbalance_diagnosis(self, blade_num: int, diag_obj: VibrationSignal):

        self.ib_indicator = 0.9 * diag_obj.harmonics[0] + \
                            0.05 * diag_obj.harmonics[1] + \
                            0.05 * (diag_obj.harmonics[2:].sum() - (
            diag_obj.harmonics[blade_num - 1] if blade_num != 0 else 0))
        self.ib_level = np.searchsorted(self.ib_threshold, self.ib_indicator)

        if self.require_phase_diff:
            phase_diff = self.phase_diff
            if (abs(phase_diff) > (np.pi / 2 + self.pd_threshold)) | (
                    abs(phase_diff) < (np.pi / 2 - self.pd_threshold)):
                self.ib_level = 0

        if diag_obj.thd > self.thd_threshold:
            self.ib_level = 0


class MisalignmentMixin:
    ma_threshold = None
    fault_num_name = 'ma_level'

    def misalignment_diagnosis(self, blade_num: int, diag_obj: VibrationSignal):
        self.ma_indicator = 0.5 * diag_obj.harmonics[0] + \
                            0.4 * diag_obj.harmonics[1] + \
                            0.1 * (diag_obj.harmonics[2:].sum() - (
            diag_obj.harmonics[blade_num - 1] if blade_num != 0 else 0))

        self.ma_level = np.searchsorted(self.ma_threshold, self.ma_indicator)


class ALooseMixin:
    al_threshold = None
    pd_threshold = None
    fault_num_name = 'al_level'

    def atype_loose_diagnosis(self, diag_obj: VibrationSignal):

        self.al_indicator = diag_obj.harmonics[0]
        self.al_level = np.searchsorted(self.al_threshold, self.al_indicator)

        if self.require_phase_diff:
            phase_diff = self.phase_diff
            if (abs(phase_diff) < (np.pi / 2 + self.pd_threshold)) & (
                    abs(phase_diff) > (np.pi / 2 - self.pd_threshold)):
                self.al_level = 0


class BLooseMixin:
    harmonic_threshold = None
    bl_threshold = None
    fault_num_name = 'bl_level'

    def btype_loose_diagnosis(self, blade_num: int, diag_obj: VibrationSignal):
        self.bl_indicator = diag_obj.half_fr_amp
        self.bl_level = np.searchsorted(self.bl_threshold, self.bl_indicator)

        harmonic_compare = diag_obj.harmonics >= self.harmonic_threshold
        hn = 0
        for index, item in enumerate(harmonic_compare):
            if (item & ((index + 1) != blade_num)):
                hn = (hn + 1)
        self.harmonic_number = hn


class RollBearingMixin:
    bw_threshold = None
    kurtosis_threshold = None
    fault_num_name = 'bw_level'

    def roll_bearing_diagnosis(self, diag_obj: VibrationSignal):
        level_list = []
        obj = diag_obj  # type : VibrationSignal
        bw_amps = obj.bearing_amp[:4]
        for item in bw_amps:
            level_list.append(np.searchsorted(self.bw_threshold, item))
        if obj.kurtosis > self.kurtosis_threshold:
            level_list.append(1)
        self.bw_level = np.array(level_list).max()


class GearMixin:
    gf_threshold = None
    sideband_order = 6
    kurtosis_threshold = None
    fault_num_name = 'gf_level'

    def gear_diagnosis(self, diag_obj: VibrationSignal):
        self.sideband_energies = np.sum(diag_obj.sideband_amps, axis=1)
        self.gf_indicator = self.sideband_energies.max()
        self.gf_level = np.searchsorted(self.gf_threshold, self.gf_indicator)

        if (diag_obj.kurtosis > self.kurtosis_threshold) & (self.gf_level < 1):
            self.gf_level += 1


class OilWhirlMixin:
    wd_threshold = None  # type: ndarray
    fault_num_name = 'ow_level'

    def oil_whirl_diagnosis(self, diag_obj: VibrationSignal):
        self.ow_indicator = diag_obj.ow_amp
        self.ow_level = np.searchsorted(self.wd_threshold, self.ow_indicator)


class RubbingMixin:
    rb_threshold = None  # type:ndarray
    subharmonic_threshold = None  # type:ndarray
    fault_num_name = 'rb_level'

    def rubbing_diagnosis(self, diag_obj: VibrationSignal, blade_num: int):

        harmonic_compare = diag_obj.harmonics >= self.harmonic_threshold
        hn = 0
        for index, item in enumerate(harmonic_compare):
            if (item & ((index + 1) != blade_num)):
                hn = (hn + 1)
        self.harmonic_number = hn

        sub_harmonic_compare = diag_obj.sub_harmonics >= self.subharmonic_threshold
        shn = 0
        for item in sub_harmonic_compare:
            shn = (shn + 1) if item else shn

        self.sub_har_num = shn
        self.rb_indicator = hn + shn
        self.rb_level = np.searchsorted(self.rb_threshold, self.rb_indicator)


class SurgeMixin:
    sg_threshold = None  # type: ndarray
    pres_threshold = None  # type: ndarray
    fault_num_name = 'sg_level'

    def surge_diagnosis(self, pressure_vector, diag_obj: VibrationSignal):
        low_index1, low_energy1 = diag_obj.get_band_energy(fr=self.fr, band_range=(0, 0.4))
        low_index2, low_energy2 = diag_obj.get_band_energy(fr=self.fr, band_range=(0.6, 0.8))

        self.sg_index = np.array([low_index1, low_index2])
        self.lt_fr_amp = np.array([low_energy1, low_energy2])

        self.sg_indicator = 0.45 * low_energy1 + \
                            0.35 * low_energy2

        self.pres_std = np.std(pressure_vector, ddof=1)
        sg_level_vib = np.searchsorted(self.sg_threshold, self.sg_indicator)
        sg_level_pres = np.searchsorted(self.pres_threshold, self.pres_std)

        self.sg_level = np.max((sg_level_pres, sg_level_vib))
