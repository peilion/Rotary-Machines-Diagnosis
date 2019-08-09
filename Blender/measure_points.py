from Blender.equipment import Blender
from mixin import UnbalanceMixin, MisalignmentMixin, ALooseMixin, BLooseMixin, RollBearingMixin, \
    OilWhirlMixin, GearMixin, RubbingMixin, SurgeMixin
from base import MeasurePoint, VibrationSignal
import numpy as np


class BD_Motor_Driven_Vertical(MeasurePoint, UnbalanceMixin, MisalignmentMixin, ALooseMixin, BLooseMixin,
                               RollBearingMixin):
    equip = Blender

    def __init__(self, ib_threshold, pd_threshold, thd_threshold, ma_threshold, al_threshold, bl_threshold,
                 bearing_ratio, kurtosis_threshold,
                 harmonic_threshold, bw_threshold, x: VibrationSignal, y: VibrationSignal, cur: float,
                 r: dict):
        self.operation_mode = 0 if (cur < 300) else 1
        r = r[self.operation_mode]

        super().__init__(x, y, r)
        self.ib_threshold = ib_threshold[self.operation_mode]
        self.ma_threshold = ma_threshold[self.operation_mode]
        self.al_threshold = al_threshold[self.operation_mode]
        self.bl_threshold = bl_threshold[self.operation_mode]
        self.harmonic_threshold = harmonic_threshold[self.operation_mode]
        self.bw_threshold = bw_threshold[self.operation_mode]

        self.pd_threshold = pd_threshold
        self.thd_threshold = thd_threshold
        self.bearing_ratio = bearing_ratio
        self.kurtosis_threshold = kurtosis_threshold

    def compute_fault_num(self):
        self.fault_num = []
        for item in [self.ib_level, self.ma_level, self.bw_level, self.al_level, self.bl_level]:
            assert item in [0, 1, 2, 3], '故障严重等级计算错误'
            self.fault_num += self.fault_num_mapper[item]
        self.fault_num = np.array(self.fault_num)

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type='poly').to_filted_signal(filter_type='lowpass',
                                                                              co_frequency=2 * 11 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_half_harmonic(fr=self.fr)

        self.unbalance_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.misalignment_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)

        self.x.compute_spectrum()

        self.x_env = self.x.to_filted_signal(filter_type='highpass',
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(bpfi=self.bearing_ratio[0],
                                             bpfo=self.bearing_ratio[1],
                                             bsf=self.bearing_ratio[2],
                                             ftf=self.bearing_ratio[3],
                                             fr=self.fr,
                                             upper=3)

        self.roll_bearing_diagnosis(diag_obj=self.x_env)
        self.compute_fault_num()


class BD_Motor_Driven_Horizontal(BD_Motor_Driven_Vertical):
    pass


class BD_Motor_NonDriven(MeasurePoint, UnbalanceMixin, ALooseMixin, BLooseMixin, RollBearingMixin):
    equip = Blender
    require_phase_diff = False

    def __init__(self, ib_threshold, pd_threshold, thd_threshold, al_threshold, bl_threshold,
                 bearing_ratio, kurtosis_threshold,
                 harmonic_threshold, bw_threshold, x: VibrationSignal, y: VibrationSignal, cur: float,
                 r: dict):
        self.operation_mode = 0 if (cur < 300) else 1
        r = r[self.operation_mode]

        super().__init__(x, y, r)
        self.ib_threshold = ib_threshold[self.operation_mode]
        self.al_threshold = al_threshold[self.operation_mode]
        self.bl_threshold = bl_threshold[self.operation_mode]
        self.harmonic_threshold = harmonic_threshold[self.operation_mode]
        self.bw_threshold = bw_threshold[self.operation_mode]

        self.pd_threshold = pd_threshold
        self.thd_threshold = thd_threshold
        self.bearing_ratio = bearing_ratio
        self.kurtosis_threshold = kurtosis_threshold

    def compute_fault_num(self):
        self.fault_num = []
        for item in [self.ib_level, self.bw_level, self.al_level, self.bl_level]:
            assert item in [0, 1, 2, 3], '故障严重等级计算错误'
            self.fault_num += self.fault_num_mapper[item]
        self.fault_num = np.array(self.fault_num)

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type='poly').to_filted_signal(filter_type='lowpass',
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_half_harmonic(fr=self.fr)

        self.unbalance_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)

        self.x.compute_spectrum()

        self.x_env = self.x.to_filted_signal(filter_type='highpass',
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(bpfi=self.bearing_ratio[0],
                                             bpfo=self.bearing_ratio[1],
                                             bsf=self.bearing_ratio[2],
                                             ftf=self.bearing_ratio[3],
                                             fr=self.fr,
                                             upper=3)

        self.roll_bearing_diagnosis(diag_obj=self.x_env)
        self.compute_fault_num()


class BD_Gearbox_Input(MeasurePoint, RollBearingMixin, GearMixin, ALooseMixin, BLooseMixin, MisalignmentMixin):
    equip = Blender
    require_phase_diff = False

    def __init__(self, al_threshold, bl_threshold,
                 bearing_ratio, kurtosis_threshold, teeth_num, gf_threshold,
                 harmonic_threshold, bw_threshold, x: VibrationSignal, y: VibrationSignal, cur: float, ma_threshold,
                 r: dict):
        self.operation_mode = 0 if (cur < 300) else 1
        r = r[self.operation_mode]

        super().__init__(x, y, r)
        self.al_threshold = al_threshold[self.operation_mode]
        self.bl_threshold = bl_threshold[self.operation_mode]
        self.harmonic_threshold = harmonic_threshold[self.operation_mode]
        self.bw_threshold = bw_threshold[self.operation_mode]
        self.gf_threshold = gf_threshold[self.operation_mode]
        self.ma_threshold = ma_threshold[self.operation_mode]

        self.bearing_ratio = bearing_ratio
        self.kurtosis_threshold = kurtosis_threshold

        self.equip.teeth_num = teeth_num

    def compute_fault_num(self):
        self.fault_num = []
        for item in [self.ma_level, self.bw_level, self.gf_level, self.al_level, self.bl_level]:
            assert item in [0, 1, 2, 3], '故障严重等级计算错误'
            self.fault_num += self.fault_num_mapper[item]
        self.fault_num = np.array(self.fault_num)

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type='poly').to_filted_signal(filter_type='lowpass',
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_half_harmonic(fr=self.fr)
        self.misalignment_diagnosis(blade_num=0, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=0, diag_obj=self.x_vel)

        self.x_env = self.x.to_filted_signal(filter_type='highpass',
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(bpfi=self.bearing_ratio[0],
                                             bpfo=self.bearing_ratio[1],
                                             bsf=self.bearing_ratio[2],
                                             ftf=self.bearing_ratio[3],
                                             fr=self.fr,
                                             upper=3)
        self.roll_bearing_diagnosis(diag_obj=self.x_env)

        self.x.compute_spectrum()
        self.x.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.equip.teeth_num[0])
        self.gear_diagnosis(diag_obj=self.x)

        self.compute_fault_num()


class BD_Gearbox_Output(BD_Gearbox_Input):

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type='poly')  # 这里本来打算两次积分至位移，但是转速太低，位移信号做FFT似乎无法收敛

        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11, tolerance=0.1)
        self.x_vel.compute_half_harmonic(fr=self.fr, tolerance=0.1)

        self.misalignment_diagnosis(blade_num=0, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=0, diag_obj=self.x_vel)

        self.x.compute_spectrum()
        self.x.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.equip.teeth_num[1], tolerance=0.1)

        self.gear_diagnosis(diag_obj=self.x)

        self.x_env = self.x.to_filted_signal(filter_type='highpass',
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(bpfi=self.bearing_ratio[0],
                                             bpfo=self.bearing_ratio[1],
                                             bsf=self.bearing_ratio[2],
                                             ftf=self.bearing_ratio[3],
                                             fr=self.fr,
                                             upper=3,
                                             tolerance=0.1)
        self.roll_bearing_diagnosis(diag_obj=self.x_env)

        self.compute_fault_num()


class BD_Blender_Driven(MeasurePoint, UnbalanceMixin, RollBearingMixin, RubbingMixin, ALooseMixin, BLooseMixin):
    equip = Blender
    require_phase_diff = False

    def __init__(self, ib_threshold, pd_threshold, thd_threshold, al_threshold, bl_threshold,
                 bearing_ratio, kurtosis_threshold, rb_threshold, subharmonic_threshold,
                 harmonic_threshold, bw_threshold, x: VibrationSignal, y: VibrationSignal, cur: float,
                 r: dict):
        self.operation_mode = 0 if (cur < 300) else 1
        r = r[self.operation_mode]

        super().__init__(x, y, r)
        self.ib_threshold = ib_threshold[self.operation_mode]
        self.al_threshold = al_threshold[self.operation_mode]
        self.bl_threshold = bl_threshold[self.operation_mode]
        self.harmonic_threshold = harmonic_threshold[self.operation_mode]
        self.bw_threshold = bw_threshold[self.operation_mode]
        self.rb_threshold = rb_threshold[self.operation_mode]
        self.subharmonic_threshold = subharmonic_threshold[self.operation_mode]

        self.pd_threshold = pd_threshold
        self.thd_threshold = thd_threshold
        self.bearing_ratio = bearing_ratio
        self.kurtosis_threshold = kurtosis_threshold

    def compute_fault_num(self):
        self.fault_num = []
        for item in [self.ib_level, self.bw_level, self.rb_level, self.al_level, self.bl_level]:
            assert item in [0, 1, 2, 3], '故障严重等级计算错误'
            self.fault_num += self.fault_num_mapper[item]
        self.fault_num = np.array(self.fault_num)

    def diagnosis(self):
        self.x_vel = self.x. \
            to_velocity(detrend_type='poly')

        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11, tolerance=0.1)
        self.x_vel.compute_half_harmonic(fr=self.fr, tolerance=0.1)
        self.x_vel.compute_sub_harmonic(fr=self.fr, upper=10,tolerance=0.1)

        self.unbalance_diagnosis(blade_num=self.equip.blender_blade, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.blender_blade, diag_obj=self.x_vel)
        self.rubbing_diagnosis(diag_obj=self.x_vel, blade_num=self.equip.blender_blade)

        self.x.compute_spectrum()

        self.x_env = self.x.to_filted_signal(filter_type='highpass',
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(bpfi=self.bearing_ratio[0],
                                             bpfo=self.bearing_ratio[1],
                                             bsf=self.bearing_ratio[2],
                                             ftf=self.bearing_ratio[3],
                                             fr=self.fr,
                                             upper=3,
                                             tolerance=0.1)

        self.roll_bearing_diagnosis(diag_obj=self.x_env)

        self.compute_fault_num()
