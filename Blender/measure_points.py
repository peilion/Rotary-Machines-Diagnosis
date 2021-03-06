from Blender.equipment import Blender
from mixin import UnbalanceMixin, MisalignmentMixin, ALooseMixin, BLooseMixin, RollBearingMixin, \
    GearMixin, RubbingMixin
from base import MeasurePoint


class BD_Motor_Driven_Vertical(MeasurePoint, UnbalanceMixin, MisalignmentMixin, RollBearingMixin, ALooseMixin,
                               BLooseMixin):
    equip = Blender

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 11 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_half_harmonic(fr=self.fr)

        self.unbalance_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.misalignment_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)

        self.x.compute_spectrum()
        self.x_env = self.x.to_filted_signal(filter_type="highpass",
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(
            bpfi=self.bearing_ratio[3],
            bpfo=self.bearing_ratio[2],
            bsf=self.bearing_ratio[1],
            ftf=self.bearing_ratio[0],
            fr=self.fr,
            upper=3,
        )

        self.roll_bearing_diagnosis(diag_obj=self.x_env)
        self.compute_fault_num()


class BD_Motor_Driven_Horizontal(MeasurePoint, UnbalanceMixin, MisalignmentMixin, RollBearingMixin, ALooseMixin,
                                 BLooseMixin):
    equip = Blender

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 11 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_half_harmonic(fr=self.fr)
        self.unbalance_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.misalignment_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)

        self.x.compute_spectrum()
        self.x_env = self.x.to_filted_signal(filter_type="highpass",
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(
            bpfi=self.bearing_ratio[3],
            bpfo=self.bearing_ratio[2],
            bsf=self.bearing_ratio[1],
            ftf=self.bearing_ratio[0],
            fr=self.fr,
            upper=3,
        )

        self.roll_bearing_diagnosis(diag_obj=self.x_env)
        self.compute_fault_num()


class BD_Motor_NonDriven(MeasurePoint, UnbalanceMixin, RollBearingMixin, ALooseMixin, BLooseMixin):
    equip = Blender
    require_phase_diff = False

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_half_harmonic(fr=self.fr)
        self.unbalance_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)

        self.x.compute_spectrum()
        self.x_env = self.x.to_filted_signal(filter_type="highpass",
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(
            bpfi=self.bearing_ratio[3],
            bpfo=self.bearing_ratio[2],
            bsf=self.bearing_ratio[1],
            ftf=self.bearing_ratio[0],
            fr=self.fr,
            upper=3,
        )

        self.roll_bearing_diagnosis(diag_obj=self.x_env)
        self.compute_fault_num()


class BD_Gearbox_Input(MeasurePoint, MisalignmentMixin, RollBearingMixin, GearMixin, ALooseMixin, BLooseMixin):
    equip = Blender
    require_phase_diff = False

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_half_harmonic(fr=self.fr)
        self.misalignment_diagnosis(blade_num=0, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=0, diag_obj=self.x_vel)

        self.x_env = self.x.to_filted_signal(filter_type="highpass",
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(
            bpfi=self.bearing_ratio[3],
            bpfo=self.bearing_ratio[2],
            bsf=self.bearing_ratio[1],
            ftf=self.bearing_ratio[0],
            fr=self.fr,
            upper=3,
        )
        self.roll_bearing_diagnosis(diag_obj=self.x_env)

        self.x.compute_spectrum()
        self.x.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.equip.teeth_num[0])
        self.gear_diagnosis(diag_obj=self.x)

        self.compute_fault_num()


class BD_Gearbox_Output(MeasurePoint, MisalignmentMixin, RollBearingMixin, GearMixin, ALooseMixin, BLooseMixin):
    equip = Blender
    require_phase_diff = False

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly")  # 这里本来打算两次积分至位移，但是转速太低，位移信号做FFT似乎无法收敛
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11, tolerance=0.1)
        self.x_vel.compute_half_harmonic(fr=self.fr, tolerance=0.1)
        self.misalignment_diagnosis(blade_num=0, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=0, diag_obj=self.x_vel)

        self.x.compute_spectrum()
        self.x.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.equip.teeth_num[0] * self.equip.teeth_num[2],
                                      tolerance=0.1, )
        self.gear_diagnosis(diag_obj=self.x)

        self.x_env = self.x.to_filted_signal(filter_type="highpass",
                                             co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(
            bpfi=self.bearing_ratio[3],
            bpfo=self.bearing_ratio[2],
            bsf=self.bearing_ratio[1],
            ftf=self.bearing_ratio[0],
            fr=self.fr,
            upper=3,
            tolerance=0.1,
        )
        self.roll_bearing_diagnosis(diag_obj=self.x_env)
        self.compute_fault_num()


class BD_Blender_Driven(MeasurePoint, UnbalanceMixin, RollBearingMixin, RubbingMixin, ALooseMixin, BLooseMixin ):
    equip = Blender
    require_phase_diff = False

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly")
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11, tolerance=0.1)
        self.x_vel.compute_half_harmonic(fr=self.fr, tolerance=0.1)
        self.x_vel.compute_sub_harmonic(fr=self.fr, upper=10, tolerance=0.1)
        self.unbalance_diagnosis(blade_num=self.equip.blender_blade, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.blender_blade, diag_obj=self.x_vel)
        self.rubbing_diagnosis(diag_obj=self.x_vel, blade_num=self.equip.blender_blade)

        self.x.compute_spectrum()
        self.x_env = self.x.to_filted_signal(filter_type="highpass", co_frequency=2 * 1000 / self.x.sampling_rate).to_envelope()
        self.x_env.compute_spectrum()
        self.x_env.compute_bearing_frequency(
            bpfi=self.bearing_ratio[3],
            bpfo=self.bearing_ratio[2],
            bsf=self.bearing_ratio[1],
            ftf=self.bearing_ratio[0],
            fr=self.fr,
            upper=3,
            tolerance=0.1,
        )
        self.roll_bearing_diagnosis(diag_obj=self.x_env)
        self.compute_fault_num()
