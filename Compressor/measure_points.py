from Compressor.equipment import Compressor
from mixin import UnbalanceMixin, MisalignmentMixin, ALooseMixin, BLooseMixin, RollBearingMixin, OilWhirlMixin, \
    GearMixin, RubbingMixin, SurgeMixin
from base import MeasurePoint


class CP_Motor_Driven_Vertical(MeasurePoint, UnbalanceMixin, MisalignmentMixin, RollBearingMixin, ALooseMixin,
                               BLooseMixin):
    # Mixin的继承顺序必须与故障代码的顺序完全一致
    equip = Compressor

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
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


class CP_Motor_Driven_Horizontal(MeasurePoint, UnbalanceMixin, MisalignmentMixin, RollBearingMixin):
    equip = Compressor

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)

        self.unbalance_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.misalignment_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)

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


class CP_Motor_NonDriven_Vertical(MeasurePoint, UnbalanceMixin, RollBearingMixin, ALooseMixin, BLooseMixin):
    equip = Compressor

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)

        self.unbalance_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)

        self.x_vel.compute_half_harmonic(fr=self.fr)
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
            upper=3)

        self.roll_bearing_diagnosis(diag_obj=self.x_env)
        self.compute_fault_num()


class CP_Motor_NonDriven_Horizontal(MeasurePoint, UnbalanceMixin, RollBearingMixin):
    equip = Compressor

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)

        self.unbalance_diagnosis(blade_num=self.equip.motor_blade, diag_obj=self.x_vel)

        self.x.compute_spectrum()
        self.x.compute_bearing_frequency(
            bpfi=self.bearing_ratio[3],
            bpfo=self.bearing_ratio[2],
            bsf=self.bearing_ratio[1],
            ftf=self.bearing_ratio[0],
            fr=self.fr,
            upper=3,
        )
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


class CP_Gearbox_Input_Vertical(
    MeasurePoint, MisalignmentMixin, OilWhirlMixin, GearMixin, ALooseMixin, BLooseMixin
):
    equip = Compressor
    require_phase_diff = False

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_oilwhirl_frequency(fr=self.fr)
        self.x_vel.compute_half_harmonic(fr=self.fr)

        self.misalignment_diagnosis(blade_num=0, diag_obj=self.x_vel)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.oil_whirl_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=0, diag_obj=self.x_vel)

        self.x_hp = self.x.to_filted_signal(filter_type="highpass", co_frequency=1000 / self.x.sampling_rate)
        self.x_hp.compute_spectrum()
        self.x_hp.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.teeth_num[0])
        self.gear_diagnosis(diag_obj=self.x_hp)
        self.compute_fault_num()


class CP_Gearbox_Input_Horizontal(
    MeasurePoint, MisalignmentMixin, OilWhirlMixin, GearMixin
):
    equip = Compressor
    require_phase_diff = False

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_oilwhirl_frequency(fr=self.fr)
        self.x_vel.compute_half_harmonic(fr=self.fr)
        self.misalignment_diagnosis(blade_num=0, diag_obj=self.x_vel)
        self.oil_whirl_diagnosis(diag_obj=self.x_vel)

        self.x_hp = self.x.to_filted_signal(filter_type="highpass", co_frequency=1000 / self.x.sampling_rate)
        self.x_hp.compute_spectrum()
        self.x_hp.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.teeth_num[0])
        self.gear_diagnosis(diag_obj=self.x_hp)

        self.compute_fault_num()


class CP_Gearbox_Inner_Ring(MeasurePoint, GearMixin):
    equip = Compressor
    require_phase_diff = False

    def diagnosis(self):
        self.x_hp = self.x.to_filted_signal(filter_type="highpass", co_frequency=1000 / self.x.sampling_rate)
        self.x_hp.compute_spectrum()
        self.x_hp.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.teeth_num[0])
        self.gear_diagnosis(diag_obj=self.x_hp)

        self.compute_fault_num()


class CP_Gearbox_Output_Vertical(MeasurePoint, MisalignmentMixin, OilWhirlMixin, GearMixin, ALooseMixin, BLooseMixin):
    equip = Compressor
    require_phase_diff = False

    def diagnosis(self):
        self.x.compute_spectrum()
        self.x_lp = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                             co_frequency=2 * 11 * self.fr / self.x.sampling_rate)
        self.x_lp.compute_spectrum()
        self.x_lp.compute_harmonics(fr=self.fr, upper=11)
        self.x_lp.compute_oilwhirl_frequency(fr=self.fr)
        self.x_lp.compute_half_harmonic(fr=self.fr)
        self.misalignment_diagnosis(blade_num=0, diag_obj=self.x_lp)
        self.atype_loose_diagnosis(diag_obj=self.x_lp)
        self.oil_whirl_diagnosis(diag_obj=self.x_lp)
        self.btype_loose_diagnosis(blade_num=0, diag_obj=self.x_lp)

        self.x_hp = self.x.to_filted_signal(filter_type="highpass",
                                            co_frequency=2 * 11 * self.fr / self.x.sampling_rate)
        self.x_hp.compute_spectrum()
        self.x_hp.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.teeth_num[0] / 6.964)  # 使用输入轴转速计算啮合频率
        self.gear_diagnosis(diag_obj=self.x_hp)

        self.compute_fault_num()


class CP_Gearbox_Output_Horizontal(MeasurePoint, MisalignmentMixin, OilWhirlMixin, GearMixin):
    equip = Compressor
    require_phase_diff = False

    def diagnosis(self):
        self.x.compute_spectrum()
        self.x_lp = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                             co_frequency=2 * 11 * self.fr / self.x.sampling_rate)
        self.x_lp.compute_spectrum()
        self.x_lp.compute_harmonics(fr=self.fr, upper=11)
        self.x_lp.compute_oilwhirl_frequency(fr=self.fr)
        self.x_lp.compute_half_harmonic(fr=self.fr)
        self.misalignment_diagnosis(blade_num=0, diag_obj=self.x_lp)
        self.oil_whirl_diagnosis(diag_obj=self.x_lp)

        self.x_hp = self.x.to_filted_signal(filter_type="highpass",
                                            co_frequency=2 * 11 * self.fr / self.x.sampling_rate)
        self.x_hp.compute_spectrum()
        self.x_hp.compute_mesh_frequency(fr=self.fr, mesh_ratio=self.teeth_num[0] / 6.964)
        self.gear_diagnosis(diag_obj=self.x_hp)

        self.compute_fault_num()


class CP_Compressor_Driven_Vertical(MeasurePoint, UnbalanceMixin, MisalignmentMixin, OilWhirlMixin, RubbingMixin,
                                    ALooseMixin, BLooseMixin, SurgeMixin, ):
    equip = Compressor
    require_phase_diff = True

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=10 * self.fr * 2 / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_sub_harmonic(fr=self.fr, upper=10)
        self.x_vel.compute_half_harmonic(fr=self.fr)
        self.unbalance_diagnosis(blade_num=self.equip.compressor_blade, diag_obj=self.x_vel)
        self.rubbing_diagnosis(diag_obj=self.x_vel, blade_num=self.equip.compressor_blade)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.compressor_blade, diag_obj=self.x_vel)

        self.x.compute_spectrum()
        self.x.compute_harmonics(fr=self.fr, upper=11)
        self.x.compute_oilwhirl_frequency(fr=self.fr)
        self.misalignment_diagnosis(blade_num=self.equip.compressor_blade, diag_obj=self.x)
        self.oil_whirl_diagnosis(diag_obj=self.x)
        self.surge_diagnosis(diag_obj=self.x, pressure_vector=self.pressure)

        self.compute_fault_num()


class CP_Compressor_Driven_Horizontal(MeasurePoint, UnbalanceMixin, MisalignmentMixin, OilWhirlMixin, RubbingMixin,
                                      SurgeMixin, ):
    equip = Compressor
    require_phase_diff = True

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_sub_harmonic(fr=self.fr, upper=10)
        self.unbalance_diagnosis(blade_num=self.equip.compressor_blade, diag_obj=self.x_vel)
        self.rubbing_diagnosis(diag_obj=self.x_vel, blade_num=self.equip.compressor_blade)

        self.x.compute_spectrum()
        self.x.compute_harmonics(fr=self.fr, upper=11)
        self.x.compute_oilwhirl_frequency(fr=self.fr)
        self.misalignment_diagnosis(blade_num=self.equip.compressor_blade, diag_obj=self.x)
        self.oil_whirl_diagnosis(diag_obj=self.x)
        self.surge_diagnosis(diag_obj=self.x, pressure_vector=self.pressure)
        self.compute_fault_num()


class CP_Compressor_NonDriven_Vertical(
    MeasurePoint, UnbalanceMixin, OilWhirlMixin, RubbingMixin, ALooseMixin, BLooseMixin, SurgeMixin):
    equip = Compressor
    require_phase_diff = True

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_sub_harmonic(fr=self.fr, upper=10)
        self.x_vel.compute_half_harmonic(fr=self.fr)
        self.unbalance_diagnosis(blade_num=self.equip.compressor_blade, diag_obj=self.x_vel)
        self.rubbing_diagnosis(diag_obj=self.x_vel, blade_num=self.equip.compressor_blade)
        self.atype_loose_diagnosis(diag_obj=self.x_vel)
        self.btype_loose_diagnosis(blade_num=self.equip.compressor_blade, diag_obj=self.x_vel)

        self.x.compute_spectrum()
        self.x.compute_harmonics(fr=self.fr, upper=11)
        self.x.compute_oilwhirl_frequency(fr=self.fr)
        self.oil_whirl_diagnosis(diag_obj=self.x)
        self.surge_diagnosis(diag_obj=self.x, pressure_vector=self.pressure)

        self.compute_fault_num()


class CP_Compressor_NonDriven_Horizontal(MeasurePoint, UnbalanceMixin, OilWhirlMixin, RubbingMixin, SurgeMixin):
    equip = Compressor
    require_phase_diff = True

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_sub_harmonic(fr=self.fr, upper=10)

        self.unbalance_diagnosis(blade_num=self.equip.compressor_blade, diag_obj=self.x_vel)
        self.rubbing_diagnosis(diag_obj=self.x_vel, blade_num=self.equip.compressor_blade)

        self.x.compute_spectrum()
        self.x.compute_harmonics(fr=self.fr, upper=11)
        self.x.compute_oilwhirl_frequency(fr=self.fr)

        self.oil_whirl_diagnosis(diag_obj=self.x)
        self.surge_diagnosis(diag_obj=self.x, pressure_vector=self.pressure)
        self.compute_fault_num()


class CP_Compressor_NonDriven_Axial(MeasurePoint, OilWhirlMixin, RubbingMixin):
    equip = Compressor
    require_phase_diff = False

    def diagnosis(self):
        self.x_vel = self.x.to_velocity(detrend_type="poly").to_filted_signal(filter_type="lowpass",
                                                                              co_frequency=2 * 10 * self.fr / self.x.sampling_rate)
        self.x_vel.compute_spectrum()
        self.x_vel.compute_harmonics(fr=self.fr, upper=11)
        self.x_vel.compute_sub_harmonic(fr=self.fr, upper=10)
        self.rubbing_diagnosis(diag_obj=self.x_vel, blade_num=self.equip.compressor_blade)

        self.x.compute_spectrum()
        self.x.compute_oilwhirl_frequency(fr=self.fr)
        self.oil_whirl_diagnosis(diag_obj=self.x)
        self.compute_fault_num()
