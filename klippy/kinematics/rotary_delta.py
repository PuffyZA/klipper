# Code for handling the kinematics of rotary delta robots
#
# Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, homing, mathutil

class RotaryDeltaKinematics:
    def __init__(self, toolhead, config):
        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.PrinterRail(
            stepper_configs[0], need_position_minmax = False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.PrinterRail(
            stepper_configs[1], need_position_minmax = False,
            default_position_endstop=a_endstop)
        rail_c = stepper.PrinterRail(
            stepper_configs[2], need_position_minmax = False,
            default_position_endstop=a_endstop)
        self.rails = [rail_a, rail_b, rail_c]
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # Setup stepper max halt velocity
        for rail in self.rails:
            rail.set_max_jerk(9999999.9, 9999999.9)
        # Read config
        self.shoulder_radius = config.getfloat('shoulder_radius', above=0.)
        self.shoulder_height = config.getfloat('shoulder_height', above=0.)
        a_upper_arm = stepper_configs[0].getfloat('upper_arm_length', above=0.)
        self.upper_arms = upper_arms = [
            sconfig.getfloat('upper_arm_length', a_upper_arm, above=0.)
            for sconfig in stepper_configs]
        a_lower_arm = stepper_configs[0].getfloat('lower_arm_length', above=0.)
        self.lower_arms = lower_arms = [
            sconfig.getfloat('lower_arm_length', a_lower_arm, above=0.)
            for sconfig in stepper_configs]
        self.angles = angles = [sconfig.getfloat('angle', angle)
                                for sconfig, angle in zip(stepper_configs,
                                                          [30., 150., 270.])]
        # Setup iterative solver
        for r, a, ua, la in zip(self.rails, angles, upper_arms, lower_arms):
            r.setup_itersolve('rotary_delta_stepper_alloc',
                              self.shoulder_radius, self.shoulder_height,
                              math.radians(a), ua, la)
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Setup boundary checks
        self.need_home = True
        epos = [r.get_homing_info().position_endstop for r in self.rails]
        eangles = [r.calc_position_from_coord([0., 0., ep])
                   for r, ep in zip(self.rails, epos)]
        self.home_position = self._actuator_to_cartesian(eangles)
        self.set_position([0., 0., 0.], ())
    def get_steppers(self, flags=""):
        return [s for rail in self.rails for s in rail.get_steppers()]
    def _elbow_coord(self, elbow_id, spos):
        # Calculate elbow position in coordinate system at shoulder joint
        sj_elbow_x = self.upper_arms[elbow_id] * math.cos(spos)
        sj_elbow_y = self.upper_arms[elbow_id] * math.sin(spos)
        # Shift and rotate to main cartesian coordinate system
        angle = math.radians(self.angles[elbow_id])
        x = (sj_elbow_x + self.shoulder_radius) * math.cos(angle)
        y = (sj_elbow_x + self.shoulder_radius) * math.sin(angle)
        z = sj_elbow_y + self.shoulder_height
        return (x, y, z)
    def _actuator_to_cartesian(self, spos):
        sphere_coords = [self._elbow_coord(i, sp) for i, sp in enumerate(spos)]
        lower_arm2 = [la**2 for la in self.lower_arms]
        return mathutil.trilateration(sphere_coords, lower_arm2)
    def calc_tag_position(self):
        spos = [rail.get_tag_position() for rail in self.rails]
        return self._actuator_to_cartesian(spos)
    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False
    def home(self, homing_state):
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = -1.0
        homing_state.home_rails(self.rails, forcepos, self.home_position)
    def _motor_off(self, print_time):
        self.need_home = True
    def check_move(self, move):
        pass
    def get_status(self):
        return {'homed_axes': '' if self.need_home else 'XYZ'}

def load_kinematics(toolhead, config):
    return RotaryDeltaKinematics(toolhead, config)
