# Code for handling printer nozzle extruders
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper, homing, chelper

class PrinterExtruder:
    def __init__(self, config, extruder_num):
        self.printer = config.get_printer()
        self.name = config.get_name()
        shared_heater = config.get('shared_heater', None)
        pheater = self.printer.lookup_object('heater')
        gcode_id = 'T%d' % (extruder_num,)
        if shared_heater is None:
            self.heater = pheater.setup_heater(config, gcode_id)
        else:
            self.heater = pheater.lookup_heater(shared_heater)
        self.stepper = stepper.PrinterStepper(config)
        self.nozzle_diameter = config.getfloat('nozzle_diameter', above=0.)
        filament_diameter = config.getfloat(
            'filament_diameter', minval=self.nozzle_diameter)
        self.filament_area = math.pi * (filament_diameter * .5)**2
        def_max_cross_section = 4. * self.nozzle_diameter**2
        def_max_extrude_ratio = def_max_cross_section / self.filament_area
        max_cross_section = config.getfloat(
            'max_extrude_cross_section', def_max_cross_section, above=0.)
        self.max_extrude_ratio = max_cross_section / self.filament_area
        logging.info("Extruder max_extrude_ratio=%.6f", self.max_extrude_ratio)
        toolhead = self.printer.lookup_object('toolhead')
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_e_velocity = config.getfloat(
            'max_extrude_only_velocity', max_velocity * def_max_extrude_ratio
            , above=0.)
        self.max_e_accel = config.getfloat(
            'max_extrude_only_accel', max_accel * def_max_extrude_ratio
            , above=0.)
        self.stepper.set_max_jerk(9999999.9, 9999999.9)
        self.max_e_dist = config.getfloat(
            'max_extrude_only_distance', 50., minval=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)
        gcode_macro = self.printer.try_load_module(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')
        self.pressure_advance = self.pressure_advance_smooth_time = 0.
        pressure_advance = config.getfloat('pressure_advance', 0., minval=0.)
        smooth_time = config.getfloat('pressure_advance_smooth_time',
                                      0.040, above=0., maxval=.200)
        self.extrude_pos = self.extrude_pa_pos = 0.
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_free_moves = ffi_lib.trapq_free_moves
        self.sk_extruder = ffi_main.gc(ffi_lib.extruder_stepper_alloc(),
                                       ffi_lib.free)
        self.stepper.set_stepper_kinematics(self.sk_extruder)
        self.stepper.set_trapq(self.trapq)
        toolhead.register_step_generator(self.stepper.generate_steps)
        self.extruder_set_pressure = ffi_lib.extruder_set_pressure
        self._set_pressure_advance(pressure_advance, smooth_time)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        if self.name == 'extruder':
            toolhead.set_extruder(self, self.extrude_pos)
            gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER", None,
                                       self.cmd_default_SET_PRESSURE_ADVANCE,
                                       desc=self.cmd_SET_PRESSURE_ADVANCE_help)
        gcode.register_mux_command("SET_PRESSURE_ADVANCE", "EXTRUDER",
                                   self.name, self.cmd_SET_PRESSURE_ADVANCE,
                                   desc=self.cmd_SET_PRESSURE_ADVANCE_help)
    def update_move_time(self, flush_time):
        self.trapq_free_moves(self.trapq, flush_time)
    def _set_pressure_advance(self, pressure_advance, smooth_time):
        old_smooth_time = self.pressure_advance_smooth_time * .5
        if not self.pressure_advance:
            old_smooth_time = 0.
        new_smooth_time = smooth_time * .5
        if not pressure_advance:
            new_smooth_time = 0.
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.note_step_generation_scan_time(new_smooth_time,
                                                old_delay=old_smooth_time)
        self.extruder_set_pressure(self.sk_extruder,
                                   pressure_advance, new_smooth_time)
        self.pressure_advance = pressure_advance
        self.pressure_advance_smooth_time = smooth_time
    def get_status(self, eventtime):
        return dict(self.get_heater().get_status(eventtime),
                    pressure_advance=self.pressure_advance,
                    smooth_time=self.pressure_advance_smooth_time)
    def get_name(self):
        return self.name
    def get_heater(self):
        return self.heater
    def set_active(self, print_time, is_active):
        return self.extrude_pos
    def get_activate_gcode(self, is_active):
        if is_active:
            return self.activate_gcode.render()
        return self.deactivate_gcode.render()
    def stats(self, eventtime):
        return self.heater.stats(eventtime)
    def check_move(self, move):
        axis_r = move.axes_r[3]
        if not self.heater.can_extrude:
            raise homing.EndstopError(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        if (not move.axes_d[0] and not move.axes_d[1]) or axis_r < 0.:
            # Extrude only move (or retraction move) - limit accel and velocity
            if abs(move.axes_d[3]) > self.max_e_dist:
                raise homing.EndstopError(
                    "Extrude only move too long (%.3fmm vs %.3fmm)\n"
                    "See the 'max_extrude_only_distance' config"
                    " option for details" % (move.axes_d[3], self.max_e_dist))
            inv_extrude_r = 1. / abs(axis_r)
            move.limit_speed(self.max_e_velocity * inv_extrude_r,
                             self.max_e_accel * inv_extrude_r)
        elif axis_r > self.max_extrude_ratio:
            if move.axes_d[3] <= self.nozzle_diameter * self.max_extrude_ratio:
                # Permit extrusion if amount extruded is tiny
                return
            area = axis_r * self.filament_area
            logging.debug("Overextrude: %s vs %s (area=%.3f dist=%.3f)",
                          axis_r, self.max_extrude_ratio, area, move.move_d)
            raise homing.EndstopError(
                "Move exceeds maximum extrusion (%.3fmm^2 vs %.3fmm^2)\n"
                "See the 'max_extrude_cross_section' config option for details"
                % (area, self.max_extrude_ratio * self.filament_area))
    def calc_junction(self, prev_move, move):
        diff_r = move.axes_r[3] - prev_move.axes_r[3]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def move(self, print_time, move):
        axis_r = move.axes_r[3]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        is_pa = 0.
        if axis_r > 0. and (move.axes_d[0] or move.axes_d[1]):
            is_pa = 1.
        # Queue movement (x is extruder movement, y is movement with pa)
        self.trapq_append(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          move.start_pos[3], self.extrude_pa_pos, 0.,
                          1., is_pa, 0.,
                          start_v, cruise_v, accel)
        self.extrude_pos = move.end_pos[3]
        if is_pa:
            self.extrude_pa_pos += move.axes_d[3]
    cmd_SET_PRESSURE_ADVANCE_help = "Set pressure advance parameters"
    def cmd_default_SET_PRESSURE_ADVANCE(self, params):
        extruder = self.printer.lookup_object('toolhead').get_extruder()
        extruder.cmd_SET_PRESSURE_ADVANCE(params)
    def cmd_SET_PRESSURE_ADVANCE(self, params):
        gcode = self.printer.lookup_object('gcode')
        pressure_advance = gcode.get_float(
            'ADVANCE', params, self.pressure_advance, minval=0.)
        smooth_time = gcode.get_float(
            'SMOOTH_TIME', params,
            self.pressure_advance_smooth_time, minval=0., maxval=.200)
        self._set_pressure_advance(pressure_advance, smooth_time)
        msg = ("pressure_advance: %.6f\n"
               "pressure_advance_smooth_time: %.6f" % (
                   pressure_advance, smooth_time))
        self.printer.set_rollover_info(self.name, "%s: %s" % (self.name, msg))
        gcode.respond_info(msg, log=False)

# Dummy extruder class used when a printer has no extruder at all
class DummyExtruder:
    def set_active(self, print_time, is_active):
        return 0.
    def update_move_time(self, flush_time):
        pass
    def check_move(self, move):
        raise homing.EndstopMoveError(
            move.end_pos, "Extrude when no extruder present")
    def calc_junction(self, prev_move, move):
        return move.max_cruise_v2
    def get_name(self):
        return ""
    def get_heater(self):
        raise homing.CommandError("Extruder not configured")

def add_printer_objects(config):
    printer = config.get_printer()
    for i in range(99):
        section = 'extruder'
        if i:
            section = 'extruder%d' % (i,)
        if not config.has_section(section):
            break
        pe = PrinterExtruder(config.getsection(section), i)
        printer.add_object(section, pe)
