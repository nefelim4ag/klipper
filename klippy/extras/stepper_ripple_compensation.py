# Stepper position correction for motor torque ripple
#
# Copyright (C) 2026  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, cmath
import chelper
from stepper import MAX_STEPCOMPRESS_ERROR as STOCK_MAX_STEPCOMPRESS_ERROR

MAX_HARMONIC = 4
TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
                    "tmc5160"]
# Maximum slope of the correction, d_correction / d_position
MAX_CORRECTION_SLOPE = .5
MAX_STEPCOMPRESS_ERROR = 0.0000025
MAX_NODES = 16

# Speed nodes of a harmonic are a list of speed, positive and negative
# direction correction tuples. The speed in mm/s, the corrections in um.
def format_node(node):
    speed, corr_pos, corr_neg = node
    return "%g, %.3f, %.1f, %.3f, %.1f" % (
        speed, abs(corr_pos), math.degrees(cmath.phase(corr_pos)) % 360.,
        abs(corr_neg), math.degrees(cmath.phase(corr_neg)) % 360.)

# Rows of speed, amplitude and phase of the positive and the negative
# direction
def build_nodes(rows):
    nodes = []
    for row in rows:
        if len(row) != 5:
            raise ValueError("expected speed, amplitude, phase, amplitude,"
                             " phase")
        speed, amp_pos, phase_pos, amp_neg, phase_neg = row
        if amp_pos < 0. or amp_neg < 0.:
            raise ValueError("amplitudes must not be negative")
        nodes.append((speed, cmath.rect(amp_pos, math.radians(phase_pos)),
                      cmath.rect(amp_neg, math.radians(phase_neg))))
    if len(nodes) > MAX_NODES:
        raise ValueError("too many nodes, maximum is %d" % (MAX_NODES,))
    for n1, n2 in zip(nodes, nodes[1:]):
        if n2[0] <= n1[0]:
            raise ValueError("speeds must increase")
    if any(n[0] < 0. for n in nodes):
        raise ValueError("speeds must not be negative")
    return nodes

# Rows separated by '/' in a G-Code parameter
def parse_nodes(text):
    return build_nodes([[float(v) for v in row.split(',')]
                        for row in text.split('/') if row.strip()])

def format_sections(sections):
    msg = []
    for section in sections:
        msg.append("stepper_ripple_compensation %s: %s" % (
            section.name, "enabled" if section.enabled else "disabled"))
        for k in range(1, MAX_HARMONIC + 1):
            nodes = section.harmonic_nodes.get(k)
            if nodes:
                msg.append("h%d_nodes:" % (k,))
                msg += ["  " + format_node(n) for n in nodes]
    return msg

class StepperRippleCompensation:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split(None, 1)[-1]
        self.max_step_error = config.getfloat(
            'max_step_error', MAX_STEPCOMPRESS_ERROR, above=0.,
            maxval=STOCK_MAX_STEPCOMPRESS_ERROR)
        self.harmonic_nodes = {}
        for k in range(1, MAX_HARMONIC + 1):
            rows = config.getlists('h%d_nodes' % (k,), None,
                                   seps=(',', '\n'), count=5, parser=float)
            if rows is None:
                continue
            try:
                nodes = build_nodes(rows)
            except ValueError as e:
                raise config.error("Option 'h%d_nodes' in section '%s': %s"
                                   % (k, config.get_name(), str(e)))
            if nodes:
                self.harmonic_nodes[k] = nodes
        self.saved_harmonics = set(self.harmonic_nodes)
        self.enabled = True
        self.stepper = self.tmc = self.electrical_period = None
        self.unshaped_sk = self.ripple_sk = None
        self.phase_origin = self.phase_period = None
        self.applied_nodes = None
        printer.register_event_handler("klippy:mcu_identify",
                                       self._handle_mcu_identify)
        printer.register_event_handler("klippy:connect",
                                       self._handle_connect)
        printer.register_event_handler("klippy:ready",
                                       self._handle_ready)
        printer.register_event_handler("input_shaper:update_kinematics",
                                       self._handle_shaper_update)
        gcode = printer.lookup_object('gcode')
        gcode.register_mux_command(
            "SET_STEPPER_RIPPLE_COMPENSATION", "NAME", self.name,
            self.cmd_SET_STEPPER_RIPPLE_COMPENSATION,
            desc=self.cmd_SET_STEPPER_RIPPLE_COMPENSATION_help)
    def _handle_mcu_identify(self):
        force_move = self.printer.lookup_object('force_move')
        self.stepper = force_move.lookup_stepper(self.name)
        self.stepper.setup_max_step_error(self.max_step_error)
        self.unshaped_sk = self.stepper.get_stepper_kinematics()
        for driver in TRINAMIC_DRIVERS:
            self.tmc = self.printer.lookup_object(
                '%s %s' % (driver, self.name), None)
            if self.tmc is not None:
                break
        else:
            raise self.printer.config_error(
                "stepper_ripple_compensation %s: no tmc driver for the"
                " stepper" % (self.name,))
    def _handle_connect(self):
        self.electrical_period = (self.tmc.get_phase_offset()[1]
                                  * self.stepper.get_step_dist())
        self._check_slope(self.harmonic_nodes, self.printer.config_error)
    # Wrap the stepper after input_shaper, which wraps it on connect
    def _handle_ready(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(ffi_lib.stepper_ripple_alloc(), ffi_lib.free)
        # The kinematics without input shaping give the direction
        if ffi_lib.stepper_ripple_set_sk(
                sk, self.stepper.get_stepper_kinematics(),
                self.unshaped_sk) < 0:
            raise self.printer.config_error(
                "Unable to setup ripple compensation for stepper '%s'"
                % (self.name,))
        self.stepper.set_stepper_kinematics(sk)
        self.ripple_sk = sk
        motion_queuing = self.printer.lookup_object('motion_queuing')
        motion_queuing.register_flush_callback(self._sync_correction)
        motion_queuing.check_step_generation_scan_windows()
        self._sync_correction()
    # Follow the step generation windows of the wrapped kinematics, modified
    # input_shaper now sends the glob event before it updates the scan windows
    def _handle_shaper_update(self):
        if self.ripple_sk is not None:
            ffi_main, ffi_lib = chelper.get_ffi()
            ffi_lib.stepper_ripple_update_sk(self.ripple_sk)
    # A too steep correction could move the corrected position backwards
    def _check_slope(self, harmonic_nodes, error):
        slope = 0.
        for k, nodes in harmonic_nodes.items():
            amp = max(max(abs(cp), abs(cn)) for v, cp, cn in nodes)
            slope += amp * .001 * k * 2. * math.pi / self.electrical_period
        if slope > MAX_CORRECTION_SLOPE:
            raise error("stepper_ripple_compensation %s: correction is too"
                        " large for the electrical period %.3fmm"
                        % (self.name, self.electrical_period))
    # Step flush callback, as the phase changes when the motor is enabled
    def _sync_correction(self, flush_time=None, step_gen_time=None):
        ffi_main, ffi_lib = chelper.get_ffi()
        mcu_phase_offset = self.tmc.get_phase_offset()[0]
        origin = period = None
        if mcu_phase_offset is not None:
            origin = self.stepper.mcu_to_commanded_position(-mcu_phase_offset)
            period = self.electrical_period
            # Keep the origin small to avoid loss of precision
            origin -= math.floor(origin / period) * period
        if (origin, period) != (self.phase_origin, self.phase_period):
            self.phase_origin, self.phase_period = origin, period
            if period is not None:
                ffi_lib.stepper_ripple_set_phase(self.ripple_sk, origin, period)
        harmonic_nodes = {}
        if self.enabled and period is not None:
            harmonic_nodes = self.harmonic_nodes
        if harmonic_nodes != self.applied_nodes:
            for k in range(1, MAX_HARMONIC + 1):
                nodes = harmonic_nodes.get(k, [])
                ffi_lib.stepper_ripple_set_nodes(
                    self.ripple_sk, k, len(nodes), [v for v, cp, cn in nodes],
                    [cp.real * .001 for v, cp, cn in nodes],
                    [cp.imag * .001 for v, cp, cn in nodes],
                    [cn.real * .001 for v, cp, cn in nodes],
                    [cn.imag * .001 for v, cp, cn in nodes])
            self.applied_nodes = dict(harmonic_nodes)
    def set_nodes(self, harmonic_nodes, enabled=True):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        # Values from numpy can't be reported by get_status
        harmonic_nodes = {k: [(float(v), complex(cp), complex(cn))
                              for v, cp, cn in nodes]
                          for k, nodes in harmonic_nodes.items() if nodes}
        self._check_slope(harmonic_nodes, self.printer.command_error)
        self.harmonic_nodes = harmonic_nodes
        self.enabled = enabled
        self._sync_correction()
    def save_nodes(self):
        configfile = self.printer.lookup_object('configfile')
        section = 'stepper_ripple_compensation ' + self.name
        for k in range(1, MAX_HARMONIC + 1):
            nodes = self.harmonic_nodes.get(k)
            if nodes:
                configfile.set(section, 'h%d_nodes' % (k,), '\n' + '\n'.join(
                    format_node(n) for n in nodes))
            elif k in self.saved_harmonics:
                configfile.set(section, 'h%d_nodes' % (k,), '')
        self.saved_harmonics = set(self.harmonic_nodes)
    cmd_SET_STEPPER_RIPPLE_COMPENSATION_help = "Set stepper ripple compensation"
    def cmd_SET_STEPPER_RIPPLE_COMPENSATION(self, gcmd):
        harmonic_nodes = dict(self.harmonic_nodes)
        for k in range(1, MAX_HARMONIC + 1):
            text = gcmd.get('H%d_NODES' % (k,), None)
            if text is None:
                continue
            try:
                nodes = parse_nodes(text)
            except ValueError as e:
                raise gcmd.error("Invalid H%d_NODES value '%s': %s"
                                 % (k, text, str(e)))
            if nodes:
                harmonic_nodes[k] = nodes
            else:
                harmonic_nodes.pop(k, None)
        enabled = gcmd.get_int('ENABLE', 1 if self.enabled else 0,
                               minval=0, maxval=1)
        save = gcmd.get_int('SAVE', 0, minval=0, maxval=1)
        self.set_nodes(harmonic_nodes, enabled=bool(enabled))
        if save:
            self.save_nodes()
        msg = format_sections([self])
        if self.phase_period is None:
            msg.insert(1, "electrical phase unknown, motor not enabled")
        gcmd.respond_info("\n".join(msg))
    def get_status(self, eventtime):
        return {
            'enabled': self.enabled,
            'phase_origin': self.phase_origin,
            'phase_period': self.phase_period,
            'nodes': {'h%d' % (k,): [
                [v, abs(cp), math.degrees(cmath.phase(cp)) % 360.,
                 abs(cn), math.degrees(cmath.phase(cn)) % 360.]
                for v, cp, cn in nodes]
                      for k, nodes in self.harmonic_nodes.items()}}


def load_config_prefix(config):
    return StepperRippleCompensation(config)
