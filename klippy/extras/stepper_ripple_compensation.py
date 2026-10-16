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
# Frequencies relative to f0 at which the peak of the correction is searched
PEAK_SEARCH_POINTS = 600
PEAK_SEARCH_DECADES = 3.

# The correction of a harmonic is the tuple of the low and the high speed
# corrections of the positive and then of the negative direction, complex
# numbers in microns. The speed model is (f0, q, delay) or None, above
# max_speed the correction fades out.

# Response of the speed model at the harmonic frequency f, the transition
# from the low to the high speed correction and the delay rotation
def model_response(model, f):
    if model is None:
        return 1., 1.
    f0, q, delay = model
    x = f / f0
    h = 1. / complex(1. - x * x, x / q)
    return h, cmath.exp(complex(0., -2. * math.pi * f * delay))

# Corrections of the positive and the negative direction at a frequency
def harmonic_correction(coeffs, model, f):
    h, rot = model_response(model, f)
    low_pos, high_pos, low_neg, high_neg = coeffs
    return (rot * (high_pos + (low_pos - high_pos) * h),
            rot * (high_neg + (low_neg - high_neg) * h))

# Largest correction amplitude of a harmonic over all speeds
def max_amplitude(coeffs, model):
    if model is None:
        return max(abs(coeffs[0]), abs(coeffs[2]))
    amp = max(abs(c) for c in coeffs)
    for i in range(PEAK_SEARCH_POINTS + 1):
        x = 10. ** (PEAK_SEARCH_DECADES * (2. * i / PEAK_SEARCH_POINTS - 1.))
        amp = max(amp, max(abs(c) for c in harmonic_correction(
            coeffs, model, model[0] * x)))
    return max(amp, max(abs(c) for c in harmonic_correction(
        coeffs, model, model[0])))

# Rows of the positive and the negative direction, each the amplitude
# and the phase at low speed, optionally followed by those at high speed
def build_harmonic(rows):
    if not 1 <= len(rows) <= 2:
        raise ValueError("expected one or two rows")
    corrs = []
    for row in rows:
        if len(row) not in (2, 4):
            raise ValueError("expected amplitude, phase and optionally the"
                             " high speed amplitude, phase")
        if any(a < 0. for a in row[::2]):
            raise ValueError("amplitudes must not be negative")
        low = cmath.rect(row[0], math.radians(row[1]))
        high = cmath.rect(row[2], math.radians(row[3])) if len(row) == 4 \
            else low
        corrs.append((low, high))
    if len(corrs) == 1:
        corrs.append(corrs[0])
    return corrs[0] + corrs[1]

# Rows separated by '/' in a G-Code parameter
def parse_harmonic(text):
    return build_harmonic([[float(v) for v in row.split(',')]
                           for row in text.split('/') if row.strip()])

def parse_model(values):
    if len(values) != 3:
        raise ValueError("expected f0, q, delay")
    f0, q, delay = values
    if f0 <= 0. or q <= 0.:
        raise ValueError("f0 and q must be positive")
    return (f0, q, delay)

def format_corr(c):
    return "%.3f, %.1f" % (abs(c), math.degrees(cmath.phase(c)) % 360.)

# One row when both directions are equal, the high speed correction only
# when it differs
def format_harmonic(coeffs):
    rows = []
    for low, high in (coeffs[:2], coeffs[2:]):
        row = format_corr(low)
        if abs(high - low) > .0005:
            row += ", " + format_corr(high)
        rows.append(row)
    return rows[:1] if rows[0] == rows[1] else rows

def format_model(model):
    return "%.1f, %.3f, %.7f" % model

def format_sections(sections):
    msg = []
    for section in sections:
        msg.append("stepper_ripple_compensation %s: %s" % (
            section.name, "enabled" if section.enabled else "disabled"))
        if section.speed_model is not None:
            msg.append("speed_model: " + format_model(section.speed_model))
        if section.max_speed is not None:
            msg.append("max_speed: %.1f" % (section.max_speed,))
        for k in range(1, MAX_HARMONIC + 1):
            coeffs = section.harmonics.get(k)
            if coeffs:
                msg.append("h%d:" % (k,))
                msg += ["  " + row for row in format_harmonic(coeffs)]
    return msg

class StepperRippleCompensation:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split(None, 1)[-1]
        self.max_step_error = config.getfloat(
            'max_step_error', MAX_STEPCOMPRESS_ERROR, above=0.,
            maxval=STOCK_MAX_STEPCOMPRESS_ERROR)
        self.harmonics = {}
        for k in range(1, MAX_HARMONIC + 1):
            rows = config.getlists('h%d' % (k,), None, seps=(',', '\n'),
                                   parser=float)
            if not rows:
                continue
            try:
                self.harmonics[k] = build_harmonic(rows)
            except ValueError as e:
                raise config.error("Option 'h%d' in section '%s': %s"
                                   % (k, config.get_name(), str(e)))
        self.speed_model = None
        values = config.getfloatlist('speed_model', None)
        if values:
            try:
                self.speed_model = parse_model(values)
            except ValueError as e:
                raise config.error("Option 'speed_model' in section '%s': %s"
                                   % (config.get_name(), str(e)))
        self.max_speed = config.getfloat('max_speed', None, above=0.)
        self.saved_options = set(self._saved_values())
        self.enabled = True
        self.test_offsets = {}
        self.stepper = self.tmc = self.electrical_period = None
        self.unshaped_sk = self.ripple_sk = None
        self.phase_origin = self.phase_period = None
        self.applied = None
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
        self._check_slope(self.harmonics, self.speed_model, {},
                          self.printer.config_error)
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
    def _check_slope(self, harmonics, model, offsets, error):
        slope = 0.
        for k in range(1, MAX_HARMONIC + 1):
            amp = abs(offsets.get(k, 0j))
            if k in harmonics:
                amp += max_amplitude(harmonics[k], model)
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
        applied = ({}, None, None, {})
        if self.enabled and period is not None:
            applied = (self.harmonics, self.speed_model, self.max_speed,
                       self.test_offsets)
        if applied == self.applied:
            return
        harmonics, model, max_speed, offsets = applied
        f0, q, delay = model or (0., 0., 0.)
        ffi_lib.stepper_ripple_set_model(self.ripple_sk, f0, q, delay,
                                         max_speed or 0.)
        for k in range(1, MAX_HARMONIC + 1):
            coeffs = harmonics.get(k, (0j,) * 4)
            low = [coeffs[0], coeffs[2]]
            high = [coeffs[1], coeffs[3]]
            offset = [offsets.get(k, 0j)] * 2
            ffi_lib.stepper_ripple_set_harmonic(
                self.ripple_sk, k, *[[v for c in cs for v in (
                    c.real * .001, c.imag * .001)]
                                    for cs in (low, high, offset)])
        self.applied = (dict(harmonics), model, max_speed, dict(offsets))
    # Offsets are added to the correction without the speed model, they
    # are the test corrections of the calibration
    def set_correction(self, harmonics, speed_model, max_speed, enabled=True,
                       offsets=None):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        # Values from numpy can't be reported by get_status
        harmonics = {k: tuple(complex(c) for c in coeffs)
                     for k, coeffs in harmonics.items() if coeffs}
        if speed_model is not None:
            speed_model = tuple(float(v) for v in speed_model)
        offsets = {k: complex(c) for k, c in (offsets or {}).items() if c}
        self._check_slope(harmonics, speed_model, offsets,
                          self.printer.command_error)
        self.harmonics = harmonics
        self.speed_model = speed_model
        self.max_speed = max_speed
        self.enabled = enabled
        self.test_offsets = offsets
        self._sync_correction()
    def _saved_values(self):
        values = {'h%d' % (k,): '\n' + '\n'.join(format_harmonic(coeffs))
                  for k, coeffs in self.harmonics.items()}
        if self.speed_model is not None:
            values['speed_model'] = format_model(self.speed_model)
        if self.max_speed is not None:
            values['max_speed'] = "%.1f" % (self.max_speed,)
        return values
    def save_correction(self):
        configfile = self.printer.lookup_object('configfile')
        section = 'stepper_ripple_compensation ' + self.name
        values = self._saved_values()
        for option in self.saved_options | set(values):
            configfile.set(section, option, values.get(option, ''))
        self.saved_options = set(values)
    cmd_SET_STEPPER_RIPPLE_COMPENSATION_help = "Set stepper ripple compensation"
    def cmd_SET_STEPPER_RIPPLE_COMPENSATION(self, gcmd):
        harmonics = dict(self.harmonics)
        for k in range(1, MAX_HARMONIC + 1):
            text = gcmd.get('H%d' % (k,), None)
            if text is None:
                continue
            if not text.strip():
                harmonics.pop(k, None)
                continue
            try:
                harmonics[k] = parse_harmonic(text)
            except ValueError as e:
                raise gcmd.error("Invalid H%d value '%s': %s"
                                 % (k, text, str(e)))
        speed_model = self.speed_model
        text = gcmd.get('SPEED_MODEL', None)
        if text is not None:
            speed_model = None
            if text.strip():
                try:
                    speed_model = parse_model(
                        [float(v) for v in text.split(',')])
                except ValueError as e:
                    raise gcmd.error("Invalid SPEED_MODEL value '%s': %s"
                                     % (text, str(e)))
        max_speed = self.max_speed
        text = gcmd.get('MAX_SPEED', None)
        if text is not None:
            max_speed = None
            if text.strip():
                max_speed = gcmd.get_float('MAX_SPEED', above=0.)
        enabled = gcmd.get_int('ENABLE', 1 if self.enabled else 0,
                               minval=0, maxval=1)
        save = gcmd.get_int('SAVE', 0, minval=0, maxval=1)
        self.set_correction(harmonics, speed_model, max_speed,
                            enabled=bool(enabled))
        if save:
            self.save_correction()
        msg = format_sections([self])
        if self.phase_period is None:
            msg.insert(1, "electrical phase unknown, motor not enabled")
        gcmd.respond_info("\n".join(msg))
    def get_status(self, eventtime):
        def polar(c):
            return [abs(c), math.degrees(cmath.phase(c)) % 360.]
        return {
            'enabled': self.enabled,
            'phase_origin': self.phase_origin,
            'phase_period': self.phase_period,
            'speed_model': (list(self.speed_model)
                            if self.speed_model is not None else None),
            'max_speed': self.max_speed,
            'harmonics': {'h%d' % (k,): [polar(c) for c in coeffs]
                          for k, coeffs in self.harmonics.items()}}


def load_config_prefix(config):
    return StepperRippleCompensation(config)
