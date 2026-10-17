# Calibration of the stepper ripple compensation with an accelerometer
#
# Copyright (C) 2026  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, cmath, os, re, time, bisect
import numpy as np
from .stepper_ripple_compensation import (
    MAX_HARMONIC, MAX_CORRECTION_SLOPE, harmonic_correction, max_amplitude,
    format_harmonic, format_model, format_sections)

TEMP_RESULTS_FOLDER = ('~/printer_data/config/adxl_results/'
                       'stepper_ripple_compensation/')

# Square of the stepper speed in mm/s below which kin_stepper_ripple.c blends
# the directions
DIRECTION_SMOOTH_V2 = 4.
TEST_AXES = ('DIAG', 'X', 'Y')

# Share of the positive direction correction at a signed stepper speed
def direction_weight(v):
    return .5 * (1. + v / math.sqrt(v*v + DIRECTION_SMOOTH_V2))

# Ratio of the neighbor speeds of the calibration grid and its smallest size
SPEED_RATIO = 1.2
MIN_SPEED_COUNT = 4

# Speeds from min_speed to max_speed with a constant ratio of neighbors
def speed_grid(min_speed, max_speed):
    count = max(MIN_SPEED_COUNT - 1, int(math.ceil(
        math.log(max_speed / min_speed) / math.log(SPEED_RATIO) - 1e-9)))
    return sorted(set(round(min_speed * (max_speed / min_speed)**(
        float(i) / count), 1) for i in range(count + 1)))

def save_result(text, prefix, label):
    folder = os.path.expanduser(TEMP_RESULTS_FOLDER)
    os.makedirs(folder, exist_ok=True)
    suffix = '_' + re.sub(r'[^A-Za-z0-9_.-]', '_', label) if label else ''
    path = os.path.join(folder, '%s_%s%s.txt' % (
        prefix, time.strftime('%Y%m%d_%H%M%S'), suffix))
    with open(path, 'w') as f:
        f.write(text + '\n')
    return path

MIN_SEGMENT_SAMPLES = 200
# Highest analyzed harmonic frequency relative to the sample rate
MAX_FREQ_TO_RATE = .4

# Median vibration and lock-in amplitudes of each accelerometer axis of
# the test move segments relative to the stepper electrical phase
def analyze_segments(batches, rest_offset, segments, phase_origin,
                     phase_period, harmonics, error):
    # One batch per numpy call gives the reactor the GIL between them
    data = [np.array(b, dtype=float) for b in batches if b]
    if not data:
        raise error("No accelerometer data")
    data = np.concatenate(data)
    data = data[np.argsort(data[:,0], kind='stable')]
    times = data[:,0]
    results = {}
    for key, t0, t1, pos0, pos1 in segments:
        i0, i1 = np.searchsorted(times, [t0, t1])
        if i1 - i0 < MIN_SEGMENT_SAMPLES:
            raise error("Not enough accelerometer data")
        t = times[i0:i1]
        accel = data[i0:i1, 1:4] - np.array(rest_offset)
        median = float(np.median(np.linalg.norm(accel, axis=1)))
        lockin = {}
        if phase_period is not None:
            rate = (len(t) - 1) / (t[-1] - t[0])
            freq = abs(pos1 - pos0) / (t1 - t0) / phase_period
            stepper_pos = pos0 + (pos1 - pos0) * ((t - t0) / (t1 - t0))
            theta = 2. * math.pi * (stepper_pos - phase_origin) / phase_period
            # Linear trend of all the axes removed at once
            idx = np.arange(len(t))
            trend = np.polyfit(idx, accel, 1)
            detrended = accel - (np.outer(idx, trend[0]) + trend[1])
            window = np.hanning(len(t))
            for k in harmonics:
                if k * freq > MAX_FREQ_TO_RATE * rate:
                    continue
                weights = window * np.exp(-1j * k * theta)
                lockin[k] = [complex(a) for a in weights.dot(detrended)
                             * 2. / np.sum(window)]
        results[key] = {'median': median, 'lockin': lockin}
    return results

# Fit of the corrections and the shared speed model to the measured
# vibrations. A test row is the vibration of one accelerometer axis in
# a test move, the response to a correction change D is B*D + Bm*D'
# with D' conjugate to D.

def _model_arrays(model, f):
    if model is None:
        return np.ones(len(f), dtype=complex), np.ones(len(f), dtype=complex)
    f0, q, delay = model
    x = f / f0
    return 1. / (1. - x*x + 1j*x/q), np.exp(-2j*math.pi*f*delay)

# Ridge toward the current coefficients relative to the median effect of
# a low speed coefficient on the tests
RIDGE = .001
# A coefficient with less effect on the tests than this many times the
# ridge is reported as poorly determined
WEAK_INFORMATION = 20.

# Least squares of the coefficients of one harmonic with a speed model, the
# unknowns are the real and the imaginary parts of the correction of each
# section, direction and part
def _solve_harmonic(hd, model):
    nparts = 2 if model is not None else 1
    ncoef = len(hd['names']) * 2 * nparts
    h, rot = _model_arrays(model, hd['f'])
    rows = len(hd['a0'])
    M = np.zeros((rows, 2 * ncoef), dtype=complex)
    for di, wd in ((0, hd['w_pos']), (1, 1. - hd['w_pos'])):
        for pi, wm in enumerate((rot * h, rot * (1. - h))[:nparts]):
            w = wd * wm
            col = (hd['name_idx'] * 2 + di) * nparts + pi
            re = hd['b'] * w[:, None] + hd['bm'] * w.conj()[:, None]
            im = 1j * (hd['b'] * w[:, None] - hd['bm'] * w.conj()[:, None])
            r = (hd['test_idx'][:, None] * 3 + np.arange(3)).ravel()
            np.add.at(M, (r, np.repeat(2 * col, 3)), re.ravel())
            np.add.at(M, (r, np.repeat(2 * col + 1, 3)), im.ravel())
    W = hd['weight']
    Mr = np.vstack([(M * W[:, None]).real, (M * W[:, None]).imag])
    Ar = np.concatenate([(hd['a0'] * W).real, (hd['a0'] * W).imag])
    # Ridge scaled by the median effect of a low speed coefficient
    norms = np.sqrt(np.sum(Mr**2, axis=0))
    s0 = float(np.median(norms[0::2 * nparts])) or 1.
    ref = np.zeros(2 * ncoef)
    for n, coeffs in enumerate(hd['current']):
        for di in range(2):
            for pi in range(nparts):
                c = coeffs[2 * di + pi]
                j = 2 * ((n * 2 + di) * nparts + pi)
                ref[j], ref[j + 1] = c.real, c.imag
    pen = math.sqrt(RIDGE) * s0
    # Normal equations, the ridge keeps them well conditioned
    A = np.vstack([Mr, pen * np.eye(2 * ncoef)])
    y = np.concatenate([-Ar, pen * ref])
    x = np.linalg.solve(A.T.dot(A), A.T.dot(y))
    cost = float(np.sum((A.dot(x) - y)**2))
    weak = [j for j in range(ncoef) if norms[2*j]**2 + norms[2*j + 1]**2
            < WEAK_INFORMATION * RIDGE * s0**2]
    coeffs = {}
    for n, name in enumerate(hd['names']):
        c = []
        for di in range(2):
            for pi in range(2):
                j = (n * 2 + di) * nparts + min(pi, nparts - 1)
                c.append(complex(x[2 * j], x[2 * j + 1]))
        coeffs[name] = tuple(c)
    weak = [(hd['names'][j // (2 * nparts)], ('low', 'high')[j % nparts],
             1. if (j // nparts) % 2 == 0 else -1.) for j in weak]
    return {'cost': cost, 'coeffs': coeffs, 'weak': weak,
            'pred': hd['a0'] + M.dot(x[:2 * ncoef])}

def _solve_all(data, model):
    sols = {k: _solve_harmonic(hd, model) for k, hd in data.items()}
    return sum(sol['cost'] for sol in sols.values()), sols

# Largest correction slope of the sections relative to slope_limit
def _slope_ratio(data, sols, model, slope_limit):
    slope = {}
    for k, hd in data.items():
        for name, c in sols[k]['coeffs'].items():
            amp = max_amplitude(c, model)
            slope[name] = slope.get(name, 0.) + (
                amp * .001 * k * 2. * math.pi / hd['periods'][name])
    return max(slope.values() or [0.]) / slope_limit

# Largest ratio of the correction between the tested frequencies to the
# largest one at a tested frequency
def _peak_ratio(data, sols, model):
    worst = 0.
    for k, hd in data.items():
        fmax = float(np.max(hd['f']))
        grid = np.linspace(0., 1.5 * fmax, 600)
        h, rot = _model_arrays(model, grid)
        ht, rt = _model_arrays(model, hd['f'])
        for name, c in sols[k]['coeffs'].items():
            for lo, hi in ((c[0], c[1]), (c[2], c[3])):
                amp = np.max(np.abs(hi + (lo - hi) * h))
                tested = np.max(np.abs(hi + (lo - hi) * ht))
                if tested > 1e-9:
                    worst = max(worst, float(amp / tested))
    return worst

# Speed model search grid, the transition has no sharp resonance
MODEL_F0_POINTS = 16
MODEL_QS = (.3, .5, .9, 1.5)
MODEL_DELAYS = tuple(i * .0001 for i in range(-3, 4))
MODEL_REFINE_ROUNDS = 30

def _search_model(data, freqs):
    f0s = np.geomspace(.5*np.min(freqs), 2.*np.max(freqs), MODEL_F0_POINTS)
    best = None
    for f0 in f0s:
        for q in MODEL_QS:
            for delay in MODEL_DELAYS:
                model = (float(f0), q, delay)
                cost = _solve_all(data, model)[0]
                if best is None or cost < best[0]:
                    best = (cost, model)
    # Coordinate refinement with shrinking steps inside the grid range
    cost, (f0, q, delay) = best
    steps = [1.1, 1.25, MODEL_DELAYS[1] - MODEL_DELAYS[0]]
    for i in range(MODEL_REFINE_ROUNDS):
        improved = False
        for trial in ((f0 * steps[0], q, delay), (f0 / steps[0], q, delay),
                      (f0, q * steps[1], delay), (f0, q / steps[1], delay),
                      (f0, q, delay + steps[2]), (f0, q, delay - steps[2])):
            if (not f0s[0] <= trial[0] <= f0s[-1]
                    or not MODEL_QS[0] <= trial[1] <= MODEL_QS[-1]
                    or not MODEL_DELAYS[0] <= trial[2] <= MODEL_DELAYS[-1]):
                continue
            c = _solve_all(data, trial)[0]
            if c < cost:
                cost, (f0, q, delay) = c, trial
                improved = True
        if not improved:
            steps = [math.sqrt(steps[0]), math.sqrt(steps[1]), .5 * steps[2]]
    return cost, (f0, q, delay)

# Distinct tested frequencies needed to fit a speed model
MIN_MODEL_FREQS = 4
# Largest correction between the tested frequencies relative to the largest
# one at a tested frequency
MAX_PEAK_RATIO = 1.5
# Relative cost reduction a speed model needs against no model
MIN_MODEL_GAIN = .02

# Fit of all the harmonics with the best speed model, or with the given
# one when the fitted model does not pass the checks
def fit_corrections(data, model, slope_limit):
    freqs = np.concatenate([hd['f'] for hd in data.values()])
    if len(np.unique(np.round(freqs, 3))) >= MIN_MODEL_FREQS:
        cost, trial = _search_model(data, freqs)
        sols = _solve_all(data, trial)[1]
        if (_peak_ratio(data, sols, trial) <= MAX_PEAK_RATIO
                and _slope_ratio(data, sols, trial, slope_limit) <= 1.):
            model = trial
            if _solve_all(data, None)[0] <= cost * (1. + MIN_MODEL_GAIN):
                model = None
    return {'model': model, 'sols': _solve_all(data, model)[1]}

# Accelerometer samples from the start of the buffer, taken by time
class AccelSampleBuffer:
    def __init__(self, printer, chip, error):
        self.reactor = printer.get_reactor()
        self.error = error
        self.batches = []
        self.finished = False
        self.first_overflows = None
        self.overflows = 0
        chip.batch_bulk.add_client(self._handle_batch)
    def _handle_batch(self, msg):
        if self.finished:
            return False
        overflows = msg.get('overflows', 0)
        if self.first_overflows is None:
            self.first_overflows = overflows
        if overflows != self.first_overflows:
            self.overflows = overflows - self.first_overflows
            self.finished = True
            return False
        if msg['data']:
            self.batches.append(tuple(msg['data']))
        return True
    def take_until(self, end_time):
        eventtime = self.reactor.monotonic()
        timeout = eventtime + 3.
        while True:
            if self.overflows:
                raise self.error("Accelerometer lost samples in %d"
                                 " overflows, check its connection and"
                                 " data rate" % (self.overflows,))
            if self.batches and self.batches[-1][-1][0] >= end_time:
                break
            if eventtime > timeout:
                raise self.error("Timeout waiting for accelerometer data")
            eventtime = self.reactor.pause(eventtime + .100)
        batches = self.batches
        i = 0
        while i < len(batches) and batches[i][-1][0] <= end_time:
            i += 1
        taken = batches[:i]
        del batches[:i]
        if batches:
            j = bisect.bisect_right(batches[0], (end_time, float('inf')))
            if j:
                taken.append(batches[0][:j])
                batches[0] = batches[0][j:]
        return taken

AXIS_LIMIT_MARGIN = 1.
# Time at the start and the end of the cruise that is not analyzed in
# addition to the input shaper smoothing, the acceleration ringing decays
CRUISE_SKIP_TIME = 0.050
# Shortest analyzed part of a test move
MIN_ANALYZED_TIME = 0.200
ACCEL_MOVE_TIME = 1.5
REST_MEASURE_TIME = 1.

# Test moves at constant speeds along an axis, or diagonally
class AccelTestMoves:
    def __init__(self, printer, gcmd, params, section, executor, axis='DIAG',
                 position=None):
        self.printer = printer
        self.section = section
        self.executor = executor
        self.reactor = printer.get_reactor()
        self.gcode = printer.lookup_object('gcode')
        self.toolhead = toolhead = printer.lookup_object('toolhead')
        self.chip_name, self.chip = params['chip_name'], params['chip']
        self.speeds = params['speeds']
        kin = toolhead.get_kinematics()
        kin_status = kin.get_status(printer.get_reactor().monotonic())
        if any(a not in kin_status['homed_axes'] for a in 'xy'):
            raise gcmd.error("Must home the X and Y axes first")
        self.z = toolhead.get_position()[2]
        self.stepper = section.stepper
        gx, gy = self.stepper_gradient(self.stepper)
        if axis != 'DIAG':
            dx, dy = (1., 0.) if axis == 'X' else (0., 1.)
        else:
            # Head direction in which the other steppers do not turn
            dx, dy = gx, gy
            for stepper in kin.get_steppers():
                if stepper is section.stepper:
                    continue
                ox, oy = self.stepper_gradient(stepper)
                if abs(gx*oy - gy*ox) > .001:
                    dx, dy = -oy, ox
                    break
        norm = math.sqrt(dx*dx + dy*dy)
        stepper_per_head = ((gx*dx + gy*dy) / norm if norm > .000001 else 0.)
        if abs(stepper_per_head) < .001:
            raise gcmd.error("Stepper %s does not move with the X and Y axes"
                             % (self.stepper.get_name(),))
        if stepper_per_head < 0.:
            dx, dy, stepper_per_head = -dx, -dy, -stepper_per_head
        self.head_dir = (dx / norm, dy / norm)
        self.stepper_per_head = stepper_per_head
        amin, amax = kin_status['axis_minimum'], kin_status['axis_maximum']
        center = []
        avail = []
        for i, d in enumerate(self.head_dir):
            if abs(d) > .000001:
                center.append(.5 * (amin[i] + amax[i]))
                avail.append((amax[i] - amin[i] - 2. * AXIS_LIMIT_MARGIN)
                             / abs(d))
                continue
            if position is None:
                position = .5 * (amin[i] + amax[i])
            center.append(position)
        max_v, max_a = toolhead.get_max_velocity()
        max_speed = max(self.speeds)
        max_head_v = max_speed / stepper_per_head
        if max_head_v > max_v:
            raise gcmd.error("Speed %.1f is above max_velocity" % (max_speed,))
        self.center = center
        self.travel_speed = .5 * max_v
        motion_queuing = printer.lookup_object('motion_queuing')
        self.cruise_margin = (CRUISE_SKIP_TIME
                              + motion_queuing.get_kin_flush_delay())
        # The speeds whose moves do not fit the axes get shorter moves
        self.move_timing = {}
        for v in self.speeds:
            head_v = v / stepper_per_head
            move_time = min(params['move_time'], min(avail) / head_v)
            t_acc = head_v / max_a
            total = move_time + t_acc
            skip = t_acc + self.cruise_margin
            if total - 2. * skip < MIN_ANALYZED_TIME:
                if move_time < params['move_time']:
                    raise gcmd.error(
                        "Speed %.1f is too high for the axis length, the"
                        " analyzed time of its test move is below %.2fs"
                        % (v, MIN_ANALYZED_TIME))
                raise gcmd.error("The test move is too short for speed %.1f"
                                 ", increase MOVE_TIME" % (v,))
            self.move_timing[v] = (head_v, move_time, t_acc, total, skip)
    # The moves of all the speeds are centered on the same point
    def start_pos(self, v):
        head_v, move_time = self.move_timing[v][:2]
        half = .5 * head_v * move_time
        return [c - half * d for c, d in zip(self.center, self.head_dir)]
    def move_time_text(self):
        times = [t[1] for t in self.move_timing.values()]
        if max(times) - min(times) < .005:
            return "%.2fs" % (max(times),)
        return "%.2f-%.2fs" % (min(times), max(times))
    # Stepper distance per head distance along X and along Y
    def stepper_gradient(self, stepper):
        calc_pos = stepper.calc_position_from_coord
        dummy_d = 100.
        p0 = calc_pos([0., 0., 0.])
        return ((calc_pos([dummy_d, 0., 0.]) - p0) / dummy_d,
                (calc_pos([0., dummy_d, 0.]) - p0) / dummy_d)
    def describe(self, states):
        est = 1. + sum(len(states) * 2. * self.move_timing[v][3]
                       for v in self.speeds)
        return ("Vibration test of %s with %s: %d speeds, %d states, head"
                " direction %.3f,%.3f, center %.3f,%.3f, move time %s,"
                " estimated time %.1f min" % (
                    self.section.name, self.chip_name, len(self.speeds),
                    len(states), self.head_dir[0], self.head_dir[1],
                    self.center[0], self.center[1], self.move_time_text(),
                    est / 60.))
    def _measure_rest_offset(self, gcmd, sample_buffer):
        toolhead = self.toolhead
        t0 = toolhead.get_last_move_time()
        toolhead.dwell(REST_MEASURE_TIME)
        t1 = toolhead.get_last_move_time()
        toolhead.wait_moves()
        rest = np.array([s for b in sample_buffer.take_until(t1)
                         for s in b if s[0] >= t0], dtype=float)
        if not len(rest):
            raise gcmd.error("No accelerometer data")
        return [float(a) for a in np.median(rest[:,1:], axis=0)]
    # Verify the move timing against the step history once the moves run
    def _check_timing(self, eventtime, moves):
        mcu = self.stepper.get_mcu()
        end_time = max(seg['t1'] for seg in moves)
        while mcu.estimated_print_time(eventtime) < end_time:
            eventtime = self.reactor.pause(eventtime + .100)
        for seg in moves:
            for t, want in ((seg['t0'], seg['pos0']), (seg['t1'], seg['pos1'])):
                got = self.stepper.mcu_to_commanded_position(
                    self.stepper.get_past_mcu_position(t))
                if abs(got - want) <= .05:
                    continue
                return ("Unexpected move timing on %s: position=%.3f "
                        "expected=%.3f" % (self.stepper.get_name(), got, want))
        return None
    # Forward and backward moves at a speed in each state without stops,
    # the move timing is checked in the background
    def _run_moves_at_speed(self, gcmd, v, states, apply_state):
        head_v, move_time, t_acc, total, skip = self.move_timing[v]
        start = self.start_pos(v)
        end = [p + d * head_v * move_time for p, d in zip(start, self.head_dir)]
        self.toolhead.manual_move(start, self.travel_speed)
        segments = []
        checks = []
        for state in states:
            if apply_state is not None:
                apply_state(state)
            moves = []
            for direction, p0, p1 in ((1., start, end), (-1., end, start)):
                self.toolhead.manual_move(p1, head_v)
                # The start of a move after a pause is not known in advance,
                # use the end time of the move instead
                t_end = self.toolhead.get_last_move_time()
                t_start = t_end - total
                seg = {'speed_dir': (v, direction), 'state': state,
                       't0': t_start + skip, 't1': t_end - skip}
                # Stepper position at constant speed
                for tk, pk in (('t0', 'pos0'), ('t1', 'pos1')):
                    d = head_v * (seg[tk] - t_start - .5 * t_acc)
                    x, y = [p + direction * u * d
                            for p, u in zip(p0, self.head_dir)]
                    seg[pk] = self.stepper.calc_position_from_coord(
                        [x, y, self.z])
                moves.append(seg)
            checks.append(self.reactor.register_callback(
                lambda e, moves=moves: self._check_timing(e, moves)))
            segments.extend(moves)
        return segments, checks
    def _analyze(self, sample_buffer, end_time, args):
        try:
            batches = sample_buffer.take_until(end_time)
            res = self.executor.submit(analyze_segments, batches, *args)
            return res, None
        except Exception as e:
            return None, e
    def _finish_analysis(self, gcmd, pending, results, report):
        v, checks, completion = pending
        for check in checks:
            error = check.wait()
            if error is not None:
                raise gcmd.error(error)
        speed_results, error = completion.wait()
        if error is not None:
            raise error
        results.update(speed_results)
        report(v, speed_results)
    # Waiting for the data and the analysis in their own reactor callback
    def _start_analysis(self, gcmd, sample_buffer, end_time, rest_offset,
                        jobs, harmonics):
        args = (rest_offset, jobs, self.section.phase_origin,
                self.section.phase_period, harmonics, gcmd.error)
        return self.printer.get_reactor().register_callback(
            lambda eventtime: self._analyze(sample_buffer, end_time, args))
    # Measure and analyze the test moves of each speed in each state,
    # apply_state sets the correction of a state
    def measure(self, gcmd, harmonics, report, states=(None,),apply_state=None):
        self.toolhead.manual_move(self.start_pos(self.speeds[0]),
                                  self.travel_speed)
        self.toolhead.wait_moves()
        sample_buffer = None
        results = {}
        try:
            sample_buffer = AccelSampleBuffer(self.printer, self.chip,
                                              gcmd.error)
            rest_offset = self._measure_rest_offset(gcmd, sample_buffer)
            # The data of a speed is analyzed while the next speed runs
            pending = None
            for v in self.speeds:
                segments, checks = self._run_moves_at_speed(gcmd, v, states,
                                                            apply_state)
                jobs = [((seg['speed_dir'], seg['state']), seg['t0'],
                         seg['t1'], seg['pos0'], seg['pos1'])
                        for seg in segments]
                if pending is not None:
                    self._finish_analysis(gcmd, pending, results, report)
                pending = (v, checks, self._start_analysis(
                    gcmd, sample_buffer, max(seg['t1'] for seg in segments),
                    rest_offset, jobs, harmonics))
            if pending is not None:
                self._finish_analysis(gcmd, pending, results, report)
        finally:
            if sample_buffer is not None:
                sample_buffer.finished = True
            if apply_state is not None:
                apply_state(None)
        return results

# A test along an axis with the sections that turn in it, their
# speed relative to the stepper of the test and their direction
class PlannedTest:
    def __init__(self, axis, moves, turning, axis_pos):
        self.axis = axis
        self.moves = moves
        self.turning = turning
        self.axis_pos = axis_pos
        self.states = ['current'] + [(s.name, a) for s, sr, sd in turning
                                     for a in (0., 90.)]

# Corrections of a speed model refitted for another model over a range of
# frequencies, used for the harmonics that are not calibrated
def refit_coeffs(coeffs, old_model, new_model, f_lo, f_hi):
    f = np.geomspace(max(f_lo, 1.), max(f_hi, 2.), 50)
    h, rot = _model_arrays(new_model, f)
    result = []
    for lo, hi in ((coeffs[0], coeffs[1]), (coeffs[2], coeffs[3])):
        target = np.array([harmonic_correction((lo, hi, lo, hi), old_model,
                                               ff)[0] for ff in f])
        if new_model is None:
            c = complex(np.mean(target))
            result += [c, c]
            continue
        A = np.stack([rot * h, rot * (1. - h)], axis=1)
        x = np.linalg.lstsq(A, target, rcond=None)[0]
        result += [complex(x[0]), complex(x[1])]
    return tuple(result)

# Share of the slope limit for the fitted and the test corrections
MAX_SLOPE_USE = .9

# One STEPPER_RIPPLE_CALIBRATE run of the harmonics of the sections
class RippleCalibration:
    def __init__(self, calibrate, gcmd, sections, harmonics, test_amplitude,
                 max_speed):
        self.printer = calibrate.printer
        self.executor = calibrate.executor
        self.gcmd = gcmd
        self.sections = sections
        self.harmonics = harmonics
        self.test_amplitude = test_amplitude
        self.saved_state = {s.name: (dict(s.harmonics), s.speed_model,
                                     s.max_speed, s.enabled)
                            for s in sections}
        self.current = {name: harmonics if enabled else {}
                        for name, (harmonics, model, ms, enabled)
                        in self.saved_state.items()}
        # A narrower calibration keeps the fade of the earlier one
        self.max_speed = max([max_speed] + [s.max_speed for s in sections
                                            if s.max_speed is not None])
        self.models = {s.name: s.speed_model for s in sections}
        self.periods = {s.name: s.electrical_period for s in sections}
        self.plan = []
        self.keys = {}
        self.obs = {}
    def plan_tests(self, params):
        for axis in TEST_AXES:
            if axis == 'DIAG':
                tests = []
                for section in self.sections:
                    moves = AccelTestMoves(self.printer, self.gcmd,
                                           params, section, self.executor)
                    if all(abs(moves.head_dir[0] - m.head_dir[0]) > .001
                           or abs(moves.head_dir[1] - m.head_dir[1]) > .001
                           for m, p in tests):
                        tests.append((moves, None))
            else:
                moves = AccelTestMoves(self.printer, self.gcmd, params,
                                       self.sections[0], self.executor, axis)
                tests = [(moves, moves.center[1 if axis == 'X' else 0])]
            for moves, axis_pos in tests:
                turning = []
                for section in self.sections:
                    gx, gy = moves.stepper_gradient(section.stepper)
                    r = gx * moves.head_dir[0] + gy * moves.head_dir[1]
                    if abs(r) > .001:
                        turning.append((section, abs(r)/moves.stepper_per_head,
                                        math.copysign(1., r)))
                self.plan.append(PlannedTest(axis, moves, turning, axis_pos))
                # The phase between the sections depends on the position on
                # the other axis, the second position shifts it by 90
                # degrees for the highest harmonic
                if (axis_pos is not None and len(tests) == 1
                        and len(turning) > 1):
                    pos2 = axis_pos + self.sections[0].phase_period / (
                        4. * max(self.harmonics))
                    tests.append((AccelTestMoves(
                        self.printer, self.gcmd, params, self.sections[0],
                        self.executor, axis, pos2), pos2))
    # The test correction is added to all the calibrated harmonics
    def apply_state(self, state):
        for section in self.sections:
            if state is None:
                harmonics, model, max_speed, enabled = self.saved_state[
                    section.name]
                section.set_correction(harmonics, model, max_speed,
                                       enabled=enabled)
                continue
            offsets = {}
            if state != 'current' and state[0] == section.name:
                t = cmath.rect(self.test_amplitude, math.radians(state[1]))
                offsets = {k: t for k in self.harmonics}
            section.set_correction(self.current[section.name],
                                   self.models[section.name], self.max_speed,
                                   offsets=offsets)
    # Slope left to a fitted correction next to the test corrections
    def slope_limit(self):
        test = max(sum(self.test_amplitude * .001 * k * 2. * math.pi
                       / self.periods[s.name] for k in self.harmonics)
                   for s in self.sections)
        return max(MAX_SLOPE_USE * MAX_CORRECTION_SLOPE - test, .01)
    # The corrections the sections apply from now on, not saved
    def set_current(self, model, result):
        for name, harmonics in result.items():
            self.current[name] = harmonics
            self.models[name] = model
        for section in self.sections:
            section.set_correction(self.current[section.name],
                                   self.models[section.name], self.max_speed)
    # Correction of a section in a state at a stepper speed and direction
    def _state_corr(self, state, name, k, speed, direction):
        c = 0j
        coeffs = self.current[name].get(k)
        if coeffs:
            w_pos = direction_weight(speed * direction)
            cp, cn = harmonic_correction(coeffs, self.models[name],
                                         k * speed / self.periods[name])
            c = w_pos * cp + (1. - w_pos) * cn
        if state != 'current' and state[0] == name:
            c += cmath.rect(self.test_amplitude, math.radians(state[1]))
        return c
    # Observations of the planned tests with the corrections applied in them
    def measure(self, full=True):
        for pi, test in enumerate(self.plan):
            moves = test.moves
            ref_section = moves.section.name
            states = test.states if full else ['current']
            report = lambda v, speed_results: self.gcmd.respond_info(
                "%s %s %.1f mm/s done" % (test.axis, ref_section, v))
            results = moves.measure(self.gcmd, self.harmonics, report,
                                    states, self.apply_state)
            for v in moves.speeds:
                for direction in (1., -1.):
                    for k in self.harmonics:
                        turning = [(section.name, v * sr, direction * sd)
                                   for section, sr, sd in test.turning]
                        key = (pi, v, direction, k)
                        info = self.keys.setdefault(key, {
                            'axis': test.axis, 'ref_section': ref_section,
                            'axis_pos': test.axis_pos, 'speed': v,
                            'direction': direction, 'k': k,
                            'turning': turning})
                        for st in states:
                            a = results[((v, direction), st)]['lockin'].get(k)
                            # Above the accelerometer bandwidth
                            if a is None:
                                continue
                            corr = [self._state_corr(st, name, k, sv, sd)
                                    for name, sv, sd in turning]
                            self.obs.setdefault(key, []).append((corr, a))
                            if st == 'current':
                                info['measured'] = a
    # Least squares of a = a0 + sum(B * c + Bm * c') over the observations
    def estimate_tests(self):
        tests = []
        resid = {}
        for key, obs in self.obs.items():
            info = self.keys[key]
            nsec = len(info['turning'])
            if len(obs) < 1 + 2 * nsec:
                continue
            X = np.array([[1.] + [v for c in corr for v in (c, c.conjugate())]
                          for corr, a in obs], dtype=complex)
            A = np.array([a for corr, a in obs], dtype=complex)
            coef = np.linalg.lstsq(X, A, rcond=None)[0]
            dof = len(obs) - X.shape[1]
            if dof > 0:
                ss, n = resid.get(info['k'], (0., 0))
                resid[info['k']] = (ss + float(np.sum(np.abs(
                    A - X.dot(coef))**2)), n + dof)
            response = [(turning, list(zip(map(complex, coef[1 + 2*i]),
                                           map(complex, coef[2 + 2*i]))))
                        for i, turning in enumerate(info['turning'])]
            test = dict(info)
            test['a0'] = [complex(v) for v in coef[0]]
            test['response'] = response
            tests.append(test)
        noise = {k: math.sqrt(ss / n) for k, (ss, n) in resid.items()}
        return tests, noise
    # Arrays of the tests of each harmonic for fit_corrections
    def fit_data(self, tests):
        data = {}
        for k in self.harmonics:
            ktests = [t for t in tests if t['k'] == k]
            if not ktests:
                continue
            names = sorted(set(key[0] for t in ktests
                               for key, resp in t['response']))
            index = {name: n for n, name in enumerate(names)}
            ent = {'test_idx': [], 'name_idx': [], 'f': [], 'w_pos': [],
                   'b': [], 'bm': []}
            a0 = []
            for j, t in enumerate(ktests):
                a0 += t['a0']
                for (name, speed, direction), axis_resp in t['response']:
                    ent['test_idx'].append(j)
                    ent['name_idx'].append(index[name])
                    ent['f'].append(k * speed / self.periods[name])
                    ent['w_pos'].append(direction_weight(speed * direction))
                    ent['b'].append([b for b, bm in axis_resp])
                    ent['bm'].append([bm for b, bm in axis_resp])
            hd = {key: np.array(val) for key, val in ent.items()}
            hd['a0'] = np.array(a0, dtype=complex)
            hd['names'] = names
            hd['periods'] = {name: self.periods[name] for name in names}
            hd['current'] = [self.current[name].get(k, (0j,) * 4)
                             for name in names]
            # Weight of the tests of one speed, axis and reference section,
            # the inverse of their rms vibration without the correction
            groups = {}
            for j, t in enumerate(ktests):
                key = (t['speed'], t['axis'], t['ref_section'])
                groups.setdefault(key, []).append(j)
            weight = np.ones(len(ktests))
            for js in groups.values():
                rms = math.sqrt(np.mean(np.abs(hd['a0'].reshape(-1, 3)[js])
                                        ** 2) * 3.)
                weight[js] = 1. / rms if rms > 0. else 1.
            hd['weight'] = np.repeat(weight, 3)
            hd['tests'] = ktests
            data[k] = hd
        return data
    # New corrections of the sections, the other harmonics are refitted to
    # a new speed model
    def result_harmonics(self, data, fit):
        model = fit['model']
        freqs = np.concatenate([hd['f'] for hd in data.values()])
        f_lo, f_hi = float(np.min(freqs)), float(np.max(freqs))
        result = {}
        for section in self.sections:
            name = section.name
            harmonics = {}
            for k, coeffs in self.current[name].items():
                if model != self.models[name]:
                    coeffs = refit_coeffs(coeffs, self.models[name], model,
                                          f_lo, f_hi)
                harmonics[k] = coeffs
            for k, sol in fit['sols'].items():
                if name in sol['coeffs']:
                    harmonics[k] = sol['coeffs'][name]
            result[name] = harmonics
        return result
    def apply_result(self, model, result):
        for section in self.sections:
            section.set_correction(result[section.name], model,
                                   self.max_speed)
            section.save_correction()

MAX_ITERATIONS = 5
# Relative improvement an iteration needs
AUTO_MIN_IMPROVEMENT = .05

# Iterations that apply the fitted correction and measure only it
class AutoCalibration:
    def __init__(self, calibrate, cal, grid):
        self.calibrate = calibrate
        self.cal = cal
        self.gcmd = cal.gcmd
        self.grid = list(grid)
        self.history = []
        self.best_iteration = None
    # Rms vibration of each harmonic with the current correction
    @staticmethod
    def _measured(tests):
        meas = {}
        for t in tests:
            if 'measured' in t:
                meas.setdefault(t['k'], []).append(
                    sum(abs(a)**2 for a in t['measured']))
        return {k: math.sqrt(np.mean(v)) for k, v in meas.items()}
    # Rms vibration of each harmonic predicted with the fitted correction
    @staticmethod
    def _predicted(fit):
        return {k: math.sqrt(np.mean(np.sum(np.abs(sol['pred']).reshape(
            -1, 3)**2, axis=1))) for k, sol in fit['sols'].items()}
    @staticmethod
    def _total(values):
        return math.sqrt(sum(v**2 for v in values.values()))
    def run(self, max_iterations):
        cal, gcmd = self.cal, self.gcmd
        cal.measure(full=True)
        best = None
        prev_total = None
        for it in range(1, max_iterations + 1):
            tests, noise = cal.estimate_tests()
            data = cal.fit_data(tests)
            if not data:
                raise gcmd.error("No harmonic is below the accelerometer"
                                 " bandwidth at the tested speeds")
            measured = self._measured(tests)
            total = self._total(measured)
            model = cal.models[cal.sections[0].name]
            fit = self.calibrate.executor.submit(fit_corrections, data,
                                                 model, cal.slope_limit())
            result = cal.result_harmonics(data, fit)
            pred = self._predicted(fit)
            state = (dict(cal.models), {n: dict(h)
                                        for n, h in cal.current.items()})
            self.history.append((it, measured, pred, noise, fit['model']))
            gcmd.respond_info(self._format_iteration(self.history[-1]))
            if best is None or total < best[0]:
                best = (total, state, fit, data, it)
            # Stop when the fit expects no gain or when an iteration no
            # longer helps
            if self._total(pred) > total * (1. - AUTO_MIN_IMPROVEMENT):
                gcmd.respond_info("Stopping, the fit expects less than %.0f%%"
                                  " lower vibrations"
                                  % (AUTO_MIN_IMPROVEMENT * 100.,))
                break
            if (prev_total is not None
                    and total > prev_total * (1. - AUTO_MIN_IMPROVEMENT)):
                gcmd.respond_info("Stopping, the last iteration did not lower"
                                  " the vibrations")
                break
            if it == max_iterations:
                break
            prev_total = total
            try:
                cal.set_current(fit['model'], result)
            except gcmd.error as e:
                gcmd.respond_info("Stopping, the fitted correction can not"
                                  " be applied: %s" % (e,))
                break
            cal.measure(full=False)
        # Correction with the lowest measured vibration and its fit
        total, (models, current), fit, data, self.best_iteration = best
        return models[cal.sections[0].name], current, fit, data
    def _format_iteration(self, entry):
        it, measured, pred, noise, model = entry
        def fmt(values):
            return " ".join("h%d=%.0f" % (k, v)
                            for k, v in sorted(values.items()))
        return ("Iteration %d: measured %s, predicted %s, noise %s,"
                " speed model %s" % (
                    it, fmt(measured), fmt(pred), fmt(noise) or "unknown",
                    format_model(model) if model is not None else "none"))
    def summary(self):
        msg = ["Automatic ripple calibration, rms vibration in mm/s^2 at the"
               " speeds %s" % (", ".join("%g" % v for v in self.grid),)]
        msg += [self._format_iteration(entry) for entry in self.history]
        return msg

# Vibration tests and the calibration of the stepper_ripple_compensation
class StepperRippleCalibrate:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.accel_chip = config.get('accel_chip', '').strip() or None
        printer.register_event_handler("klippy:connect", self._handle_connect)
        aio = printer.load_object(config, 'aio_executor')
        self.executor = aio.allocate_executor("stepper_ripple")
        gcode = printer.lookup_object('gcode')
        gcode.register_command(
            "STEPPER_RIPPLE_VIBRATION_MEASURE",
            self.cmd_STEPPER_RIPPLE_VIBRATION_MEASURE,
            desc=self.cmd_STEPPER_RIPPLE_VIBRATION_MEASURE_help)
        gcode.register_command(
            "STEPPER_RIPPLE_CALIBRATE",
            self.cmd_STEPPER_RIPPLE_CALIBRATE,
            desc=self.cmd_STEPPER_RIPPLE_CALIBRATE_help)
    def _all_sections(self):
        return [section for name, section in self.printer.lookup_objects(
            'stepper_ripple_compensation')]
    def _lookup_chip(self, chip_name, error):
        chip = self.printer.lookup_object(chip_name, None)
        if chip is None or not hasattr(chip, 'batch_bulk'):
            raise error("Unknown accelerometer '%s'" % (chip_name,))
        return chip
    def _handle_connect(self):
        if self.accel_chip is not None:
            self._lookup_chip(self.accel_chip, self.printer.config_error)
    def _parse_test_params(self, gcmd, speeds=None):
        chip_name = gcmd.get('ACCEL_CHIP', self.accel_chip)
        if chip_name is None:
            raise gcmd.error("ACCEL_CHIP is required when accel_chip is not"
                             " set in [stepper_ripple_calibrate]")
        chip_name = chip_name.strip()
        chip = self._lookup_chip(chip_name, gcmd.error)
        text = gcmd.get('SPEEDS', None)
        if speeds is not None:
            pass
        elif text is not None:
            try:
                speeds = [float(s) for s in text.split(',') if s.strip()]
            except ValueError:
                raise gcmd.error("Invalid SPEEDS")
        else:
            min_speed = gcmd.get_float('MIN_SPEED', above=0.)
            max_speed = gcmd.get_float('MAX_SPEED', minval=min_speed)
            step = gcmd.get_float('SPEED_STEP', 1., above=0.)
            count = int((max_speed - min_speed) / step + .000001)
            speeds = [round(min_speed + i*step, 6) for i in range(count + 1)]
        if not speeds or min(speeds) <= 0.:
            raise gcmd.error("Invalid SPEEDS")
        move_time = gcmd.get_float('MOVE_TIME', ACCEL_MOVE_TIME, above=0.)
        return {'chip_name': chip_name, 'chip': chip, 'speeds': speeds,
                'move_time': move_time}
    def _lookup_sections(self, gcmd):
        names = gcmd.get('NAME', None)
        if names is None:
            return self._all_sections()
        by_name = {s.name: s for s in self._all_sections()}
        sections = []
        for name in [n.strip() for n in names.split(',') if n.strip()]:
            if name not in by_name:
                raise gcmd.error("Unknown stepper_ripple_compensation '%s'"
                                 % (name,))
            sections.append(by_name[name])
        return sections
    def _format_table(self, moves, results):
        section = moves.section
        msg = ["Ripple vibration test, %s, %s, head direction %.3f,%.3f,"
               " center %.3f,%.3f, move time %s" % (
                   section.name, moves.chip_name, moves.head_dir[0],
                   moves.head_dir[1], moves.center[0],
                   moves.center[1], moves.move_time_text())]
        msg += format_sections([section] + [s for s in self._all_sections()
                                            if s is not section])
        msg.append("units: mm/s^2")
        msg.append("median, hN: forward moves, median_rev, hN_rev: backward"
                   " moves, hN: amplitude over the X/Y/Z axes")
        rot_dist = moves.stepper.get_rotation_distance()[0]
        table = [['speed', 'rpm', 'median', 'median_rev']
                 + ['h%d%s' % (k, s) for k in range(1, MAX_HARMONIC + 1)
                    for s in ('', '_rev')]]
        for v in moves.speeds:
            fwd, rev = [results[((v, d), None)] for d in (1., -1.)]
            row = ['%g' % (v,), '%.1f' % (v / rot_dist * 60.),
                   '%.1f' % (fwd['median'],), '%.1f' % (rev['median'],)]
            for k in range(1, MAX_HARMONIC + 1):
                for r in (fwd, rev):
                    amps = r['lockin'].get(k)
                    row.append('-' if amps is None else '%.1f' % (
                        math.sqrt(sum(abs(a)**2 for a in amps)),))
            table.append(row)
        widths = [max(len(row[i]) for row in table)
                  for i in range(len(table[0]))]
        msg += ['  '.join(c.rjust(w) for c, w in zip(row, widths))
                for row in table]
        return "\n".join(msg)
    cmd_STEPPER_RIPPLE_VIBRATION_MEASURE_help = \
        "Measure the vibrations of a stepper at constant speeds"
    def cmd_STEPPER_RIPPLE_VIBRATION_MEASURE(self, gcmd):
        sections = self._lookup_sections(gcmd)
        if len(sections) != 1:
            raise gcmd.error("NAME must specify one"
                             " stepper_ripple_compensation")
        label = gcmd.get('LABEL', '')
        axis = gcmd.get('AXIS', 'DIAG').strip().upper()
        if axis not in TEST_AXES:
            raise gcmd.error("AXIS must be DIAG, X or Y")
        moves = AccelTestMoves(self.printer, gcmd,
                               self._parse_test_params(gcmd), sections[0],
                               self.executor, axis)
        gcmd.respond_info(moves.describe([None]))
        def report(v, speed_results):
            gcmd.respond_info("%.1f mm/s: forward %.0f, backward %.0f" % (
                v, speed_results[((v, 1.), None)]['median'],
                speed_results[((v, -1.), None)]['median']))
        results = moves.measure(gcmd, list(range(1, MAX_HARMONIC + 1)), report)
        text = self._format_table(moves, results)
        try:
            path = save_result(text, 'stepper_ripple_vibration', label)
        except Exception as e:
            raise gcmd.error("%s\nUnable to save the table: %s" % (text, e))
        gcmd.respond_info("Table saved to: %s" % (path,))
    cmd_STEPPER_RIPPLE_CALIBRATE_help = (
        "Calibrate stepper ripple compensation over a range of speeds")
    def cmd_STEPPER_RIPPLE_CALIBRATE(self, gcmd):
        min_speed = gcmd.get_float('MIN_SPEED', above=0.)
        max_speed = gcmd.get_float('MAX_SPEED', above=min_speed)
        max_iterations = gcmd.get_int('MAX_ITERATIONS', MAX_ITERATIONS,
                                      minval=1)
        apply_result = gcmd.get_int('APPLY', 1, minval=0, maxval=1)
        label = gcmd.get('LABEL', '')
        sections = self._lookup_sections(gcmd)
        try:
            harmonics = sorted(set(int(k) for k in gcmd.get(
                'HARMONICS', '1,2,4').split(',') if k.strip()))
        except ValueError:
            raise gcmd.error("Invalid HARMONICS")
        if not harmonics or harmonics[0] < 1 or harmonics[-1] > MAX_HARMONIC:
            raise gcmd.error("HARMONICS must be a list of 1 to %d"
                             % (MAX_HARMONIC,))
        test_amplitude = gcmd.get_float('AMPLITUDE', 4., above=0.)
        if not sections:
            raise gcmd.error("No stepper_ripple_compensation sections")
        for section in sections:
            if section.phase_period is None:
                raise gcmd.error("Electrical phase of %s is unknown"
                                 % (section.name,))
        grid = speed_grid(min_speed, max_speed)
        cal = RippleCalibration(self, gcmd, sections, harmonics, test_amplitude,
                                max_speed)
        cal.plan_tests(self._parse_test_params(gcmd, grid))
        gcmd.respond_info(
            "Accelerometer ripple calibration of harmonics %s of %s,"
            " %d test states per speed\n%s"
            % (",".join(map(str, harmonics)),
               ", ".join(s.name for s in sections),
               sum(len(test.states) for test in cal.plan),
               "\n".join(test.moves.describe(test.states)
                         for test in cal.plan)))
        auto = AutoCalibration(self, cal, grid)
        model, result, fit, data = auto.run(max_iterations)
        msg = ["Using the correction measured in iteration %d"
               % (auto.best_iteration,)]
        try:
            msg.append("Test results saved to: %s" % (save_result(
                "\n".join(auto.summary() + self._format_table_fit(data, fit)),
                'stepper_ripple_calibrate', label),))
        except Exception as e:
            msg.append("Unable to save the test results: %s" % (e,))
        msg += self._format_result(fit, model, cal.max_speed, result)
        if apply_result:
            cal.apply_result(model, result)
            msg.append("New values applied, use SAVE_CONFIG to store them.")
        else:
            cal.apply_state(None)
        gcmd.respond_info("\n".join(msg))
    def _format_result(self, fit, model, max_speed, result):
        msg = []
        weak = ["%s h%d %s%s" % (name, k, part, '+' if d > 0. else '-')
                for k, sol in sorted(fit['sols'].items())
                for name, part, d in sol['weak']]
        if weak:
            msg.append("Poorly determined coefficients, kept nearer the"
                       " current ones: " + ", ".join(weak))
        for name, harmonics_res in result.items():
            msg.append("[stepper_ripple_compensation %s]" % (name,))
            if model is not None:
                msg.append("speed_model: " + format_model(model))
            msg.append("max_speed: %.1f" % (max_speed,))
            for k in sorted(harmonics_res):
                msg.append("h%d:" % (k,))
                msg += ["  " + row for row in format_harmonic(harmonics_res[k])]
        return msg
    # Table of all the tests in mm/s^2
    def _format_table_fit(self, data, fit):
        table = ["axis position speed dir harmonic measured pred no_corr"]
        for k, hd in sorted(data.items()):
            pred = np.linalg.norm(fit['sols'][k]['pred'].reshape(-1, 3), axis=1)
            a0 = np.linalg.norm(hd['a0'].reshape(-1, 3), axis=1)
            meas = [np.linalg.norm(t['measured']) for t in hd['tests']]
            for t, m, p, o in zip(hd['tests'], meas, pred, a0):
                where = t['ref_section']
                if t['axis_pos'] is not None:
                    where = "%.2f" % (t['axis_pos'],)
                table.append("%s %s %.1f %s h%d %.0f %.0f %.0f" % (
                    t['axis'], where, t['speed'],
                    '+' if t['direction'] > 0. else '-', k, m, p, o))
        return table

def load_config(config):
    return StepperRippleCalibrate(config)
