# Calibration of the stepper ripple compensation with an accelerometer
#
# Copyright (C) 2026  Maksim Bolgov <maksim8024@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, cmath, os, re, time, bisect
import numpy as np
from .stepper_ripple_compensation import (MAX_HARMONIC, MAX_NODES,
                                          format_node, format_sections)

TEMP_RESULTS_FOLDER = ('~/printer_data/config/adxl_results/'
                       'stepper_ripple_compensation/')

# Square of the stepper speed in mm/s below which kin_stepper_ripple.c blends
# the directions
DIRECTION_SMOOTH_V2 = 4.
AXIS_LIMIT_MARGIN = 1.
# Time at the start and the end of the cruise that is not analyzed in
# addition to the input shaper smoothing, the acceleration ringing decays
CRUISE_SKIP_TIME = 0.100
# Shortest analyzed part of a test move
MIN_ANALYZED_TIME = 0.200
MIN_SEGMENT_SAMPLES = 200
ACCEL_AXES = ('x', 'y', 'z')
ACCEL_MOVE_TIME = 1.5
REST_MEASURE_TIME = 1.
# Highest analyzed harmonic frequency relative to the sample rate
MAX_FREQ_TO_RATE = .4
TEST_AXES = ('DIAG', 'X', 'Y')
# Relative singular value below which a node change is not determined
MIN_RELATIVE_SV = .05

# Positive and negative direction corrections at a speed, interpolated as
# in kin_stepper_ripple.c
def interpolate_nodes(nodes, speed):
    if speed <= nodes[0][0]:
        return nodes[0][1], nodes[0][2]
    for (v1, p1, n1), (v2, p2, n2) in zip(nodes, nodes[1:]):
        if speed < v2:
            f = (speed - v1) / (v2 - v1)
            return p1 + f * (p2 - p1), n1 + f * (n2 - n1)
    return nodes[-1][1], nodes[-1][2]

def save_result(text, prefix, label):
    folder = os.path.expanduser(TEMP_RESULTS_FOLDER)
    os.makedirs(folder, exist_ok=True)
    suffix = '_' + re.sub(r'[^A-Za-z0-9_.-]', '_', label) if label else ''
    path = os.path.join(folder, '%s_%s%s.txt' % (
        prefix, time.strftime('%Y%m%d_%H%M%S'), suffix))
    with open(path, 'w') as f:
        f.write(text + '\n')
    return path

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

# Node changes that minimize the vibrations of all the tests together, the
# response to a correction change D is B*D + Bm*D' with D' conjugate to D
def solve_node_changes(tests, change_model):
    naxes = len(ACCEL_AXES)
    # The real and the imaginary part of the change of each node
    nodes = sorted(set(node for test in tests
                       for key, axis_resp in test['response']
                       for node, w in change_model[key][0]))
    index = {node: j for j, node in enumerate(nodes)}
    # One complex equation per test and accelerometer axis, the vibration with
    # the new nodes at the current correction plus effect of the node changes
    ripple_off, rows_a, rows_m = [], [], []
    for test in tests:
        a_off, a_new = list(test['ripple']), list(test['ripple'])
        rows = np.zeros((naxes, 2 * len(nodes)), dtype=complex)
        for key, axis_resp in test['response']:
            columns, d_new, d_off = change_model[key]
            for i, (b, bm) in enumerate(axis_resp):
                a_off[i] += b * d_off + bm * d_off.conjugate()
                a_new[i] += b * d_new + bm * d_new.conjugate()
                # A real change x of a node gives b*x + bm*x, an imaginary
                # change i*x gives b*i*x - bm*i*x
                for node, w in columns:
                    j = index[node]
                    rows[i, 2*j] += w * (b + bm)
                    rows[i, 2*j+1] += w * 1j * (b - bm)
        ripple_off.append(math.sqrt(sum(abs(a)**2 for a in a_off)))
        rows_a += a_new
        rows_m.append(rows)
    A = np.array(rows_a, dtype=complex)
    M = np.concatenate(rows_m) if rows_m else np.zeros((0, 2 * len(nodes)))
    # Weight of the tests of one speed, axis and reference section, the inverse
    # of their rms vibration without the correction
    group_keys = [(t['speed'], t['axis'], t['ref_section']) for t in tests]
    groups = {}
    for key, off in zip(group_keys, ripple_off):
        groups.setdefault(key, []).append(off**2)
    group_weight = {key: 1. / math.sqrt(sum(sq) / len(sq))
                    if sum(sq) > 0. else 1.
                    for key, sq in groups.items()}
    W = np.repeat([group_weight[key] for key in group_keys], naxes)
    x = np.zeros(2 * len(nodes))
    weak = []
    if nodes:
        # Truncated least squares of the real and the imaginary equations,
        # with the same scale of each node so that a node which affects
        # more tests is not taken as better determined
        real_m = np.vstack([M.real * W[:, None], M.imag * W[:, None]])
        real_a = np.concatenate([A.real * W, A.imag * W])
        scale = np.sqrt(np.sum(real_m**2, axis=0).reshape(-1, 2)
                        .sum(axis=1)).repeat(2)
        scale[scale == 0.] = 1.
        u, sv, vt = np.linalg.svd(real_m / scale, full_matrices=False)
        # Solve A + M*x = 0 only in the directions with a significant
        # effect on the vibrations, a node that mostly lies in the dropped
        # directions is reported
        keep = sv > MIN_RELATIVE_SV * sv[0]
        x = vt[keep].T.dot(u[:, keep].T.dot(-real_a) / sv[keep]) / scale
        drop = vt[~keep]
        weak = [node for node, j in index.items()
                if float(np.sum(drop[:, 2*j:2*j+2]**2)) > .5]
    pred = np.sqrt(np.sum(np.abs(A + M.dot(x)).reshape(-1, naxes)**2, axis=1))
    return {'delta': {node: complex(x[2*j], x[2*j+1])
                      for node, j in index.items()},
            'pred': [float(p) for p in pred], 'weak': sorted(weak),
            'pred_off': ripple_off}

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
            elif not amin[i] <= position <= amax[i]:
                raise gcmd.error("POSITION %.3f is outside of the %s axis"
                                 % (position, 'xy'[i]))
            center.append(position)
        max_v, max_a = toolhead.get_max_velocity()
        max_speed = max(self.speeds)
        max_head_v = max_speed / stepper_per_head
        if max_head_v > max_v:
            raise gcmd.error("Speed %.1f is above max_velocity"
                             % (max_speed,))
        self.move_time = min(params['move_time'], min(avail) / max_head_v)
        max_move_length = max_head_v * self.move_time
        self.start_pos = [c - .5 * max_move_length * d
                          for c, d in zip(center, self.head_dir)]
        self.travel_speed = .5 * max_v
        self.move_accel = params['accel'] or max_a
        motion_queuing = printer.lookup_object('motion_queuing')
        cruise_margin = CRUISE_SKIP_TIME + motion_queuing.get_kin_flush_delay()
        self.move_timing = {}
        for v in self.speeds:
            head_v = v / stepper_per_head
            t_acc = head_v / self.move_accel
            total = self.move_time + t_acc
            skip = t_acc + cruise_margin
            if total - 2. * skip < MIN_ANALYZED_TIME:
                raise gcmd.error("The test move is too short for speed %.1f"
                                 ", increase ACCEL or MOVE_TIME" % (v,))
            self.move_timing[v] = (head_v, t_acc, total, skip)
    # Stepper distance per head distance along X and along Y
    def stepper_gradient(self, stepper):
        calc_pos = stepper.calc_position_from_coord
        dummy_d = 100.
        p0 = calc_pos([0., 0., 0.])
        return ((calc_pos([dummy_d, 0., 0.]) - p0) / dummy_d,
                (calc_pos([0., dummy_d, 0.]) - p0) / dummy_d)
    def describe(self, states):
        est = 1. + sum(len(states) * 2. * self.move_timing[v][2]
                       for v in self.speeds)
        return ("Vibration test of %s with %s: %d speeds, %d states, head"
                " direction %.3f,%.3f, start %.3f,%.3f, move time %.2fs,"
                " estimated time %.1f min" % (
                    self.section.name, self.chip_name, len(self.speeds),
                    len(states), self.head_dir[0], self.head_dir[1],
                    self.start_pos[0], self.start_pos[1], self.move_time,
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
        head_v, t_acc, total, skip = self.move_timing[v]
        start = self.start_pos
        end = [p + d * head_v * self.move_time
               for p, d in zip(start, self.head_dir)]
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
                lambda eventtime, moves=moves: \
                    self._check_timing(eventtime, moves)))
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
        old_accel = self.toolhead.get_max_velocity()[1]
        self.toolhead.manual_move(self.start_pos, self.travel_speed)
        self.toolhead.wait_moves()
        sample_buffer = None
        results = {}
        try:
            if self.move_accel != old_accel:
                self.gcode.run_script_from_command(
                    "SET_VELOCITY_LIMIT ACCEL=%.3f" % (self.move_accel,))
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
            if self.move_accel != old_accel:
                self.gcode.run_script_from_command(
                    "SET_VELOCITY_LIMIT ACCEL=%.3f" % (old_accel,))
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

# One STEPPER_RIPPLE_CALIBRATE run of a harmonic of the sections
class RippleCalibration:
    def __init__(self, calibrate, gcmd, sections, harmonic, test_amplitude):
        self.printer = calibrate.printer
        self.executor = calibrate.executor
        self.gcmd = gcmd
        self.sections = sections
        self.harmonic = harmonic
        self.test_amplitude = test_amplitude
        self.saved_state = {s.name: (dict(s.harmonic_nodes), s.enabled)
                            for s in sections}
        self.current_nodes = {name: nodes if enabled else {}
                              for name, (nodes, enabled)
                              in self.saved_state.items()}
        self.plan = []
        self.new_nodes = {}
    def plan_tests(self, axes, params, position):
        for axis in axes:
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
                moves = AccelTestMoves(
                    self.printer, self.gcmd, params, self.sections[0],
                    self.executor, axis, position)
                tests = [(moves, moves.start_pos[1 if axis == 'X' else 0])]
            for moves, axis_pos in tests:
                turning = []
                for section in self.sections:
                    gx, gy = moves.stepper_gradient(section.stepper)
                    r = gx * moves.head_dir[0] + gy * moves.head_dir[1]
                    if abs(r) > .001:
                        turning.append((
                            section, abs(r) / moves.stepper_per_head,
                            math.copysign(1., r)))
                self.plan.append(PlannedTest(axis, moves, turning, axis_pos))
                # The phase between the sections depends on the position on
                # the other axis, the second position inverts it on corexy
                if (axis_pos is not None and len(tests) == 1
                        and len(turning) > 1):
                    pos2 = axis_pos + self.sections[0].phase_period / (
                        4. * self.harmonic)
                    tests.append((AccelTestMoves(
                        self.printer, self.gcmd, params, self.sections[0],
                        self.executor, axis, pos2), pos2))
    def init_new_nodes(self, node_speeds):
        tested = {}
        for test in self.plan:
            for section, sr, sd in test.turning:
                tested.setdefault(section.name, set()).update(
                    round(v * sr, 1) for v in test.moves.speeds)
        for name, speeds in tested.items():
            old = self.current_nodes[name].get(self.harmonic) or []
            grid = node_speeds or sorted(speeds)
            span = list(grid) + list(speeds)
            lo, hi = min(span) - .5, max(span) + .5
            kept = [n for n in old if not lo <= n[0] <= hi]
            # The new nodes start from the current correction
            nodes = [(v, True) + (interpolate_nodes(old, v) if old
                                  else (0j, 0j)) for v in grid]
            nodes += [(v, False, cp, cn) for v, cp, cn in kept]
            nodes.sort(key=lambda n: n[0])
            if len(nodes) > MAX_NODES:
                raise self.gcmd.error(
                    "Too many h%d nodes for %s: %d new and %d kept, maximum"
                    " is %d" % (self.harmonic, name, len(grid), len(kept),
                                MAX_NODES))
            self.new_nodes[name] = nodes
    def apply_state(self, state):
        for section in self.sections:
            if state is None:
                harmonic_nodes, enabled = self.saved_state[section.name]
                section.set_nodes(harmonic_nodes, enabled=enabled)
                continue
            harmonic_nodes = dict(self.current_nodes[section.name])
            if state != 'current' and state[0] == section.name:
                t = cmath.rect(self.test_amplitude, math.radians(state[1]))
                nodes = harmonic_nodes.get(self.harmonic)
                harmonic_nodes[self.harmonic] = (
                    [(v, cp + t, cn + t) for v, cp, cn in nodes]
                    if nodes else [(0., t, t)])
            section.set_nodes(harmonic_nodes)
    # Base lock-in amplitudes of each test move and the response B, Bm of
    # each axis to the correction of each section that turns in it
    def measure_responses(self):
        tests = []
        for test in self.plan:
            ref_section = test.moves.section.name
            report = lambda v, speed_results: self.gcmd.respond_info(
                "%s %s %.1f mm/s done" % (test.axis, ref_section, v))
            results = test.moves.measure(self.gcmd, [self.harmonic], report,
                                         test.states, self.apply_state)
            for v in test.moves.speeds:
                for direction in (1., -1.):
                    lockin = {st: results[((v, direction), st)]['lockin']
                              .get(self.harmonic) for st in test.states}
                    if any(a is None for a in lockin.values()):
                        raise self.gcmd.error(
                            "The h%d frequency at %.1f mm/s is too high for"
                            " the accelerometer" % (self.harmonic, v))
                    ripple = lockin['current']
                    response = []
                    for section, sr, sd in test.turning:
                        axis_resp = []
                        for i in range(len(ACCEL_AXES)):
                            d1 = ((lockin[(section.name, 0.)][i] - ripple[i])
                                  / self.test_amplitude)
                            d2 = ((lockin[(section.name, 90.)][i] - ripple[i])
                                  / (self.test_amplitude * 1j))
                            axis_resp.append((.5 * (d1 + d2), .5 * (d1 - d2)))
                        response.append(((section.name, round(v * sr, 1),
                                          direction * sd), axis_resp))
                    tests.append({'axis': test.axis,
                                  'ref_section': ref_section,
                                  'axis_pos': test.axis_pos, 'speed': v,
                                  'direction': direction, 'ripple': ripple,
                                  'response': response})
        return tests
    # The correction change of each test as the weighted node changes plus
    # the change with the new nodes, and the change that disables it
    def node_change_model(self, tests):
        change_model = {}
        for key in set(key for test in tests
                       for key, axis_resp in test['response']):
            name, speed, direction = key
            nodes = self.new_nodes[name]
            w_pos = .5 * (1. + speed * direction / math.sqrt(
                speed**2 + DIRECTION_SMOOTH_V2))
            old = self.current_nodes[name].get(self.harmonic)
            change = 0j
            if old:
                cp, cn = interpolate_nodes(old, speed)
                change = -(w_pos * cp + (1. - w_pos) * cn)
            disable_change = change
            if speed <= nodes[0][0]:
                weights = [(0, 1.)]
            elif speed >= nodes[-1][0]:
                weights = [(len(nodes) - 1, 1.)]
            else:
                j = next(j for j in range(1, len(nodes))
                         if speed < nodes[j][0])
                f = (speed - nodes[j-1][0]) / (nodes[j][0] - nodes[j-1][0])
                weights = [(j - 1, 1. - f), (j, f)]
            columns = []
            for j, w in weights:
                v, is_new, cp, cn = nodes[j]
                for d, wd, c in ((1., w_pos, cp), (-1., 1. - w_pos, cn)):
                    if w * wd:
                        change += w * wd * c
                        if is_new:
                            columns.append(((name, v, d), w * wd))
            change_model[key] = (columns, change, disable_change)
        return change_model
    def result_nodes(self, delta):
        return {name: [(v, cp + delta.get((name, v, 1.), 0j),
                        cn + delta.get((name, v, -1.), 0j))
                       for v, is_new, cp, cn in nodes]
                for name, nodes in self.new_nodes.items()}
    def apply_nodes(self, result_nodes):
        for section in self.sections:
            harmonic_nodes = dict(self.current_nodes[section.name])
            if section.name in result_nodes:
                harmonic_nodes[self.harmonic] = result_nodes[section.name]
            section.set_nodes(harmonic_nodes)
            section.save_nodes()

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
    def _parse_test_params(self, gcmd):
        chip_name = gcmd.get('ACCEL_CHIP', self.accel_chip)
        if chip_name is None:
            raise gcmd.error("ACCEL_CHIP is required when accel_chip is not"
                             " set in [stepper_ripple_calibrate]")
        chip_name = chip_name.strip()
        chip = self._lookup_chip(chip_name, gcmd.error)
        text = gcmd.get('SPEEDS', None)
        if text is not None:
            try:
                speeds = [float(s) for s in text.split(',') if s.strip()]
            except ValueError:
                raise gcmd.error("Invalid SPEEDS")
        else:
            min_speed = gcmd.get_float('MIN_SPEED', above=0.)
            max_speed = gcmd.get_float('MAX_SPEED', minval=min_speed)
            step = gcmd.get_float('SPEED_STEP', 1., above=0.)
            count = int((max_speed - min_speed) / step + .000001)
            speeds = [round(min_speed + i * step, 6)
                      for i in range(count + 1)]
        if not speeds or min(speeds) <= 0.:
            raise gcmd.error("Invalid SPEEDS")
        return {'chip_name': chip_name, 'chip': chip, 'speeds': speeds,
                'accel': gcmd.get_float('ACCEL', None, above=0.),
                'move_time': gcmd.get_float('MOVE_TIME', ACCEL_MOVE_TIME,
                                            above=0.)}
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
               " start %.3f,%.3f, move time %.2fs" % (
                   section.name, moves.chip_name, moves.head_dir[0],
                   moves.head_dir[1], moves.start_pos[0],
                   moves.start_pos[1], moves.move_time)]
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
        position = (gcmd.get_float('POSITION', None) if axis != 'DIAG'
                    else None)
        moves = AccelTestMoves(self.printer, gcmd,
                               self._parse_test_params(gcmd), sections[0],
                               self.executor, axis, position)
        gcmd.respond_info(moves.describe([None]))
        def report(v, speed_results):
            gcmd.respond_info("%.1f mm/s: forward %.0f, backward %.0f" % (
                v, speed_results[((v, 1.), None)]['median'],
                speed_results[((v, -1.), None)]['median']))
        results = moves.measure(gcmd, list(range(1, MAX_HARMONIC + 1)),
                                report)
        text = self._format_table(moves, results)
        try:
            path = save_result(text, 'stepper_ripple_vibration', label)
        except Exception as e:
            raise gcmd.error("%s\nUnable to save the table: %s" % (text, e))
        gcmd.respond_info("Table saved to: %s" % (path,))
    cmd_STEPPER_RIPPLE_CALIBRATE_help = (
        "Calibrate stepper ripple compensation using an accelerometer")
    def cmd_STEPPER_RIPPLE_CALIBRATE(self, gcmd):
        axes = [a.strip().upper() for a in gcmd.get('AXIS', 'DIAG')
                .split(',') if a.strip()]
        if (not axes or len(set(axes)) != len(axes)
            or any(a not in TEST_AXES for a in axes)):
            raise gcmd.error("AXIS must be a list of DIAG, X and Y")
        sections = self._lookup_sections(gcmd)
        harmonic = gcmd.get_int('HARMONIC', minval=1, maxval=MAX_HARMONIC)
        test_amplitude = gcmd.get_float('AMPLITUDE', 4., above=0.)
        apply_result = gcmd.get_int('APPLY', 1, minval=0, maxval=1)
        node_speeds = None
        text = gcmd.get('NODE_SPEEDS', None)
        if text is not None:
            try:
                node_speeds = [round(float(v), 1) for v in text.split(',')
                               if v.strip()]
            except ValueError:
                raise gcmd.error("Invalid NODE_SPEEDS")
            if (not node_speeds or min(node_speeds) < 0.
                    or any(v2 <= v1 for v1, v2 in
                           zip(node_speeds, node_speeds[1:]))):
                raise gcmd.error("NODE_SPEEDS must be increasing speeds")
        if not sections:
            raise gcmd.error("No stepper_ripple_compensation sections")
        for section in sections:
            if section.phase_period is None:
                raise gcmd.error("Electrical phase of %s is unknown"
                                 % (section.name,))
        cal = RippleCalibration(self, gcmd, sections, harmonic, test_amplitude)
        cal.plan_tests(axes, self._parse_test_params(gcmd),
                       gcmd.get_float('POSITION', None))
        cal.init_new_nodes(node_speeds)
        gcmd.respond_info(
            "Accelerometer ripple calibration of harmonic %d of %s, axes"
            " %s, %d test states per speed\n%s"
            % (harmonic, ", ".join(s.name for s in sections),
               ",".join(axes), sum(len(test.states) for test in cal.plan),
               "\n".join(test.moves.describe(test.states)
                         for test in cal.plan)))
        tests = cal.measure_responses()
        solution = self.executor.submit(solve_node_changes, tests,
                                        cal.node_change_model(tests))
        msg = ["Harmonic %d vibration in mm/s^2, measured with the current"
               " correction, predicted with the new nodes, predicted without"
               " the correction" % (harmonic,)]
        for test, pred, pred_off in zip(tests, solution['pred'],
                                        solution['pred_off']):
            where = test['ref_section']
            if test['axis_pos'] is not None:
                where = "%.2f" % (test['axis_pos'],)
            sign = '+' if test['direction'] > 0. else '-'
            measured = math.sqrt(sum(abs(a)**2 for a in test['ripple']))
            msg.append("  %s %s %.1f mm/s %s: measured=%.0f pred=%.0f"
                       " no_corr=%.0f" % (test['axis'], where, test['speed'],
                                          sign, measured, pred, pred_off))
        weak = ["%s %.1f%s" % (name, v, '+' if d > 0. else '-')
                for name, v, d in solution['weak']]
        if weak:
            msg.append("Poorly determined nodes, kept nearer the current"
                       " ones: " + ", ".join(weak))
        result_nodes = cal.result_nodes(solution['delta'])
        for name, nodes in result_nodes.items():
            msg.append("[stepper_ripple_compensation %s]" % (name,))
            msg.append("h%d_nodes:" % (harmonic,))
            msg += ["  " + format_node(n) for n in nodes]
        if apply_result:
            cal.apply_nodes(result_nodes)
            msg.append("New values applied, use SAVE_CONFIG to store them.")
        gcmd.respond_info("\n".join(msg))

def load_config(config):
    return StepperRippleCalibrate(config)
