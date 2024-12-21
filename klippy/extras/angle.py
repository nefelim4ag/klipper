# Support for reading SPI magnetic angle sensors
#
# Copyright (C) 2021,2022  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
from . import bus, bulk_sensor

MIN_MSG_TIME = 0.100
TCODE_ERROR = 0xff

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
    "tmc5160"]

CALIBRATION_BITS = 6 # 64 entries
ANGLE_BITS = 16 # angles range from 0..65535

class AngleCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.stepper_name = config.get('stepper', None)
        if self.stepper_name is None:
            # No calibration
            return
        try:
            import numpy
        except:
            raise config.error("Angle calibration requires numpy module")
        sconfig = config.getsection(self.stepper_name)
        sconfig.getint('microsteps', note_valid=False)
        self.tmc_module = self.mcu_stepper = None
        # Current calibration data
        self.mcu_pos_offset = None
        self.angle_phase_offset = 0.
        self.calibration_reversed = False
        self.calibration = []
        cal = config.get('calibrate', None)
        if cal is not None:
            data = [d.strip() for d in cal.split(',')]
            angles = [float(d) for d in data if d]
            self.load_calibration(angles)
        # Register commands
        self.printer.register_event_handler("stepper:sync_mcu_position",
                                            self.handle_sync_mcu_pos)
        self.printer.register_event_handler("klippy:connect", self.connect)
        cname = self.name.split()[-1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("ANGLE_CALIBRATE", "CHIP",
                                   cname, self.cmd_ANGLE_CALIBRATE,
                                   desc=self.cmd_ANGLE_CALIBRATE_help)
    def handle_sync_mcu_pos(self, mcu_stepper):
        if mcu_stepper.get_name() == self.stepper_name:
            self.mcu_pos_offset = None
    def calc_mcu_pos_offset(self, sample):
        # Lookup phase information
        mcu_phase_offset, phases = self.tmc_module.get_phase_offset()
        if mcu_phase_offset is None:
            return
        # Find mcu position at time of sample
        angle_time, angle_pos = sample
        mcu_pos = self.mcu_stepper.get_past_mcu_position(angle_time)
        # Convert angle_pos to mcu_pos units
        microsteps, full_steps = self.get_microsteps()
        angle_to_mcu_pos = full_steps * microsteps / float(1<<ANGLE_BITS)
        angle_mpos = angle_pos * angle_to_mcu_pos
        # Calculate adjustment for stepper phases
        phase_diff = ((angle_mpos + self.angle_phase_offset * angle_to_mcu_pos)
                      - (mcu_pos + mcu_phase_offset)) % phases
        if phase_diff > phases//2:
            phase_diff -= phases
        # Store final offset
        self.mcu_pos_offset = mcu_pos - (angle_mpos - phase_diff)
    def apply_calibration(self, samples):
        calibration = self.calibration
        if not calibration:
            return None
        calibration_reversed = self.calibration_reversed
        interp_bits = ANGLE_BITS - CALIBRATION_BITS
        interp_mask = (1 << interp_bits) - 1
        interp_round = 1 << (interp_bits - 1)
        for i, (samp_time, angle) in enumerate(samples):
            bucket = (angle & 0xffff) >> interp_bits
            cal1 = calibration[bucket]
            cal2 = calibration[bucket + 1]
            adj = (angle & interp_mask) * (cal2 - cal1)
            adj = cal1 + ((adj + interp_round) >> interp_bits)
            angle_diff = (adj - angle) & 0xffff
            angle_diff -= (angle_diff & 0x8000) << 1
            new_angle = angle + angle_diff
            if calibration_reversed:
                new_angle = -new_angle
            samples[i] = (samp_time, new_angle)
        if self.mcu_pos_offset is None:
            self.calc_mcu_pos_offset(samples[0])
            if self.mcu_pos_offset is None:
                return None
        return self.mcu_stepper.mcu_to_commanded_position(self.mcu_pos_offset)
    def load_calibration(self, angles):
        # Calculate linear intepolation calibration buckets by solving
        # linear equations
        angle_max = 1 << ANGLE_BITS
        calibration_count = 1 << CALIBRATION_BITS
        bucket_size = angle_max // calibration_count
        full_steps = len(angles)
        nominal_step = float(angle_max) / full_steps
        self.angle_phase_offset = (angles.index(min(angles)) & 3) * nominal_step
        self.calibration_reversed = angles[-2] > angles[-1]
        if self.calibration_reversed:
            angles = list(reversed(angles))
        first_step = angles.index(min(angles))
        angles = angles[first_step:] + angles[:first_step]
        import numpy
        eqs = numpy.zeros((full_steps, calibration_count))
        ans = numpy.zeros((full_steps,))
        for step, angle in enumerate(angles):
            int_angle = int(angle + .5) % angle_max
            bucket = int(int_angle / bucket_size)
            bucket_start = bucket * bucket_size
            ang_diff = angle - bucket_start
            ang_diff_per = ang_diff / bucket_size
            eq = eqs[step]
            eq[bucket] = 1. - ang_diff_per
            eq[(bucket + 1) % calibration_count] = ang_diff_per
            ans[step] = float(step * nominal_step)
            if bucket + 1 >= calibration_count:
                ans[step] -= ang_diff_per * angle_max
        sol = numpy.linalg.lstsq(eqs, ans, rcond=None)[0]
        isol = [int(s + .5) for s in sol]
        self.calibration = isol + [isol[0] + angle_max]
    def lookup_tmc(self):
        for driver in TRINAMIC_DRIVERS:
            driver_name = "%s %s" % (driver, self.stepper_name)
            module = self.printer.lookup_object(driver_name, None)
            if module is not None:
                return module
        raise self.printer.command_error("Unable to find TMC driver for %s"
                                         % (self.stepper_name,))
    def connect(self):
        self.tmc_module = self.lookup_tmc()
        fmove = self.printer.lookup_object('force_move')
        self.mcu_stepper = fmove.lookup_stepper(self.stepper_name)
    def get_microsteps(self):
        configfile = self.printer.lookup_object('configfile')
        sconfig = configfile.get_status(None)['settings']
        stconfig = sconfig.get(self.stepper_name, {})
        microsteps = stconfig['microsteps']
        full_steps = stconfig['full_steps_per_rotation']
        return microsteps, full_steps
    def get_stepper_phase(self):
        mcu_phase_offset, phases = self.tmc_module.get_phase_offset()
        if mcu_phase_offset is None:
            raise self.printer.command_error("Driver phase not known for %s"
                                             % (self.stepper_name,))
        mcu_pos = self.mcu_stepper.get_mcu_position()
        return (mcu_pos + mcu_phase_offset) % phases
    def do_calibration_moves(self):
        move = self.printer.lookup_object('force_move').manual_move
        # Start data collection
        msgs = []
        is_finished = False
        def handle_batch(msg):
            if is_finished:
                return False
            msgs.append(msg)
            return True
        self.printer.lookup_object(self.name).add_client(handle_batch)
        # Move stepper several turns (to allow internal sensor calibration)
        microsteps, full_steps = self.get_microsteps()
        mcu_stepper = self.mcu_stepper
        step_dist = mcu_stepper.get_step_dist()
        full_step_dist = step_dist * microsteps
        rotation_dist = full_steps * full_step_dist
        align_dist = step_dist * self.get_stepper_phase()
        move_time = 0.010
        move_speed = full_step_dist / move_time
        move(mcu_stepper, -(rotation_dist+align_dist), move_speed)
        move(mcu_stepper, 2. * rotation_dist, move_speed)
        move(mcu_stepper, -2. * rotation_dist, move_speed)
        move(mcu_stepper, .5 * rotation_dist - full_step_dist, move_speed)
        # Move to each full step position
        toolhead = self.printer.lookup_object('toolhead')
        times = []
        samp_dist = full_step_dist
        for i in range(2 * full_steps):
            move(mcu_stepper, samp_dist, move_speed)
            start_query_time = toolhead.get_last_move_time() + 0.050
            end_query_time = start_query_time + 0.050
            times.append((start_query_time, end_query_time))
            toolhead.dwell(0.150)
            if i == full_steps-1:
                # Reverse direction and test each full step again
                move(mcu_stepper, .5 * rotation_dist, move_speed)
                move(mcu_stepper, -.5 * rotation_dist + samp_dist, move_speed)
                samp_dist = -samp_dist
        move(mcu_stepper, .5*rotation_dist + align_dist, move_speed)
        toolhead.wait_moves()
        # Finish data collection
        is_finished = True
        # Correlate query responses
        cal = {}
        step = 0
        for msg in msgs:
            for query_time, pos in msg['data']:
                # Add to step tracking
                while step < len(times) and query_time > times[step][1]:
                    step += 1
                if step < len(times) and query_time >= times[step][0]:
                    cal.setdefault(step, []).append(pos)
        if len(cal) != len(times):
            raise self.printer.command_error(
                "Failed calibration - incomplete sensor data")
        fcal = { i: cal[i] for i in range(full_steps) }
        rcal = { full_steps-i-1: cal[i+full_steps] for i in range(full_steps) }
        return fcal, rcal
    def calc_angles(self, meas):
        total_count = total_variance = 0
        angles = {}
        for step, data in meas.items():
            count = len(data)
            angle_avg = float(sum(data)) / count
            angles[step] = angle_avg
            total_count += count
            total_variance += sum([(d - angle_avg)**2 for d in data])
        return angles, math.sqrt(total_variance / total_count), total_count
    cmd_ANGLE_CALIBRATE_help = "Calibrate angle sensor to stepper motor"
    def cmd_ANGLE_CALIBRATE(self, gcmd):
        # Perform calibration movement and capture
        old_calibration = self.calibration
        self.calibration = []
        try:
            fcal, rcal = self.do_calibration_moves()
        finally:
            self.calibration = old_calibration
        # Calculate each step position average and variance
        microsteps, full_steps = self.get_microsteps()
        fangles, fstd, ftotal = self.calc_angles(fcal)
        rangles, rstd, rtotal = self.calc_angles(rcal)
        if (len({a: i for i, a in fangles.items()}) != len(fangles)
            or len({a: i for i, a in rangles.items()}) != len(rangles)):
            raise self.printer.command_error(
                "Failed calibration - sensor not updating for each step")
        merged = { i: fcal[i] + rcal[i] for i in range(full_steps) }
        angles, std, total = self.calc_angles(merged)
        gcmd.respond_info("angle: stddev=%.3f (%.3f forward / %.3f reverse)"
                          " in %d queries" % (std, fstd, rstd, total))
        # Order data with lowest/highest magnet position first
        anglist = [angles[i] % 0xffff for i in range(full_steps)]
        if angles[0] > angles[1]:
            first_ang = max(anglist)
        else:
            first_ang = min(anglist)
        first_phase = anglist.index(first_ang) & ~3
        anglist = anglist[first_phase:] + anglist[:first_phase]
        # Save results
        cal_contents = []
        for i, angle in enumerate(anglist):
            if not i % 8:
                cal_contents.append('\n')
            cal_contents.append("%.1f" % (angle,))
            cal_contents.append(',')
        cal_contents.pop()
        configfile = self.printer.lookup_object('configfile')
        configfile.remove_section(self.name)
        configfile.set(self.name, 'calibrate', ''.join(cal_contents))

class AngleTMCCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.stepper_name = config.get('stepper', None)
        if self.stepper_name is None:
            # No calibration
            return
        try:
            from numpy import interp
            from numpy import linspace
            from numpy.polynomial import Polynomial
            from numpy import std
        except:
            raise config.error("Angle TMC calibration requires numpy module")

        self.std = std
        self.Polynomial = Polynomial
        self.interp = interp
        self.linspace = linspace

        sconfig = config.getsection(self.stepper_name)
        sconfig.getint('microsteps', note_valid=False)
        self.tmc = self.mcu_stepper = None
        self.driver_name = None
        self.driver = None
        self.mcu = None
        self.reactor = self.printer.get_reactor()
        self.msgs = []
        self.is_finished = True
        self.dir = 0
        self.return_offset = 0
        self.microsteps = 0
        self.full_steps = 0
        self.step_dist = 0
        self.full_step_dist = 0
        self.mscnt = 0
        self.real_resolution = 1 << 16
        self.ms_angle = 0
        self.angle_dir = 1
        self.misalign = 0.225
        self.start_offset = 0

        # Register commands
        self.printer.register_event_handler("klippy:connect", self.connect)
        cname = self.name.split()[-1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("ANGLE_TMC_CALIBRATE", "CHIP",
                                   cname, self.cmd_ANGLE_TMC_CALIBRATE,
                                   desc=self.cmd_ANGLE_TMC_CALIBRATE_help)
        gcode.register_mux_command("ANGLE_TMC_MSLUT_DUMP", "CHIP",
                                   cname, self.cmd_ANGLE_TMC_MSLUT_DUMP,
                                   desc=self.cmd_ANGLE_TMC_MSLUT_DUMP_help)
    def lookup_tmc(self):
        for driver in TRINAMIC_DRIVERS:
            self.driver_name = "%s %s" % (driver, self.stepper_name)
            self.driver = driver
            module = self.printer.lookup_object(self.driver_name, None)
            if module is not None:
                return module
        raise self.printer.command_error("Unable to find TMC driver for %s"
                                         % (self.stepper_name,))
    def connect(self):
        tmc_module = self.lookup_tmc()
        self.tmc = tmc_module.mcu_tmc
        fmove = self.printer.lookup_object('force_move')
        self.mcu_stepper = fmove.lookup_stepper(self.stepper_name)
        configfile = self.printer.lookup_object('configfile')
        sconfig = configfile.get_status(None)['settings']
        stconfig = sconfig.get(self.stepper_name, {})
        self.microsteps = stconfig['microsteps']
        self.full_steps = stconfig['full_steps_per_rotation']
        self.step_dist = self.mcu_stepper.get_step_dist()
        self.full_step_dist = self.step_dist * self.microsteps
        self.mscnt_quant = 256 // self.microsteps
        self.mscnt_min = (self.mscnt_quant // 2)
        positions = [i for i in range(self.mscnt_min, 1024, self.mscnt_quant)]
        self.positions = positions
        self.start_offset = 0 - self.mscnt_min

    def set_field(self, field_name, value):
        reg_name = self.tmc.fields.lookup_register(field_name)
        reg_val = self.tmc.fields.set_field(field_name, value)
        self.tmc.set_register(reg_name, reg_val)

    def get_field(self, field_name):
        return self.tmc.fields.get_field(field_name)

    def _write_to_file(self, filename, samples):
        import os, multiprocessing
        def write_impl():
            try:
                # Try to re-nice writing process
                os.nice(20)
            except:
                pass
            import json
            with open(filename, "w") as fd:
                for k in samples:
                    sample = {
                        k: samples[k],
                    }
                    fd.write(json.dumps(sample) + '\n')

        write_proc = multiprocessing.Process(target=write_impl)
        write_proc.daemon = True
        write_proc.start()

    def handle_batch(self, msg):
        if self.is_finished:
            return False
        self.msgs.append({
            "data_raw": msg["data_raw"]
        })
        return True

    def move(self, distance):
        move = self.printer.lookup_object('force_move').manual_move
        move_time = 0.010
        move_speed = self.full_step_dist / move_time
        move(self.mcu_stepper, distance, move_speed, 1000)
        self.return_offset -= distance

    def move_reset(self):
        self.move(self.return_offset)

    def intpol_wait(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        intpol = self.get_field("intpol")
        if intpol:
            self.pause(0.1)

    def stepper_align(self, target):
        self.move(self.step_dist * 3)
        self.intpol_wait()
        mscnt = self.tmc.get_register("MSCNT")
        # It works bad at cold start, move it to different position
        if mscnt == self.mscnt_min:
            self.move(self.step_dist * 3)
        self.intpol_wait()
        mscnt = self.tmc.get_register("MSCNT")
        mscnt_prev = mscnt
        logging.info("MSCNT Cur: %i" % (mscnt))
        if self.dir == 0:
            self.dir = 1
            self.move(self.dir * self.step_dist)
            self.intpol_wait()
            mscnt = self.tmc.get_register("MSCNT")
            if mscnt < mscnt_prev:
                self.dir = -1

        # Normalize target within [0, 1023]
        target = target % 1024
        fwd_distance = (target - mscnt + 1024) % 1024
        bwd_distance = (mscnt - target + 1024) % 1024

        move_direction = -self.dir
        steps = bwd_distance // self.mscnt_quant
        if fwd_distance <= bwd_distance:
            move_direction = self.dir
            steps = fwd_distance // self.mscnt_quant

        if (steps > 1):
            self.move(move_direction * self.step_dist * (steps - 1))
            self.intpol_wait()
            logging.info("MSCNT: %i -> %i" % (mscnt, target))
            mscnt = self.tmc.get_register("MSCNT")

        while mscnt != target:
            self.move(move_direction * self.step_dist)
            self.intpol_wait()
            logging.info("MSCNT: %i -> %i" % (mscnt, target))
            mscnt = self.tmc.get_register("MSCNT")
        logging.info("MSCNT Cur: %i" % (mscnt))
        self.mscnt = target

    def pause(self, pause = 0.01):
        self.reactor.pause(self.reactor.monotonic() + pause)

    cmd_ANGLE_TMC_MSLUT_DUMP_help = "Dump microstep angle data"
    def cmd_ANGLE_TMC_MSLUT_DUMP(self, gcmd):
        # Start data collection
        self.is_finished = False
        self.printer.lookup_object(self.name).add_client(self.handle_batch)

        # Align stepper to zero
        toolhead = self.printer.lookup_object('toolhead')
        self.stepper_align(0 - self.mscnt_min - self.mscnt_quant)

        # Move to each microstep position
        times = {}
        angles = {}
        min_pos = 0 - self.mscnt_min
        max_pos = 1024 + self.mscnt_min
        for i in range(min_pos, max_pos, self.mscnt_quant):
            self.move(self.dir * self.step_dist)
            start_query_time = toolhead.get_last_move_time() + 0.050
            end_query_time = start_query_time + 0.050
            times[i] = (start_query_time, end_query_time)
            toolhead.dwell(0.150)
            angles[i] = {
                "samples": [],
                "sum": 0,
                "count": 0
            }

            # spread computation load
            deadline = self.reactor.monotonic() + .050
            while self.msgs and self.reactor.monotonic() < deadline:
                msg = self.msgs.pop(0)
                for mscnt in times:
                    start, end = times[mscnt]
                    for query_time, pos in msg["data_raw"]:
                        if query_time >= start and query_time < end:
                            pos = pos + (1 << 15)
                            angles[mscnt]["samples"].append(pos)
                            angles[mscnt]["sum"] += pos
                            angles[mscnt]["count"] += 1
                        if query_time > end:
                            break

        toolhead.wait_moves()
        # Finish data collection
        self.is_finished = True
        self.move_reset()
        gcmd.respond_info("Start processing tail data: %i" % (len(self.msgs)))

        # Correlate query responses
        while self.msgs:
            msg = self.msgs.pop(0)
            self.pause(0.001)
            for mscnt in times:
                start, end = times[mscnt]
                for query_time, pos in msg["data_raw"]:
                    if query_time >= start and query_time < end:
                        pos = pos + (1 << 15)
                        angles[mscnt]["samples"].append(pos)
                        angles[mscnt]["sum"] += pos
                        angles[mscnt]["count"] += 1
                    if query_time > end:
                        break

        for mscnt in angles:
            avg = angles[mscnt]["sum"]/angles[mscnt]["count"]
            angles[mscnt]["avg"] = avg
            angles[mscnt]["angle_avg"] = avg * 360 / (1 << 16)

        file = f"/tmp/angle-tmc-samples-{self.microsteps}ms.json"
        gcmd.respond_info(
            f"Write to {file}")
        self._write_to_file(file, angles)

    def mslut_decoder(self):
        MSLUTS = [
            self.get_field("mslut0"),
            self.get_field("mslut1"),
            self.get_field("mslut2"),
            self.get_field("mslut3"),
            self.get_field("mslut4"),
            self.get_field("mslut5"),
            self.get_field("mslut6"),
            self.get_field("mslut7"),
        ]
        bit_diffs = []
        for mslut in MSLUTS:
            for shift in range(0, 32):
                bit_val = (mslut >> shift) & 1
                bit_diffs.append(bit_val)

        decoded_val = []

        # Width control bit coding W0…W3:
        # %00: MSLUT entry 0, 1 select: -1, +0
        # %01: MSLUT entry 0, 1 select: +0, +1
        # %10: MSLUT entry 0, 1 select: +1, +2
        # %11: MSLUT entry 0, 1 select: +2, +3
        W = {
            0: self.get_field("w0"),
            1: self.get_field("w1"),
            2: self.get_field("w2"),
            3: self.get_field("w3")
        }
        # Unused, just for inmind decoding
        # _W_lookup_table = {0: {-1, 0}, 1: {0, +1}, 2: {1, 2}, 3: {2, 3}}
        # _W_Fast_offset_table = {0: -1, 1: 0, 2: 1, 3: 2}
        # offset = W - 1

        # The sine wave look-up table can be divided into up to
        # four segments using an individual step width control
        # entry Wx. The segment borders are selected by X1, X2
        # and X3.
        # Segment 0 goes from 0 to X1-1.
        # Segment 1 goes from X1 to X2-1.
        # Segment 2 goes from X2 to X3-1.
        # Segment 3 goes from X3 to 255.
        # For defined response the values shall satisfy:
        # 0<X1<X2<X3
        X = {
            1: self.get_field("x1"),
            2: self.get_field("x2"),
            3: self.get_field("x3"),
        }
        START_SIN = self.get_field("start_sin")
        START_SIN90 = self.get_field("start_sin90")
        # START_SIN90 gives the absolute current for
        # microstep table entry at positions 256

        for i in range(0, 256):
            if i < X[1] - 1:
                # Segment 0
                offset = W[0] - 1
            elif i < X[2] - 1:
                # Segment 1
                offset = W[1] - 1
            elif i < X[3] - 1:
                # Segment 2
                offset = W[2] - 1
            else:
                # Segment 3
                offset = W[3] - 1
            bit = bit_diffs[i]
            bit_dec = bit + offset
            decoded_val.append(bit_dec)

        # Sum diffs one by one
        sin_value = [START_SIN] # first value is always START_SIN
        for val in decoded_val:
            last = sin_value[-1]
            sin_value.append(last + val)

        if sin_value[-1] != START_SIN90:
            print("Are you sure? End of 1/4 of sin is not in sync with 2 of 4")
        if sin_value[0] != START_SIN:
            print(f"Are you sure? first value is {sin_value[0]} you miss zero crossing")

        # Maybe i'm stupid, as last value is equal it can be ignored..? 257 -> 256
        sin_value.pop()

        return sin_value

    def mslut_encoder(self, quarter_seg, safe=False):
        START_SIN90 = quarter_seg[-1]
        if len(quarter_seg) != 256:
            logging.error("Wrong quarter segment size")
            raise self.printer.command_error("Quarter segment should have exact 256 elements")

        deltas = [0]
        for i in range(1, 256):
            delta = quarter_seg[i] - quarter_seg[i-1]
            if delta > 3:
                if safe:
                    return None
                raise self.printer.command_error(f"Can't be encoded delta > 3: {quarter_seg}")
            if delta < -1:
                if safe:
                    return None
                raise self.printer.command_error(f"Can't be encoded delta < -1: {quarter_seg}")
            deltas.append(delta)

        # Search for segments
        segments = {
            0: {
                "start": 0,
                "min": 2,
                "max": -1,
            },
            1: {
                "start": 255,
                "min": 2,
                "max": -1,
            },
            2: {
                "start": 255,
                "min": 3,
                "max": -1,
            },
            3: {
                "start": 255,
                "min": 3,
                "max": -1,
            },
        }
        cur_seg = 0
        for i in range(1, len(deltas)):
            smin = segments[cur_seg]["min"]
            smax = segments[cur_seg]["max"]
            delta = deltas[i]
            nsmin = min(smin, delta)
            nsmax = max(smax, delta)
            if nsmax - nsmin > 1:
                cur_seg += 1
                if cur_seg > 3:
                    if safe:
                        return None
                    logging.error("Can't be encoded: %s" % (quarter_seg))
                    raise self.printer.command_error("Too many swings")
                segments[cur_seg] = {
                    "min": delta,
                    "max": delta,
                    "start": i
                }
                continue
            else:
                segments[cur_seg]["min"] = nsmin
                segments[cur_seg]["max"] = nsmax

        X = {}
        X[1] = segments[1]["start"]
        X[2] = segments[2]["start"]
        X[3] = segments[3]["start"]

        W = {}
        W[0] = (segments[0]["min"] + 1)
        W[1] = (segments[1]["min"] + 1) % 4
        W[2] = (segments[2]["min"] + 1) % 4
        if X[2] == 255:
            W[2] = W[1]
        W[3] = (segments[3]["min"] + 1) % 4
        if X[3] == 255:
            W[3] = W[2]


        bit_diffs = []
        # print(deltas)
        # Width control bit coding W0…W3:
        # %00: MSLUT entry 0, 1 select: -1, +0
        # %01: MSLUT entry 0, 1 select: +0, +1
        # %10: MSLUT entry 0, 1 select: +1, +2
        # %11: MSLUT entry 0, 1 select: +2, +3
        WReverseTable = {
            0: {
               -1: 0,
                0: 1,
            },
            1: {
                0: 0,
                1: 1,
            },
            2: {
                1: 0,
                2: 1,
            },
            3: {
                2: 0,
                3: 1,
            },
        }
        _width = 0
        for pair in [[1, X[1]], [X[1], X[2]], [X[2], X[3]], [X[3], 255]]:
            width = W[_width]
            for i in range(pair[0], pair[1]):
                delta = deltas[i]
                # print(f"{i}, {width}, {delta} -> {WReverseTable[width][delta]}")
                bit = WReverseTable[width][delta]
                if bit != 0 and bit != 1:
                    logging.error("Something very wrong happens here")
                    bit = 0
                bit_diffs.append(bit)
            _width += 1

        # print(bit_diffs, len(bit_diffs))
        # Where is 2 last bit missing, possibly becuase ...
        # first is sin_start and last is sin90?
        bit_diffs.append(0)
        bit_diffs.append(0)

        MSLUTS = [
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        ]
        for m in range(0, len(MSLUTS)):
            for i in range(0, 32):
                bit = bit_diffs[m * 32 + i]
                MSLUTS[m] = MSLUTS[m] | (bit << i)

        return {
            "START_SIN": quarter_seg[0], # Fisrt value is always START_SIN
            "START_SIN90": START_SIN90,
            "X": X,
            "W": W,
            "MSLUTS": MSLUTS
        }

    # Encoder is limited to 4 segments with 1 bit diffs
    # Diffs is limited -1..3
    def mslut_normalize(self, sin_value):
        sin_new = [round(i) for i in sin_value]
        while sin_new[0] < 0:
            for i in range(0, 256):
                sin_new[i] += 1
        y_max = 248
        if self.driver == "tmc2240":
            y_max = 247
            if self.get_field("offset_sin90") < -8 or self.get_field("offset_sin90") > 8:
                y_max = 246
        for i in range(1, 256):
            d = sin_new[i] - sin_new[i-1]
            if d > 3:
                sin_new[i] = sin_new[i-1] + 3
                if i < 255:
                    sin_new[i+1] += d - 3
            if d < -1:
                sin_new[i] = sin_new[i-1] - 1
                if i < 255:
                    sin_new[i+1] += d + 1
            sin_new[i] = min(y_max, sin_new[i])

        return sin_new

    def twindow_to_angle(self, start):
        samples = []
        while len(samples) < 10:
            while not self.msgs:
                self.pause()
            msg = self.msgs.pop(0)
            for query_time, pos in msg["data_raw"]:
                if query_time >= start:
                    samples.append(pos)

        avg = sum(samples)/len(samples)
        angle_avg = avg * 360 / (1 << 16)
        if self.microsteps == 256:
            return round(angle_avg, 8)
        if self.microsteps == 128:
            return round(angle_avg, 7)
        if self.microsteps == 64:
            return round(angle_avg, 6)
        if self.microsteps == 32:
            return round(angle_avg, 5)
        return round(angle_avg, 4)

    def last_move_angle(self):
        toolhead = self.printer.lookup_object('toolhead')
        start = toolhead.get_last_move_time() + 0.055
        return self.twindow_to_angle(start)

    def guess_real_resolution(self):
        if self.is_finished:
            self.is_finished = False
            self.printer.lookup_object(self.name).add_client(self.handle_batch)
        self.pause()
        self.move(self.full_step_dist * self.full_steps / 8)
        self.move(-self.full_step_dist * self.full_steps / 8)
        toolhead = self.printer.lookup_object('toolhead')
        time_end = toolhead.get_last_move_time() + 0.1
        mask = 0
        search = True
        while search:
            while len(self.msgs) == 0:
                self.pause()
            msg = self.msgs.pop(0)
            for query_time, pos in msg["data_raw"]:
                if query_time > time_end:
                    search = False
                    break
                mask |= pos
        mask &= 0xff
        zero_bits = mask ^ 0xff
        self.real_resolution = 1 << 16
        while zero_bits:
            self.real_resolution = self.real_resolution >> 1
            zero_bits = zero_bits >> 1

        # Allow missaligment up to 1 angle sensor step
        self.misalign = (360 / self.real_resolution) * 1.2

    def sin_interp(self, sin_value):
        x = [i for i in range(self.mscnt_min, 256, self.mscnt_quant)]
        y = [sin_value[i] for i in x]
        logging.info(f"y = {y}")
        y_max = [y[-1]/(y[-2]/y[-1])]
        if self.driver == "tmc2240":
            if self.get_field("offset_sin90") < -8 or self.get_field("offset_sin90") > 8:
                y_max = 246
        y_i =[0] + y + [y_max]
        x_i = [0] + x + [255]
        y_new = [i for i in range(0, 256)]
        for i in range(0, 256):
            y_new[i] = self.interp(i, x_i, y_i)
        return self.mslut_normalize(y_new)

    # def fit_safe(self, sin_value):
    #     sin_value = sin_value[:256]
    #     x = [0] + [i for i in range(self.mscnt_min, 256, self.mscnt_quant)] + [255]
    #     y = [sin_value[i] for i in x]
    #     p = self.Polynomial.fit(x, y, 4)
    #     x_new = self.linspace(0, 256, 256)
    #     y_new = p(x_new)
    #     return self.mslut_normalize(y_new)

    def fit(self, sin_value):
        sin_value = sin_value[:256]
        x = [i for i in range(self.mscnt_min, 256, self.mscnt_quant)] + [255]
        y = [sin_value[i] for i in x]
        x = [0] + x
        y = [0] + y
        logging.info(f"y = {y}")
        p = self.Polynomial.fit(x, y, 4)
        x_new = self.linspace(0, 256, 256)
        y_new = p(x_new)
        # if y_new[0] < 0 or y_new[0] > 3:
        #     logging.warning("fit - fallback to fit safe")
        #     return self.fit_safe(sin_value)
        return self.mslut_normalize(y_new)

    def interp_or_fit(self, sin_value):
        interp = self.sin_interp(sin_value)
        if self.mslut_encoder(interp, safe=True):
            return interp
        logging.warning("interp - fallback to fit")
        return self.fit(sin_value)

    def measure_sin(self, sin):
        matched = 0
        not_matched = 0
        up = 0
        down = 0
        self.sin_apply(sin)
        self.move_reset()
        self.stepper_align(self.start_offset)
        # abs angle distance
        def adist(a1, a2):
            diff = (a1 - a2 + 180) % 360 - 180
            return abs(diff)
        # Recalculate fullstep angles
        fs_angles = {
            0: self.last_move_angle()
        }
        fs_diffs = []
        for i in range(1, 5):
            self.move(self.dir * self.full_step_dist)
            fs_angles[i] = self.last_move_angle()
            if i > 0:
                adiff = adist(fs_angles[i], fs_angles[i-1])
                fs_diffs.append(adiff)
        self.move(-self.dir * self.full_step_dist * 4)
        self.ms_angle = [fs_diffs[0]/self.microsteps, fs_diffs[1]/self.microsteps,
                           fs_diffs[2]/self.microsteps, fs_diffs[3]/self.microsteps]
        pos_angle = self.last_move_angle()
        # Try converge to ideal steps
        ideal_angle = pos_angle
        pos_angle_prev = pos_angle
        logging.info(f"pos: {1024-self.mscnt_min}, tgt: {ideal_angle:.3f}, act: {pos_angle:.3f}, fs_angle: {fs_angles[0]:3.3f}")
        ideal_angle += self.ms_angle[0] * self.angle_dir
        min_dist = 1
        max_dist = 0
        ms_dist = []
        pos_dist = []
        sin_up = sin.copy()
        sin_down = sin.copy()
        stealth = False
        if self.get_field("stealth"):
            stealth = True
        for pos in self.positions:
            # compensate interpolation lag and backlash
            self.move(self.dir * self.step_dist * 2)
            self.move(-self.dir * self.step_dist * 2)
            self.move(self.dir * self.step_dist)
            pos_angle = self.last_move_angle()
            pos_diff = self.angle_dist(pos_angle, pos_angle_prev)
            pos_diff = abs(pos_diff)
            pos_dist.append(pos_diff)
            pos_angle_prev = pos_angle
            distance = self.angle_dist(ideal_angle, pos_angle)
            ms_dist.append(distance)
            logging.info(f"pos: {pos:4}, tgt: {ideal_angle:3.3f}, act: {pos_angle:3.3f}, dist: {distance:1.3f}, diff: {pos_diff:3.3f}, tgt_fs_angle: {fs_angles[1+pos//256]:3.3f}")
            ideal_angle = pos_angle + self.ms_angle[pos//256] * self.angle_dir
            if (pos % 256) == (256 - self.mscnt_min):
                # Compensate error
                ideal_angle = pos_angle + self.ms_angle[pos//256] * self.angle_dir
                logging.info("---")
            min_dist = min(min_dist, distance)
            max_dist = max(max_dist, distance)
            # Average over fullstep
            change = 0.26
            if stealth:
                change = 1
            pos = pos % 256

            if abs(distance) > 1:
                logging.error("Driver insane - abort")
                raise Exception()

            if distance < -self.misalign:
                up += 1
                if self.microsteps <= 128 and sin_up[pos-1]-sin_up[pos] == 3:
                    # Decrease right side if left can't be increased
                    pos = 256 - (pos % 256)
                    sin_up[pos] -= change
                else:
                    sin_up[pos] += change
            elif distance > self.misalign:
                down += 1
                if self.microsteps <= 128 and sin_down[pos-1] - sin_down[pos] == -1:
                    # Increase right side if left can't be decreased
                    pos = 256 - (pos % 256)
                    sin_up[pos] += change
                else:
                    sin_down[pos] -= change
            else:
                matched += 1

        not_matched = up + down
        return {
            "stddev": self.std(pos_dist),
            "ms_dist": [round(i,2) for i in ms_dist],
            "min_dist": min_dist,
            "max_dist": max_dist,
            "abs_dist": abs(min_dist) + abs(max_dist),
            "sin_up": sin_up,
            "up": up,
            "sin_down": sin_down,
            "down": down,
            "not_matched": not_matched,
            "matched": matched
        }

    def choise_best(self, left, right):
        left_res = self.measure_sin(left)
        right_res = self.measure_sin(right)
        if left_res["stddev"] < right_res["stddev"]:
            return left, left_res
        return right, right_res

    def _force_disable(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        disable = stepper_enable.lookup_enable(self.stepper_name)
        was_enable = disable.is_motor_enabled()
        if was_enable:
            disable.motor_disable(print_time)
            toolhead.dwell(0.100)


    def sin_apply(self, sin_new):
        # self._force_disable()
        mslut = self.mslut_encoder(sin_new)
        for i in range(0, 8):
            self.set_field("mslut%i" % (i), mslut["MSLUTS"][i])
        self.tmc.fields.set_field("start_sin", mslut["START_SIN"])
        MSLUTSTART = self.tmc.fields.set_field("start_sin90", mslut["START_SIN90"])
        self.tmc.set_register("MSLUTSTART", MSLUTSTART)
        for i in (3, 2, 1):
            f = "x" + str(i)
            self.tmc.fields.set_field(f, mslut["X"][i])
            f = "w" + str(i)
            self.tmc.fields.set_field(f, mslut["W"][i])
        MSLUTSEL = self.tmc.fields.set_field(f, mslut["W"][0])
        self.tmc.set_register("MSLUTSEL", MSLUTSEL)

        # Force reread mslut
        self.move(self.full_step_dist * 8)
        self.move(self.full_step_dist * -8)

    def angle_dist(self, target, actual):
        diff = (target - actual + 180) % 360 - 180
        if self.angle_dir > 0:
            if target > actual:
                return -abs(diff)
            else:
                return abs(diff)
        if self.angle_dir < 0:
            if target < actual:
                return -abs(diff)
            else:
                return abs(diff)

    cmd_ANGLE_TMC_CALIBRATE_help = "Calibrate stepper driver by angle sensor"
    def cmd_ANGLE_TMC_CALIBRATE(self, gcmd):
        # Start data collection
        self.is_finished = False
        self.printer.lookup_object(self.name).add_client(self.handle_batch)

        # Looks like mslut only really applied when mscnt == 0
        gcmd.respond_info("Enable interpolation")
        self.set_field("intpol", 1)

        # abs angle distance
        def adist(a1, a2):
            diff = (a1 - a2 + 180) % 360 - 180
            return abs(diff)

        self.guess_real_resolution()
        fs_angle = 360 / self.full_steps
        fs_resolution = (self.real_resolution / self.full_steps)
        gcmd.respond_info("Real resolution: %i, per fs: %.2f" % (
                          self.real_resolution,
                          fs_resolution))

        self.stepper_align(self.start_offset)
        # Align stepper to zero
        fs_angles = {
            0: self.last_move_angle()
        }
        fs_diffs = []
        for i in range(1, 5):
            self.move(self.dir * self.full_step_dist)
            fs_angles[i] = self.last_move_angle()
            if i > 0:
                adiff = adist(fs_angles[i], fs_angles[i-1])
                fs_diffs.append(adiff)
        fs_4_diff = adist(fs_angles[4], fs_angles[0])
        gcmd.respond_info("FullStep %.1f angles: %.2f %.2f %.2f %.2f ~ %.2f" %
                          (fs_angle, fs_diffs[0], fs_diffs[1],
                           fs_diffs[2], fs_diffs[3], fs_4_diff))
        self.move(-4 * self.dir * self.full_step_dist)
        self.angle_dir = 1
        if fs_angles[0] > fs_angles[2]:
            self.angle_dir = -1

        self.ms_angle = [fs_diffs[0]/self.microsteps, fs_diffs[1]/self.microsteps,
                           fs_diffs[2]/self.microsteps, fs_diffs[3]/self.microsteps]
        # # Assume 1/4 is fine for large microsteps
        # self.misalign = max(self.misalign, sum(self.ms_angle)/len(self.ms_angle) / 5)
        gcmd.respond_info(
            "Ideal step angle: %.4f, allowed drift: %.4f" % (
            sum(self.ms_angle)/len(self.ms_angle), self.misalign))

        tries = 48
        sin_value = self.mslut_decoder()
        try:
            res = self.measure_sin(sin_value)
        except Exception:
            self.is_finished = True
            self.msgs = []
            gcmd.respond_info("Something really broken here")
            return
        stddev = res["stddev"]
        ms_dist = res["ms_dist"]
        min_dist = res["min_dist"]
        max_dist = res["max_dist"]
        abs_dist = res["abs_dist"]
        up = res["up"]
        down = res["down"]
        sin_up = res["sin_up"]
        sin_down = res["sin_down"]
        not_matched = res["not_matched"]
        matched = res["matched"]
        gcmd.respond_info("Initial Values")
        history = [{
                "sin": sin_value,
                "stddev": stddev,
                "abs_dist": abs_dist,
                "not_matched": not_matched,
            }]
        while tries and not_matched > 0:
            tries -= 1
            gcmd.respond_info(
                "Step distance Min %.6f, Max %.6f, Abs: %.6f" % (
                    min_dist, max_dist, abs_dist))
            gcmd.respond_info(
                "Unaligned steps: %i/%i, stddev: %.4f, up: %i, down: %i" % (
                not_matched, not_matched + matched, stddev, up, down))
            if (len(history) > 4 and
                (history[-4]["stddev"] < history[-3]["stddev"]) and
                (history[-3]["stddev"] < history[-2]["stddev"]) and
                (history[-2]["stddev"] < history[-1]["stddev"])):
                gcmd.respond_info("stddev only increasing - abort")
                break

            logging.info(f"ms_dist = {ms_dist}")
            sin_up = self.interp_or_fit(sin_up)
            sin_down = self.interp_or_fit(sin_down)
            logging.info(f"sin_up = {sin_up}")
            logging.info(f"sin_down = {sin_down}")
            sin_new = sin_down
            try:
                if up > 0 and down > 0:
                    sin_new, res = self.choise_best(sin_up, sin_down)
                elif down == 0:
                    sin_new = sin_up
                    res = self.measure_sin(sin_up)
                else:
                    sin_new = sin_down
                    res = self.measure_sin(sin_down)
            except Exception:
                break

            if sin_up == sin_new:
                gcmd.respond_info("Follow up")
            else:
                gcmd.respond_info("Follow down")

            stddev = res["stddev"]
            ms_dist = res["ms_dist"]
            min_dist = res["min_dist"]
            max_dist = res["max_dist"]
            abs_dist = res["abs_dist"]
            up = res["up"]
            down = res["down"]
            sin_up = res["sin_up"]
            sin_down = res["sin_down"]
            not_matched = res["not_matched"]
            matched = res["matched"]

            history.append({
                "sin": sin_new,
                "stddev": stddev,
                "abs_dist": abs_dist,
                "not_matched": not_matched,
            })
            self.sin_apply(sin_new)

        gcmd.respond_info("Choise best from history")
        best = min(history, key=lambda x: x["abs_dist"])
        gcmd.respond_info(f"Best STDDev: {best["stddev"]:.3f} ABS Distance: {best["abs_dist"]:.3f}")
        sin_new = best["sin"]
        mslut = self.mslut_encoder(sin_new)
        self.sin_apply(sin_new)
        self.move_reset()

        cfgname = self.driver_name
        configfile = self.printer.lookup_object('configfile')
        for i in range(0, 8):
            configfile.set(cfgname, 'driver_MSLUT%i' % (i), mslut["MSLUTS"][i])
        for i in range(1, 4):
            configfile.set(cfgname, 'driver_X%i' % (i), mslut["X"][i])
        for i in range(0, 4):
            configfile.set(cfgname, 'driver_W%i' % (i), mslut["W"][i])
        configfile.set(cfgname, 'driver_START_SIN', mslut["START_SIN"])
        configfile.set(cfgname, 'driver_START_SIN90', mslut["START_SIN90"])
        self.is_finished = True
        self.msgs = []
        gcmd.respond_info("Can't improve farther, you can click save config")

class HelperA1333:
    SPI_MODE = 3
    SPI_SPEED = 10000000
    def __init__(self, config, spi, oid):
        self.spi = spi
        self.is_tcode_absolute = False
        self.last_temperature = None
    def get_static_delay(self):
        return .000001
    def start(self):
        # Setup for angle query
        self.spi.spi_transfer([0x32, 0x00])

class HelperAS5047D:
    SPI_MODE = 1
    SPI_SPEED = int(1. / .000000350)
    def __init__(self, config, spi, oid):
        self.spi = spi
        self.is_tcode_absolute = False
        self.last_temperature = None
    def get_static_delay(self):
        return .000100
    def start(self):
        # Clear any errors from device
        self.spi.spi_transfer([0xff, 0xfc]) # Read DIAAGC
        self.spi.spi_transfer([0x40, 0x01]) # Read ERRFL
        self.spi.spi_transfer([0xc0, 0x00]) # Read NOP

class HelperTLE5012B:
    SPI_MODE = 1
    SPI_SPEED = 4000000
    def __init__(self, config, spi, oid):
        self.printer = config.get_printer()
        self.spi = spi
        self.oid = oid
        self.is_tcode_absolute = True
        self.last_temperature = None
        self.mcu = spi.get_mcu()
        self.mcu.register_config_callback(self._build_config)
        self.spi_angle_transfer_cmd = None
        self.last_chip_mcu_clock = self.last_chip_clock = 0
        self.chip_freq = 0.
        name = config.get_name().split()[-1]
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("ANGLE_DEBUG_READ", "CHIP", name,
                                   self.cmd_ANGLE_DEBUG_READ,
                                   desc=self.cmd_ANGLE_DEBUG_READ_help)
        gcode.register_mux_command("ANGLE_DEBUG_WRITE", "CHIP", name,
                                   self.cmd_ANGLE_DEBUG_WRITE,
                                   desc=self.cmd_ANGLE_DEBUG_WRITE_help)
    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.spi_angle_transfer_cmd = self.mcu.lookup_query_command(
            "spi_angle_transfer oid=%c data=%*s",
            "spi_angle_transfer_response oid=%c clock=%u response=%*s",
            oid=self.oid, cq=cmdqueue)
    def get_tcode_params(self):
        return self.last_chip_mcu_clock, self.last_chip_clock, self.chip_freq
    def _calc_crc(self, data):
        crc = 0xff
        for d in data:
            crc ^= d
            for i in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x1d
                else:
                    crc <<= 1
        return (~crc) & 0xff
    def _send_spi(self, msg):
        for retry in range(5):
            if msg[0] & 0x04:
                params = self.spi_angle_transfer_cmd.send([self.oid, msg])
            else:
                params = self.spi.spi_transfer(msg)
            resp = bytearray(params['response'])
            crc = self._calc_crc(bytearray(msg[:2]) + resp[2:-2])
            if crc == resp[-1]:
                return params
        raise self.printer.command_error("Unable to query tle5012b chip")
    def _read_reg(self, reg):
        cw = 0x8000 | ((reg & 0x3f) << 4) | 0x01
        if reg >= 0x05 and reg <= 0x11:
            cw |= 0x5000
        msg = [cw >> 8, cw & 0xff, 0, 0, 0, 0]
        params = self._send_spi(msg)
        resp = bytearray(params['response'])
        return (resp[2] << 8) | resp[3]
    def _write_reg(self, reg, val):
        cw = ((reg & 0x3f) << 4) | 0x01
        if reg >= 0x05 and reg <= 0x11:
            cw |= 0x5000
        msg = [cw >> 8, cw & 0xff, (val >> 8) & 0xff, val & 0xff, 0, 0]
        for retry in range(5):
            self._send_spi(msg)
            rval = self._read_reg(reg)
            if rval == val:
                return
        raise self.printer.command_error("Unable to write to tle5012b chip")
    def _mask_reg(self, reg, off, on):
        rval = self._read_reg(reg)
        self._write_reg(reg, (rval & ~off) | on)
    def _query_clock(self):
        # Read frame counter (and normalize to a 16bit counter)
        msg = [0x84, 0x42, 0, 0, 0, 0, 0, 0] # Read with latch, AREV and FSYNC
        params = self._send_spi(msg)
        resp = bytearray(params['response'])
        mcu_clock = self.mcu.clock32_to_clock64(params['clock'])
        chip_clock = ((resp[2] & 0x7e) << 9) | ((resp[4] & 0x3e) << 4)
        # Calculate temperature
        temper = resp[5] - ((resp[4] & 0x01) << 8)
        self.last_temperature = (temper + 152) / 2.776
        return mcu_clock, chip_clock
    def update_clock(self):
        mcu_clock, chip_clock = self._query_clock()
        mdiff = mcu_clock - self.last_chip_mcu_clock
        chip_mclock = self.last_chip_clock + int(mdiff * self.chip_freq + .5)
        cdiff = (chip_clock - chip_mclock) & 0xffff
        cdiff -= (cdiff & 0x8000) << 1
        new_chip_clock = chip_mclock + cdiff
        self.chip_freq = float(new_chip_clock - self.last_chip_clock) / mdiff
        self.last_chip_clock = new_chip_clock
        self.last_chip_mcu_clock = mcu_clock
    def start(self):
        # Clear any errors from device
        self._read_reg(0x00) # Read STAT
        # Initialize chip (so different chip variants work the same way)
        self._mask_reg(0x06, 0xc003, 0x4000) # MOD1: 42.7us, IIF disable
        self._mask_reg(0x08, 0x0007, 0x0001) # MOD2: Predict off, autocal=1
        self._mask_reg(0x0e, 0x0003, 0x0000) # MOD4: IIF mode
        # Setup starting clock values
        mcu_clock, chip_clock = self._query_clock()
        self.last_chip_clock = chip_clock
        self.last_chip_mcu_clock = mcu_clock
        self.chip_freq = float(1<<5) / self.mcu.seconds_to_clock(1. / 750000.)
        self.update_clock()
    cmd_ANGLE_DEBUG_READ_help = "Query low-level angle sensor register"
    def cmd_ANGLE_DEBUG_READ(self, gcmd):
        reg = gcmd.get("REG", minval=0, maxval=0x30, parser=lambda x: int(x, 0))
        val = self._read_reg(reg)
        gcmd.respond_info("ANGLE REG[0x%02x] = 0x%04x" % (reg, val))
    cmd_ANGLE_DEBUG_WRITE_help = "Set low-level angle sensor register"
    def cmd_ANGLE_DEBUG_WRITE(self, gcmd):
        reg = gcmd.get("REG", minval=0, maxval=0x30, parser=lambda x: int(x, 0))
        val = gcmd.get("VAL", minval=0, maxval=0xffff,
                       parser=lambda x: int(x, 0))
        self._write_reg(reg, val)

class HelperMT6816:
    SPI_MODE = 3
    SPI_SPEED = 10000000
    def __init__(self, config, spi, oid):
        self.printer = config.get_printer()
        self.spi = spi
        self.oid = oid
        self.mcu = spi.get_mcu()
        self.mcu.register_config_callback(self._build_config)
        self.spi_angle_transfer_cmd = None
        self.is_tcode_absolute = False
        self.last_temperature = None
        name = config.get_name().split()[-1]
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("ANGLE_DEBUG_READ", "CHIP", name,
                                   self.cmd_ANGLE_DEBUG_READ,
                                   desc=self.cmd_ANGLE_DEBUG_READ_help)
    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.spi_angle_transfer_cmd = self.mcu.lookup_query_command(
            "spi_angle_transfer oid=%c data=%*s",
            "spi_angle_transfer_response oid=%c clock=%u response=%*s",
            oid=self.oid, cq=cmdqueue)
    def _send_spi(self, msg):
        return self.spi.spi_transfer(msg)
    def get_static_delay(self):
        return .000001
    def _read_reg(self, reg):
        msg = [reg, 0, 0]
        params = self._send_spi(msg)
        resp = bytearray(params['response'])
        val =  (resp[1] << 8) | resp[2]
        return val
    def start(self):
        pass
    cmd_ANGLE_DEBUG_READ_help = "Query low-level angle sensor register"
    def cmd_ANGLE_DEBUG_READ(self, gcmd):
        reg = 0x83
        val = self._read_reg(reg)
        gcmd.respond_info("ANGLE REG[0x%02x] = 0x%04x" % (reg, val))
        angle = val >> 2
        parity = bin(val >> 1).count("1") % 2
        gcmd.respond_info("Angle %i ~ %.2f" % (angle, angle * 360 / (1 << 14)))
        gcmd.respond_info("No Mag: %i" % (val >> 1 & 0x1))
        gcmd.respond_info("Parity: %i == %i" % (parity, val & 0x1))

class HelperMT6826S:
    SPI_MODE = 3
    SPI_SPEED = 10000000
    def __init__(self, config, spi, oid):
        self.printer = config.get_printer()
        self.stepper_name = config.get('stepper', None)
        self.spi = spi
        self.oid = oid
        self.mcu = spi.get_mcu()
        self.mcu.register_config_callback(self._build_config)
        self.spi_angle_transfer_cmd = None
        self.is_tcode_absolute = False
        self.last_temperature = None
        name = config.get_name().split()[-1]
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("ANGLE_DEBUG_READ", "CHIP", name,
                                   self.cmd_ANGLE_DEBUG_READ,
                                   desc=self.cmd_ANGLE_DEBUG_READ_help)
        gcode.register_mux_command("ANGLE_CHIP_CALIBRATE", "CHIP", name,
                                   self.cmd_ANGLE_CHIP_CALIBRATE,
                                   desc=self.cmd_ANGLE_CHIP_CALIBRATE_help)
        self.status_map = {
            0: "No Calibration",
            1: "Running Calibration",
            2: "Calibration Failed",
            3: "Calibration Successful"
        }
    def _build_config(self):
        cmdqueue = self.spi.get_command_queue()
        self.spi_angle_transfer_cmd = self.mcu.lookup_query_command(
            "spi_angle_transfer oid=%c data=%*s",
            "spi_angle_transfer_response oid=%c clock=%u response=%*s",
            oid=self.oid, cq=cmdqueue)
    def _send_spi(self, msg):
        params = self.spi.spi_transfer(msg)
        return params
    def get_static_delay(self):
        return .00001
    def _read_reg(self, reg):
        reg = 0x3000 | reg
        msg = [reg >> 8, reg & 0xff, 0]
        params = self._send_spi(msg)
        resp = bytearray(params['response'])
        return resp[2]
    def _write_reg(self, reg, data):
        reg = 0x6000 | reg
        msg = [reg >> 8, reg & 0xff, data]
        self._send_spi(msg)
    def crc8(self, data):
        polynomial = 0x07
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc
    def _read_angle(self, reg):
        reg = 0x3000 | reg
        msg = [reg >> 8, reg & 0xff, 0, 0, 0, 0]
        params = self._send_spi(msg)
        resp = bytearray(params['response'])
        angle = (resp[2] << 7) | (resp[3] >> 1)
        status = resp[4]
        crc_computed = self.crc8([resp[2], resp[3], resp[4]])
        crc = resp[5]
        return angle, status, crc, crc_computed
    def start(self):
        val = self._read_reg(0x00d)
        # Set histeresis to 0.003 degree
        self._write_reg(0x00d, (val & 0xf8) | 0x5)
    def get_microsteps(self):
        configfile = self.printer.lookup_object('configfile')
        sconfig = configfile.get_status(None)['settings']
        stconfig = sconfig.get(self.stepper_name, {})
        microsteps = stconfig['microsteps']
        full_steps = stconfig['full_steps_per_rotation']
        return microsteps, full_steps
    cmd_ANGLE_CHIP_CALIBRATE_help = "Run MT6826s calibration sequence"
    def cmd_ANGLE_CHIP_CALIBRATE(self, gcmd):
        fmove = self.printer.lookup_object('force_move')
        mcu_stepper = fmove.lookup_stepper(self.stepper_name)
        if self.stepper_name is None:
            gcmd.respond_info("stepper not defined")
            return

        gcmd.respond_info("MT6826S Run calibration sequence")
        gcmd.respond_info("Motor will do 18+ rotations -" +
                          " ensure pulley is disconnected")
        req_freq = self._read_reg(0x00e) >> 4 & 0x7
        # Minimal calibration speed
        rpm = (3200 >> req_freq) + 1
        rps = rpm / 60
        move = fmove.manual_move
        # Move stepper several turns (to allow internal sensor calibration)
        microsteps, full_steps = self.get_microsteps()
        step_dist = mcu_stepper.get_step_dist()
        full_step_dist = step_dist * microsteps
        rotation_dist = full_steps * full_step_dist
        move(mcu_stepper, 2 * rotation_dist, rps * rotation_dist)
        self._write_reg(0x155, 0x5e)
        move(mcu_stepper, 20 * rotation_dist, rps * rotation_dist)
        val = self._read_reg(0x113)
        code = val >> 6
        gcmd.respond_info("Status: %s" % (self.status_map[code]))
        while code == 1:
            move(mcu_stepper, 5 * rotation_dist, rps * rotation_dist)
            val = self._read_reg(0x113)
            code = val >> 6
            gcmd.respond_info("Status: %s" % (self.status_map[code]))
        if code == 2:
            gcmd.respond_info("Calibration failed")
        if code == 3:
            gcmd.respond_info("Calibration success, please poweroff sensor")
    cmd_ANGLE_DEBUG_READ_help = "Query low-level angle sensor register"
    def cmd_ANGLE_DEBUG_READ(self, gcmd):
        reg = gcmd.get("REG", minval=0, maxval=0x155,
                       parser=lambda x: int(x, 0))
        if reg == 0x003:
            angle, status, crc1, crc2 = self._read_angle(reg)
            gcmd.respond_info("ANGLE REG[0x003] = 0x%02x" %
                              (angle >> 7))
            gcmd.respond_info("ANGLE REG[0x004] = 0x%02x" %
                              ((angle << 1) & 0xff))
            gcmd.respond_info("Angle %i ~ %.2f" % (angle,
                                                   angle * 360 / (1 << 15)))
            gcmd.respond_info("Weak Mag: %i" % (status >> 1 & 0x1))
            gcmd.respond_info("Under Voltage: %i" % (status >> 2 & 0x1))
            gcmd.respond_info("CRC: 0x%02x == 0x%02x" % (crc1, crc2))
        elif reg == 0x00e:
            val = self._read_reg(reg)
            gcmd.respond_info("GPIO_DS = %i" % (val >> 7))
            gcmd.respond_info("AUTOCAL_FREQ = %i" % (val >> 4 & 0x7))
        elif reg == 0x113:
            val = self._read_reg(reg)
            gcmd.respond_info("Status: %s" % (self.cal_status[val >> 6]))
        else:
            val = self._read_reg(reg)
            gcmd.respond_info("REG[0x%04x] = 0x%02x" % (reg, val))


BYTES_PER_SAMPLE = 3
SAMPLES_PER_BLOCK = bulk_sensor.MAX_BULK_MSG_SIZE // BYTES_PER_SAMPLE

SAMPLE_PERIOD = 0.000400
BATCH_UPDATES = 0.100

class Angle:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sample_period = config.getfloat('sample_period', SAMPLE_PERIOD,
                                             above=0.)
        self.calibration = AngleCalibration(config)
        self.tmc_debug = AngleTMCCalibration(config)
        # Measurement conversion
        self.start_clock = self.time_shift = self.sample_ticks = 0
        self.last_sequence = self.last_angle = 0
        # Sensor type
        sensors = { "a1333": HelperA1333,
                    "as5047d": HelperAS5047D,
                    "tle5012b": HelperTLE5012B,
                    "mt6816": HelperMT6816,
                    "mt6826s": HelperMT6826S }
        sensor_type = config.getchoice('sensor_type', {s: s for s in sensors})
        sensor_class = sensors[sensor_type]
        self.spi = bus.MCU_SPI_from_config(config, sensor_class.SPI_MODE,
                                           default_speed=sensor_class.SPI_SPEED)
        self.mcu = mcu = self.spi.get_mcu()
        self.oid = oid = mcu.create_oid()
        self.sensor_helper = sensor_class(config, self.spi, oid)
        # Setup mcu sensor_spi_angle bulk query code
        self.query_spi_angle_cmd = None
        mcu.add_config_cmd(
            "config_spi_angle oid=%d spi_oid=%d spi_angle_type=%s"
            % (oid, self.spi.get_oid(), sensor_type))
        mcu.add_config_cmd(
            "query_spi_angle oid=%d clock=0 rest_ticks=0 time_shift=0"
            % (oid,), on_restart=True)
        mcu.register_config_callback(self._build_config)
        self.bulk_queue = bulk_sensor.BulkDataQueue(mcu, oid=oid)
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch,
            self._start_measurements, self._finish_measurements, BATCH_UPDATES)
        self.name = config.get_name().split()[1]
        api_resp = {'header': ('time', 'angle')}
        self.batch_bulk.add_mux_endpoint("angle/dump_angle",
                                         "sensor", self.name, api_resp)
    def _build_config(self):
        freq = self.mcu.seconds_to_clock(1.)
        while float(TCODE_ERROR << self.time_shift) / freq < 0.002:
            self.time_shift += 1
        cmdqueue = self.spi.get_command_queue()
        self.query_spi_angle_cmd = self.mcu.lookup_command(
            "query_spi_angle oid=%c clock=%u rest_ticks=%u time_shift=%c",
            cq=cmdqueue)
    def get_status(self, eventtime=None):
        return {'temperature': self.sensor_helper.last_temperature}
    def add_client(self, client_cb):
        self.batch_bulk.add_client(client_cb)
    # Measurement decoding
    def _extract_samples(self, raw_samples):
        # Load variables to optimize inner loop below
        sample_ticks = self.sample_ticks
        start_clock = self.start_clock
        clock_to_print_time = self.mcu.clock_to_print_time
        last_sequence = self.last_sequence
        last_angle = self.last_angle
        time_shift = 0
        static_delay = 0.
        last_chip_mcu_clock = last_chip_clock = chip_freq = inv_chip_freq = 0.
        is_tcode_absolute = self.sensor_helper.is_tcode_absolute
        if is_tcode_absolute:
            tparams = self.sensor_helper.get_tcode_params()
            last_chip_mcu_clock, last_chip_clock, chip_freq = tparams
            inv_chip_freq = 1. / chip_freq
        else:
            time_shift = self.time_shift
            static_delay = self.sensor_helper.get_static_delay()
        # Process every message in raw_samples
        count = error_count = 0
        samples = [None] * (len(raw_samples) * SAMPLES_PER_BLOCK)
        for params in raw_samples:
            seq_diff = (params['sequence'] - last_sequence) & 0xffff
            last_sequence += seq_diff
            samp_count = last_sequence * SAMPLES_PER_BLOCK
            msg_mclock = start_clock + samp_count*sample_ticks
            d = bytearray(params['data'])
            for i in range(len(d) // BYTES_PER_SAMPLE):
                d_ta = d[i*BYTES_PER_SAMPLE:(i+1)*BYTES_PER_SAMPLE]
                tcode = d_ta[0]
                if tcode == TCODE_ERROR:
                    error_count += 1
                    continue
                raw_angle = d_ta[1] | (d_ta[2] << 8)
                angle_diff = (raw_angle - last_angle) & 0xffff
                angle_diff -= (angle_diff & 0x8000) << 1
                last_angle += angle_diff
                mclock = msg_mclock + i*sample_ticks
                if is_tcode_absolute:
                    # tcode is tle5012b frame counter
                    mdiff = mclock - last_chip_mcu_clock
                    chip_mclock = last_chip_clock + int(mdiff * chip_freq + .5)
                    cdiff = ((tcode << 10) - chip_mclock) & 0xffff
                    cdiff -= (cdiff & 0x8000) << 1
                    sclock = mclock + (cdiff - 0x800) * inv_chip_freq
                else:
                    # tcode is mcu clock offset shifted by time_shift
                    sclock = mclock + (tcode<<time_shift)
                ptime = round(clock_to_print_time(sclock) - static_delay, 6)
                samples[count] = (ptime, last_angle)
                count += 1
        self.last_sequence = last_sequence
        self.last_angle = last_angle
        del samples[count:]
        return samples, error_count
    # Start, stop, and process message batches
    def _is_measuring(self):
        return self.start_clock != 0
    def _start_measurements(self):
        logging.info("Starting angle '%s' measurements", self.name)
        self.sensor_helper.start()
        # Start bulk reading
        self.bulk_queue.clear_queue()
        self.last_sequence = 0
        systime = self.printer.get_reactor().monotonic()
        print_time = self.mcu.estimated_print_time(systime) + MIN_MSG_TIME
        self.start_clock = reqclock = self.mcu.print_time_to_clock(print_time)
        rest_ticks = self.mcu.seconds_to_clock(self.sample_period)
        self.sample_ticks = rest_ticks
        self.query_spi_angle_cmd.send([self.oid, reqclock, rest_ticks,
                                       self.time_shift], reqclock=reqclock)
    def _finish_measurements(self):
        # Halt bulk reading
        self.query_spi_angle_cmd.send_wait_ack([self.oid, 0, 0, 0])
        self.bulk_queue.clear_queue()
        self.sensor_helper.last_temperature = None
        logging.info("Stopped angle '%s' measurements", self.name)
    def _process_batch(self, eventtime):
        if self.sensor_helper.is_tcode_absolute:
            self.sensor_helper.update_clock()
        raw_samples = self.bulk_queue.pull_queue()
        if not raw_samples:
            return {}
        samples, error_count = self._extract_samples(raw_samples)
        if not samples:
            return {}
        rsamples = samples.copy()
        offset = self.calibration.apply_calibration(samples)
        return {'data': samples,
                'data_raw': rsamples,
                'errors': error_count,
                'position_offset': offset}

def load_config_prefix(config):
    return Angle(config)
