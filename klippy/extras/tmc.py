# Common helper code for TMC stepper drivers
#
# Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, collections
import stepper
from . import bulk_sensor


######################################################################
# Field helpers
######################################################################

# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1

class FieldHelper:
    def __init__(self, all_fields, signed_fields=[], field_formatters={},
                 registers=None):
        self.all_fields = all_fields
        self.signed_fields = {sf: 1 for sf in signed_fields}
        self.field_formatters = field_formatters
        self.registers = registers
        if self.registers is None:
            self.registers = collections.OrderedDict()
        self.field_to_register = { f: r for r, fields in self.all_fields.items()
                                   for f in fields }
    def lookup_register(self, field_name, default=None):
        return self.field_to_register.get(field_name, default)
    def get_field(self, field_name, reg_value=None, reg_name=None):
        # Returns value of the register field
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        field_value = (reg_value & mask) >> ffs(mask)
        if field_name in self.signed_fields and ((reg_value & mask)<<1) > mask:
            field_value -= (1 << field_value.bit_length())
        return field_value
    def set_field(self, field_name, field_value, reg_value=None, reg_name=None):
        # Returns register value with field bits filled with supplied value
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        new_value = (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
        self.registers[reg_name] = new_value
        return new_value
    def set_config_field(self, config, field_name, default):
        # Allow a field to be set from the config file
        config_name = "driver_" + field_name.upper()
        reg_name = self.field_to_register[field_name]
        mask = self.all_fields[reg_name][field_name]
        maxval = mask >> ffs(mask)
        if maxval == 1:
            val = config.getboolean(config_name, default)
        elif field_name in self.signed_fields:
            val = config.getint(config_name, default,
                                minval=-(maxval//2 + 1), maxval=maxval//2)
        else:
            val = config.getint(config_name, default, minval=0, maxval=maxval)
        return self.set_field(field_name, val)
    def pretty_format(self, reg_name, reg_value):
        # Provide a string description of a register
        reg_fields = self.all_fields.get(reg_name, {})
        reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
        fields = []
        for mask, field_name in reg_fields:
            field_value = self.get_field(field_name, reg_value, reg_name)
            sval = self.field_formatters.get(field_name, str)(field_value)
            if sval and sval != "0":
                fields.append(" %s=%s" % (field_name, sval))
        return "%-11s %08x%s" % (reg_name + ":", reg_value, "".join(fields))
    def get_reg_fields(self, reg_name, reg_value):
        # Provide fields found in a register
        reg_fields = self.all_fields.get(reg_name, {})
        return {field_name: self.get_field(field_name, reg_value, reg_name)
                for field_name, mask in reg_fields.items()}


######################################################################
# Periodic error checking
######################################################################

class TMCErrorCheck:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        name_parts = config.get_name().split()
        self.stepper_name = ' '.join(name_parts[1:])
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.check_timer = None
        self.last_drv_status = self.last_drv_fields = None
        # Setup for GSTAT query
        reg_name = self.fields.lookup_register("drv_err")
        if reg_name is not None:
            self.gstat_reg_info = [0, reg_name, 0xffffffff, 0xffffffff, 0]
        else:
            self.gstat_reg_info = None
        self.clear_gstat = True
        # Setup for DRV_STATUS query
        self.irun_field = "irun"
        reg_name = "DRV_STATUS"
        mask = err_mask = cs_actual_mask = 0
        if name_parts[0] == 'tmc2130':
            # TMC2130 driver quirks
            self.clear_gstat = False
            cs_actual_mask = self.fields.all_fields[reg_name]["cs_actual"]
        elif name_parts[0] == 'tmc2660':
            # TMC2660 driver quirks
            self.irun_field = "cs"
            reg_name = "READRSP@RDSEL2"
            cs_actual_mask = self.fields.all_fields[reg_name]["se"]
        err_fields = ["ot", "s2ga", "s2gb", "s2vsa", "s2vsb"]
        warn_fields = ["otpw", "t120", "t143", "t150", "t157"]
        for f in err_fields + warn_fields:
            if f in self.fields.all_fields[reg_name]:
                mask |= self.fields.all_fields[reg_name][f]
                if f in err_fields:
                    err_mask |= self.fields.all_fields[reg_name][f]
        self.drv_status_reg_info = [0, reg_name, mask, err_mask, cs_actual_mask]
        # Setup for temperature query
        self.adc_temp = None
        self.adc_temp_reg = self.fields.lookup_register("adc_temp")
        if self.adc_temp_reg is not None:
            pheaters = self.printer.load_object(config, 'heaters')
            pheaters.register_monitor(config)
    def _query_register(self, reg_info, try_clear=False):
        last_value, reg_name, mask, err_mask, cs_actual_mask = reg_info
        cleared_flags = 0
        count = 0
        while 1:
            try:
                val = self.mcu_tmc.get_register(reg_name)
            except self.printer.command_error as e:
                count += 1
                if count < 3 and str(e).startswith("Unable to read tmc uart"):
                    # Allow more retries on a TMC UART read error
                    reactor = self.printer.get_reactor()
                    reactor.pause(reactor.monotonic() + 0.050)
                    continue
                raise
            if val & mask != last_value & mask:
                fmt = self.fields.pretty_format(reg_name, val)
                logging.info("TMC '%s' reports %s", self.stepper_name, fmt)
            reg_info[0] = last_value = val
            if not val & err_mask:
                if not cs_actual_mask or val & cs_actual_mask:
                    break
                irun = self.fields.get_field(self.irun_field)
                if self.check_timer is None or irun < 4:
                    break
                if (self.irun_field == "irun"
                    and not self.fields.get_field("ihold")):
                    break
                # CS_ACTUAL field of zero - indicates a driver reset
            count += 1
            if count >= 3:
                fmt = self.fields.pretty_format(reg_name, val)
                raise self.printer.command_error("TMC '%s' reports error: %s"
                                                 % (self.stepper_name, fmt))
            if try_clear and val & err_mask:
                try_clear = False
                cleared_flags |= val & err_mask
                self.mcu_tmc.set_register(reg_name, val & err_mask)
        return cleared_flags
    def _query_temperature(self):
        try:
            self.adc_temp = self.mcu_tmc.get_register(self.adc_temp_reg)
        except self.printer.command_error as e:
            # Ignore comms error for temperature
            self.adc_temp = None
            return
    def _do_periodic_check(self, eventtime):
        try:
            self._query_register(self.drv_status_reg_info)
            if self.gstat_reg_info is not None:
                self._query_register(self.gstat_reg_info)
            if self.adc_temp_reg is not None:
                self._query_temperature()
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
            return self.printer.get_reactor().NEVER
        return eventtime + 1.
    def stop_checks(self):
        if self.check_timer is None:
            return
        self.printer.get_reactor().unregister_timer(self.check_timer)
        self.check_timer = None
    def start_checks(self):
        if self.check_timer is not None:
            self.stop_checks()
        cleared_flags = 0
        self._query_register(self.drv_status_reg_info)
        if self.gstat_reg_info is not None:
            cleared_flags = self._query_register(self.gstat_reg_info,
                                                 try_clear=self.clear_gstat)
        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        self.check_timer = reactor.register_timer(self._do_periodic_check,
                                                  curtime + 1.)
        if cleared_flags:
            reset_mask = self.fields.all_fields["GSTAT"]["reset"]
            if cleared_flags & reset_mask:
                return True
        return False
    def get_status(self, eventtime=None):
        if self.check_timer is None:
            return {'drv_status': None, 'temperature': None}
        temp = None
        if self.adc_temp is not None:
            temp = round((self.adc_temp - 2038) / 7.7, 2)
        last_value, reg_name = self.drv_status_reg_info[:2]
        if last_value != self.last_drv_status:
            self.last_drv_status = last_value
            fields = self.fields.get_reg_fields(reg_name, last_value)
            self.last_drv_fields = {n: v for n, v in fields.items() if v}
        return {'drv_status': self.last_drv_fields, 'temperature': temp}

######################################################################
# Record driver status
######################################################################

class TMCStallguardDump:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.mcu_tmc = mcu_tmc
        self.mcu = self.mcu_tmc.get_mcu()
        self.fields = self.mcu_tmc.get_fields()
        self.sg2_supp = False
        self.sg4_reg_name = None
        # It is possible to support TMC2660, just disable it for now
        if not self.fields.all_fields.get("DRV_STATUS", None):
            return
        # Collect driver capabilities
        if self.fields.all_fields["DRV_STATUS"].get("sg_result", None):
            self.sg2_supp = True
        # New drivers have separate register for SG4 result
        if self.mcu_tmc.name_to_reg.get("SG_RESULT", 0):
            self.sg4_reg_name = "SG_RESULT"
        # 2240 supports both SG2 & SG4
        if self.sg4_reg_name is None:
            if self.mcu_tmc.name_to_reg.get("SG4_RESULT", 0):
                self.sg4_reg_name = "SG4_RESULT"
        # TMC2208
        if self.sg2_supp is None and self.sg4_reg_name is None:
            return
        self.optimized_spi = False
        # Bulk API
        self.samples = []
        self.query_timer = None
        self.error = None
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._dump, self._start, self._stop)
        api_resp = {'header': ('time', 'sg_result', 'cs_actual')}
        self.batch_bulk.add_mux_endpoint("tmc/stallguard_dump", "name",
                                         self.stepper_name, api_resp)
    def _start(self):
        self.error = None
        status = self.mcu_tmc.get_register_raw("DRV_STATUS")
        if status.get("spi_status"):
            self.optimized_spi = True
        reactor = self.printer.get_reactor()
        self.query_timer = reactor.register_timer(self._query_tmc,
                                                  reactor.NOW)
    def _stop(self):
        self.printer.get_reactor().unregister_timer(self.query_timer)
        self.query_timer = None
        self.samples = []
    def _query_tmc(self, eventtime):
        sg_result = -1
        cs_actual = -1
        recv_time = eventtime
        try:
            if self.optimized_spi or self.sg4_reg_name == "SG4_RESULT":
                #TMC2130/TMC5160/TMC2240
                status = self.mcu_tmc.get_register_raw("DRV_STATUS")
                reg_val = status["data"]
                cs_actual = self.fields.get_field("cs_actual", reg_val)
                sg_result = self.fields.get_field("sg_result", reg_val)
                is_stealth = self.fields.get_field("stealth", reg_val)
                recv_time = status["#receive_time"]
                if is_stealth and self.sg4_reg_name == "SG4_RESULT":
                    sg4_ret = self.mcu_tmc.get_register_raw("SG4_RESULT")
                    sg_result = sg4_ret["data"]
                    recv_time = sg4_ret["#receive_time"]
            else:
                # TMC2209
                if self.sg4_reg_name == "SG_RESULT":
                    sg4_ret = self.mcu_tmc.get_register_raw("SG_RESULT")
                    sg_result = sg4_ret["data"]
                    recv_time = sg4_ret["#receive_time"]
        except self.printer.command_error as e:
            self.error = e
            return self.printer.get_reactor().NEVER
        print_time = self.mcu.estimated_print_time(recv_time)
        self.samples.append((print_time, sg_result, cs_actual))
        if self.optimized_spi:
            return eventtime + 0.001
        # UART queried as fast as possible
        return eventtime + 0.005
    def _dump(self, eventtime):
            if self.error:
                raise self.error
            samples = self.samples
            self.samples = []
            return {"data": samples}


######################################################################
# G-Code command helpers
######################################################################

class TMCCommandHelper:
    def __init__(self, config, mcu_tmc, current_helper):
        self.printer = config.get_printer()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.current_helper = current_helper
        # Handle current settings
        _, hold_support, max_cur = self.current_helper.get_current()
        self.run_current = config.getfloat('run_current',
                                      above=0., maxval=max_cur)
        self.hold_current = None
        run_cur = self.run_current
        if hold_support:
            hold = config.getfloat('hold_current', run_cur,
                                   above=0., maxval=run_cur)
            self.hold_current = hold
        self.ready_current = config.getfloat('ready_current', run_cur,
                                             above=0., maxval=run_cur)
        self._restore_current = [self.run_current, self.hold_current]
        self._in_ready = False
        self.printer.register_event_handler("idle_timeout:ready",
                                            self._handle_ready)
        # Misc
        self.echeck_helper = TMCErrorCheck(config, mcu_tmc)
        self.record_helper = TMCStallguardDump(config, mcu_tmc)
        self.fields = mcu_tmc.get_fields()
        self.read_registers = self.read_translate = None
        self.toff = None
        self.mcu_phase_offset = None
        self.stepper = None
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.printer.register_event_handler("stepper:sync_mcu_position",
                                            self._handle_sync_mcu_pos)
        self.printer.register_event_handler("stepper:set_sdir_inverted",
                                            self._handle_sync_mcu_pos)
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        # Set microstep config options
        TMCMicrostepHelper(config, mcu_tmc)
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_TMC_FIELD", "STEPPER", self.name,
                                   self.cmd_SET_TMC_FIELD,
                                   desc=self.cmd_SET_TMC_FIELD_help)
        gcode.register_mux_command("INIT_TMC", "STEPPER", self.name,
                                   self.cmd_INIT_TMC,
                                   desc=self.cmd_INIT_TMC_help)
        gcode.register_mux_command("SET_TMC_CURRENT", "STEPPER", self.name,
                                   self.cmd_SET_TMC_CURRENT,
                                   desc=self.cmd_SET_TMC_CURRENT_help)
    def _init_registers(self, print_time=None):
        # Send registers
        for reg_name in list(self.fields.registers.keys()):
            val = self.fields.registers[reg_name] # Val may change during loop
            self.mcu_tmc.set_register(reg_name, val, print_time)
    cmd_INIT_TMC_help = "Initialize TMC stepper driver registers"
    def cmd_INIT_TMC(self, gcmd):
        logging.info("INIT_TMC %s", self.name)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self._init_registers(print_time)
    cmd_SET_TMC_FIELD_help = "Set a register field of a TMC driver"
    def cmd_SET_TMC_FIELD(self, gcmd):
        field_name = gcmd.get('FIELD').lower()
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise gcmd.error("Unknown field name '%s'" % (field_name,))
        value = gcmd.get_int('VALUE', None)
        velocity = gcmd.get_float('VELOCITY', None, minval=0.)
        if (value is None) == (velocity is None):
            raise gcmd.error("Specify either VALUE or VELOCITY")
        if velocity is not None:
            if self.mcu_tmc.get_tmc_frequency() is None:
                raise gcmd.error(
                    "VELOCITY parameter not supported by this driver")
            value = TMCtstepHelper(self.mcu_tmc, velocity,
                                   pstepper=self.stepper)
        reg_val = self.fields.set_field(field_name, value)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self.mcu_tmc.set_register(reg_name, reg_val, print_time)
    cmd_SET_TMC_CURRENT_help = "Set the current of a TMC driver"
    def cmd_SET_TMC_CURRENT(self, gcmd):
        ch = self.current_helper
        prev_cur, prev_hold_cur, max_cur = ch.get_current()
        run_current = gcmd.get_float('CURRENT', None,
                                     minval=0., maxval=max_cur)
        hold_current = None
        if prev_hold_cur is not None:
            hold_current = gcmd.get_float('HOLDCURRENT', None,
                                          above=0., maxval=max_cur)
        ready_current = gcmd.get_float('READYCURRENT', None,
                                       above=0., maxval=max_cur)
        if run_current is not None or hold_current is not None:
            if run_current is None:
                run_current = prev_cur
            if hold_current is None:
                hold_current = prev_hold_cur
            toolhead = self.printer.lookup_object('toolhead')
            print_time = toolhead.get_last_move_time()
            ch.set_current(run_current, hold_current, print_time)
            prev_cur, prev_hold_cur, max_cur = ch.get_current()
        if ready_current is not None:
            self.ready_current = ready_current
        # Don't silently miss user input to change ihold upon restore
        if self._in_ready and hold_current is not None:
            self._restore_current[1] = hold_current
        # Report values
        if prev_hold_cur is None:
            gcmd.respond_info("Run Current: %0.2fA "
                              "Ready Current: %0.2fA"
                              % (prev_cur, self.ready_current))
        else:
            gcmd.respond_info("Run Current: %0.2fA "
                              "Hold Current: %0.2fA "
                              "Ready Current: %0.2fA"
                              % (prev_cur, prev_hold_cur, self.ready_current))
    # Handle printer ready state current reduction
    def _handle_ready(self, print_time):
        if self.ready_current >= self.run_current:
            return
        ch = self.current_helper
        run, hold, max_cur = ch.get_current()
        self._restore_current = [run, hold]
        def callback(eventtime):
            if hold is not None:
                ch.set_current(run, self.ready_current, print_time)
            else:
                #TMC2660
                ch.set_current(self.ready_current, None, print_time)
        self.printer.get_reactor().register_callback(callback)
        self._in_ready = True
        force_move = self.printer.lookup_object("force_move")
        stepper = force_move.lookup_stepper(self.stepper_name)
        stepper.add_active_callback(self._restore)
    def _restore(self, flush_time):
        ch = self.current_helper
        run, hold = self._restore_current
        def callback(eventtime):
            ch.set_current(run, hold, flush_time)
        self.printer.get_reactor().register_callback(callback)
        self._in_ready = False
    # Stepper phase tracking
    def _get_phases(self):
        return (256 >> self.fields.get_field("mres")) * 4
    def get_phase_offset(self):
        return self.mcu_phase_offset, self._get_phases()
    def _query_phase(self):
        field_name = "mscnt"
        if self.fields.lookup_register(field_name, None) is None:
            # TMC2660 uses MSTEP
            field_name = "mstep"
        reg = self.mcu_tmc.get_register(self.fields.lookup_register(field_name))
        return self.fields.get_field(field_name, reg)
    def _handle_sync_mcu_pos(self, stepper):
        if stepper.get_name() != self.stepper_name:
            return
        try:
            driver_phase = self._query_phase()
        except self.printer.command_error as e:
            logging.info("Unable to obtain tmc %s phase", self.stepper_name)
            self.mcu_phase_offset = None
            enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
            if enable_line.is_motor_enabled():
                raise
            return
        if not stepper.get_dir_inverted()[0]:
            driver_phase = 1023 - driver_phase
        phases = self._get_phases()
        phase = int(float(driver_phase) / 1024 * phases + .5) % phases
        moff = (phase - stepper.get_mcu_position()) % phases
        if self.mcu_phase_offset is not None and self.mcu_phase_offset != moff:
            logging.warning("Stepper %s phase change (was %d now %d)",
                            self.stepper_name, self.mcu_phase_offset, moff)
        self.mcu_phase_offset = moff
    # Stepper enable/disable tracking
    def _do_enable(self, print_time):
        try:
            if self.toff is not None:
                # Shared enable via comms handling
                self.fields.set_field("toff", self.toff)
            self._init_registers()
            did_reset = self.echeck_helper.start_checks()
            if did_reset:
                self.mcu_phase_offset = None
            # Calculate phase offset
            if self.mcu_phase_offset is not None:
                return
            gcode = self.printer.lookup_object("gcode")
            with gcode.get_mutex():
                if self.mcu_phase_offset is not None:
                    return
                logging.info("Pausing toolhead to calculate %s phase offset",
                             self.stepper_name)
                self.printer.lookup_object('toolhead').wait_moves()
                self._handle_sync_mcu_pos(self.stepper)
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
    def _do_disable(self, print_time):
        try:
            if self.toff is not None:
                val = self.fields.set_field("toff", 0)
                reg_name = self.fields.lookup_register("toff")
                self.mcu_tmc.set_register(reg_name, val, print_time)
            self.echeck_helper.stop_checks()
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
    def _handle_mcu_identify(self):
        # Lookup stepper object
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        # Note pulse duration and step_both_edge optimizations available
        self.stepper.setup_default_pulse_duration(.000000100, True)
    def _handle_stepper_enable(self, print_time, is_enable):
        if is_enable:
            cb = (lambda ev: self._do_enable(print_time))
        else:
            cb = (lambda ev: self._do_disable(print_time))
        self.printer.get_reactor().register_callback(cb)
    def _handle_connect(self):
        # Check if using step on both edges optimization
        pulse_duration, step_both_edge = self.stepper.get_pulse_duration()
        if step_both_edge:
            self.fields.set_field("dedge", 1)
        # Check for soft stepper enable/disable
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        enable_line.register_state_callback(self._handle_stepper_enable)
        if not enable_line.has_dedicated_enable():
            self.toff = self.fields.get_field("toff")
            self.fields.set_field("toff", 0)
            logging.info("Enabling TMC virtual enable for '%s'",
                         self.stepper_name)
        # Send init
        try:
            ch = self.current_helper
            ch.set_current(self.run_current, self.hold_current, None)
            self._init_registers()
        except self.printer.command_error as e:
            logging.info("TMC %s failed to init: %s", self.name, str(e))
    # get_status information export
    def get_status(self, eventtime=None):
        cpos = None
        if self.stepper is not None and self.mcu_phase_offset is not None:
            cpos = self.stepper.mcu_to_commanded_position(self.mcu_phase_offset)
        current = self.current_helper.get_current()
        res = {'mcu_phase_offset': self.mcu_phase_offset,
               'phase_offset_position': cpos,
               'run_current': current[0],
               'hold_current': current[1],
               'ready_current': self.ready_current}
        res.update(self.echeck_helper.get_status(eventtime))
        return res
    # DUMP_TMC support
    def setup_register_dump(self, read_registers, read_translate=None):
        self.read_registers = read_registers
        self.read_translate = read_translate
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("DUMP_TMC", "STEPPER", self.name,
                                   self.cmd_DUMP_TMC,
                                   desc=self.cmd_DUMP_TMC_help)
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, gcmd):
        logging.info("DUMP_TMC %s", self.name)
        reg_name = gcmd.get('REGISTER', None)
        if reg_name is not None:
            reg_name = reg_name.upper()
            val = self.fields.registers.get(reg_name)
            if (val is not None) and (reg_name not in self.read_registers):
                # write-only register
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            elif reg_name in self.read_registers:
                # readable register
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            else:
                raise gcmd.error("Unknown register name '%s'" % (reg_name))
        else:
            gcmd.respond_info("========== Write-only registers ==========")
            for reg_name, val in self.fields.registers.items():
                if reg_name not in self.read_registers:
                    gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            gcmd.respond_info("========== Queried registers ==========")
            for reg_name in self.read_registers:
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))

######################################################################
# TMC passive braking
######################################################################

class TMCPassiveBrake:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        name_parts = config.get_name().split()
        self.stepper_name = ' '.join(name_parts[1:])
        self.state = {}
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("TMC_PASSIVE_BRAKE", "STEPPER",
                                   self.stepper_name,
                                   self.cmd_TMC_PASSIVE_BRAKE,
                                   desc=self.cmd_TMC_PASSIVE_BRAKE_help)
    cmd_TMC_PASSIVE_BRAKE_help = "Enable the passive braking"
    def cmd_TMC_PASSIVE_BRAKE(self, gcmd):
        force_move = self.printer.lookup_object("force_move")
        stepper = force_move.lookup_stepper(self.stepper_name)
        stepper.add_active_callback(self.restore)
        state = {}
        reg = self.fields.lookup_register("en_pwm_mode", None)
        if reg is not None:
            en_pwm = self.fields.get_field("en_pwm_mode")
            if en_pwm == 0:
                state["en_pwm_mode"] = 0
                val = self.fields.set_field("en_pwm_mode", 1)
                self.mcu_tmc.set_register("GCONF", val)
        if reg is None:
            en_spread = not self.fields.get_field("en_spreadcycle")
            if en_spread == 1:
                val = self.fields.set_field("en_spreadcycle", 0)
                state["en_spreadcycle"] = 1
                self.mcu_tmc.set_register("GCONF", val)
        pwmthrs = self.fields.get_field("tpwmthrs")
        if pwmthrs == 0xfffff:
            state["tpwmthrs"] = 0xfffff
            val = self.fields.set_field("tpwmthrs", 0xffff0)
            self.mcu_tmc.set_register("TPWMTHRS", val)
        ihold = self.fields.get_field("ihold")
        if ihold:
            state["ihold"] = ihold
            val = self.fields.set_field("ihold", 0)
            self.mcu_tmc.set_register("IHOLD_IRUN", val)
        self.state = state
    def restore(self, flush_time):
        state = self.state
        self.state = {}
        for field in state:
            reg = self.fields.lookup_register(field)
            v = self.fields.set_field(field, state[field])
            self.mcu_tmc.set_register(reg, v)

######################################################################
# TMC virtual pins
######################################################################

# Helper class for "sensorless homing"
class TMCVirtualPinHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        if self.fields.lookup_register('diag0_stall') is not None:
            if config.get('diag0_pin', None) is not None:
                self.diag_pin = config.get('diag0_pin')
                self.diag_pin_field = 'diag0_stall'
            else:
                self.diag_pin = config.get('diag1_pin', None)
                self.diag_pin_field = 'diag1_stall'
        else:
            self.diag_pin = config.get('diag_pin', None)
            self.diag_pin_field = None
        self.mcu_endstop = None
        self.en_pwm = False
        self.pwmthrs = self.coolthrs = self.thigh = 0
        # Register virtual_endstop pin
        name_parts = config.get_name().split()
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)
    def setup_pin(self, pin_type, pin_params):
        # Validate pin
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise ppins.error("tmc virtual endstop only useful as endstop")
        if pin_params['invert'] or pin_params['pullup']:
            raise ppins.error("Can not pullup/invert tmc virtual pin")
        if self.diag_pin is None:
            raise ppins.error("tmc virtual endstop requires diag pin config")
        # Setup for sensorless homing
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self.handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self.handle_homing_move_end)
        self.mcu_endstop = ppins.setup_pin('endstop', self.diag_pin)
        return self.mcu_endstop
    def handle_homing_move_begin(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # Enable/disable stealthchop
        self.pwmthrs = self.fields.get_field("tpwmthrs")
        reg = self.fields.lookup_register("en_pwm_mode", None)
        if reg is None:
            # On "stallguard4" drivers, "stealthchop" must be enabled
            self.en_pwm = not self.fields.get_field("en_spreadcycle")
            tp_val = self.fields.set_field("tpwmthrs", 0)
            self.mcu_tmc.set_register("TPWMTHRS", tp_val)
            val = self.fields.set_field("en_spreadcycle", 0)
        else:
            # On earlier drivers, "stealthchop" must be disabled
            self.en_pwm = self.fields.get_field("en_pwm_mode")
            self.fields.set_field("en_pwm_mode", 0)
            val = self.fields.set_field(self.diag_pin_field, 1)
        self.mcu_tmc.set_register("GCONF", val)
        # Enable tcoolthrs (if not already)
        self.coolthrs = self.fields.get_field("tcoolthrs")
        if self.coolthrs == 0:
            tc_val = self.fields.set_field("tcoolthrs", 0xfffff)
            self.mcu_tmc.set_register("TCOOLTHRS", tc_val)
        # Disable thigh
        reg = self.fields.lookup_register("thigh", None)
        if reg is not None:
            self.thigh = self.fields.get_field("thigh")
            th_val = self.fields.set_field("thigh", 0)
            self.mcu_tmc.set_register(reg, th_val)
    def handle_homing_move_end(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # Restore stealthchop/spreadcycle
        reg = self.fields.lookup_register("en_pwm_mode", None)
        if reg is None:
            tp_val = self.fields.set_field("tpwmthrs", self.pwmthrs)
            self.mcu_tmc.set_register("TPWMTHRS", tp_val)
            val = self.fields.set_field("en_spreadcycle", not self.en_pwm)
        else:
            self.fields.set_field("en_pwm_mode", self.en_pwm)
            val = self.fields.set_field(self.diag_pin_field, 0)
        self.mcu_tmc.set_register("GCONF", val)
        # Restore tcoolthrs
        tc_val = self.fields.set_field("tcoolthrs", self.coolthrs)
        self.mcu_tmc.set_register("TCOOLTHRS", tc_val)
        # Restore thigh
        reg = self.fields.lookup_register("thigh", None)
        if reg is not None:
            th_val = self.fields.set_field("thigh", self.thigh)
            self.mcu_tmc.set_register(reg, th_val)


######################################################################
# Config reading helpers
######################################################################

# Helper to initialize the wave table from config or defaults
def TMCWaveTableHelper(config, mcu_tmc):
    set_config_field = mcu_tmc.get_fields().set_config_field
    set_config_field(config, "mslut0", 0xAAAAB554)
    set_config_field(config, "mslut1", 0x4A9554AA)
    set_config_field(config, "mslut2", 0x24492929)
    set_config_field(config, "mslut3", 0x10104222)
    set_config_field(config, "mslut4", 0xFBFFFFFF)
    set_config_field(config, "mslut5", 0xB5BB777D)
    set_config_field(config, "mslut6", 0x49295556)
    set_config_field(config, "mslut7", 0x00404222)
    set_config_field(config, "w0", 2)
    set_config_field(config, "w1", 1)
    set_config_field(config, "w2", 1)
    set_config_field(config, "w3", 1)
    set_config_field(config, "x1", 128)
    set_config_field(config, "x2", 255)
    set_config_field(config, "x3", 255)
    set_config_field(config, "start_sin", 0)
    set_config_field(config, "start_sin90", 247)

# Helper to configure the microstep settings
def TMCMicrostepHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    stepper_name = " ".join(config.get_name().split()[1:])
    if not config.has_section(stepper_name):
        raise config.error(
            "Could not find config section '[%s]' required by tmc driver"
            % (stepper_name,))
    sconfig = config.getsection(stepper_name)
    steps = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
    mres = sconfig.getchoice('microsteps', steps)
    fields.set_field("mres", mres)
    fields.set_field("intpol", config.getboolean("interpolate", True))

# Helper for calculating TSTEP based values from velocity
def TMCtstepHelper(mcu_tmc, velocity, pstepper=None, config=None):
    if velocity <= 0.:
        return 0xfffff
    if pstepper is not None:
        step_dist = pstepper.get_step_dist()
    else:
        stepper_name = " ".join(config.get_name().split()[1:])
        sconfig = config.getsection(stepper_name)
        rotation_dist, steps_per_rotation = stepper.parse_step_distance(sconfig)
        step_dist = rotation_dist / steps_per_rotation
    mres = mcu_tmc.get_fields().get_field("mres")
    step_dist_256 = step_dist / (1 << mres)
    tmc_freq = mcu_tmc.get_tmc_frequency()
    threshold = int(tmc_freq * step_dist_256 / velocity + .5)
    return max(0, min(0xfffff, threshold))

# Helper to configure stealthChop-spreadCycle transition velocity
class TMCStealthchopHelper:
    def __init__(self, config, mcu_tmc):
        fields = mcu_tmc.get_fields()
        en_pwm_mode = False
        velocity = config.getfloat('stealthchop_threshold', None, minval=0.)
        tpwmthrs = 0xfffff

        if velocity is not None:
            en_pwm_mode = True
            tpwmthrs = TMCtstepHelper(mcu_tmc, velocity, config=config)
        fields.set_field("tpwmthrs", tpwmthrs)

        reg = fields.lookup_register("en_pwm_mode", None)
        if reg is not None:
            fields.set_field("en_pwm_mode", en_pwm_mode)
        else:
            # TMC2208 uses en_spreadCycle
            fields.set_field("en_spreadcycle", not en_pwm_mode)

        self.velocity, self.tpwmthrs = velocity, tpwmthrs
    def get_velocity_threshold(self):
        # Returns a threshold velocity when the stepper is switching
        # to spreadcycle mode. Returns None when no such threshold
        # exists (i.e. the stepper always run in stealthchop mode).
        if self.tpwmthrs == 0xfffff:
            return 0
        if self.tpwmthrs == 0:
            return None
        return self.velocity


# Helper to configure StallGuard and CoolStep minimum velocity
def TMCVcoolthrsHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat('coolstep_threshold', None, minval=0.)
    tcoolthrs = 0
    if velocity is not None:
        tcoolthrs = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("tcoolthrs", tcoolthrs)

# Helper to configure StallGuard and CoolStep maximum velocity and
# SpreadCycle-FullStepping (High velocity) mode threshold.
def TMCVhighHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat('high_velocity_threshold', None, minval=0.)
    thigh = 0
    if velocity is not None:
        thigh = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("thigh", thigh)
