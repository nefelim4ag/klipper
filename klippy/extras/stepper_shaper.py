import chelper
import logging

TRINAMIC_DRIVERS = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660",
    "tmc5160"]

class StepperShaper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.stepper_name = config.get('stepper', None)
        self.period = config.getint('period', 0, minval=0, maxval=1024)
        self.offset = config.getint('offset', self.period // 2,
                                    minval=0, maxval=self.period)
        self.amplitude = config.getfloat('amplitude', 10)
        self.tmc = None
        self.microsteps = 0
        # Register gcode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_STEPPER_SHAPER", "STEPPER", self.stepper_name,
                                   self.cmd_SET_STEPPER_SHAPER,
                                   desc=self.cmd_SET_STEPPER_SHAPER_help)
        gcode.register_mux_command("GET_STEPPER_SHAPER", "STEPPER", self.stepper_name,
                                   self.cmd_GET_STEPPER_SHAPER,
                                   desc=self.cmd_GET_STEPPER_SHAPER_help)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self.handle_homing_move_end)
    def lookup_tmc(self):
        for driver in TRINAMIC_DRIVERS:
            driver_name = "%s %s" % (driver, self.stepper_name)
            module = self.printer.lookup_object(driver_name, None)
            if module is not None:
                return module
        raise self.printer.command_error("Unable to find TMC driver for %s"
                                         % (self.stepper_name,))
    def connect(self):
        tmc_module = self.lookup_tmc()
        self.tmc = tmc_module.mcu_tmc
        configfile = self.printer.lookup_object('configfile')
        sconfig = configfile.get_status(None)['settings']
        stconfig = sconfig.get(self.stepper_name, {})
        self.microsteps = stconfig['microsteps']
        # Configure initial values
        self.stepper_set_shaper_params()

    def get_stepper(self):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        stepper = None
        for s in kin.get_steppers():
            if s.get_trapq() is None:
                continue
            if self.stepper_name == s.get_name():
                stepper = s
                break
        if stepper is None:
            raise self.printer.command_error("Unable to find stepper object for %s"
                                         % (self.stepper_name,))
        return stepper

    def stepper_set_shaper_params(self):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.wait_moves()
        ffi_main, ffi_lib = chelper.get_ffi()
        stepper = self.get_stepper()
        oid = stepper.get_oid()
        mscnt = self.tmc.get_register("MSCNT")
        ffi_lib.stepper_set_shaper_params(oid, mscnt, 256//self.microsteps, self.period, self.offset, self.amplitude)

    def handle_homing_move_end(self, hmoves):
        logging.info("Pausing toolhead to reset %s offset",
                        self.stepper_name)
        self.printer.lookup_object('toolhead').wait_moves()
        self.stepper_set_shaper_params()

    cmd_SET_STEPPER_SHAPER_help = "Set stepper shaper params"
    def cmd_SET_STEPPER_SHAPER(self, gcmd):
        self.amplitude = gcmd.get_int('AMP',
                                      self.amplitude,
                                      minval=1, maxval=100)
        period = gcmd.get_int('PERIOD',
                              self.period,
                              minval=0, maxval=1024)
        if period not in (0, 256, 512, 1024):
            gcmd.respond_error("Period must be 0, 256, 512, 1024")
            return
        self.offset = gcmd.get_int('OFFSET',
                                   period//2,
                                   minval=0, maxval=period)
        self.period = period
        self.stepper_set_shaper_params()
    cmd_GET_STEPPER_SHAPER_help = "Set stepper shaper params"
    def cmd_GET_STEPPER_SHAPER(self, gcmd):
        ffi_main, ffi_lib = chelper.get_ffi()
        stepper = self.get_stepper()
        oid = stepper.get_oid()
        mscnt = ffi_lib.stepper_get_shaper_mscnt(oid)
        gcmd.respond_info("MSCNT: %i" % (mscnt))

def load_config_prefix(config):
    return StepperShaper(config)
