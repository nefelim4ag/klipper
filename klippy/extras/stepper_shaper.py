import chelper
import logging

class StealthChopShaper:
    def __init__(self, config):
        self.inductance = config.getfloat('inductance_mh', above=0.) / 1000
        self.resistance = config.getfloat('resistance_ohm', above=0.)
    def configure(self, sk, rt_dist, fsteps_per_rev):
        _, ffi_lib = chelper.get_ffi()
        ffi_lib.stealthchop_shaper(sk, self.inductance, self.resistance
                                   , rt_dist, int(fsteps_per_rev))

SHAPER_TYPE = {
    'stealthchop': StealthChopShaper,
}

class StepperShaper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        shaper_type = config.getchoice('shaper', SHAPER_TYPE)
        self.shaper = shaper_type(config)
        name_parts = config.get_name().split()
        if len(name_parts) != 2:
            raise config.error("Section name '%s' is not valid"
                               % (config.get_name(),))
        self.stepper_name = name_parts[1]
        sconfig = config.getsection(self.stepper_name)
        self.microsteps = sconfig.getint("microsteps", minval=1)
        self.stepper_shaper_kin = None
        self.orig_stepper_kin = None
    def connect(self):
        # Configure initial values
        self._set_stepper_shaping(error=self.printer.config_error)
    def _replace_stepper_kinematics(self, stepper):
        # Lookup stepper kinematics
        sk = stepper.get_stepper_kinematics()
        self.orig_stepper_kin = sk
        ffi_main, ffi_lib = chelper.get_ffi()
        ss_sk = ffi_main.gc(ffi_lib.stepper_shaper_alloc(), ffi_lib.free)
        ffi_lib.stepper_shaper_set_sk(ss_sk, sk)
        self.stepper_shaper_kin = ss_sk
        stepper.set_stepper_kinematics(ss_sk)
    def _set_stepper_shaping(self, error=None):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()
        force_move = self.printer.lookup_object("force_move")
        s = force_move.lookup_stepper(self.stepper_name)
        s.get_stepper_kinematics()
        if s.get_trapq() is None:
            return
        self._replace_stepper_kinematics(s)
        _, ffi_lib = chelper.get_ffi()
        ss_sk = self.stepper_shaper_kin
        old_delay = ffi_lib.stepper_shaper_get_step_generation_window(ss_sk)
        rt_distance, msteps_per_rev = s.get_rotation_distance()
        fsteps_per_rev = msteps_per_rev / self.microsteps
        self.shaper.configure(ss_sk, rt_distance, fsteps_per_rev)
        new_delay = ffi_lib.stepper_shaper_get_step_generation_window(ss_sk)
        if old_delay != new_delay:
            toolhead.note_step_generation_scan_time(new_delay, old_delay)

def load_config_prefix(config):
    return StepperShaper(config)
