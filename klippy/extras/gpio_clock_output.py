# Define GPIO as clock output
#
# Copyright (C) 2025  Timofey Titovets <nefelim4ag@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class PrinterClockOutputPin:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object('pins')
        self.mcu_pin = ppins.setup_pin('pwm', config.get('pin'))
        self.mcu = self.mcu_pin.get_mcu()
        mdur = self.mcu_pin.get_mcu().max_nominal_duration()
        min_freq = 1 / mdur
        self.frequency = config.getfloat('frequency', 1000, above=min_freq,
                                         maxval=520000000)
        # Determine start and shutdown values
        self.scale = config.getfloat('scale', 1., above=0.)
        self.value = config.getfloat('pulse_width', 0.5, minval=0.,
                                     maxval=self.scale)
        self.value = self.value / self.scale
        self.mcu.register_config_callback(self._build_config)
    def _build_config(self):
        # Check support
        self.mcu.lookup_command(
            "stm32_timer_output pin=%u cycle_ticks=%u on_ticks=%hu")
        mcu_freq = self.mcu.seconds_to_clock(1.0)
        cycle_ticks = mcu_freq // self.frequency
        value = int(self.value * cycle_ticks)
        pin = self.mcu_pin.get_pin()
        self.mcu.add_config_cmd(
            "stm32_timer_output pin=%s cycle_ticks=%d on_ticks=%d"
            % (pin, cycle_ticks, value))

def load_config_prefix(config):
    return PrinterClockOutputPin(config)
