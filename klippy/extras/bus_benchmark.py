# Klipper bus benchmark
#
# Copyright (C) 2024  Timofey Titovets <nefelim4ag@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import mcu
import math
import logging

class BusBenchmark:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mcu_name = config.get('mcu', 'mcu')
        self.mcu = mcu.get_printer_mcu(self.printer, self.mcu_name)
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.reactor = self.printer.get_reactor()

        # Register commands
        self.name = config.get_name().split()[1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("BUS_LATENCY", "MCU", self.mcu_name,
                                   self.cmd_BUS_LATENCY,
                                   desc=self.cmd_BUS_LATENCY_help)
        gcode.register_mux_command("BUS_LATENCY_NOP", "MCU", self.mcu_name,
                                   self.cmd_BUS_LATENCY_NOP,
                                   desc=self.cmd_BUS_LATENCY_NOP_help)
        gcode.register_mux_command("BUS_BANDWIDTH", "MCU", self.mcu_name,
                                   self.cmd_BUS_BANDWIDTH,
                                   desc=self.cmd_BUS_BANDWIDTH_help)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
    def handle_connect(self):
        self.ping = self.mcu.lookup_query_command(
                    "debug_ping data=%*s",
                    "pong data=%*s",
                    cq=self.cmd_queue)
        self.nop = self.mcu.lookup_command(
                    "debug_nop",cq=self.cmd_queue)
    cmd_BUS_LATENCY_help = "Test e2e command dispatch latency"
    def cmd_BUS_LATENCY(self, gcmd):
        count = gcmd.get_int('COUNT', 10)
        times_reactor = []
        times_serial = []

        for i in range(1, count + 1):
            start = self.reactor.monotonic()
            res = self.ping.send([[]])
            sent_time = res["#sent_time"]
            recv_time = res["#receive_time"]
            stop = self.reactor.monotonic()
            diff = stop - start
            times_reactor.append(diff)
            diff_serial = recv_time - sent_time
            times_serial.append(diff_serial)
        mean_reactor = sum(times_reactor) / len(times_reactor)
        d = [ (i - mean_reactor) ** 2 for i in times_reactor]
        stddev = math.sqrt(sum(d) / len(d))
        gcmd.respond_info("PING: Reactor latency mean=%.9fs stddev=%.9fs" % (
                            mean_reactor, stddev))
        mean_serial = sum(times_serial) / len(times_serial)
        d = [ (i - mean_serial) ** 2 for i in times_serial]
        stddev = math.sqrt(sum(d) / len(d))
        gcmd.respond_info("PING: Serial latency mean=%.9fs stddev=%.9fs" % (
                            mean_serial, stddev))
    cmd_BUS_LATENCY_NOP_help = "Test e2e ack latency"
    def cmd_BUS_LATENCY_NOP(self, gcmd):
        count = gcmd.get_int('COUNT', 10)
        times_reactor = []
        times_serial = []

        for i in range(1, count + 1):
            start = self.reactor.monotonic()
            res = self.nop.send_wait_ack()
            sent_time = res["#sent_time"]
            recv_time = res["#receive_time"]
            stop = self.reactor.monotonic()
            diff = stop - start
            times_reactor.append(diff)
            diff_serial = recv_time - sent_time
            times_serial.append(diff_serial)
        mean_reactor = sum(times_reactor) / len(times_reactor)
        d = [ (i - mean_reactor) ** 2 for i in times_reactor]
        stddev = math.sqrt(sum(d) / len(d))
        gcmd.respond_info("NOP: Reactor latency: mean=%.9fs stddev=%.9fs" % (
                            mean_reactor, stddev))
        mean_serial = sum(times_serial) / len(times_serial)
        d = [ (i - mean_serial) ** 2 for i in times_serial]
        stddev = math.sqrt(sum(d) / len(d))
        gcmd.respond_info("NOP: Serial latency: mean=%.9fs stddev=%.9fs" % (
                            mean_serial, stddev))

    cmd_BUS_BANDWIDTH_help = "Test e2e bus bandwidht"
    def cmd_BUS_BANDWIDTH(self, gcmd):
        count = gcmd.get_int('COUNT', 10)
        size = 48
        times = []
        data = []
        for i in range(1, size + 1):
            data.append(i % 16)

        for i in range(1, count + 1):
            start = self.reactor.monotonic()
            self.ping.send([data])
            stop = self.reactor.monotonic()
            data.append(data.pop(0))
            diff = stop - start
            times.append(diff)
        mean = sum(times) / len(times)
        d = [ (i - mean) ** 2 for i in times]
        stddev = math.sqrt(sum(d) / len(d))
        gcmd.respond_info("Latency: mean=%.9fs stddev=%.9fs" % (mean, stddev))
        # write == read size
        bps = len(data) * 8 * count * 2 / sum(times)
        gcmd.respond_info("Approximate bandwidth: %.2f bps" % (bps))
        gcmd.respond_info("Transfered: %d * %d * 2 = %.2f bytes" %
                          (len(data), count, len(data) * count * 2))

def load_config_prefix(config):
    return BusBenchmark(config)
