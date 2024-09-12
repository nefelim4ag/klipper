# Klipper bus benchmark
#
# Copyright (C) 2024  Timofey Titovets <nefelim4ag@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import mcu
import math
import logging

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

class BusBenchmark:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mcu_name = config.get('mcu', 'mcu')
        self.mcu = mcu.get_printer_mcu(self.printer, self.mcu_name)
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.reactor = self.printer.get_reactor()
        self.times = {}
        self.counter = 0

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
        self.ping_async = self.mcu.lookup_command(
                    "debug_ping data=%*s", cq=self.cmd_queue)
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

    def _pong(self, res):
        stop = self.reactor.monotonic()
        data = res["data"]
        if len(data) == 0:
            return
        self.counter += 1
        sent_time = res["#sent_time"]
        recv_time = res["#receive_time"]
        data = data.decode("utf-8")
        start, i = data.split(",")
        start = float(start)
        diff = stop - start
        self.times["reactor"].append(diff)
        self.times["stop"] = stop
        diff_serial = recv_time - sent_time
        self.times["serial"].append(diff_serial)

    cmd_BUS_BANDWIDTH_help = "Test e2e bus bandwidht"
    def cmd_BUS_BANDWIDTH(self, gcmd):
        count = gcmd.get_int('COUNT', 10)
        size = 53
        self.times["reactor"] = []
        self.times["serial"] = []
        self.counter = 0
        self.mcu.register_response(self._pong, "pong")
        _start = self.reactor.monotonic()
        for i in range(1, count + 1):
            start = self.reactor.monotonic()
            data = "%*.9f,%*d" % (size//2, start, size//2, i)
            self.ping_async.send([data.encode("utf-8")],
                                 reqclock=BACKGROUND_PRIORITY_CLOCK)
        self.reactor.pause(self.reactor.monotonic() + 0.1)
        self.nop.send_wait_ack()
        self.mcu.register_response(None, "pong")

        mean = sum(self.times["reactor"]) / len(self.times["reactor"])
        d = [ (i - mean) ** 2 for i in self.times["reactor"]]
        stddev = math.sqrt(sum(d) / len(d))
        gcmd.respond_info("Reactor latency mean=%.6fs stddev=%.6fs" % (
            mean, stddev))

        size += 1 + 3 + 3 # ???
        mean = sum(self.times["serial"]) / len(self.times["serial"])
        d = [ (i - mean) ** 2 for i in self.times["serial"]]
        stddev = math.sqrt(sum(d) / len(d))
        gcmd.respond_info("Serial latency mean=%.6fs stddev=%.6fs" % (
            mean, stddev))

        bps = size * count / (self.times["stop"] - _start)
        gcmd.respond_info("Bandwidth to mcu: ~%.2f Byte/s" % (bps))
        bps = size * self.counter / (self.times["stop"] - _start)
        gcmd.respond_info("Bandwidth from mcu: ~%.2f Byte/s" % (bps))
        gcmd.respond_info("Transfered: %d * (%d + %d) = %.2f bytes" %
                          (size, count, self.counter,
                           size * (count + self.counter)))
        self.counter = 0
        self.times["reactor"] = []
        self.times["serial"] = []

def load_config_prefix(config):
    return BusBenchmark(config)
