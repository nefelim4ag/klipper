# Wrapper around embedded RISCV VM
#
# Copyright (C) 2026 Timofey Titovets <nefelim4ag@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os
import mcu
import logging
import subprocess

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

class EmbeddedVM:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.mcu_name = config.get('mcu', 'mcu')
        self.mcu = mcu.get_printer_mcu(self.printer, self.mcu_name)
        self.cmd_queue = self.mcu.alloc_command_queue()
        # Compile something
        self.CC      = "riscv64-linux-gnu-gcc"
        self.OBJCOPY = "riscv64-linux-gnu-objcopy"
        self.CFLAGS  = "-O2 -march=rv32e -mabi=ilp32e -ffreestanding " \
                       + " -nostdlib -fPIE -Wvla -Werror=vla"
        self._run(["which", self.CC], )
        self._run(["which", self.OBJCOPY])
        dir = os.path.dirname(os.path.abspath(__file__))
        dir = str(dir)
        self._run([self.CC, self.CFLAGS, "-c", "-o", "%s/code.o" % (dir),
                   "%s/code.c" % (dir)])
        self._run([self.CC, self.CFLAGS,
                   "-T %s/rv32e.ld -static -Wl,--build-id=none -o" % dir,
                   "%s/code.elf" % dir, "%s/code.o" % dir])
        self._run([self.OBJCOPY, "-O binary", "%s/code.elf" % dir,
                   "%s/code.bin" % dir])

        # Configure MCU objects
        self._oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self._build_config)
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
    def _run(self, cmd):
        logging.info(cmd)
        cmd = " ".join(cmd)
        r = subprocess.run(cmd, shell=True)
        if r.returncode != 0:
            raise self.printer.command_error("Command failed (exit %d})" %
                                             r.returncode)

    def read_bytes(self, path):
        with open(path, "rb") as f:
            return f.read()

    def _build_config(self):
        dir = os.path.dirname(os.path.abspath(__file__))
        dir = str(dir)
        byte_code = self.read_bytes("%s/code.bin" % dir)

        self.mcu.add_config_cmd("config_evm oid=%d data_size=%d" % (
                self._oid, len(byte_code)))
        # Lookup commands
        self._evm_update_cmd = self.mcu.lookup_command(
            "evm_update oid=%c pos=%hu data=%*s",
            cq=self.cmd_queue)
        self._send_evm = self.mcu.lookup_command(
            "send_evm oid=%c data=%*s", cq=self.cmd_queue)

    def handle_connect(self):
        dir = os.path.dirname(os.path.abspath(__file__))
        dir = str(dir)
        byte_code = self.read_bytes("%s/code.bin" % dir)
        ucmd = self._evm_update_cmd.send
        chunk_size = 48
        for pos in range(0, len(byte_code), chunk_size):
            ucmd([self._oid, pos, byte_code[pos:pos+chunk_size]],
                 reqclock=BACKGROUND_PRIORITY_CLOCK)
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        self._send_evm.send([self._oid, bytes("1234".encode())])