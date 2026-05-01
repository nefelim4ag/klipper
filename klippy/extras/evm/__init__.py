# Some code for testing
#
# Copyright (C) 2026 Timofey Titovets <nefelim4ag@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import evm

def load_config_prefix(config):
    return evm.EmbeddedVM(config)
