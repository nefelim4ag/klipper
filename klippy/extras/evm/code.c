// Code for eVM code generation testing
//
// Copyright (C) 2026  Timofey Titovets <nefelim4ag@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include "uapi-gen.h"

static uint8_t counter = 0;

__section(".start")
void task(uint32_t *args, uint8_t *data)
{
    uint8_t oid = args[0];
    uint8_t data_len = args[1];
    uint8_t new_data[48];
    for (uint8_t i = 0; i < data_len; i++) {
        new_data[i] = data[i];
    }
    new_data[data_len-1] += counter;
    new_data[data_len] = 0;
    uint32_t time = timer_read_time();
    uint8_t *ptr = (uint8_t *) &time;
    for (uint8_t i = 0; i < sizeof(time); i++) {
        new_data[data_len + 1 + i] = ptr[i];
    }
    counter++;
    evm_response(oid, data_len + 1 + 4, new_data);
}
