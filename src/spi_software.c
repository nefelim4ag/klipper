// Software SPI emulation
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"   // CONFIG_*
#include "board/gpio.h" // gpio_out_setup
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_shutdown
#include "spicmds.h" // spidev_set_software_bus

struct spi_software {
    struct gpio_in miso;
    struct gpio_out mosi, sclk;
    uint16_t sck_ticks;
    uint8_t mode;
};

void
command_spi_set_software_bus(uint32_t *args)
{
    uint8_t mode = args[4];
    // 550Mhz/100Khz = 5500 ticks
    uint32_t pulse_ticks = args[5];
    if (mode > 3)
        shutdown("Invalid spi config");

    struct spidev_s *spi = spidev_oid_lookup(args[0]);
    struct spi_software *ss = alloc_chunk(sizeof(*ss));
    ss->miso = gpio_in_setup(args[1], 1);
    ss->mosi = gpio_out_setup(args[2], 0);
    ss->sclk = gpio_out_setup(args[3], 0);
    ss->mode = mode;
    ss->sck_ticks = pulse_ticks;
    spidev_set_software_bus(spi, ss);
}
DECL_COMMAND(command_spi_set_software_bus,
             "spi_set_sw_bus oid=%c miso_pin=%u mosi_pin=%u sclk_pin=%u"
             " mode=%u pulse_ticks=%u");

void
spi_software_prepare(struct spi_software *ss)
{
    gpio_out_write(ss->sclk, ss->mode & 0x02);
}

static void
spi_delay(uint32_t *time, uint32_t ticks)
{
    *time += ticks;
    while (timer_is_before(timer_read_time(), *time));
}

void
spi_software_transfer(struct spi_software *ss, uint8_t receive_data
                      , uint8_t len, uint8_t *data)
{
    uint32_t t1 = ss->sck_ticks >> 1;
    uint32_t t2 = ss->sck_ticks - t1;
    uint32_t time = timer_read_time() - t1;
    while (len--) {
        uint8_t outbuf = *data;
        uint8_t inbuf = 0;
        for (uint_fast8_t i = 0; i < 8; i++) {
            spi_delay(&time, t1);
            if (ss->mode & 0x01) {
                // MODE 1 & 3
                gpio_out_toggle(ss->sclk);
                gpio_out_write(ss->mosi, outbuf & 0x80);
                time = timer_read_time();
                outbuf <<= 1;
                inbuf <<= 1;
                spi_delay(&time, t2);
                gpio_out_toggle(ss->sclk);
                inbuf |= gpio_in_read(ss->miso);
            } else {
                // MODE 0 & 2
                gpio_out_write(ss->mosi, outbuf & 0x80);
                gpio_out_toggle(ss->sclk);
                time = timer_read_time();
                outbuf <<= 1;
                inbuf <<= 1;
                spi_delay(&time, t2);
                inbuf |= gpio_in_read(ss->miso);
                gpio_out_toggle(ss->sclk);
            }
            time = timer_read_time();
        }

        if (receive_data)
            *data = inbuf;
        data++;
    }
}
