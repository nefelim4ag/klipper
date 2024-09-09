// I2C functions on stm32f0
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/misc.h" // timer_is_before
#include "command.h" // shutdown
#include "gpio.h" // i2c_setup
#include "internal.h" // GPIO
#include "sched.h" // sched_shutdown
#include "i2ccmds.h" // IF_ACTIVE

struct i2c_info {
    I2C_TypeDef *i2c;
    uint8_t scl_pin, sda_pin, function;
};

#if CONFIG_MACH_STM32F0
  DECL_ENUMERATION("i2c_bus", "i2c1_PB6_PB7", 0);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
  DECL_ENUMERATION("i2c_bus", "i2c1_PF1_PF0", 1);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PF1_PF0", "PF1,PF0");
  DECL_ENUMERATION("i2c_bus", "i2c1_PB8_PB9", 2);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
  DECL_ENUMERATION("i2c_bus", "i2c1_PB8_PB7", 3);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB8_PB7", "PB8,PB7");
  // Deprecated "i2c1a" style mappings
  DECL_ENUMERATION("i2c_bus", "i2c1", 0);
  DECL_CONSTANT_STR("BUS_PINS_i2c1", "PB6,PB7");
  DECL_ENUMERATION("i2c_bus", "i2c1a", 1);
  DECL_CONSTANT_STR("BUS_PINS_i2c1a", "PF1,PF0");
#elif CONFIG_MACH_STM32F7
  DECL_ENUMERATION("i2c_bus", "i2c1_PB6_PB7", 0);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
#elif CONFIG_MACH_STM32G0
  DECL_ENUMERATION("i2c_bus", "i2c1_PB6_PB7", 0);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
  DECL_ENUMERATION("i2c_bus", "i2c1_PB8_PB9", 1);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
  DECL_ENUMERATION("i2c_bus", "i2c1_PA9_PA10", 2);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PA9_PA10", "PA9,PA10");
  DECL_ENUMERATION("i2c_bus", "i2c2_PB10_PB11", 3);
  DECL_CONSTANT_STR("BUS_PINS_i2c2_PB10_PB11", "PB10,PB11");
  DECL_ENUMERATION("i2c_bus", "i2c2_PB13_PB14", 4);
  DECL_CONSTANT_STR("BUS_PINS_i2c2_PB13_PB14", "PB13,PB14");
  #ifdef I2C3
    DECL_ENUMERATION("i2c_bus", "i2c3_PB3_PB4", 5);
    DECL_CONSTANT_STR("BUS_PINS_i2c3_PB3_PB4", "PB3,PB4");
    DECL_ENUMERATION("i2c_bus", "i2c3_PC0_PC1", 6);
    DECL_CONSTANT_STR("BUS_PINS_i2c3_PC0_PC1", "PC0,PC1");
  #endif
#elif CONFIG_MACH_STM32L4
  DECL_ENUMERATION("i2c_bus", "i2c1_PB6_PB7", 0);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
  DECL_ENUMERATION("i2c_bus", "i2c1_PB8_PB9", 1);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
  DECL_ENUMERATION("i2c_bus", "i2c1_PA9_PA10", 2);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PA9_PA10", "PA9,PA10");
  DECL_ENUMERATION("i2c_bus", "i2c2_PB10_PB11", 3);
  DECL_CONSTANT_STR("BUS_PINS_i2c2_PB10_PB11", "PB10,PB11");
  DECL_ENUMERATION("i2c_bus", "i2c2_PB13_PB14", 4);
  DECL_CONSTANT_STR("BUS_PINS_i2c2_PB13_PB14", "PB13,PB14");
  DECL_ENUMERATION("i2c_bus", "i2c3_PA7_PB4", 5);
  DECL_CONSTANT_STR("BUS_PINS_i2c3_PA7_PB4", "PA7,PB4");
#elif CONFIG_MACH_STM32G4
  DECL_ENUMERATION("i2c_bus", "i2c1_PA13_PA14", 0);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PA13_PA14", "PA13,PA14");
  DECL_ENUMERATION("i2c_bus", "i2c1_PA15_PA14", 1);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PA15_PA14", "PA15,PA14");
  DECL_ENUMERATION("i2c_bus", "i2c1_PB8_PB7", 2);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB8_PB7", "PB8,PB7");
  DECL_ENUMERATION("i2c_bus", "i2c1_PB8_PB9", 3);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
  DECL_ENUMERATION("i2c_bus", "i2c2_PA9_PA8", 4);
  DECL_CONSTANT_STR("BUS_PINS_i2c2_PA9_PA8", "PA9,PA8");
  DECL_ENUMERATION("i2c_bus", "i2c2_PC4_PF0", 5);
  DECL_CONSTANT_STR("BUS_PINS_i2c2_PC4_PF0", "PC4,PF0");
  DECL_ENUMERATION("i2c_bus", "i2c3_PC8_PC9", 6);
  DECL_CONSTANT_STR("BUS_PINS_i2c3_PC8_PC9", "PC8,PC9");
  DECL_ENUMERATION("i2c_bus", "i2c3_PC8_PC11", 7);
  DECL_CONSTANT_STR("BUS_PINS_i2c3_PC8_PC11", "PC8,PC11");
#elif CONFIG_MACH_STM32H7
  DECL_ENUMERATION("i2c_bus", "i2c1_PB6_PB7", 0);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB6_PB7", "PB6,PB7");
  DECL_ENUMERATION("i2c_bus", "i2c1_PB8_PB9", 1);
  DECL_CONSTANT_STR("BUS_PINS_i2c1_PB8_PB9", "PB8,PB9");
  DECL_ENUMERATION("i2c_bus", "i2c2_PB10_PB11", 2);
  DECL_CONSTANT_STR("BUS_PINS_i2c2_PB10_PB11", "PB10,PB11");
  DECL_ENUMERATION("i2c_bus", "i2c3_PA8_PC9", 3);
  DECL_CONSTANT_STR("BUS_PINS_i2c3_PA8_PC9", "PA8,PC9");
#endif

static const struct i2c_info i2c_bus[] = {
#if CONFIG_MACH_STM32F0
    { I2C1, GPIO('B', 6), GPIO('B', 7), GPIO_FUNCTION(1) },
    { I2C1, GPIO('F', 1), GPIO('F', 0), GPIO_FUNCTION(1) },
    { I2C1, GPIO('B', 8), GPIO('B', 9), GPIO_FUNCTION(1) },
    { I2C1, GPIO('B', 8), GPIO('B', 7), GPIO_FUNCTION(1) },
#elif CONFIG_MACH_STM32F7
    { I2C1, GPIO('B', 6), GPIO('B', 7), GPIO_FUNCTION(1) },
#elif CONFIG_MACH_STM32G0
    { I2C1, GPIO('B', 6), GPIO('B', 7), GPIO_FUNCTION(6) },
    { I2C1, GPIO('B', 8), GPIO('B', 9), GPIO_FUNCTION(6) },
    { I2C1, GPIO('A', 9), GPIO('A', 10), GPIO_FUNCTION(6) },
    { I2C2, GPIO('B', 10), GPIO('B', 11), GPIO_FUNCTION(6) },
    { I2C2, GPIO('B', 13), GPIO('B', 14), GPIO_FUNCTION(6) },
  #ifdef I2C3
    { I2C3, GPIO('B', 3), GPIO('B', 4), GPIO_FUNCTION(6) },
    { I2C3, GPIO('C', 0), GPIO('C', 1), GPIO_FUNCTION(6) },
  #endif
#elif CONFIG_MACH_STM32L4
    { I2C1, GPIO('B', 6), GPIO('B', 7), GPIO_FUNCTION(4) },
    { I2C1, GPIO('B', 8), GPIO('B', 9), GPIO_FUNCTION(4) },
    { I2C1, GPIO('A', 9), GPIO('A', 10), GPIO_FUNCTION(4) },
    { I2C2, GPIO('B', 10), GPIO('B', 11), GPIO_FUNCTION(4) },
    { I2C2, GPIO('B', 13), GPIO('B', 14), GPIO_FUNCTION(4) },
    { I2C3, GPIO('A', 7), GPIO('B', 4), GPIO_FUNCTION(4) },
#elif CONFIG_MACH_STM32G4
    { I2C1, GPIO('A', 13), GPIO('A', 14), GPIO_FUNCTION(4) },
    { I2C1, GPIO('A', 15), GPIO('A', 14), GPIO_FUNCTION(4) },
    { I2C1, GPIO('B', 8), GPIO('B', 7), GPIO_FUNCTION(4) },
    { I2C1, GPIO('B', 8), GPIO('B', 9), GPIO_FUNCTION(4) },
    { I2C2, GPIO('A', 9), GPIO('A', 8), GPIO_FUNCTION(4) },
    { I2C2, GPIO('C', 4), GPIO('F', 0), GPIO_FUNCTION(4) },
    { I2C3, GPIO('C', 8), GPIO('C', 9), GPIO_FUNCTION(8) },
    { I2C3, GPIO('C', 8), GPIO('C', 11), GPIO_FUNCTION(8) },
#elif CONFIG_MACH_STM32H7
    { I2C1, GPIO('B', 6), GPIO('B', 7), GPIO_FUNCTION(4) },
    { I2C1, GPIO('B', 8), GPIO('B', 9), GPIO_FUNCTION(4) },
    { I2C2, GPIO('B', 10), GPIO('B', 11), GPIO_FUNCTION(4) },
    { I2C3, GPIO('A', 8), GPIO('C', 9), GPIO_FUNCTION(4) },
#endif
};

struct i2c_config
i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr)
{
    // Lookup requested i2c bus
    if (bus >= ARRAY_SIZE(i2c_bus))
        shutdown("Unsupported i2c bus");
    const struct i2c_info *ii = &i2c_bus[bus];
    I2C_TypeDef *i2c = ii->i2c;

    if (!is_enabled_pclock((uint32_t)i2c)) {
        // Enable i2c clock and gpio
        enable_pclock((uint32_t)i2c);
        gpio_peripheral(ii->scl_pin, ii->function | GPIO_OPEN_DRAIN, 1);
        gpio_peripheral(ii->sda_pin, ii->function | GPIO_OPEN_DRAIN, 1);

        // Set 100Khz frequency and enable
        i2c->TIMINGR = ((0xB << I2C_TIMINGR_PRESC_Pos)
                        | (0x13 << I2C_TIMINGR_SCLL_Pos)
                        | (0xF << I2C_TIMINGR_SCLH_Pos)
                        | (0x2 << I2C_TIMINGR_SDADEL_Pos)
                        | (0x4 << I2C_TIMINGR_SCLDEL_Pos));
        i2c->CR1 = I2C_CR1_PE;
    }

    return (struct i2c_config){ .i2c=i2c, .addr=addr<<1 };
}

static uint32_t
i2c_wait(I2C_TypeDef *i2c, uint32_t set, uint32_t timeout)
{
    for (;;) {
        uint32_t isr = i2c->ISR;
        if (isr & set)
            return isr;
        if (!timer_is_before(timer_read_time(), timeout))
            shutdown("i2c timeout");
    }
}

void
i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write)
{
    I2C_TypeDef *i2c = config.i2c;
    uint32_t timeout = timer_read_time() + timer_from_us(5000);

    // Send start and address
    i2c->CR2 = (I2C_CR2_START | config.addr | (write_len << I2C_CR2_NBYTES_Pos)
                | I2C_CR2_AUTOEND);
    while (write_len--) {
        i2c_wait(i2c, I2C_ISR_TXIS, timeout);
        i2c->TXDR = *write++;
    }
    i2c_wait(i2c, I2C_ISR_TXE, timeout);
}

void
i2c_read(struct i2c_config config, uint8_t reg_len, uint8_t *reg
         , uint8_t read_len, uint8_t *read)
{
    I2C_TypeDef *i2c = config.i2c;
    uint32_t timeout = timer_read_time() + timer_from_us(5000);

    // Send start, address, reg
    i2c->CR2 = (I2C_CR2_START | config.addr |
               (reg_len << I2C_CR2_NBYTES_Pos));
    while (reg_len--) {
        i2c_wait(i2c, I2C_ISR_TXIS, timeout);
        i2c->TXDR = *reg++;
    }
    i2c_wait(i2c, I2C_ISR_TC, timeout);

    // send restart, read data
    i2c->CR2 = (I2C_CR2_START | I2C_CR2_RD_WRN | config.addr |
               (read_len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND);
    while (read_len--) {
        i2c_wait(i2c, I2C_ISR_RXNE, timeout);
        *read++ = i2c->RXDR;
    }
    i2c_wait(i2c, I2C_ISR_STOPF, timeout);
}

static uint_fast8_t i2c_async_read_end(struct timer *timer){
    struct i2cdev_s *i2c_slv = container_of(timer, struct i2cdev_s, timer);
    I2C_TypeDef *i2c_reg = i2c_slv->i2c_config.i2c;
    uint32_t timeout = timer_read_time() + timer_from_us(50);
    i2c_wait(i2c_reg, I2C_ISR_STOPF, timeout);

    timer->func = i2c_slv->callback;
    timer->waketime = timer_read_time() + timer_from_us(50);
    return SF_RESCHEDULE;
}

static uint_fast8_t i2c_async_read(struct timer *timer){
    struct i2cdev_s *i2c_slv = container_of(timer, struct i2cdev_s, timer);
    I2C_TypeDef *i2c_reg = i2c_slv->i2c_config.i2c;
    uint32_t timeout = timer_read_time() + timer_from_us(50);

    i2c_wait(i2c_reg, I2C_ISR_RXNE, timeout);
    i2c_slv->data_len[0]--;
    i2c_slv->buf[i2c_slv->cur] = i2c_reg->RXDR;
    i2c_slv->cur++;

    if (i2c_slv->data_len[0] == 0) {
        i2c_slv->timer.func = i2c_async_read_end;
    }
    timer->waketime = timer_read_time() + timer_from_us(50);
    return SF_RESCHEDULE;
}

static uint_fast8_t i2c_async_read_start(struct timer *timer)
{
    struct i2cdev_s *i2c_slv = container_of(timer, struct i2cdev_s, timer);
    struct i2c_config config = i2c_slv->i2c_config;
    I2C_TypeDef *i2c_reg = config.i2c;
    uint8_t read_len;
    i2c_slv->data_len[0] &= ~(I2C_R);
    read_len = i2c_slv->data_len[0];

    i2c_reg->CR2 = (I2C_CR2_START | I2C_CR2_RD_WRN | config.addr |
                    (read_len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND);

    i2c_slv->cur = i2c_slv->tail;
    timer->func = i2c_async_read;
    timer->waketime = timer_read_time() + timer_from_us(50);
    return SF_RESCHEDULE;
}

static uint_fast8_t i2c_async_write_end(struct timer *timer)
{
    struct i2cdev_s *i2c_slv = container_of(timer, struct i2cdev_s, timer);
    I2C_TypeDef *i2c_reg = i2c_slv->i2c_config.i2c;
    uint32_t timeout = timer_read_time() + timer_from_us(50);

    if (i2c_slv->flags & IF_RW) {
        i2c_wait(i2c_reg, I2C_ISR_TC, timeout);
        return i2c_async_read_start(timer);
    }

    i2c_wait(i2c_reg, I2C_ISR_TXE, timeout);
    timer->func = i2c_async;
    timer->waketime = timer_read_time() + timer_from_us(50);
    return SF_RESCHEDULE;
}

static uint_fast8_t i2c_async_write(struct timer *timer)
{
    struct i2cdev_s *i2c_slv = container_of(timer, struct i2cdev_s, timer);
    I2C_TypeDef *i2c_reg = i2c_slv->i2c_config.i2c;
    uint32_t timeout = timer_read_time() + timer_from_us(50);

    i2c_wait(i2c_reg, I2C_ISR_TXIS, timeout);
    i2c_reg->TXDR = i2c_buf_read_b(i2c_slv);
    i2c_slv->data_len[0]--;
    if (i2c_slv->data_len[0]) {
        timer->waketime = timer_read_time() + timer_from_us(50);
        return SF_RESCHEDULE;
    }

    i2c_cmd_done(i2c_slv);
    timer->func = i2c_async_write_end;
    timer->waketime = timer_read_time() + timer_from_us(50);
    return SF_RESCHEDULE;
}

uint_fast8_t i2c_async(struct timer *timer)
{
    struct i2cdev_s *i2c_slv = container_of(timer, struct i2cdev_s, timer);
    struct i2c_config config = i2c_slv->i2c_config;
    I2C_TypeDef *i2c_reg = config.i2c;

    // write register + read
    if (i2c_slv->data_len[0] || i2c_slv->data_len[1] & I2C_R) {
        if (i2c_slv->data_len[0]) {
            uint8_t to_write = i2c_slv->data_len[0];
            uint32_t autoend = I2C_CR2_AUTOEND;
            if (i2c_slv->data_len[1] & I2C_R)
                autoend = 0;
            i2c_reg->CR2 = (I2C_CR2_START | config.addr |
                            (to_write << I2C_CR2_NBYTES_Pos) | autoend);
            timer->func = i2c_async_write;
            // 100kHz 5 microsend per pulse, 18 pulses for byte + ack
            // But by some strange circumstances in vitro there is 50
            timer->waketime = timer_read_time() + timer_from_us(50);
            return SF_RESCHEDULE;
        }

        // cleanup empty write, start read
        i2c_cmd_done(i2c_slv);
        return i2c_async_read_start(timer);
    }

    i2c_slv->flags &= ~IF_ACTIVE;
    return SF_DONE;
}
