#ifndef __LINUX_GPIO_H
#define __LINUX_GPIO_H

#include <stdint.h> // uint8_t

struct gpio_out {
    struct gpio_line* line;
};
struct gpio_out gpio_out_setup(uint32_t pin, uint8_t val);
void gpio_out_reset(struct gpio_out g, uint8_t val);
void gpio_out_toggle_noirq(struct gpio_out g);
void gpio_out_toggle(struct gpio_out g);
void gpio_out_write(struct gpio_out g, uint8_t val);

struct gpio_in {
    struct gpio_line* line;
};
struct gpio_in gpio_in_setup(uint32_t pin, int8_t pull_up);
void gpio_in_reset(struct gpio_in g, int8_t pull_up);
uint8_t gpio_in_read(struct gpio_in g);

struct gpio_adc {
    int fd;
};
struct gpio_adc gpio_adc_setup(uint32_t pin);
uint32_t gpio_adc_sample(struct gpio_adc g);
uint16_t gpio_adc_read(struct gpio_adc g);
void gpio_adc_cancel_sample(struct gpio_adc g);

struct spi_config {
    int fd;
    int rate;
};
struct spi_config spi_setup(uint32_t bus, uint8_t mode, uint32_t rate);
void spi_prepare(struct spi_config config);
void spi_transfer(struct spi_config config, uint8_t receive_data
                  , uint8_t len, uint8_t *data);

struct gpio_pwm {
    int duty_fd, enable_fd;
    uint32_t period;
};
struct gpio_pwm gpio_pwm_setup(uint32_t pin, uint32_t cycle_time, uint16_t val);
void gpio_pwm_write(struct gpio_pwm g, uint16_t val);

struct i2c_bus {
    int fd;
};

struct i2c_bus i2c_setup(uint32_t bus, uint32_t rate);
void i2c_write(struct i2c_bus bus, uint8_t addr, uint8_t write_len
               , uint8_t *write);
void i2c_read(struct i2c_bus bus, uint8_t addr, uint8_t reg_len, uint8_t *reg
              , uint8_t read_len, uint8_t *read);

#endif // gpio.h
