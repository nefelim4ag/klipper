#include <stdint.h>
#include "uapi-gen.h"

#ifndef CHIP_TYPE
#define CHIP_TYPE 255
#endif

#define MAX31865_RTDMSB_REG 0x01
#define MAX31865_FAULTSTAT_REG 0x07

static uint32_t min_value;           // Min allowed ADC value
static uint32_t max_value;           // Max allowed ADC value
static uint8_t max_invalid, invalid_count;
struct spidev_s *dev;

__section(".command")
void command(uint8_t oid, uint8_t data_len, uint8_t *data)
{
    uint32_t *ptr = (uint32_t *)data;
    min_value = ptr[0];
    data = &data[4];
    max_value = ptr[1];
    data = &data[4];
    max_invalid = ptr[2];
    data = &data[4];
    uint8_t spi_oid = data[0];
    dev = spidev_oid_lookup(spi_oid);
}

__section(".task")
void task(uint8_t oid, struct evm *vm)
{
    update_evm_flags(oid, 0, EVM_PENDING);
    if (CHIP_TYPE == 0) {
        uint8_t msg[4] = { MAX31865_RTDMSB_REG, 0x00, 0x00, 0x00 };
        spidev_transfer(dev, 1, 3, msg);
        // be32 to le32
        uint32_t value = msg[1] << 8 | msg[2];
        value &= 0xffff;
        // Read faults
        msg[0] = MAX31865_FAULTSTAT_REG;
        msg[1] = 0x00;
        spidev_transfer(dev, 1, 2, msg);
        uint8_t fault = (msg[1] & ~0x03) | (value & 0x0001);
        /* check the result and stop if below or above allowed range */
        if (fault || value < min_value || value > max_value) {
            invalid_count++;
            if (invalid_count < max_invalid)
                return;
            // try_shutdown("Thermocouple reader fault");
        }
        invalid_count = 0;
        uint32_t next_wake_time = get_next_wake_time(oid);
        uint8_t response[9] = {};
        uint32_t *ptr = (uint32_t *)response;
        ptr[0] = value;
        ptr[1] = next_wake_time;
        response[8] = fault;
        evm_response(oid, sizeof(response), response);
    }
}
