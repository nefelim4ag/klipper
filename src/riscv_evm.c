#include <stdio.h>
#include <stdarg.h>

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_WANT_SPI
#include "board/gpio.h" // irq_disable
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "sensor_bulk.h" // sensor_bulk_report
#include "spicmds.h" // spidev_transfer
#include "i2ccmds.h" // i2cdev_s
#include "riscv-evm.h"

static struct task_wake evm_wake;

void evm_print(const char *fmt, ...) {
    // va_list ap;
    // va_start(ap, fmt);
    // vprintf(fmt, ap);
    // va_end(ap);
}

void
platform_ecall(uint32_t id, uint32_t *a0, uint32_t a1,
               uint32_t a2, uint32_t a3, uint32_t a4)
{
    uint32_t ret = *a0;
    switch (id) {
        case 0:
            // printf("%d\n", ret);
            break;
        case 1:
            ret = timer_read_time();
            break;
        case 2:
            uint8_t oid = *a0;
            uint8_t rlen = a1;
            uint8_t *rdata = (uint8_t *) a2;
            sendf("evm_response oid=%c response=%*s",
                    oid, rlen, rdata);
            break;
    }
    *a0 = ret;
}

struct evm {
    struct timer timer;
    uint32_t rest_ticks;
    uint16_t data_size;
    uint16_t flags;
    uint8_t byte_code[];
};

static uint_fast8_t
evm_event(struct timer *timer)
{
    struct evm *vm = container_of(timer, struct evm, timer);
    sched_wake_task(&evm_wake);
    vm->timer.waketime += vm->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_evm(uint32_t *args)
{
    uint32_t byte_code_size = args[1];
    struct evm *bc = oid_alloc(args[0], command_config_evm
                               , sizeof(struct evm) + byte_code_size);
    bc->data_size = args[1];
}
DECL_COMMAND(command_config_evm, "config_evm oid=%c data_size=%hu");

void
command_update_evm(uint32_t *args)
{
    uint8_t oid = args[0];
    struct evm *bc = oid_lookup(oid, command_config_evm);
    uint_fast16_t pos = args[1];
    uint_fast8_t data_len = args[2];
    uint8_t *data = command_decode_ptr(args[3]);
    if (pos & 0x8000 || pos + data_len > bc->data_size)
        shutdown("Invalid evm update command");
    memcpy(&bc->byte_code[pos], data, data_len);
}
DECL_COMMAND(command_update_evm, "evm_update oid=%c pos=%hu data=%*s");

// Execution on receive
void
command_send_evm(uint32_t *args)
{
    uint8_t oid = args[0];
    struct evm *bc = oid_lookup(oid, command_config_evm);
    uint32_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    struct evm_args vm_args = {
        .a0 = (uint32_t) args,
        .a1 = (uint32_t) data,
    };
    evm_interpreter(bc->byte_code, EVM_MODE_INT, &vm_args);
}
DECL_COMMAND(command_send_evm, "send_evm oid=%c data=%*s");

void
evm_task(void)
{
    if (!sched_check_wake(&evm_wake))
        return;
    uint8_t oid;
    struct evm *vm;
    foreach_oid(oid, vm, command_config_evm) {
        // uint_fast8_t flags = ld->flags;
        // if (!(flags & LDC_PENDING))
        //     continue;
        // ldc1612_query(ld, oid);
    }
}
DECL_TASK(evm_task);
