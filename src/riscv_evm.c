#include <stdio.h>
#include <stdarg.h>

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "riscv_evm.h" // DECL_FW_CALL
#include "riscv-evm.h" // evm_interpreter
#include "sensor_bulk.h"

DECL_FW_CALL(uint32_t, timer_read_time);

static struct task_wake evm_wake;

void evm_print(const char *fmt, ...) {
    // va_list ap;
    // va_start(ap, fmt);
    // vprintf(fmt, ap);
    // va_end(ap);
}

enum {
    MODE_RESPONSE,
    MODE_BULK,
};

DECL_ENUMERATION("evm_mode", "mode_command", MODE_RESPONSE);
DECL_ENUMERATION("evm_mode", "mode_bulk", MODE_BULK);

enum {
    EVM_PENDING = 1,
};
DECL_ENUMERATION("evm_flags", "EVM_PENDING", EVM_PENDING);

struct evm {
    struct timer timer;
    uint32_t rest_ticks;
    struct sensor_bulk *sb;
    uint16_t flags;
    uint16_t task_offset;
    uint16_t data_size;
    uint8_t *byte_code;
};

void sched_wake_evm(void) {
    sched_wake_task(&evm_wake);
}
DECL_FW_CALL(void, sched_wake_evm);

static uint_fast8_t
evm_event(struct timer *timer)
{
    struct evm *vm = container_of(timer, struct evm, timer);
    sched_wake_evm();
    vm->flags |= EVM_PENDING;
    vm->timer.waketime += vm->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_evm(uint32_t *args)
{
    uint8_t oid = args[0];
    struct evm *bc = oid_alloc(oid, command_config_evm, sizeof(struct evm));
    bc->timer.func = evm_event;
    bc->data_size = args[1];
    bc->byte_code = alloc_chunk(bc->data_size);
    bc->sb = NULL;
    bc->task_offset = args[2];
    if (args[3] == MODE_BULK) {
        bc->sb = alloc_chunk(sizeof(struct sensor_bulk));
    }
}
DECL_COMMAND(command_config_evm, "config_evm oid=%c data_size=%hu "
    "task_offset=%hu evm_mode=%hu");

struct evm *
evm_oid_lookup(uint8_t oid)
{
    return oid_lookup(oid, command_config_evm);
}
DECL_FW_CALL1(struct evm *, evm_oid_lookup, uint8_t oid);

void
command_update_evm(uint32_t *args)
{
    uint8_t oid = args[0];
    struct evm *bc = evm_oid_lookup(oid);
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
command_evm_send(uint32_t *args)
{
    uint8_t oid = args[0];
    struct evm *bc = oid_lookup(oid, command_config_evm);
    uint32_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    struct evm_args vm_args = {
        .a0 = oid,
        .a1 = data_len,
        .a2 = (uint32_t) data,
    };
    uint8_t *end = bc->byte_code + bc->data_size;
    uint8_t *entry = bc->byte_code;
    evm_interpreter(entry, end,
        EVM_MODE_INT_DEBUG, &vm_args);
}
DECL_COMMAND(command_evm_send, "evm_send oid=%c data=%*s");

void
evm_response(uint8_t oid, uint8_t rlen, uint8_t *rdata)
{
    sendf("evm_response oid=%c response=%*s", oid, rlen, rdata);
}
DECL_FW_CALL3(void, evm_response, uint8_t oid, uint8_t rlen, uint8_t *rdata);

uint16_t
update_evm_flags(uint8_t oid, uint16_t set_bits, uint16_t clear_bits)
{
    struct evm *vm = evm_oid_lookup(oid);
    vm->flags |= set_bits;
    vm->flags &= ~clear_bits;
    return vm->flags;
}
DECL_FW_CALL3(uint16_t, update_evm_flags, uint8_t oid, uint16_t set_bits,
              uint16_t clear_bits);

void
evm_reschedule_timer(struct evm *bc)
{
    irq_disable();
    bc->timer.waketime = timer_read_time() + bc->rest_ticks;
    sched_add_timer(&bc->timer);
    irq_enable();
}

uint32_t get_next_wake_time(uint8_t oid) {
    struct evm *vm = evm_oid_lookup(oid);
    return vm->timer.waketime;
}
DECL_FW_CALL1(uint32_t, get_next_wake_time, uint8_t oid);

void
command_evm_query(uint32_t *args)
{
    uint8_t oid = args[0];
    struct evm *bc = oid_lookup(oid, command_config_evm);

    sched_del_timer(&bc->timer);
    bc->flags = 0;
    if (!args[1])
        // End measurements
        return;

    // Start new measurements query
    bc->rest_ticks = args[1];
    if (bc->sb != NULL)
        sensor_bulk_reset(bc->sb);
    evm_reschedule_timer(bc);
}
DECL_COMMAND(command_evm_query, "evm_query oid=%c rest_ticks=%u");

void
evm_query(struct evm *vm, uint8_t oid)
{
    struct evm_args vm_args = {
        .a0 = oid,
        .a1 = (uint32_t) vm,
    };
    uint8_t *entry = vm->byte_code + vm->task_offset;
    uint8_t *end = vm->byte_code + vm->data_size;
    evm_interpreter(entry, end,
        EVM_MODE_INT, &vm_args);
}

void
evm_task(void)
{
    if (!sched_check_wake(&evm_wake))
        return;
    uint8_t oid;
    struct evm *vm;
    foreach_oid(oid, vm, command_config_evm) {
        uint_fast8_t flags = vm->flags;
        if (!(flags & EVM_PENDING))
            continue;
        evm_query(vm, oid);
    }
}
DECL_TASK(evm_task);
