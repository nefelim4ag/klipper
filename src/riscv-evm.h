#ifndef RISCV_EVM
#define RISCV_EVM
#include <stdint.h>

// Runtime
#define EVM_MODE_INT 0
#define EVM_MODE_INT_DEBUG 1
#define EVM_MODE_DISASM 2

void evm_print(const char *fmt, ...);

struct evm_args {
    uint32_t a0, a1, a2, a3, a4;
};

int evm_interpreter(uint8_t *prog_start, uint8_t mode, struct evm_args *args);

// Platform helpers
void
platform_ecall(uint32_t id, uint32_t *a0, uint32_t a1,
               uint32_t a2, uint32_t a3, uint32_t a4);

#endif // RISCV_EVM
