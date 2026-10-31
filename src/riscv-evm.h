#ifndef RISCV_EVM
#define RISCV_EVM
#include <stdint.h>

// Runtime
#define EVM_MODE_INT 0
#define EVM_MODE_INT_DEBUG 1
#define EVM_MODE_DISASM 2

// Errors
#define EVM_MISSING_OPCODE (-1)
#define EMV_MEMORY_FAULT (-2)
#define EVM_ENOMEM (-3)

void evm_print(const char *fmt, ...);

struct evm_ctx;

struct evm_mmap {
    uint8_t *host_base;  // host address of shared thing
    uint16_t guest_base; // guest address of the same memory
    uint16_t size;       // size of memory region
};

#define EVM_MMAP_MAX 8
#define EVM_STACK_PAGE 0
#define EVM_PROG_PAGE 1

struct evm_ctx {
    struct evm_mmap pages[EVM_MMAP_MAX];
    int8_t page_cnt;
    int8_t fault;
    uint32_t X[16];
};

int evm_setup_prog(struct evm_ctx *ctx, uint8_t *start, uint8_t *end);
int evm_map_memory(struct evm_ctx *ctx, uint8_t *ptr, uint16_t size);

// host -> guest: any pointer inside the struct (or the struct itself).
uint16_t evm_h2g(const struct evm_ctx *ctx, const void *host_ptr);

// guest -> host, for platform_ecall (window or stack). NULL on failure.
uint8_t *evm_g2h(const struct evm_ctx *ctx, uint16_t addr, uint_fast8_t len);

int evm_interpreter(struct evm_ctx *ctx, uint16_t entry_offset, uint8_t mode);

// Platform helpers
void
platform_ecall(struct evm_ctx *ctx, uint16_t id, uint32_t *regs);

#endif // RISCV_EVM
