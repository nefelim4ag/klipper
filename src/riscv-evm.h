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

struct evm_mmap {
    uint8_t *mem;  // host address of the shared struct
    uint16_t base; // guest address of mem[0]
    uint16_t size;
};

struct evm_ctx {
    uint8_t *prog_start;
    uint16_t img_size;
    uint8_t *stack;
    struct evm_mmap win;
    uint32_t X[16];
    int fault;
};

// EVM_ENOMEM if the image leaves no room below the stack
int evm_window_init(struct evm_mmap *w, uint8_t *prog_start, uint8_t *prog_end,
                    void *mem, uint16_t size);

// host -> guest: any pointer inside the struct (or the struct itself).
uint16_t evm_h2g(const struct evm_mmap *w, const void *host_ptr);

// guest -> host, for platform_ecall (window or stack). NULL on failure.
uint8_t *evm_g2h(const struct evm_ctx *ctx, uint16_t addr, uint_fast8_t len);

struct evm_args {
    uint32_t a0, a1, a2, a3, a4;
};

int evm_interpreter(uint8_t *prog_start, uint8_t *end, uint16_t offset,
    uint8_t mode, struct evm_args *args, struct evm_mmap *w);

// Platform helpers
void
platform_ecall(struct evm_ctx *ctx, uint16_t id, uint32_t *regs);

#endif // RISCV_EVM
