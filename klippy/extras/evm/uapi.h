#include <stdint.h>

#ifndef RISC_V_EVM
#define RISC_V_EVM
// Macro magic
#define RISC_V_EVM_CALL_N(N) \
    ({ register unsigned int _r  __asm__("a0"); \
       register unsigned int _n  __asm__("a5") = (unsigned int)(N); \
       __asm__ volatile ("ecall" \
           : "=r"(_r)            \
           : "r"(_n)             \
           : "memory");          \
       _r; })

#define RISC_V_EVM_CALL_N1(N, A0) \
    ({ register unsigned int _r  __asm__("a0"); \
       register unsigned int _n  __asm__("a5") = (unsigned int)(N); \
       register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
       __asm__ volatile ("ecall" \
           : "=r"(_r)            \
           : "r"(_n), "r"(_a0)   \
           : "memory");          \
       _r; })

#define RISC_V_EVM_CALL_N2(N, A0, A1) \
    ({ register unsigned int _r  __asm__("a0"); \
       register unsigned int _n  __asm__("a5") = (unsigned int)(N); \
       register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
       register unsigned int _a1 __asm__("a1") = (unsigned int)(A1); \
       __asm__ volatile ("ecall" \
           : "=r"(_r)            \
           : "r"(_n), "r"(_a0), "r"(_a1) \
           : "memory");          \
       _r; })

#define RISC_V_EVM_CALL_N3(N, A0, A1, A2) \
    ({ register unsigned int _r  __asm__("a0"); \
       register unsigned int _n  __asm__("a5") = (unsigned int)(N); \
       register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
       register unsigned int _a1 __asm__("a1") = (unsigned int)(A1); \
       register unsigned int _a2 __asm__("a2") = (unsigned int)(A2); \
       __asm__ volatile ("ecall" \
           : "=r"(_r)            \
           : "r"(_n), "r"(_a0), "r"(_a1), "r"(_a2) \
           : "memory");         \
       _r; })

#define RISC_V_EVM_CALL_N4(N, A0, A1, A2, A3) \
    ({ register unsigned int _r  __asm__("a0"); \
       register unsigned int _n  __asm__("a5") = (unsigned int)(N); \
       register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
       register unsigned int _a1 __asm__("a1") = (unsigned int)(A1); \
       register unsigned int _a2 __asm__("a2") = (unsigned int)(A2); \
       register unsigned int _a3 __asm__("a3") = (unsigned int)(A3); \
       __asm__ volatile ("ecall" \
           : "=r"(_r)            \
           : "r"(_n), "r"(_a0), "r"(_a1), "r"(_a2), "r"(_a3) \
           : "memory");          \
       _r; })

#define RISC_V_EVM_CALL_N5(N, A0, A1, A2, A3, A4) \
    ({ register unsigned int _r  __asm__("a0"); \
       register unsigned int _n  __asm__("a5") = (unsigned int)(N); \
       register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
       register unsigned int _a1 __asm__("a1") = (unsigned int)(A1); \
       register unsigned int _a2 __asm__("a2") = (unsigned int)(A2); \
       register unsigned int _a3 __asm__("a3") = (unsigned int)(A3); \
       register unsigned int _a4 __asm__("a4") = (unsigned int)(A4); \
       __asm__ volatile ("ecall" \
           : "=r"(_r)            \
           : "r"(_n), "r"(_a0), "r"(_a1), "r"(_a2), "r"(_a3), "r"(_a4) \
           : "memory");          \
       _r; })

#ifndef __section
#define __section(NAME) \
__attribute__((section(NAME), used))
#endif

// "syscalls"

#define _print(d) (RISC_V_EVM_CALL_N1(0, d))

static inline void
print(uint32_t d)
{
    _print(d);
}

#define _timer_read_time() (RISC_V_EVM_CALL_N(1))

static inline uint32_t
timer_read_time()
{
    return _timer_read_time();
}

#define _sendf(oid, rlen, rdata) (RISC_V_EVM_CALL_N3(2, oid, rlen, rdata))

static inline void
sendf(uint8_t oid, uint8_t rlen, uint8_t *rdata)
{
    _sendf(oid, rlen, rdata);
}

#endif // RISC_V_EVM
