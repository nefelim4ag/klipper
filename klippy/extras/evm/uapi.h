#include <stdint.h>

#ifndef RISC_V_EVM
#define RISC_V_EVM

// Syscalls
#define _c_ecall(N) ".word ((" #N ") << 20) | 0x73"

#define RISC_V_EVM_CALL_N1(N, A0) ( \
    { register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
        __asm__ volatile (_c_ecall(N) : "+r"(_a0) : : "memory"); \
      _a0; })

#define RISC_V_EVM_CALL_N0(N) RISC_V_EVM_CALL_N1(N, 0)

#define RISC_V_EVM_CALL_N2(N, A0, A1) ( \
    { register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
      register unsigned int _a1 __asm__("a1") = (unsigned int)(A1); \
        __asm__ volatile (_c_ecall(N) : "+r"(_a0) : "r"(_a1) : "memory"); \
      _a0; })

#define RISC_V_EVM_CALL_N3(N, A0, A1, A2) \
    ({ register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
       register unsigned int _a1 __asm__("a1") = (unsigned int)(A1); \
       register unsigned int _a2 __asm__("a2") = (unsigned int)(A2); \
         __asm__ volatile (_c_ecall(N) : "+r"(_a0) : "r"(_a1), "r"(_a2) \
           : "memory"); \
       _a0; })

#define RISC_V_EVM_CALL_N4(N, A0, A1, A2, A3) \
    ({ register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
       register unsigned int _a1 __asm__("a1") = (unsigned int)(A1); \
       register unsigned int _a2 __asm__("a2") = (unsigned int)(A2); \
       register unsigned int _a3 __asm__("a3") = (unsigned int)(A3); \
         __asm__ volatile (_c_ecall(N) : "+r"(_a0) \
           : "r"(_a1), "r"(_a2), "r"(_a3) \
           : "memory"); \
       _a0; })

#define RISC_V_EVM_CALL_N5(N, A0, A1, A2, A3, A4) \
    ({ register unsigned int _a0 __asm__("a0") = (unsigned int)(A0); \
       register unsigned int _a1 __asm__("a1") = (unsigned int)(A1); \
       register unsigned int _a2 __asm__("a2") = (unsigned int)(A2); \
       register unsigned int _a3 __asm__("a3") = (unsigned int)(A3); \
       register unsigned int _a4 __asm__("a4") = (unsigned int)(A4); \
       __asm__ volatile (_c_ecall(N) : "+r"(_a0) \
           : "r"(_a1), "r"(_a2), "r"(_a3), "r"(_a4) \
           : "memory"); \
       _a0; })

#ifndef __section
#define __section(NAME) \
__attribute__((section(NAME), used))
#endif

#endif // RISC_V_EVM
