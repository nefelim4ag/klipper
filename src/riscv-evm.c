// Portable RISC-V eVM interpreter
//
// Copyright (C) 2026  Timofey Titovets <nefelim4ag@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include "riscv-evm.h"

int evm_interpreter(uint8_t *prog_start, uint8_t mode, struct evm_args *args) {
    const uint8_t MODE = mode;
    if (MODE == EVM_MODE_DISASM)
        evm_print("Run disassembler\n");
    if (MODE == EVM_MODE_INT_DEBUG)
        evm_print("Run interrupt debug mode\n");
    uint32_t X[16];
    uint8_t stack[256];
    X[0] = 0; // always
    X[2] = (uint32_t) &stack[sizeof(stack)]; // stack pointer
    X[10] = args->a0;
    X[11] = args->a1;
    X[12] = args->a2;
    X[13] = args->a3;
    X[14] = args->a4;
    // Program counter
    uint8_t *PC = prog_start;
    for (;; PC+=4) {
        const uint32_t *ptr = (uint32_t *) PC;
        const uint8_t opcode = *ptr & 0x7f; // 6 bit
        if (MODE)
            evm_print("%4x | ", PC - prog_start);
        if (opcode == 0x03) {
            uint8_t rd = (*ptr >> 7) & 0x1f;
            uint8_t func3 = (*ptr >> 12) & 0x7;
            uint8_t rs1 = (*ptr >> 15) & 0x1f;
            uint32_t offset = (*ptr >> 20) & 0xffff;
            if (offset & (1 << 11))
                offset |= 0xfffff800;
            int8_t    *s8 = (int8_t *)   (X[rs1] + offset);
            int16_t  *s16 = (int16_t *)  (X[rs1] + offset);
            int32_t  *s32 = (int32_t *)  (X[rs1] + offset);
            uint8_t   *u8 = (uint8_t *)  (X[rs1] + offset);
            uint16_t *u16 = (uint16_t *) (X[rs1] + offset);
            switch (func3) {
                case 0: // lb
                    if (MODE)
                        evm_print("lb x[%d] = M[x[%d] + %d] // = %p", rd, rs1, offset, s8);
                    if (MODE == EVM_MODE_DISASM) {
                        evm_print("\n");
                        continue;
                    }
                    X[rd] = *s8;
                    if (MODE)
                        evm_print(" %d\n", X[rd]);
                    break;
                case 1: // lh
                    if (MODE)
                        evm_print("lh x[%d] = M[x[%d] + %d] // = %p", rd, rs1, offset, s16);
                    if (MODE == EVM_MODE_DISASM) {
                        evm_print("\n");
                        continue;
                    }
                    X[rd] = *s16;
                    if (MODE)
                        evm_print(" %d\n", X[rd]);
                    break;
                case 2: // lw
                    if (MODE)
                        evm_print("lw x[%d] = M[x[%d] + %d] // = %p", rd, rs1, offset, s32);
                    if (MODE == EVM_MODE_DISASM) {
                        evm_print("\n");
                        continue;
                    }
                    X[rd] = *s32;
                    if (MODE)
                        evm_print(" %d\n", X[rd]);
                    break;
                case 4: // lbu
                    if (MODE)
                        evm_print("lbu x[%d] = M[x[%d] + %d] // = %p", rd, rs1, offset, u8);
                    if (MODE == EVM_MODE_DISASM) {
                        evm_print("\n");
                        continue;
                    }
                    X[rd] = *u8;
                    if (MODE)
                        evm_print(" %d\n", X[rd]);
                    break;
                case 5: // lhu
                    if (MODE)
                        evm_print("lbu x[%d] = M[x[%d] + %d] // = %p", rd, rs1, offset, u16);
                    if (MODE == EVM_MODE_DISASM) {
                        evm_print("\n");
                        continue;
                    }
                    X[rd] = *u16;
                    if (MODE)
                        evm_print(" %d\n", X[rd]);
                    break;
            }
        } else if (opcode == 0x13) {
            uint8_t rd = (*ptr >> 7) & 0x1f;
            uint8_t func3 = (*ptr >> 12) & 0x7;
            uint8_t rs1 = (*ptr >> 15) & 0x1f;
            uint32_t imm = (*ptr >> 20) & 0xffff;
            if (imm & (1 << 11))
                imm |= 0xfffff800;
            uint8_t shamt = imm & 0x1f;
            int32_t srs1 = X[rs1];
            switch (func3) {
                case 0: // addi
                    if (MODE)
                        evm_print("addi x[%d] = x[%d] (0x%08x) + sext(%d)\n", rd, rs1, X[rs1], (int32_t)imm);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] + imm;
                    break;
                case 1: // slli
                    if (MODE)
                        evm_print("slli x[%d] = x[%d] << %d\n", rd, rs1, shamt);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] << shamt;
                    break;
                case 2: // slti
                    if (MODE)
                        evm_print("slti x[%d] = x[%d] <s sext(%d)\n", rd, rs1, imm);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    int32_t i_imm = imm;
                    X[rd] = srs1 < i_imm;
                    break;
                case 3: // sltiu
                    if (MODE)
                        evm_print("sltiu x[%d] = x[%d] <u sext(%d)\n", rd, rs1, imm);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] < imm;
                    break;
                case 4: // xori
                    if (MODE)
                        evm_print("xori x[%d] = x[%d] ^ sext(%d)\n", rd, rs1, imm);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] ^ imm;
                    break;
                case 5: // srli srai
                    if (((imm >> 30) & 1) == 0) {
                        if (MODE)
                            evm_print("srli x[%d] = x[%d] >> %d\n", rd, rs1, shamt);
                        if (MODE == EVM_MODE_DISASM)
                            continue;
                        X[rd] = X[rs1] >> shamt;
                    } else {
                        if (MODE)
                            evm_print("srai x[%d] = x[%d] >>s %d\n", rd, rs1, shamt);
                        if (MODE == EVM_MODE_DISASM)
                            continue;
                        X[rd] = srs1 >> shamt;
                    }
                    continue;
                case 6: // ori
                    if (MODE)
                        evm_print("ori x[%d] = x[%d] | sext(%d)\n", rd, rs1, imm);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] | imm;
                    break;
                case 7: // andi
                    if (MODE)
                        evm_print("andi x[%d] = x[%d] & sext(%d)\n", rd, rs1, imm);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] & imm;
                    break;
            }
        } else if (opcode == 0x17) {
            uint8_t rd = (*ptr >> 7) & 0x1f;
            int32_t imm = *ptr & 0xffffff000;
            // AUIPC
            if (MODE)
                evm_print("auipc x[%d] = pc + sext(%d)\n", rd, imm);
            if (MODE == EVM_MODE_DISASM)
                continue;
            X[rd] = (uint32_t) (PC + imm);
        } else if (opcode == 0x23) {
            uint8_t func3 = (*ptr >> 12) & 0x7;
            uint8_t rs1 = (*ptr >> 15) & 0x1f;
            uint8_t rs2 = (*ptr >> 20) & 0x1f;
            uint32_t offset = ((*ptr >> 7) & 0x1f) | ((*ptr >> 25) << 5) ;
            if (offset & (1 << 11))
                offset |= 0xfffff000;
            uint8_t *byte = (uint8_t *) (X[rs1] + offset);
            uint16_t *hword = (uint16_t *) (X[rs1] + offset);
            uint32_t *word = (uint32_t *) (X[rs1] + offset);
            switch (func3) {
                case 0: // sh
                    if (MODE)
                        evm_print("sb M[x[%d] + sext(%d)] = x[%d][7:0] // * (u8 *) %p = %d\n",
                            rs1, offset, rs2, byte, X[rs2]);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    *byte = X[rs2];
                    break;
                case 1: // sh
                    if (MODE)
                        evm_print("sh M[x[%d] + sext(%d)] = x[%d][15:0] // * (u16 *) %p = %d\n",
                            rs1, offset, rs2, hword, X[rs2]);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    *hword = X[rs2];
                    break;
                case 2: // sw
                    if (MODE)
                        evm_print("sw M[x[%d] + sext(%d)] = x[%d][31:0] // * (u32 *) %p = %d\n",
                            rs1, offset, rs2, word, X[rs2]);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    *word = X[rs2];
                    break;
            }
        } else if (opcode == 0x33) {
            uint8_t rd = (*ptr >> 7) & 0x1f;
            uint8_t func3 = (*ptr >> 12) & 0x7;
            uint8_t rs1 = (*ptr >> 15) & 0x1f;
            uint32_t rs2 = (*ptr >> 20) & 0x1f;
            uint8_t func7 = (*ptr >> 25);
            uint16_t func = (func7 << 3) | func3;
            int32_t srs1 = X[rs1];
            int32_t srs2 = X[rs2];
            switch (func) {
                case 0x000:
                    // add
                    if (MODE)
                        evm_print("add x[%d] = x[%d] + x[%d]\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] + X[rs2];
                    break;
                case 0x001:
                    // sll
                    if (MODE)
                        evm_print("sll x[%d] = x[%d] << x[%d]\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] << (X[rs2] & 0x1f);
                    break;
                case 0x002:
                    // slt
                    if (MODE)
                        evm_print("slt x[%d] = bool(x[%d] <s x[%d])\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = srs1 < srs2;
                    break;
                case 0x003:
                    // sltu
                    if (MODE)
                        evm_print("sltu x[%d] = bool(x[%d] <u x[%d])\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] < X[rs2];
                    break;
                case 0x004:
                    // xor
                    if (MODE)
                        evm_print("xor x[%d] = x[%d] ^ x[%d]\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] ^ X[rs2];
                    break;
                case 0x005:
                    // slr
                    if (MODE)
                        evm_print("slr x[%d] = x[%d] >> x[%d]\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] >> (X[rs2] & 0x1f);
                    break;
                case 0x006:
                    // or
                    if (MODE)
                        evm_print("or x[%d] = x[%d] | x[%d]\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] | X[rs2];
                    break;
                case 0x007:
                    // and
                    if (MODE)
                        evm_print("and x[%d] = x[%d] & x[%d]\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] & X[rs2];
                    break;
                case 0x105:
                    // sra
                    if (MODE)
                        evm_print("slr x[%d] = x[%d] >>s x[%d]\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = srs1 >> (X[rs2] & 0x1f);
                    break;
                case 0x180:
                    // sub
                    if (MODE)
                        evm_print("sub x[%d] = x[%d] - x[%d]\n", rd, rs1, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    X[rd] = X[rs1] - X[rs2];
                    break;
            }
        } else if (opcode == 0x37) {
            // lui
            uint8_t rd = (*ptr >> 7) & 0x1f;
            int32_t imm = *ptr & 0xffffff000;
            if (MODE)
                evm_print("lui x[%d] = sext(%d)\n", rd, imm);
            if (MODE == EVM_MODE_DISASM)
                continue;
            X[rd] = imm;
        } else if (opcode == 0x63) {
            uint8_t func3 = (*ptr >> 12) & 0x7;
            uint8_t rs1 = (*ptr >> 15) & 0x1f;
            uint8_t rs2 = (*ptr >> 20) & 0x1f;
            uint32_t offset =
                (((*ptr >> 31) & 0x1)  << 12) |  // imm[12]
                (((*ptr >> 25) & 0x3f) << 5)  |  // imm[10:5]
                (((*ptr >>  8) & 0xf)  << 1)  |  // imm[4:1]
                (((*ptr >>  7) & 0x1)  << 11);   // imm[11]
            if (offset & (1 << 11))
                offset |= 0xfffff800;
            uint32_t urs1 = X[rs1];
            uint32_t urs2 = X[rs2];
            int32_t srs1 = X[rs1];
            int32_t srs2 = X[rs2];
            switch (func3) {
                case 0: // beq
                    if (MODE)
                        evm_print("beq if (x[%d] == x[%d]) pc += sext(%d)) // %d == %d\n",
                            rs1, rs2, offset, X[rs1], X[rs2]);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    if (X[rs1] == X[rs2])
                        PC += offset - 4;
                    break;
                case 1: // bne
                    if (MODE)
                        evm_print("bne if (x[%d] != x[%d]) pc += sext(%d)) // %d != %d\n",
                            rs1, rs2, offset, X[rs1], X[rs2]);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    if (X[rs1] != X[rs2])
                        PC += offset - 4;
                    break;
                case 4: // blt
                    if (MODE)
                        evm_print("blt if (x[%d] < x[%d]) pc += sext(%d)) // %d < %d\n",
                            rs1, rs2, offset, srs1, srs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    if (srs1 < srs2)
                        PC += offset - 4;
                    break;
                case 5: // bge
                    if (MODE)
                        evm_print("bge if (x[%d] >= x[%d]) pc += sext(%d)) // %d >= %d\n",
                            rs1, rs2, offset, srs1, srs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    if (srs1 >= srs2)
                        PC += offset - 4;
                    break;
                case 6: // bltu
                    if (MODE)
                        evm_print("bltu if (x[%d] < x[%d]) pc += sext(%d)) // %d < %d\n",
                            rs1, rs2, offset, urs1, urs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    if (urs1 < urs2)
                        PC += offset - 4;
                    break;
                case 7: // bgeu
                    if (MODE)
                        evm_print("bgeu if (x[%d] >= x[%d]) pc += sext(%d)) // %d >= %d\n",
                            rs1, rs2, offset, urs1, urs2);
                    if (MODE == EVM_MODE_DISASM)
                        continue;
                    if (urs1 >= urs2)
                        PC += offset - 4;
                    break;
            }
        } else if (opcode == 0x67) { // jalr
            // uint8_t rd = (*ptr >> 7) & 0x1f;
            // uint8_t func3 = (*ptr >> 12) & 0x7;
            // uint8_t rs1 = (*ptr >> 15) & 0x1f;
            // uint32_t offset = (*ptr >> 20) & 0xffff;
            // if (offset & (1 << 11))
            //     offset |= 0xfffff800;
            if (MODE)
                evm_print("jalr // return \n");
            goto out;
        } else if (opcode == 0x6f) { // JAL
            uint8_t rd = (*ptr >> 7) & 0x1f;
            uint32_t offset =
                (((*ptr >> 31) & 0x1)  << 20) |  // imm[20]
                (((*ptr >> 21) & 0x3ff)<< 1)  |  // imm[10:1]
                (((*ptr >> 20) & 0x1)  << 11) |  // imm[11]
                (((*ptr >> 12) & 0xff) << 12);   // imm[19:12]
            if (offset & (1 << 20))
                offset |= 0xffe00000;
            if (MODE)
                evm_print("jal x[%d] = pc+4; pc += sext(%d)\n", rd, offset);
            if (MODE == EVM_MODE_DISASM)
                continue;
            if (rd) // x0 is readonly, and always zero, it is a special case
                X[rd] = (uint32_t)(PC + 4);
            PC += offset - 4;
        } else if (opcode == 0x73) {
            uint32_t *a0 = &X[10];
            uint32_t a1 = X[11];
            uint32_t a2 = X[12];
            uint32_t a3 = X[13];
            uint32_t a4 = X[14];
            uint32_t a5 = X[15];
            if (MODE)
                evm_print("ecall args a5: 0x%x, a 0:%x 1:%x 2:%x 3:%x 4:%x\n",
                       a5, *a0, a1, a2, a3, a4);
            if (MODE == EVM_MODE_DISASM)
                continue;
            platform_ecall(a5, a0, a1, a2, a3, a4);
        } else {
            if (MODE)
                evm_print("opcode 0x%02x - not implemented\n", opcode);
            return -1;
        }
    }
out:
    args->a0 = X[10];
    args->a1 = X[11];
    args->a2 = X[12];
    args->a3 = X[13];
    args->a4 = X[14];
    return 0;
}
