// Portable RISC-V eVM interpreter
//
// Copyright (C) 2026  Timofey Titovets <nefelim4ag@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include "riscv-evm.h"

static uint8_t *
lx(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    uint8_t rd = (inst >> 7) & 0x1f;
    uint8_t func3 = (inst >> 12) & 0x7;
    uint8_t rs1 = (inst >> 15) & 0x1f;
    uint32_t offset = (inst >> 20) & 0xfff;
    if (inst & 0x80000000)
        offset |= 0xfffff000;
    uint8_t *mem = (uint8_t *)(X[rs1] + offset);
    uint8_t width = 1 << (func3 & 0x3); // 1, 2, 4 bytes
    uint8_t is_signed = !(func3 & 0x4);

    if (MODE) {
        static const char *names[] = {"lb","lh","lw","?","lbu","lhu"};
        evm_print("%s x[%d] = M[x[%d] + %d] // = %p", names[func3],
            rd, rs1, offset, mem);
    }
    if (MODE == EVM_MODE_DISASM) {
        goto out;
    }

    uint32_t val = 0;
    for (int i = 0; i < width; i++)
        val |= (uint32_t)mem[i] << (i * 8);

    uint32_t sign_bit = 1u << (width * 8 - 1);
    if (is_signed && val & sign_bit)
        val |= ~(sign_bit - 1);

    X[rd] = val;
out:
    if (MODE)
        evm_print(" %d\n", X[rd]);
    return PC + 4;
}

static uint8_t *
alu(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    uint8_t rd = (inst >> 7) & 0x1f;
    uint8_t func3 = (inst >> 12) & 0x7;
    uint8_t rs1 = (inst >> 15) & 0x1f;
    uint32_t imm = (inst >> 20) & 0xfff;
    if (inst & 0x80000000)
        imm |= 0xfffff000;
    uint8_t shamt = imm & 0x1f;
    int32_t srs1 = X[rs1];
    switch (func3) {
        case 0: // addi
            if (MODE)
                evm_print("addi x[%d] = x[%d] (0x%08x) + sext(%d) //", rd, rs1, X[rs1], (int32_t)imm);
            if (MODE == EVM_MODE_DISASM) {
                evm_print("\n");
                goto out;
            }
            X[rd] = X[rs1] + imm;
            if (MODE)
                evm_print( " 0x%x = 0x%x + 0x%x\n", X[rd], X[rs1], imm);
            break;
        case 1: // slli
            if (MODE)
                evm_print("slli x[%d] = x[%d] << %d\n", rd, rs1, shamt);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] << shamt;
            break;
        case 2: // slti
            if (MODE)
                evm_print("slti x[%d] = x[%d] <s sext(%d)\n", rd, rs1, imm);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            int32_t i_imm = imm;
            X[rd] = srs1 < i_imm;
            break;
        case 3: // sltiu
            if (MODE)
                evm_print("sltiu x[%d] = x[%d] <u sext(%d)\n", rd, rs1, imm);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] < imm;
            break;
        case 4: // xori
            if (MODE)
                evm_print("xori x[%d] = x[%d] ^ sext(%d)\n", rd, rs1, imm);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] ^ imm;
            break;
        case 5: // srli srai
            if ((inst & 0x40000000) == 0) {
                if (MODE)
                    evm_print("srli x[%d] = x[%d] >> %d //", rd, rs1, shamt);
                if (MODE == EVM_MODE_DISASM) {
                    evm_print("\n");
                    goto out;
                }
                X[rd] = X[rs1] >> shamt;
                if (MODE)
                    evm_print("%d >> %d = %d\n", srs1, shamt, X[rd]);
            } else {
                if (MODE)
                    evm_print("srai x[%d] = x[%d] >>s %d //", rd, rs1, shamt);
                if (MODE == EVM_MODE_DISASM) {
                    evm_print("\n");
                    goto out;
                }
                X[rd] = srs1 >> shamt;
                if (MODE)
                    evm_print("%d >> %d = %d\n", srs1, shamt, X[rd]);
            }
            break;
        case 6: // ori
            if (MODE)
                evm_print("ori x[%d] = x[%d] | sext(%d)\n", rd, rs1, imm);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] | imm;
            break;
        case 7: // andi
            if (MODE)
                evm_print("andi x[%d] = x[%d] & sext(%d)\n", rd, rs1, imm);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] & imm;
            break;
    }
out:
    return PC + 4;
}

static uint8_t *
sx(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    uint8_t rd = (inst >> 7) & 0x1f;
    uint8_t func3 = (inst >> 12) & 0x7;
    uint8_t rs1 = (inst >> 15) & 0x1f;
    uint32_t rs2 = (inst >> 20) & 0x1f;
    int32_t srs1 = X[rs1];
    int32_t srs2 = X[rs2];
    switch (func3) {
        case 0:
            if (inst & 0x40000000) {
                // sub
                if (MODE)
                    evm_print("sub x[%d] = x[%d] - x[%d]\n", rd, rs1, rs2);
                if (MODE == EVM_MODE_DISASM)
                    goto out;
                X[rd] = X[rs1] - X[rs2];
                break;
            }
            // add
            if (MODE)
                evm_print("add x[%d] = x[%d] + x[%d]\n", rd, rs1, rs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] + X[rs2];
            break;
        case 1:
            // sll
            if (MODE)
                evm_print("sll x[%d] = x[%d] << x[%d]\n", rd, rs1, rs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] << (X[rs2] & 0x1f);
            break;
        case 2:
            // slt
            if (MODE)
                evm_print("slt x[%d] = bool(x[%d] <s x[%d])\n", rd, rs1, rs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = srs1 < srs2;
            break;
        case 3:
            // sltu
            if (MODE)
                evm_print("sltu x[%d] = bool(x[%d] <u x[%d])\n", rd, rs1, rs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] < X[rs2];
            break;
        case 4:
            // xor
            if (MODE)
                evm_print("xor x[%d] = x[%d] ^ x[%d]\n", rd, rs1, rs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] ^ X[rs2];
            break;
        case 5:
            if (inst & 0x40000000) {
                // sra
                if (MODE)
                    evm_print("slr x[%d] = x[%d] >>s x[%d]\n", rd, rs1, rs2);
                if (MODE == EVM_MODE_DISASM)
                    goto out;
                X[rd] = srs1 >> (X[rs2] & 0x1f);
                break;
            }
            // slr
            if (MODE)
                evm_print("slr x[%d] = x[%d] >> x[%d]\n", rd, rs1, rs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] >> (X[rs2] & 0x1f);
            break;
        case 6:
            // or
            if (MODE)
                evm_print("or x[%d] = x[%d] | x[%d]\n", rd, rs1, rs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] | X[rs2];
            break;
        case 7:
            // and
            if (MODE)
                evm_print("and x[%d] = x[%d] & x[%d]\n", rd, rs1, rs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            X[rd] = X[rs1] & X[rs2];
            break;
    }
out:
    return PC+4;
}

static uint8_t *
bxx(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    uint8_t func3 = (inst >> 12) & 0x7;
    uint8_t rs1 = (inst >> 15) & 0x1f;
    uint8_t rs2 = (inst >> 20) & 0x1f;
    uint32_t offset =
        (((inst >> 25) & 0x3f) << 5)  |  // imm[10:5]
        (((inst >>  8) & 0xf)  << 1)  |  // imm[4:1]
        (((inst >>  7) & 0x1)  << 11);   // imm[11]
    if (inst & 0x80000000)
        offset |= 0xfffff000;
    uint32_t urs1 = X[rs1];
    uint32_t urs2 = X[rs2];
    int32_t srs1 = X[rs1];
    int32_t srs2 = X[rs2];
    switch (func3) {
        case 0: // beq
            if (MODE)
                evm_print(
                    "beq if (x[%d] == x[%d]) pc += sext(%d)) // %d == %d\n",
                    rs1, rs2, offset, X[rs1], X[rs2]);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            if (X[rs1] == X[rs2])
                return PC + offset;
            break;
        case 1: // bne
            if (MODE)
                evm_print(
                    "bne if (x[%d] != x[%d]) pc += sext(%d)) // %d != %d\n",
                    rs1, rs2, offset, X[rs1], X[rs2]);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            if (X[rs1] != X[rs2])
                return PC + offset;
            break;
        case 4: // blt
            if (MODE)
                evm_print(
                    "blt if (x[%d] < x[%d]) pc += sext(%d)) // %d < %d\n",
                    rs1, rs2, offset, srs1, srs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            if (srs1 < srs2)
                return PC + offset;
            break;
        case 5: // bge
            if (MODE)
                evm_print(
                    "bge if (x[%d] >= x[%d]) pc += sext(%d)) // %d >= %d\n",
                    rs1, rs2, offset, srs1, srs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            if (srs1 >= srs2)
                return PC + offset;
            break;
        case 6: // bltu
            if (MODE)
                evm_print(
                    "bltu if (x[%d] < x[%d]) pc += sext(%d)) // %d < %d\n",
                    rs1, rs2, offset, urs1, urs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            if (urs1 < urs2)
                return PC + offset;
            break;
        case 7: // bgeu
            if (MODE)
                evm_print(
                    "bgeu if (x[%d] >= x[%d]) pc += sext(%d)) // %d >= %d\n",
                    rs1, rs2, offset, urs1, urs2);
            if (MODE == EVM_MODE_DISASM)
                goto out;
            if (urs1 >= urs2)
                return PC + offset;
            break;
    }
out:
    return PC + 4;
}

static uint8_t *
sw(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    uint8_t func3 = (inst >> 12) & 0x7;
    uint8_t rs1 = (inst >> 15) & 0x1f;
    uint8_t rs2 = (inst >> 20) & 0x1f;
    uint32_t offset =
        ((inst >> 7) & 0x1f) |
        (((inst >> 25) & 0x7f) << 5);
    if (inst & 0x80000000)
        offset |= 0xfffff000;
    uint8_t *mem = (uint8_t *)(X[rs1] + offset);
    uint8_t width = 1 << (func3 & 0x3); // 1, 2, 4 bytes
    if (MODE) {
        static const char *names[] = {"sb","sh","sw"};
        evm_print("%s M[x[%d] + sext(%d)] = x[%d][%d:0] // * (u8 *) %p = %d\n",
            names[func3], rs1, offset, rs2, width * 8 -1, mem, X[rs2]);
    }
    if (MODE == EVM_MODE_DISASM)
        goto out;
    for (int i = 0; i < width; i++)
        mem[i] = X[rs2] >> (i * 8);
out:
    return PC + 4;
}

static uint8_t *
auipc(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    uint8_t rd = (inst >> 7) & 0x1f;
    int32_t imm = inst & 0xffffff000;
    // AUIPC
    if (MODE)
        evm_print("auipc x[%d] = pc + sext(%d)\n", rd, imm);
    if (MODE == EVM_MODE_DISASM)
        goto out;
    X[rd] = (uint32_t) (PC + imm);
out:
    return PC + 4;
}

static uint8_t *
lui(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    // LUI
    uint8_t rd = (inst >> 7) & 0x1f;
    int32_t imm = inst & 0xffffff000;
    if (MODE)
        evm_print("lui x[%d] = sext(0x%x)\n", rd, imm);
    if (MODE == EVM_MODE_DISASM)
        goto out;
    X[rd] = imm;
out:
    return PC + 4;
}

static uint8_t *
jalr(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
        // JALR
    uint8_t rd = (inst >> 7) & 0x1f;
    uint8_t rs1 = (inst >> 15) & 0x1f;
    uint32_t offset = (inst >> 20) & 0xfff;
    if (inst & 0x80000000)
        offset |= 0xfffff000;
    if (MODE)
        evm_print(
            "jalr t=pc+4; pc=(x[%d]+sext(%d))&∼1; x[%d]=t\n",
            rs1, offset, rd);
    if (MODE == EVM_MODE_DISASM)
        goto out;
    uint32_t t = (uint32_t) PC + 4;
    PC = (uint8_t *) ((X[rs1] + offset) & ~1);
    if (rd)
        X[rd] = t;
    return PC;
out:
    return PC + 4;
}

static uint8_t *
jal(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    // JAL
    uint8_t rd = (inst >> 7) & 0x1f;
    uint32_t offset =
        (((inst >> 21) & 0x3ff)<< 1)  |  // imm[10:1]
        (((inst >> 20) & 0x1)  << 11) |  // imm[11]
        (((inst >> 12) & 0xff) << 12);   // imm[19:12]
    if (inst & 0x80000000)
        offset |= 0xfff00000;
    if (MODE)
        evm_print("jal x[%d] = pc+4; pc += sext(%d)\n", rd, offset);
    if (MODE == EVM_MODE_DISASM)
        goto out;
    if (rd) // x0 is readonly, and always zero, it is a special case
        X[rd] = (uint32_t)(PC + 4);
    return PC + offset;
out:
    return PC + 4;
}

static uint8_t *
ecall(uint8_t *PC, uint32_t *X, uint32_t inst, uint8_t MODE)
{
    uint16_t id = inst >> 20;
    if (MODE)
        evm_print(
            "ecall id: %d a0 %x, a1 %x, a2 %x, a3 %x, a4 %x\n",
                id, X[10], X[11], X[12], X[13], X[14]);
    if (MODE != EVM_MODE_DISASM)
        platform_ecall(id, &X[10]);
    return PC + 4;
}
static uint8_t *
c0(uint8_t *PC, uint32_t *X, uint16_t inst, uint8_t MODE) {
    const uint8_t opcode = inst >> 13;
    uint8_t rd = ((inst >> 2) & 0x7) + 8;
    uint8_t rs1 = ((inst >> 7) & 0x7) + 8;
    uint32_t imm = ((inst >> 6) & 0x1) << 2 |
                   ((inst >> 10) & 0x7) << 3 |
                   ((inst >> 5) & 0x1) << 6;
    uint8_t *mem = (uint8_t *) (X[rs1] + imm);
    switch (opcode) {
        case 0: // c.addi4spn
            imm = ((inst >> 6) & 0x1) << 2;
            imm |= ((inst >> 5) & 0x1) << 3;
            imm |= ((inst >> 11) & 0x3) << 4;
            imm |= ((inst >> 7) & 0xf) << 6;
            if (MODE)
                evm_print("c.addi4spn x[%d] = X[2] + %d\n", rd, imm);
            if (MODE == EVM_MODE_DISASM)
                break;
            X[rd] = X[2] + imm;
            break;
        case 2: // c.lw
            if (MODE)
                evm_print("c.lw x[%d] = M[x[%d] + %d]\n", rd, rs1, imm);
            if (MODE == EVM_MODE_DISASM)
                break;
            uint32_t val = 0;
            for (int i = 0; i < 4; i++)
                val |= mem[i] << (i * 8);
            X[rd] = val;
            break;
        case 6: // c.sw
            uint8_t rs2 = rd;
            if (MODE)
                evm_print("c.sw M[x[%d] + %d] = x[%d]\n", rs1,
                    imm, rs2);
            if (MODE == EVM_MODE_DISASM)
                break;
            for (int i = 0; i < 4; i++)
                mem[i] = X[rs2] >> (i * 8);
            break;
        default:
            if (MODE)
                evm_print("c.opcode 0x%1x.%d - not implemented\n",
                    opcode, 0);
    }
    return PC + 2;
}
static uint8_t *
c1(uint8_t *PC, uint32_t *X, uint16_t inst, uint8_t MODE) {
    const uint8_t opcode = inst >> 13;
    uint8_t rd = (inst >> 7) & 0x1f;
    uint32_t imm = (inst >> 2) & 0x1f;
    uint32_t offset;
    uint8_t rs1;
    switch (opcode) {
        case 0: // c.addi
            if (inst & 0x1000)
                imm |= 0xffffffe0;
            if (MODE)
                evm_print("c.addi x[%d] += sext(%d)\n", rd, (int32_t)imm);
            if (MODE == EVM_MODE_DISASM)
                break;
            X[rd] += imm;
            break;
        case 2: // c.li
            if (inst & 0x1000)
                imm |= 0xffffffe0;
            if (MODE)
                evm_print("c.li x[%d] = sext(%d)\n", rd, (int32_t)imm);
            if (MODE == EVM_MODE_DISASM)
                break;
            X[rd] = imm;
            break;
        case 3:
            if (rd == 2) { // c.addi16sp
                imm = ((inst >> 6) & 0x1) << 4;
                imm |= ((inst >> 5) & 0x1) << 6;
                imm |= ((inst >> 4) & 0x1) << 8;
                imm |= ((inst >> 3) & 0x1) << 7;
                imm |= ((inst >> 2) & 0x1) << 5;
                if (inst & 0x1000)
                    imm |= 0xffffff00;
                if (MODE)
                    evm_print("c.addi16sp x[2] += sext(%d)\n",
                        (int32_t)imm);
                if (MODE == EVM_MODE_DISASM)
                    break;
                X[2] = X[2] + imm;
                break;
            } else { // c.lui
                imm = ((inst >> 2) & 0x1f) << 12;
                if (inst & 0x1000)
                    imm |= 0xfffe0000;
                if (MODE)
                    evm_print("c.lui x[%d] = sext(%d)\n",
                        rd, (int32_t)imm);
                if (MODE == EVM_MODE_DISASM)
                    break;
                X[rd] = imm;
            }
            break;
        case 4:
            uint8_t microop = (inst >> 10) & 0x7;
            imm = (inst >> 2) & 0x1f;
            uint8_t shamt = imm;
            rd = ((inst >> 7) & 0x7) + 8;
            if (microop == 0x0) { // c.srli
                if (MODE)
                    evm_print("c.srli x[%d] = x[%d] >>u %d // %x -> %x\n",
                        rd, rd, shamt, X[rd], X[rd] >> shamt);
                if (MODE == EVM_MODE_DISASM)
                    break;
                X[rd] = X[rd] >> shamt;
            } else if (microop == 0x1) { // c.srai
                if (MODE)
                    evm_print("c.srai x[%d] = x[%d] >>s %d\n",
                        rd, rd, shamt);
                if (MODE == EVM_MODE_DISASM)
                    break;
                int32_t v = X[rd];
                X[rd] = v >> shamt;
            } else if (microop == 0x2) { // c.andi
                if (MODE)
                    evm_print("c.andi x[%d] = x[%d] & sext(0x%x)\n",
                        rd, rd, imm);
                if (MODE == EVM_MODE_DISASM)
                    break;
                X[rd] = X[rd] & imm;
            } else if (microop == 0x6) { // c.andi
                if (MODE)
                    evm_print("c.andi x[%d] = x[%d] & sext(0x%x)\n",
                        rd, rd, imm);
                if (MODE == EVM_MODE_DISASM)
                    break;
                imm |= 0xffffffe0;
                X[rd] = X[rd] & imm;
            } else if (microop == 0x3) {
                uint8_t mmop = (inst >> 5) & 0x3;
                uint8_t rs2 = ((inst >> 2) & 0x7) + 8;
                if (MODE && mmop == 0x0)
                    evm_print("c.sub x[%d] = x[%d] - x[%d] // %d - %d\n",
                        rd, rd, rs2, X[rd], X[rs2]);
                if (MODE && mmop == 0x1)
                    evm_print("c.xor x[%d] = x[%d] ^ x[%d]\n", rd, rd, rs2);
                if (MODE && mmop == 0x2)
                    evm_print("c.or x[%d] = x[%d] | x[%d]\n", rd, rd, rs2);
                if (MODE && mmop == 0x3)
                    evm_print("c.and x[%d] = x[%d] & x[%d] // %x & %x\n",
                        rd, rd, rs2, X[rd], X[rs2]);
                if (MODE == EVM_MODE_DISASM)
                    break;
                if (mmop == 0x0) // c.sub
                    X[rd] = X[rd] - X[rs2];
                if (mmop == 0x1) // c.xor
                    X[rd] = X[rd] ^ X[rs2];
                if (mmop == 0x2) // c.or
                    X[rd] = X[rd] | X[rs2];
                if (mmop == 0x3) // c.and
                    X[rd] = X[rd] & X[rs2];
            }
            break;
        case 1: // c.jal imm[11|4|9:8|10|6|7|3:1|5]
        case 5: // c.j imm[11|4|9:8|10|6|7|3:1|5]
            imm = ((inst >> 2) & 0x1) << 5;
            imm |= ((inst >> 3) & 0x7) << 1;
            imm |= ((inst >> 6) & 0x1) << 7;
            imm |= ((inst >> 7) & 0x1) << 6;
            imm |= ((inst >> 8) & 0x1) << 10;
            imm |= ((inst >> 9) & 0x3) << 8;
            imm |= ((inst >> 11) & 0x1) << 4;
            if (inst & 0x1000)
                imm |= 0xfffff800;
            if (MODE && opcode == 1)
                evm_print("c.jal x[1] = pc+2; pc += sext(%d)\n",
                    (int32_t)imm);
            if (MODE && opcode == 5)
                evm_print("c.j pc += sext(%d)\n", imm);
            if (MODE == EVM_MODE_DISASM)
                break;
            if (opcode == 1)
                X[1] = (uint32_t) PC + 2;
            return PC + imm;
        case 6: // c.beqz
        case 7: // c.bnez
            rs1 = ((inst >> 7) & 0x7) + 8;
            offset = ((inst >> 3) & 0x3) << 1;
            offset |= ((inst >> 10) & 0x3) << 3;
            offset |= ((inst >> 2) & 0x1) << 5;
            offset |= ((inst >> 5) & 0x3) << 6;
            if (inst & 0x1000)
                offset |= 0xffffff00;
            if (MODE && (opcode & 1) == 0)
                evm_print("c.beqz if (x[%d] == 0) pc += sext(%d)\n",
                    rs1, offset);
            if (MODE && (opcode & 1))
                evm_print("c.bnez if (x[%d] != 0) pc += sext(%d)\n",
                    rs1, offset);
            if (MODE == EVM_MODE_DISASM)
                break;
            int eqz = (X[rs1] == 0);
            // For documentation
            // 1 && 0 == 0
            // if (eqz && (opcode & 1) == 0)
            //     return PC + offset;
            // !0 && 1
            // if (!eqz && (opcode & 1)) //
            //     return PC + offset;
            if (eqz ^ (opcode & 1))
                return PC + offset;
            break;
        default:
            if (MODE)
                evm_print("c.opcode 0x%1x.%d - not implemented\n",
                    opcode, 1);
    }
    return PC + 2;
}

static uint8_t *
c2(uint8_t *PC, uint32_t *X, uint16_t inst, uint8_t MODE) {
    const uint8_t opcode = inst >> 13;
    uint8_t rd = (inst >> 7) & 0x1f;
    uint8_t rs1 = rd;
    uint8_t rs2 = (inst >> 2) & 0x1f;
    uint8_t *mem;
    uint32_t imm;
    switch (opcode) {
        case 0: // c.slli
            uint8_t shamt = (inst >> 2) & 0x1f;
            if (MODE)
                evm_print("c.slli x[%d] = x[%d] << %d\n", rd, shamt);
            if (MODE == EVM_MODE_DISASM)
                break;
            X[rd] = X[rd] << shamt;
            break;
        case 2: // c.lwsp
            imm = ((inst >> 2) & 0x3) << 6;
            imm |= ((inst >> 4) & 0x7) << 2;
            imm |= ((inst >> 12) & 0x1) << 5;
            if (MODE)
                evm_print("c.lwsp x[%d] = M[x[2] + %d]\n", rd, imm);
            if (MODE == EVM_MODE_DISASM)
                break;
            mem = (uint8_t *) (X[2] + imm);
            uint32_t val = 0;
            for (int i = 0; i < 4; i++)
                val |= mem[i] << (i * 8);
            X[rd] = val;
            break;
        case 4:
            if ((inst & 0x1000) == 0) {
                if (rs2) { // c.mv
                    if (MODE)
                        evm_print("c.mv x[%d] = x[%d]\n", rd, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        break;
                    X[rd] = X[rs2];
                } else { // c.jr
                    if (MODE)
                        evm_print("c.jr pc = x[%d] // 0x%x\n", rs1, X[rs1]);
                    if (MODE == EVM_MODE_DISASM)
                        break;
                    return (uint8_t *) X[rs1];
                }
            } else {
                if (rs2 == 0) {
                    if (rs1 == 0) { // c.ebreak
                        if (MODE)
                            evm_print("c.ebreak\n");
                        if (MODE == EVM_MODE_DISASM)
                            break;
                    } else { // c.jalr
                        if (MODE)
                            evm_print("c.jalr t = pc+2; pc = x[%d] (%p); x[1] = t\n",
                                rs1, X[rs1]);
                        if (MODE == EVM_MODE_DISASM)
                            break;
                        uint32_t t = (uint32_t) (PC + 2);
                        PC = (uint8_t *) X[rs1];
                        X[1] = t;
                        return PC;
                    }
                } else { // c.add
                    uint8_t rd = rs1;
                    if (MODE)
                        evm_print("c.add x[%d] = x[%d] + x[%d]\n",
                            rd, rd, rs2);
                    if (MODE == EVM_MODE_DISASM)
                        break;
                    X[rd] = X[rd] + X[rs2];
                }
            }
            break;
        case 6: // c.swsp
            imm = ((inst >> 9) & 0xf) | ((inst >> 7) & 0x3) << 4;
            imm = imm << 2;
            if (MODE)
                evm_print("c.swsp M[x[2] + %d] = x[%d] // = 0x%x\n",
                    imm, rs2, X[rs2]);
            if (MODE == EVM_MODE_DISASM)
                break;
            mem = (uint8_t *) (X[2] + imm);
            for (int i = 0; i < 4; i++)
                mem[i] = X[rs2] >> (i * 8);
            break;
        default:
            if (MODE)
                evm_print("c.opcode 0x%1x.%d - not implemented\n",
                    opcode, 2);
    }
    return PC + 2;
}

int
evm_interpreter(uint8_t *prog_start, uint8_t *end,
                uint8_t mode, struct evm_args *args)
{
    const uint8_t MODE = mode;
    if (MODE == EVM_MODE_DISASM)
        evm_print("Run disassembler\n");
    if (MODE == EVM_MODE_INT_DEBUG)
        evm_print("Run interpreter debug mode\n");
    uint32_t X[16];
    uint8_t stack[256];
    X[0] = 0; // always
    X[1] = (uint32_t) end; // dummy return point
    X[2] = (uint32_t) &stack[sizeof(stack)]; // stack pointer
    X[10] = args->a0;
    X[11] = args->a1;
    X[12] = args->a2;
    X[13] = args->a3;
    X[14] = args->a4;
    // Program counter
    uint8_t *PC = prog_start;
    while (PC < end) {
        const uint16_t *ptr = (uint16_t *) PC;
        const uint8_t prefix = *ptr & 0x3;
        if (prefix == 0x3) {
            const uint8_t opcode = (*ptr >> 2) & 0x1f; // 5 bit
            uint32_t inst = *ptr;
            uint16_t hi = *(uint16_t *)(PC + 2);
            inst |= (uint32_t)hi << 16;
            if (MODE)
                evm_print("%4x (x%02x.%x) | ", PC - prog_start, opcode, 3);
            switch (opcode) {
                case 0x00: PC = lx(PC, X, inst, MODE); break;
                case 0x04: PC = alu(PC, X, inst, MODE); break;
                case 0x05: PC = auipc(PC, X, inst, MODE); break;
                case 0x08: PC = sw(PC, X, inst, MODE); break;
                case 0x0c: PC = sx(PC, X, inst, MODE); break;
                case 0x0d: PC = lui(PC, X, inst, MODE); break;
                case 0x18: PC = bxx(PC, X, inst, MODE); break;
                case 0x19: PC = jalr(PC, X, inst, MODE); break;
                case 0x1b: PC = jal(PC, X, inst, MODE); break;
                case 0x1c: PC = ecall(PC, X, inst, MODE); break;
                default:
                    if (MODE)
                        evm_print("opcode 0x%02x - not implemented\n",
                            opcode);
                    return -1;
            }
        } else {
            const uint16_t inst = *ptr;
            const uint8_t opcode = inst >> 13;
            if (MODE)
                evm_print("%4x (x%02x.%x) | ", PC - prog_start, opcode,
                    prefix);
            switch (prefix) {
                case 0: PC = c0(PC, X, *ptr, MODE); break;
                case 1: PC = c1(PC, X, *ptr, MODE); break;
                case 2: PC = c2(PC, X, *ptr, MODE); break;
            }
        }
    }
    args->a0 = X[10];
    args->a1 = X[11];
    args->a2 = X[12];
    args->a3 = X[13];
    args->a4 = X[14];
    return 0;
}
