#include <stdint.h>
#include <stdio.h>

#include "system.h"

uint8_t *mapper0(uint16_t addr, nes_t *nes) {
	if (addr >= 0x6000 && addr < 0x8000) {
		if (nes->sram.size < 1) {
			fprintf(stderr, "No SRAM\n");
			return NULL;
		}
		return &nes->sram.mem[(addr - 0x6000) % nes->sram.size];
	} else {
		if (nes->rom->prg_size != 16384 && nes->rom->prg_size != 32768) {
			fprintf(stderr, "prg_size %lu, not 16K or 32K\n", nes->rom->prg_size);
			return NULL;
		}
		return &nes->rom->prg[(addr - 0x8000) % nes->rom->prg_size];
	}
}

typedef uint16_t mapper_t;

typedef uint8_t *(*mapper_func_t)(uint16_t, nes_t *);

mapper_func_t mapper_func(mapper_t mapper) {
	if (mapper == 0) {
		return mapper0;
	}
	fprintf(stderr, "No mapper function for mapper %d\n", mapper);
	return NULL;
}

mapper_t mapper(rom_t *rom) {
	return (rom->header[6] >> 4) | (rom->header[7] & 0xF0);
}

uint8_t *ppu_reg(uint16_t addr, ppu_t *ppu) {
	if (addr >= 0x2000 && addr < 0x4000) {
		uint8_t regno = addr % 8;
		switch (regno) {
			case 0x0:
				return &ppu->regfile.ctrl;
			case 0x1:
				return &ppu->regfile.mask;
			case 0x2:
				return &ppu->regfile.status;
			case 0x3:
				return &ppu->regfile.oamaddr;
			case 0x4:
				return &ppu->regfile.oamdata;
			case 0x5:
				return &ppu->regfile.scroll;
			case 0x6:
				return &ppu->regfile.addr;
			case 0x7:
				return &ppu->regfile.data;
		}
	} else if (addr == 0x4014) {
		return &ppu->regfile.oamdma;
	}
	return NULL;
}

uint8_t *apu_reg(uint16_t addr, apu_t *apu) {
	if (addr >= 0x4000) {
        if (addr < 0x4004) {
            return &apu->regfile.pulse[0][addr - 0x4000];
        } else if (addr < 0x4008) {
            return &apu->regfile.pulse[1][addr - 0x4004];
        } else if (addr < 0x400C) {
            return &apu->regfile.triangle[addr - 0x4008];
        } else if (addr < 0x4010) {
            return &apu->regfile.noise[addr - 0x400C];
        } else if (addr < 0x4014) {
            return &apu->regfile.dmc[addr - 0x4010];
        } else if (addr == 0x4015) {
            return &apu->regfile.status;
        } else if (addr == 0x4017) {
            return &apu->regfile.frame;
        }
    }
    return NULL;
}

uint8_t *mem(uint16_t addr, nes_t *nes, int write) {
	if (addr < 0x2000) {
		return &nes->cpu.ram[addr % 0x800];
	}
    if (addr == 0x4016) {
		// TODO JOY1
		return NULL;
	}
    if (addr == 0x4017 && !write) {
        // TODO JOY2
        return NULL;
    }
    if (addr < 0x4018) {
        uint8_t *ppu_addr = ppu_reg(addr, &nes->ppu);
        if (ppu_addr) {
            return ppu_addr;
        }
        uint8_t *apu_addr = apu_reg(addr, &nes->apu);
        if (apu_addr) {
            return apu_addr;
        }
	}
    if (addr < 0x4020) {
		// TODO cpu test mode
		return NULL;
	}
    if (addr < 0x6000) {
		// TODO cartridge expansion ROM
		return NULL;
	}
    mapper_t m = mapper(nes->rom);
    mapper_func_t mf = mapper_func(m);
    return mf(addr, nes);
}

uint8_t mem_read(uint16_t addr, nes_t *nes) {
    uint8_t value = *mem(addr, nes, 0);
    ++nes->cpu.cycles;
#ifdef DEBUG
    printf("Read %02X from %04X\n", value, addr);
#endif 
    return value;
}

void mem_write(uint16_t addr, uint8_t value, nes_t *nes) {
    *mem(addr, nes, 1) = value;
    ++nes->cpu.cycles;
    nes->cpu.just_wrote = 1;
#ifdef DEBUG
    printf("Wrote %02X to %04X\n", value, addr);
#endif
}

void push(uint8_t value, nes_t *nes) {
    mem_write(nes->cpu.regfile.s, value, nes);
    nes->cpu.regfile.s -= 1;
}

uint8_t pull(nes_t *nes) {
    nes->cpu.regfile.s += 1;
    ++nes->cpu.cycles;
    return mem_read(nes->cpu.regfile.s, nes);
}

uint16_t reset_vector(nes_t *nes) {
    return mem_read(0xFFFC, nes) | (mem_read(0xFFFD, nes) << 8);
}

uint16_t irq_vector(nes_t *nes) {
    return mem_read(0xFFFE, nes) | (mem_read(0xFFFF, nes) << 8);
}

uint8_t get_negative(nes_t *nes) {
    return (nes->cpu.regfile.p >> 7) & 1;
}

uint8_t get_overflow(nes_t *nes) {
    return (nes->cpu.regfile.p >> 6) & 1;
}

uint8_t get_zero(nes_t *nes) {
    return (nes->cpu.regfile.p >> 1) & 1;
}

uint8_t get_carry(nes_t *nes) {
    return nes->cpu.regfile.p & 1;
}

void change_bit(uint8_t *flags, uint8_t position, uint8_t value) {
    *flags ^= (-!!value ^ *flags) & (1 << position);
}

void change_negative(uint8_t n, nes_t *nes) {
    change_bit(&nes->cpu.regfile.p, 7, n);
}

void change_overflow(uint8_t v, nes_t *nes) {
    change_bit(&nes->cpu.regfile.p, 6, v);
}

void change_decimal(uint8_t d, nes_t *nes) {
    change_bit(&nes->cpu.regfile.p, 3, d);
}

void change_interrupt_disable(uint8_t i, nes_t *nes) {
    change_bit(&nes->cpu.regfile.p, 2, i);
}

void change_zero(uint8_t z, nes_t *nes) {
    change_bit(&nes->cpu.regfile.p, 1, z);
}

void change_carry(uint8_t c, nes_t *nes) {
    change_bit(&nes->cpu.regfile.p, 0, c);
}

void adc(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    uint8_t a = nes->cpu.regfile.a;
    uint16_t sum = a + arg + get_carry(nes);
    change_negative((sum >> 7) & 1, nes);
    change_overflow(~(a ^ arg) & (a ^ sum) & 0x80, nes);
    change_zero(sum == 0, nes);
    change_carry(sum > 0xFF, nes);
    nes->cpu.regfile.a = (uint8_t) sum;
}

void and(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    nes->cpu.regfile.a &= arg;
    change_negative(nes->cpu.regfile.a >> 7, nes);
    change_zero(nes->cpu.regfile.a == 0, nes);
}

uint8_t asl_impl(uint8_t old, nes_t *nes) {
    uint8_t new = old << 1;
    change_negative(new >> 7, nes);
    change_zero(new == 0, nes);
    change_carry(old >> 7, nes);
    return new;
}

void asl_a(nes_t *nes) {
    nes->cpu.regfile.a = asl_impl(nes->cpu.regfile.a, nes);
}

void asl_mem(uint16_t addr, nes_t *nes) {
    mem_write(addr, asl_impl(mem_read(addr, nes), nes), nes);
    ++nes->cpu.cycles;
}

void bcc(uint16_t addr, nes_t *nes) {
    if (!get_carry(nes)) {
        nes->cpu.regfile.pc = addr;
        nes->cpu.just_branched = 1;
    }
}

void bcs(uint16_t addr, nes_t *nes) {
    if (get_carry(nes)) {
        nes->cpu.regfile.pc = addr;
        nes->cpu.just_branched = 1;
    }
}

void beq(uint16_t addr, nes_t *nes) {
    if (get_zero(nes)) {
        nes->cpu.regfile.pc = addr;
        nes->cpu.just_branched = 1;
    }
}

void bit(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    change_negative(arg >> 7, nes);
    change_overflow((arg >> 6) & 1, nes);
    change_zero((arg & nes->cpu.regfile.a) == 0, nes);
}

void bmi(uint16_t addr, nes_t *nes) {
    if (get_negative(nes)) {
        nes->cpu.regfile.pc = addr;
        nes->cpu.just_branched = 1;
    }
}

void bne(uint16_t addr, nes_t *nes) {
    if (!get_zero(nes)) {
        nes->cpu.regfile.pc = addr;
        nes->cpu.just_branched = 1;
    }
}

void bpl(uint16_t addr, nes_t *nes) {
    if (!get_negative(nes)) {
        nes->cpu.regfile.pc = addr;
        nes->cpu.just_branched = 1;
    }
}

void brk(nes_t *nes) {
    push((uint8_t) (nes->cpu.regfile.pc >> 8), nes);
    push((uint8_t) nes->cpu.regfile.pc, nes);
    push(nes->cpu.regfile.p, nes);
    nes->cpu.regfile.pc = irq_vector(nes);
    nes->cpu.just_branched = 1;
}

void bvc(uint16_t addr, nes_t *nes) {
    if (!get_overflow(nes)) {
        nes->cpu.regfile.pc = addr;
        nes->cpu.just_branched = 1;
    }
}

void bvs(uint16_t addr, nes_t *nes) {
    if (get_overflow(nes)) {
        nes->cpu.regfile.pc = addr;
        nes->cpu.just_branched = 1;
    }
}

void clc(nes_t *nes) {
    change_carry(0, nes);
}

void cld(nes_t *nes) {
    change_decimal(0, nes);
}

void cli(nes_t *nes) {
    change_interrupt_disable(0, nes);
}

void clv(nes_t *nes) {
    change_overflow(0, nes);
}

void compare(uint8_t lhs, uint8_t rhs, nes_t *nes) {
    change_negative((lhs - rhs) >> 7, nes);
    change_zero(lhs == rhs, nes);
    change_carry(lhs >= rhs, nes);
}

void cmp(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    compare(nes->cpu.regfile.a, arg, nes);
}
   
void cpx(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    compare(nes->cpu.regfile.x, arg, nes);
}

void cpy(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    compare(nes->cpu.regfile.y, arg, nes);
}

void dec(uint16_t addr, nes_t *nes) {
    uint8_t new = mem_read(addr, nes) - 1;
    ++nes->cpu.cycles;
    mem_write(addr, new, nes);
    change_negative(new >> 7, nes);
    change_zero(new == 0, nes);
}

void dex(nes_t *nes) {
    --nes->cpu.regfile.x;
    change_negative(nes->cpu.regfile.x >> 7, nes);
    change_zero(nes->cpu.regfile.x == 0, nes);
}

void dey(nes_t *nes) {
    --nes->cpu.regfile.y;
    change_negative(nes->cpu.regfile.y >> 7, nes);
    change_zero(nes->cpu.regfile.y == 0, nes);
}

void eor(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    nes->cpu.regfile.a ^= arg;
    change_negative(nes->cpu.regfile.a >> 7, nes);
    change_zero(nes->cpu.regfile.a == 0, nes);
}

void inc(uint16_t addr, nes_t *nes) {
    uint8_t new = mem_read(addr, nes) + 1;
    ++nes->cpu.cycles;
    mem_write(addr, new, nes);
    change_negative(new >> 7, nes);
    change_zero(new == 0, nes);
}

void inx(nes_t *nes) {
    ++nes->cpu.regfile.x;
    change_negative(nes->cpu.regfile.x >> 7, nes);
    change_zero(nes->cpu.regfile.x == 0, nes);
}

void iny(nes_t *nes) {
    ++nes->cpu.regfile.y;
    change_negative(nes->cpu.regfile.y >> 7, nes);
    change_zero(nes->cpu.regfile.y == 0, nes);
}

void jmp(uint16_t target, nes_t *nes) {
    nes->cpu.regfile.pc = target;
    nes->cpu.just_branched = 1;
}

void jsr(uint16_t target, nes_t *nes) {
    uint16_t pc_plus_2 = nes->cpu.regfile.pc + 2;
    push((uint8_t) (pc_plus_2 >> 8), nes);
    push((uint8_t) pc_plus_2, nes);
    nes->cpu.regfile.pc = target;
    nes->cpu.just_branched = 1;
    ++nes->cpu.cycles;
}

void load(uint8_t *target, uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    *target = arg;
    change_negative(*target >> 7, nes);
    change_zero(*target == 0, nes);
}

void lda(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    load(&nes->cpu.regfile.a, arg, nes);   
}

void ldx(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    load(&nes->cpu.regfile.x, arg, nes);
}

void ldy(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    load(&nes->cpu.regfile.y, arg, nes);
}

uint8_t lsr_impl(uint8_t old, nes_t *nes) {
    uint8_t new = old >> 1;
    change_negative(0, nes);
    change_zero(new == 0, nes);
    change_carry(old & 1, nes);
    return new;
}

void lsr_a(nes_t *nes) {
    nes->cpu.regfile.a = lsr_impl(nes->cpu.regfile.a, nes);
}

void lsr_mem(uint16_t addr, nes_t *nes) {
    mem_write(addr, lsr_impl(mem_read(addr, nes), nes), nes);
    ++nes->cpu.cycles;
}

void nop(nes_t *nes) {

}

void ora(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    nes->cpu.regfile.a |= arg;
    change_negative(nes->cpu.regfile.a >> 7, nes);
    change_zero(nes->cpu.regfile.a == 0, nes);
}

void pha(nes_t *nes) {
    push(nes->cpu.regfile.a, nes);
}

void php(nes_t *nes) {
    push(nes->cpu.regfile.p, nes);
}

void pla(nes_t *nes) {
    nes->cpu.regfile.a = pull(nes);
    change_negative(nes->cpu.regfile.a >> 7, nes);
    change_zero(nes->cpu.regfile.a == 0, nes);
}

void plp(nes_t *nes) {
    nes->cpu.regfile.p = pull(nes);
}

uint8_t rol_impl(uint8_t old, nes_t *nes) {
    uint8_t new = (old << 1) | get_carry(nes);
    change_negative(new >> 7, nes);
    change_zero(new == 0, nes);
    change_carry(old >> 7, nes);
    return new;
}

void rol_a(nes_t *nes) {
    nes->cpu.regfile.a = rol_impl(nes->cpu.regfile.a, nes);
}

void rol_mem(uint16_t addr, nes_t *nes) {
    mem_write(addr, rol_impl(mem_read(addr, nes), nes), nes);
    ++nes->cpu.cycles;
}

uint8_t ror_impl(uint8_t old, nes_t *nes) {
    uint8_t new = (old >> 1) | (get_carry(nes) << 7);
    change_negative(new >> 7, nes);
    change_zero(new == 0, nes);
    change_carry(old & 1, nes);
    return new;
}

void ror_a(nes_t *nes) {
    nes->cpu.regfile.a = ror_impl(nes->cpu.regfile.a, nes);
}

void ror_mem(uint16_t addr, nes_t *nes) {
    mem_write(addr, ror_impl(mem_read(addr, nes), nes), nes);
    ++nes->cpu.cycles;
}

void rti(nes_t *nes) {
    nes->cpu.regfile.p = pull(nes);
    nes->cpu.regfile.pc = pull(nes) | (pull(nes) << 8);
    nes->cpu.just_branched = 1;
}

void rts(nes_t *nes) {
    nes->cpu.regfile.pc = (pull(nes) | (pull(nes) << 8)) + 1;
    nes->cpu.just_branched = 1;
    ++nes->cpu.cycles;
}

void sbc(uint16_t addr, nes_t *nes) {
    uint8_t arg = mem_read(addr, nes);
    adc(~arg, nes);
}

void sec(nes_t *nes) {
    change_carry(1, nes);
}

void sed(nes_t *nes) {
    change_decimal(1, nes);
}

void sei(nes_t *nes) {
    change_interrupt_disable(1, nes);
}

void sta(uint16_t addr, nes_t *nes) {
    mem_write(addr, nes->cpu.regfile.a, nes);
}

void stx(uint16_t addr, nes_t *nes) {
    mem_write(addr, nes->cpu.regfile.x, nes);
}

void sty(uint16_t addr, nes_t *nes) {
    mem_write(addr, nes->cpu.regfile.y, nes);
}

void tax(nes_t *nes) {
    nes->cpu.regfile.x = nes->cpu.regfile.a;
    change_negative(nes->cpu.regfile.x >> 7, nes);
    change_zero(nes->cpu.regfile.x == 0, nes);
}

void tay(nes_t *nes) {
    nes->cpu.regfile.y = nes->cpu.regfile.a;
    change_negative(nes->cpu.regfile.y >> 7, nes);
    change_zero(nes->cpu.regfile.y == 0, nes);
}

void tsx(nes_t *nes) {
    nes->cpu.regfile.x = nes->cpu.regfile.s;
    change_negative(nes->cpu.regfile.x >> 7, nes);
    change_zero(nes->cpu.regfile.x == 0, nes);
}

void txa(nes_t *nes) {
    nes->cpu.regfile.a = nes->cpu.regfile.x;
    change_negative(nes->cpu.regfile.a >> 7, nes);
    change_zero(nes->cpu.regfile.a == 0, nes);
}

void txs(nes_t *nes) {
    nes->cpu.regfile.s = nes->cpu.regfile.x;
    change_negative(nes->cpu.regfile.s >> 7, nes);
    change_zero(nes->cpu.regfile.s == 0, nes);
}

void tya(nes_t *nes) {
    nes->cpu.regfile.a = nes->cpu.regfile.y;
    change_negative(nes->cpu.regfile.a >> 7, nes);
    change_zero(nes->cpu.regfile.a == 0, nes);
}

typedef enum {
    ERROR = -1, ACCUMULATOR, IMPLIED, IMMEDIATE, RELATIVE, ABSOLUTE, ZERO_PAGE,
    INDIRECT, ABSOLUTE_X, ABSOLUTE_Y, ZERO_PAGE_X, ZERO_PAGE_Y,
    ZERO_PAGE_X_INDIRECT, ZERO_PAGE_INDIRECT_Y
} address_mode_t;

uint16_t actual_address(address_mode_t address_mode, nes_t *nes) {
    uint16_t argp = nes->cpu.regfile.pc + 1;
    switch (address_mode) {
        case ACCUMULATOR:
        case IMPLIED:
            return 0; // not relevant
        case IMMEDIATE:
            return argp;
        case RELATIVE:
            return nes->cpu.regfile.pc + (int8_t) mem_read(argp, nes);
        case ABSOLUTE:
            return mem_read(argp, nes) | (((uint16_t) mem_read(argp + 1, nes)) << 8);
        case ZERO_PAGE:
            return mem_read(argp, nes);
        case INDIRECT: {
            uint16_t addr_of_addr = mem_read(argp, nes) | (((uint16_t) mem_read(argp + 1, nes)) << 8);
            return mem_read(addr_of_addr, nes) | (((uint16_t) mem_read(addr_of_addr +  1, nes)) << 8);
        }
        case ABSOLUTE_X:
            return (mem_read(argp, nes) | (((uint16_t) mem_read(argp + 1, nes)) << 8)) + nes->cpu.regfile.x;
        case ABSOLUTE_Y:
            return (mem_read(argp, nes) | (((uint16_t) mem_read(argp + 1, nes)) << 8)) + nes->cpu.regfile.y;
        case ZERO_PAGE_X:
            return mem_read(argp, nes) + nes->cpu.regfile.x;
        case ZERO_PAGE_Y:
            return mem_read(argp, nes) + nes->cpu.regfile.y;
        case ZERO_PAGE_X_INDIRECT: {
            uint16_t addr_of_addr = mem_read(argp, nes) + nes->cpu.regfile.x;
            return mem_read(addr_of_addr, nes) | (((uint16_t) mem_read(addr_of_addr + 1, nes)) << 8);
        }
        case ZERO_PAGE_INDIRECT_Y: {
            uint16_t addr_of_addr = mem_read(argp, nes);
            return (mem_read(addr_of_addr, nes) | (((uint16_t) mem_read(addr_of_addr + 1, nes)) << 8)) + nes->cpu.regfile.y;
        }
        default: {
            fprintf(stderr, "Unknown address_mode %d\n", address_mode);
            return 0;
        }
    }
}

void print_registers(cpu_regfile_t *regfile) {
    printf("A=%02X, X=%02X, Y=%02X, S=%02X, P=%02X (N=%d, V=%d, D=%d, I=%d, Z=%d, C=%d), PC=%04X\n", regfile->a, regfile->x, regfile->y, regfile->s, regfile->p, !!(regfile->p & 0x80), !!(regfile->p & 0x40), !!(regfile->p & 0x08), !!(regfile->p & 0x04), !!(regfile->p & 0x02), !!(regfile->p & 0x01), regfile->pc);
}

#ifdef DEBUG
unsigned int sleep(unsigned int seconds);
int usleep(unsigned int usec);
#endif

char *inst_mnemonics[256] = {
    "BRK", "ORA", NULL, NULL, NULL, "ORA", "ASL", NULL, "PHP", "ORA", "ASL", NULL, NULL, "ORA", "ASL", NULL,
    "BPL", "ORA", NULL, NULL, NULL, "ORA", "ASL", NULL, "CLC", "ORA", NULL, NULL, NULL, "ORA", "ASL", NULL,
    "JSR", "AND", NULL, NULL, "BIT", "AND", "ROL", NULL, "PLP", "AND", "ROL", NULL, "BIT", "AND", "ROL", NULL,
    "BMI", "AND", NULL, NULL, NULL, "AND", "ROL", NULL, "SEC", "AND", NULL, NULL, NULL, "AND", "ROL", NULL,
    "RTI", "EOR", NULL, NULL, NULL, "EOR", "LSR", NULL, "PHA", "EOR", "LSR", NULL, "JMP", "EOR", "LSR", NULL,
    "BVC", "EOR", NULL, NULL, NULL, "EOR", "LSR", NULL, "CLI", "EOR", NULL, NULL, NULL, "EOR", "LSR", NULL,
    "RTS", "ADC", NULL, NULL, NULL, "ADC", "ROR", NULL, "PLA", "ADC", "ROR", NULL, "JMP", "ADC", "ROR", NULL,
    "BVS", "ADC", NULL, NULL, NULL, "ADC", "ROR", NULL, "SEI", "ADC", NULL, NULL, NULL, "ADC", "ROR", NULL,
    NULL, "STA", NULL, NULL, "STY", "STA", "STX", NULL, "DEY", NULL, "TXA", NULL, "STY", "STA", "STX", NULL,
    "BCC", "STA", NULL, NULL, "STY", "STA", "STX", NULL, "TYA", "STA", "TXS", NULL, NULL, "STA", NULL, NULL,
    "LDY", "LDA", "LDX", NULL, "LDY", "LDA", "LDX", NULL, "TAY", "LDA", "TAX", NULL, "LDY", "LDA", "LDX", NULL,
    "BCS", "LDA", NULL, NULL, "LDY", "LDA", "LDX", NULL, "CLV", "LDA", "TSX", NULL, "LDY", "LDA", "LDX", NULL,
    "CPY", "CMP", NULL, NULL, "CPY", "CMP", "DEC", NULL, "INY", "CMP", "DEX", NULL, "CPY", "CMP", "DEC", NULL,
    "BNE", "CMP", NULL, NULL, NULL, "CMP", "DEC", NULL, "CLD", "CMP", NULL, NULL, NULL, "CMP", "DEC", NULL,
    "CPX", "SBC", NULL, NULL, "CPX", "SBC", "INC", NULL, "INX", "SBC", "NOP", NULL, "CPX", "SBC", "INC", NULL,
    "BEQ", "SBC", NULL, NULL, NULL, "SBC", "INC", NULL, "SED", "SBC", NULL, NULL, NULL, "SBC", "INC", NULL
};

address_mode_t inst_address_modes[256] = {
    IMPLIED, ZERO_PAGE_X_INDIRECT, ERROR, ERROR, ERROR, ZERO_PAGE, ZERO_PAGE, ERROR, IMPLIED, IMMEDIATE, ACCUMULATOR, ERROR, ERROR, ABSOLUTE, ABSOLUTE, ERROR,
    RELATIVE, ZERO_PAGE_INDIRECT_Y, ERROR, ERROR, ERROR, ZERO_PAGE_X, ZERO_PAGE_X, ERROR, IMPLIED, ABSOLUTE_Y, ERROR, ERROR, ERROR, ABSOLUTE_X, ABSOLUTE_X, ERROR,
    ABSOLUTE, ZERO_PAGE_X_INDIRECT, ERROR, ERROR, ZERO_PAGE, ZERO_PAGE, ZERO_PAGE, ERROR, IMPLIED, IMMEDIATE, ACCUMULATOR, ERROR, ABSOLUTE, ABSOLUTE, ABSOLUTE, ERROR,
    RELATIVE, ZERO_PAGE_INDIRECT_Y, ERROR, ERROR, ERROR, ZERO_PAGE_X, ZERO_PAGE_X, ERROR, IMPLIED, ABSOLUTE_Y, ERROR, ERROR, ERROR, ABSOLUTE_X, ABSOLUTE_X, ERROR,
    IMPLIED, ZERO_PAGE_X_INDIRECT, ERROR, ERROR, ERROR, ZERO_PAGE, ZERO_PAGE, ERROR, IMPLIED, IMMEDIATE, ACCUMULATOR, ERROR, ABSOLUTE, ABSOLUTE, ABSOLUTE, ERROR,
    RELATIVE, ZERO_PAGE_INDIRECT_Y, ERROR, ERROR, ERROR, ZERO_PAGE_X, ZERO_PAGE_X, ERROR, IMPLIED, ABSOLUTE_Y, ERROR, ERROR, ERROR, ABSOLUTE_X, ABSOLUTE_X, ERROR,
    IMPLIED, ZERO_PAGE_X_INDIRECT, ERROR, ERROR, ERROR, ZERO_PAGE, ZERO_PAGE, ERROR, IMPLIED, IMMEDIATE, ACCUMULATOR, ERROR, INDIRECT, ABSOLUTE, ABSOLUTE, ERROR,
    RELATIVE, ZERO_PAGE_INDIRECT_Y, ERROR, ERROR, ERROR, ZERO_PAGE_X, ZERO_PAGE_X, ERROR, IMPLIED, ABSOLUTE_Y, ERROR, ERROR, ERROR, ABSOLUTE_X, ABSOLUTE_X, ERROR,
    ERROR, ZERO_PAGE_X_INDIRECT, ERROR, ERROR, ZERO_PAGE, ZERO_PAGE, ZERO_PAGE, ERROR, IMPLIED, ERROR, IMPLIED, ERROR, ABSOLUTE, ABSOLUTE, ABSOLUTE, ERROR,
    RELATIVE, ZERO_PAGE_INDIRECT_Y, ERROR, ERROR, ZERO_PAGE_X, ZERO_PAGE_X, ZERO_PAGE_Y, ERROR, IMPLIED, ABSOLUTE_Y, IMPLIED, ERROR, ERROR, ABSOLUTE_X, ERROR, ERROR,
    IMMEDIATE, ZERO_PAGE_X_INDIRECT, IMMEDIATE, ERROR, ZERO_PAGE, ZERO_PAGE, ZERO_PAGE, ERROR, IMPLIED, IMMEDIATE, IMPLIED, ERROR, ABSOLUTE, ABSOLUTE, ABSOLUTE, ERROR,
    RELATIVE, ZERO_PAGE_INDIRECT_Y, ERROR, ERROR, ZERO_PAGE_X, ZERO_PAGE_X, ZERO_PAGE_Y, ERROR, IMPLIED, ABSOLUTE_Y, IMPLIED, ERROR, ABSOLUTE_X, ABSOLUTE_X, ABSOLUTE_Y, ERROR,
    IMMEDIATE, ZERO_PAGE_X_INDIRECT, ERROR, ERROR, ZERO_PAGE, ZERO_PAGE, ZERO_PAGE, ERROR, IMPLIED, IMMEDIATE, IMPLIED, ERROR, ABSOLUTE, ABSOLUTE, ABSOLUTE, ERROR,
    RELATIVE, ZERO_PAGE_INDIRECT_Y, ERROR, ERROR, ERROR, ZERO_PAGE_X, ZERO_PAGE_X, ERROR, IMPLIED, ABSOLUTE_Y, ERROR, ERROR, ERROR, ABSOLUTE_X, ABSOLUTE_X, ERROR,
    IMMEDIATE, ZERO_PAGE_X_INDIRECT, ERROR, ERROR, ZERO_PAGE, ZERO_PAGE, ZERO_PAGE, ERROR, IMPLIED, IMMEDIATE, IMPLIED, ERROR, ABSOLUTE, ABSOLUTE, ABSOLUTE, ERROR,
    RELATIVE, ZERO_PAGE_INDIRECT_Y, ERROR, ERROR, ERROR, ZERO_PAGE_X, ZERO_PAGE_X, ERROR, IMPLIED, ABSOLUTE_Y, ERROR, ERROR, ERROR, ABSOLUTE_X, ABSOLUTE_X, ERROR
};

void *inst_funcs[256] = {
    brk, ora, NULL, NULL, NULL, ora, asl_mem, NULL, php, ora, asl_a, NULL, NULL, ora, asl_mem, NULL,
    bpl, ora, NULL, NULL, NULL, ora, asl_mem, NULL, clc, ora, NULL, NULL, NULL, ora, asl_mem, NULL,
    jsr, and, NULL, NULL, bit, and, rol_mem, NULL, plp, and, rol_a, NULL, bit, and, rol_mem, NULL,
    bmi, and, NULL, NULL, NULL, and, rol_mem, NULL, sec, and, NULL, NULL, NULL, and, rol_mem, NULL,
    rti, eor, NULL, NULL, NULL, eor, lsr_mem, NULL, pha, eor, lsr_a, NULL, jmp, eor, lsr_mem, NULL,
    bvc, eor, NULL, NULL, NULL, eor, lsr_mem, NULL, cli, eor, NULL, NULL, NULL, eor, lsr_mem, NULL,
    rts, adc, NULL, NULL, NULL, adc, ror_mem, NULL, pla, adc, ror_a, NULL, jmp, adc, ror_mem, NULL,
    bvs, adc, NULL, NULL, NULL, adc, ror_mem, NULL, sei, adc, NULL, NULL, NULL, adc, ror_mem, NULL,
    NULL, sta, NULL, NULL, sty, sta, stx, NULL, dey, NULL, txa, NULL, sty, sta, stx,  NULL,
    bcc, sta, NULL, NULL, sty, sta, stx, NULL, tya, sta, txs, NULL, NULL, sta, NULL, NULL,
    ldy, lda, ldx, NULL, ldy, lda, ldx, NULL, tay, lda, tax, NULL, ldy, lda, ldx, NULL,
    bcs, lda, NULL, NULL, ldy, lda, ldx, NULL, clv, lda, tsx, NULL, ldy, lda, ldx, NULL,
    cpy, cmp, NULL, NULL, cpy, cmp, dec, NULL, iny, cmp, dex, NULL, cpy, cmp, dec, NULL,
    bne, cmp, NULL, NULL, NULL, cmp, dec, NULL, cld, cmp, NULL, NULL, NULL, cmp, dec, NULL,
    cpx, sbc, NULL, NULL, cpx, sbc, inc, NULL, inx, sbc, nop, NULL, cpx, sbc, inc, NULL,
    beq, sbc, NULL, NULL, NULL, sbc, inc, NULL, sed, sbc, NULL, NULL, NULL, sbc, inc, NULL
};

uint8_t get_instruction_size(uint8_t opcode) {
    if (opcode == 0x00) {
        // BRK
        return 7;
    }
    
    address_mode_t address_mode = inst_address_modes[opcode];
    switch (address_mode) {
        case IMPLIED:
        case ACCUMULATOR:
            return 1;
        case IMMEDIATE:
        case RELATIVE:
        case ZERO_PAGE:
        case ZERO_PAGE_X:
        case ZERO_PAGE_Y:
        case ZERO_PAGE_X_INDIRECT:
        case ZERO_PAGE_INDIRECT_Y:
            return 2;
        case ABSOLUTE:
        case ABSOLUTE_X:
        case ABSOLUTE_Y:
        case INDIRECT:
            return 3;
        default:
            return 0;
    }
}

void instruction_disassembly(uint16_t ip, char *buf, size_t buf_size, nes_t *nes) {
    uint8_t opcode = *mem(ip, nes, 0);

    char *mnemonic = inst_mnemonics[opcode];
    if (!mnemonic) {
        snprintf(buf, buf_size, "???");
        return;
    }
    
    address_mode_t address_mode = inst_address_modes[opcode];

    switch (address_mode) {
        case IMPLIED:
            snprintf(buf, buf_size, "%s", mnemonic);
            break;
        case ACCUMULATOR:
            snprintf(buf, buf_size, "%s A", mnemonic);
            break;
        case IMMEDIATE:
            snprintf(buf, buf_size, "%s #$%02X", mnemonic, *mem(ip + 1, nes, 0));
            break;
        case RELATIVE:
            snprintf(buf, buf_size, "%s $%04X", mnemonic, ip + (int8_t) *mem(ip + 1, nes, 0));
            break;
        case ZERO_PAGE:
            snprintf(buf, buf_size, "%s $%02X", mnemonic, *mem(ip + 1, nes, 0));
            break;
        case ZERO_PAGE_X:
            snprintf(buf, buf_size, "%s $%02X,X", mnemonic, *mem(ip + 1, nes, 0));
            break;
        case ZERO_PAGE_Y:
            snprintf(buf, buf_size, "%s $%02X,Y", mnemonic, *mem(ip + 1, nes, 0));
            break;
        case ZERO_PAGE_X_INDIRECT:
            snprintf(buf, buf_size, "%s ($%02X,X)", mnemonic, *mem(ip + 1, nes, 0));
            break;
        case ZERO_PAGE_INDIRECT_Y:
            snprintf(buf, buf_size, "%s ($%02X),Y", mnemonic, *mem(ip + 1, nes, 0));
            break;
        case ABSOLUTE:
            snprintf(buf, buf_size, "%s $%04X", mnemonic, *mem(ip + 1, nes, 0) | (((uint16_t) *mem(ip + 2, nes, 0)) << 8));
            break;
        case ABSOLUTE_X:
            snprintf(buf, buf_size, "%s $%04X,X", mnemonic, *mem(ip + 1, nes, 0) | (((uint16_t) *mem(ip + 2, nes, 0)) << 8));
            break;
        case ABSOLUTE_Y:
            snprintf(buf, buf_size, "%s $%04X,Y", mnemonic, *mem(ip + 1, nes, 0) | (((uint16_t) *mem(ip + 2, nes, 0)) << 8));
            break;
        case INDIRECT:
            snprintf(buf, buf_size, "%s ($%04X)", mnemonic, *mem(ip + 1, nes, 0) | (((uint16_t) *mem(ip + 2, nes, 0)) << 8));
            break;
        default:
            snprintf(buf, buf_size, "%s ???", mnemonic);
            break;
    }
}

void init_nes(rom_t *rom) {
	nes_t nes;
    
    nes.rom = rom;
    
    nes.sram.size = rom->prg_ram_size;

    nes.cpu.regfile.p = 0x34;
    nes.cpu.regfile.a = 0;
    nes.cpu.regfile.x = 0;
    nes.cpu.regfile.y = 0;
    nes.cpu.regfile.s = 0xFD;
    nes.cpu.regfile.pc = reset_vector(&nes);
    nes.cpu.cycles = 0;
    nes.cpu.just_branched = 0;

    mem_write(0x4017, 0, &nes);
    mem_write(0x4015, 0, &nes);
    for (uint16_t addr = 0x4000; addr <= 0x4013; ++addr) {
        mem_write(addr, 0, &nes);
    }

    nes.ppu.regfile.ctrl = 0;
    nes.ppu.regfile.mask = 0;
    nes.ppu.regfile.status = 0xC0;
    nes.ppu.regfile.oamaddr = 0;
    nes.ppu.regfile.scroll = 0;
    nes.ppu.regfile.addr = 0;
    nes.ppu.regfile.data = 0;

    nes.apu.regfile.status = 0;

    for (;;) {
        print_registers(&nes.cpu.regfile);

        uint16_t instruction_addr = nes.cpu.regfile.pc;
        uint8_t opcode = mem_read(instruction_addr, &nes);
        
        address_mode_t address_mode = inst_address_modes[opcode];
        uint16_t address;
        if (address_mode >= IMMEDIATE) {
            address = actual_address(address_mode, &nes);
        }

        char disassembly[16];
        instruction_disassembly(instruction_addr, disassembly, sizeof(disassembly), &nes);
        uint8_t instruction_size = get_instruction_size(opcode); 
        printf("%s (%d bytes)\n", disassembly, instruction_size);

        void *instruction_function = inst_funcs[opcode];
        if (!instruction_function) {
            fprintf(stderr, "Unknown opcode %02X\n", opcode);
        } else {
            if (address_mode >= IMMEDIATE) {
                ((void (*)(uint16_t, nes_t *)) instruction_function)(address, &nes);
            } else {
                ((void (*)(nes_t *)) instruction_function)(&nes);
            }
        }

        nes.cpu.regfile.pc += instruction_size;

        if (instruction_size == 1) {
            // Single-byte instructions burn a cycle reading the next byte
            ++nes.cpu.cycles;
        }
        if (address_mode == ZERO_PAGE_X || address_mode == ZERO_PAGE_Y || address_mode == ZERO_PAGE_X_INDIRECT) {
            // Burns a cycle reading the unindexed zero page address
            ++nes.cpu.cycles;
        }
        if (address_mode == ABSOLUTE_X || address_mode == ABSOLUTE_Y || address_mode == ZERO_PAGE_INDIRECT_Y) {
            if ((instruction_addr & 0xFF00) != (address & 0xFF00) || nes.cpu.just_wrote) {
                // Takes an extra cycle when crossing page boundary or writing
                ++nes.cpu.cycles;
            }
        }
        if (nes.cpu.just_branched && (opcode & 0x1F) == 0x10) {
            // Took a conditional branch
            // Branch takes a cycle
            ++nes.cpu.cycles;
            if ((nes.cpu.regfile.pc & 0xFF00) != (instruction_addr & 0xFF00)) {
                // Takes another cycle when crossing page boundary
                ++nes.cpu.cycles;
            }
        }

        nes.cpu.just_branched = 0;
        nes.cpu.just_wrote = 0;

#ifdef DEBUG
        usleep(500000);
        printf("\n");
#endif
    }
}

