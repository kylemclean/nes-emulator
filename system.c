#include <stdint.h>
#include <stdio.h>

#include "system.h"

uint8_t *mapper0(uint16_t addr, nes_t *nes) {
	if (addr >= 0x6000 && addr < 0x8000) {
		if (nes->sram.size < 1) {
			fprintf(stderr, "no sram\n");
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
	++nes->cpu.cycles;
    return *mem(addr, nes, 0);
}

void mem_write(uint16_t addr, uint8_t value, nes_t *nes) {
    ++nes->cpu.cycles;
    nes->cpu.just_wrote = 1;
    *mem(addr, nes, 1) = value;
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

void adc(uint8_t arg, nes_t *nes) {
    uint8_t a = nes->cpu.regfile.a;
    uint16_t sum = a + arg + get_carry(nes);
    change_negative((sum >> 7) & 1, nes);
    change_overflow(~(a ^ arg) & (a ^ sum) & 0x80, nes);
    change_zero(sum == 0, nes);
    change_carry(sum > 0xFF, nes);
    nes->cpu.regfile.a = (uint8_t) sum;
}

void and(uint8_t arg, nes_t *nes) {
    nes->cpu.regfile.a &= arg;
    change_negative(nes->cpu.regfile.a >> 7, nes);
    change_zero(nes->cpu.regfile.a == 0, nes);
}

uint8_t asl(uint8_t old, nes_t *nes) {
    uint8_t new = old << 1;
    change_negative(new >> 7, nes);
    change_zero(new == 0, nes);
    change_carry(old >> 7, nes);
    return new;
}

void asl_a(nes_t *nes) {
    nes->cpu.regfile.a = asl(nes->cpu.regfile.a, nes);
}

void asl_mem(uint16_t addr, nes_t *nes) {
    mem_write(addr, asl(mem_read(addr, nes), nes), nes);
    ++nes->cpu.cycles;
}

void bcc(uint8_t arg, nes_t *nes) {
    if (!get_carry(nes)) {
        nes->cpu.regfile.pc += (int8_t) arg;
    }
}

void bcs(uint8_t arg, nes_t *nes) {
    if (get_carry(nes)) {
        nes->cpu.regfile.pc += (int8_t) arg;
    }
}

void beq(uint8_t arg, nes_t *nes) {
    if (get_zero(nes)) {
        nes->cpu.regfile.pc += (int8_t) arg;
    }
}

void bit(uint8_t arg, nes_t *nes) {
    change_negative(arg >> 7, nes);
    change_overflow((arg >> 6) & 1, nes);
    change_zero((arg & nes->cpu.regfile.a) == 0, nes);
}

void bmi(uint8_t arg, nes_t *nes) {
    if (get_negative(nes)) {
        nes->cpu.regfile.pc += (int8_t) arg;
    }
}

void bne(uint8_t arg, nes_t *nes) {
    if (!get_zero(nes)) {
        nes->cpu.regfile.pc += (int8_t) arg;
    }
}

void bpl(uint8_t arg, nes_t *nes) {
    if (!get_negative(nes)) {
        nes->cpu.regfile.pc += (int8_t) arg;
    }
}

void brk(nes_t *nes) {
    push((uint8_t) (nes->cpu.regfile.pc >> 8), nes);
    push((uint8_t) nes->cpu.regfile.pc, nes);
    push(nes->cpu.regfile.p, nes);
    nes->cpu.regfile.pc = irq_vector(nes);
    nes->cpu.just_branched = 1;
}

void bvc(uint8_t arg, nes_t *nes) {
    if (!get_overflow(nes)) {
        nes->cpu.regfile.pc += (int8_t) arg;
    }
}

void bvs(uint8_t arg, nes_t *nes) {
    if (get_overflow(nes)) {
        nes->cpu.regfile.pc += (int8_t) arg;
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

void cmp(uint8_t arg, nes_t *nes) {
    compare(nes->cpu.regfile.a, arg, nes);
}
   
void cpx(uint8_t arg, nes_t *nes) {
    compare(nes->cpu.regfile.x, arg, nes);
}

void cpy(uint8_t arg, nes_t *nes) {
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

void eor(uint8_t arg, nes_t *nes) {
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

void load(uint8_t *target, uint8_t arg, nes_t *nes) {
    *target = arg;
    change_negative(*target >> 7, nes);
    change_zero(*target == 0, nes);
}

void lda(uint8_t arg, nes_t *nes) {
    load(&nes->cpu.regfile.a, arg, nes);   
}

void ldx(uint8_t arg, nes_t *nes) {
    load(&nes->cpu.regfile.x, arg, nes);
}

void ldy(uint8_t arg, nes_t *nes) {
    load(&nes->cpu.regfile.y, arg, nes);
}

uint8_t lsr(uint8_t old, nes_t *nes) {
    uint8_t new = old >> 1;
    change_negative(0, nes);
    change_zero(new == 0, nes);
    change_carry(old & 1, nes);
    return new;
}

void lsr_a(nes_t *nes) {
    nes->cpu.regfile.a = lsr(nes->cpu.regfile.a, nes);
}

void lsr_mem(uint16_t addr, nes_t *nes) {
    mem_write(addr, lsr(mem_read(addr, nes), nes), nes);
    ++nes->cpu.cycles;
}

void nop(nes_t *nes) {

}

void ora(uint8_t arg, nes_t *nes) {
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

uint8_t rol(uint8_t old, nes_t *nes) {
    uint8_t new = (old << 1) | get_carry(nes);
    change_negative(new >> 7, nes);
    change_zero(new == 0, nes);
    change_carry(old >> 7, nes);
    return new;
}

void rol_a(nes_t *nes) {
    nes->cpu.regfile.a = rol(nes->cpu.regfile.a, nes);
}

void rol_mem(uint16_t addr, nes_t *nes) {
    mem_write(addr, rol(mem_read(addr, nes), nes), nes);
    ++nes->cpu.cycles;
}

uint8_t ror(uint8_t old, nes_t *nes) {
    uint8_t new = (old >> 1) | (get_carry(nes) << 7);
    change_negative(new >> 7, nes);
    change_zero(new == 0, nes);
    change_carry(old & 1, nes);
    return new;
}

void ror_a(nes_t *nes) {
    nes->cpu.regfile.a = ror(nes->cpu.regfile.a, nes);
}

void ror_mem(uint16_t addr, nes_t *nes) {
    mem_write(addr, ror(mem_read(addr, nes), nes), nes);
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

void sbc(uint8_t arg, nes_t *nes) {
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

address_mode_t get_address_mode(uint8_t opcode) {
    if (opcode == 0x20) {
        // JSR
        return ABSOLUTE;
    }
    uint8_t aaa = opcode >> 5;
    uint8_t bbb = ((opcode >> 2) & 7);
    uint8_t cc = opcode & 3;
    if (cc == 1) {
        switch (bbb) {
            case 0:
                return ZERO_PAGE_X_INDIRECT;
            case 1:
                return ZERO_PAGE;
            case 2:
                return IMMEDIATE;
            case 3:
                return ABSOLUTE;
            case 4:
                return ZERO_PAGE_INDIRECT_Y;
            case 5:
                return ZERO_PAGE_X;
            case 6:
                return ABSOLUTE_X;
            case 7:
                return ABSOLUTE_Y;
        }
    }
    if (cc == 2) {
        switch (bbb) {
            case 0:
                return IMMEDIATE;
            case 1:
                return ZERO_PAGE;
            case 2:
                return ACCUMULATOR;
            case 3:
                return ABSOLUTE;
            case 5:
                return (aaa == 4 || aaa == 5) ? ZERO_PAGE_Y : ZERO_PAGE_X;
            case 7:
                return (aaa == 4 || aaa == 5) ? ABSOLUTE_Y : ABSOLUTE_X;
        }
    }
    if (cc == 0) {
        switch (bbb) {
            case 0:
                return IMMEDIATE;
            case 1:
                return ZERO_PAGE;
            case 3:
                return ABSOLUTE;
            case 5:
                return ZERO_PAGE_X;
            case 7:
                return ABSOLUTE_X;
        }
    }
    return ERROR;
}

uint16_t actual_address(address_mode_t address_mode, nes_t *nes) {
    uint16_t argp = nes->cpu.regfile.pc + 1;
    switch (address_mode) {
        case ACCUMULATOR:
        case IMPLIED:
            return 0; // not relevant
        case IMMEDIATE:
            return argp;
        case RELATIVE:
            return nes->cpu.regfile.pc + mem_read(argp, nes);
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

uint8_t get_instruction_size(uint8_t opcode) {
    if (opcode == 0x00) {
        // BRK
        return 7;
    }
    
    address_mode_t address_mode = get_address_mode(opcode);
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

void run(rom_t *rom) {
	nes_t nes;
    
    nes.rom = rom;
    
    nes.sram.size = rom->prg_ram_size;

    nes.cpu.regfile.s = 0x34;
    nes.cpu.regfile.a = 0;
    nes.cpu.regfile.x = 0;
    nes.cpu.regfile.y = 0;
    nes.cpu.regfile.p = 0xFD;
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
        uint16_t instruction_addr = nes.cpu.regfile.pc;
        uint8_t opcode = mem_read(instruction_addr, &nes);
        
        address_mode_t address_mode = get_address_mode(opcode);
        uint16_t address;
        if (address_mode >= IMMEDIATE) {
            address = actual_address(address_mode, &nes);
        }

        uint8_t instruction_size = get_instruction_size(opcode); 

        if ((opcode & 0x3) == 1) {
            switch (opcode >> 5) {
                case 0:
                    ora(mem_read(address, &nes), &nes);
                    break;
                case 1:
                    and(mem_read(address, &nes), &nes);
                    break;
                case 2:
                    eor(mem_read(address, &nes), &nes);
                    break;
                case 3:
                    adc(mem_read(address, &nes), &nes);
                    break;
                case 4:
                    sta(address, &nes);
                    break;
                case 5:
                    lda(mem_read(address, &nes), &nes);
                    break;
                case 6:
                    cmp(mem_read(address, &nes), &nes);
                    break;
                case 7:
                    sbc(mem_read(address, &nes), &nes);
                    break;
            }
        } else if ((opcode & 0x3) == 2) {
            switch (opcode >> 5) {
                case 0: {
                    if (address_mode == ACCUMULATOR) {
                        asl_a(&nes);
                    } else {
                        asl_mem(address, &nes);
                    }
                    break;
                }
                case 1: {
                    if (address_mode == ACCUMULATOR) {
                        rol_a(&nes);
                    } else {
                        rol_mem(address, &nes);
                    }
                    break;
                }
                case 2: {
                    if (address_mode == ACCUMULATOR) {
                        lsr_a(&nes);
                    } else {
                        lsr_mem(address, &nes);
                    }
                    break;
                }
                case 3: {
                    if (address_mode == ACCUMULATOR) {
                        ror_a(&nes);
                    } else {
                        ror_mem(address, &nes);
                    }
                    break;
                }
                case 4:
                    stx(address, &nes);
                    break;
                case 5:
                    ldx(mem_read(address, &nes), &nes);
                    break;
                case 6:
                    dec(address, &nes);
                    break;
                case 7:
                    inc(address, &nes);
                    break;
            }
        } else if ((opcode & 0x1) == 0) {
            switch (opcode >> 3) {
                case 1:
                    bit(mem_read(address, &nes), &nes);
                    break;
                case 2:
                case 3:
                    jmp(address, &nes);
                    break;
                case 4:
                    sty(address, &nes);
                    break;
                case 5:
                    ldy(mem_read(address, &nes), &nes);
                    break;
                case 6:
                    cpy(mem_read(address, &nes), &nes);
                    break;
                case 7:
                    cpx(mem_read(address, &nes), &nes);
                    break;
            }
        } else {
            switch (opcode) {
                case 0x10:
                    bpl(mem_read(address, &nes), &nes);
                    break;
                case 0x30:
                    bmi(mem_read(address, &nes), &nes);
                    break;
                case 0x50:
                    bvc(mem_read(address, &nes), &nes);
                    break;
                case 0x70:
                    bvs(mem_read(address, &nes), &nes);
                    break;
                case 0x90:
                    bcc(mem_read(address, &nes), &nes);
                    break;
                case 0xB0:
                    bcs(mem_read(address, &nes), &nes);
                    break;
                case 0xD0:
                    bne(mem_read(address, &nes), &nes);
                    break;
                case 0xF0:
                    beq(mem_read(address, &nes), &nes);
                    break;
                case 0x00:
                    brk(&nes);
                    break;
                case 0x20:
                    jsr(address, &nes);
                    break;
                case 0x40:
                    rti(&nes);
                    break;
                case 0x60:
                    rts(&nes);
                    break;
                case 0x08:
                    php(&nes);
                    break;
                case 0x28:
                    plp(&nes);
                    break;
                case 0x48:
                    pha(&nes);
                    break;
                case 0x68:
                    pla(&nes);
                    break;
                case 0x88:
                    dey(&nes);
                    break;
                case 0xA8:
                    tay(&nes);
                    break;
                case 0xC8:
                    iny(&nes);
                    break;
                case 0xE8:
                    inx(&nes);
                    break;
                case 0x18:
                    clc(&nes);
                    break;
                case 0x38:
                    sec(&nes);
                    break;
                case 0x58:
                    cli(&nes);
                    break;
                case 0x78:
                    sei(&nes);
                    break;
                case 0x98:
                    tya(&nes);
                    break;
                case 0xB8:
                    clv(&nes);
                    break;
                case 0xD8:
                    cld(&nes);
                    break;
                case 0xF8:
                    sed(&nes);
                    break;
                case 0x8A:
                    txa(&nes);
                    break;
                case 0x9A:
                    txs(&nes);
                    break;
                case 0xAA:
                    tax(&nes);
                    break;
                case 0xBA:
                    tsx(&nes);
                    break;
                case 0xCA:
                    dex(&nes);
                    break;
                case 0xEA:
                    nop(&nes);
                    break;
            }
        }
        
        if (!nes.cpu.just_branched) {
            nes.cpu.regfile.pc += instruction_size;
        }

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
    }
}

