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
	return *mem(addr, nes, 0);
}

void mem_write(uint16_t addr, uint8_t value, nes_t *nes) {
	*mem(addr, nes, 1) = value;
}

void push(uint8_t value, nes_t *nes) {
    mem_write(nes->cpu.regfile.s, value, nes);
    nes->cpu.regfile.s -= 1;
}

uint8_t pull(nes_t *nes) {
    nes->cpu.regfile.s += 1;
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

void asl(uint8_t *target, nes_t *nes) {
    change_carry(*target >> 7, nes);
    *target <<= 1;
    change_negative(*target >> 7, nes);
    change_zero(*target == 0, nes);
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

void dec(uint8_t *target, nes_t *nes) {
    *target -= 1;
    change_negative(*target >> 7, nes);
    change_zero(*target == 0, nes);
}

void dex(nes_t *nes) {
    dec(&nes->cpu.regfile.x, nes);
}

void dey(nes_t *nes) {
    dec(&nes->cpu.regfile.y, nes);
}

void eor(uint8_t arg, nes_t *nes) {
    nes->cpu.regfile.a ^= arg;
    change_negative(nes->cpu.regfile.a >> 7, nes);
    change_zero(nes->cpu.regfile.a == 0, nes);
}

void inc(uint8_t *target, nes_t *nes) {
    *target += 1;
    change_negative(*target >> 7, nes);
    change_zero(*target == 0, nes);
}

void inx(nes_t *nes) {
    inc(&nes->cpu.regfile.x, nes);
}

void iny(nes_t *nes) {
    inc(&nes->cpu.regfile.y, nes);
}

void jmp(uint16_t target, nes_t *nes) {
    nes->cpu.regfile.pc = target;
}

void jsr(uint16_t target, nes_t *nes) {
    uint16_t pc_plus_2 = nes->cpu.regfile.pc + 2;
    push((uint8_t) (pc_plus_2 >> 8), nes);
    push((uint8_t) pc_plus_2, nes);
    nes->cpu.regfile.pc = target;
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

void lsr(uint8_t *target, nes_t *nes) {
    change_carry(*target & 1, nes);
    *target >>= 1;
    change_negative(0, nes);
    change_zero(*target == 0, nes);
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

void rol(uint8_t *target, nes_t *nes) {
    uint8_t new_carry = *target >> 7;
    *target = (*target << 1) | get_carry(nes);
    change_negative(*target >> 7, nes);
    change_zero(*target == 0, nes);
    change_carry(new_carry, nes);
}

void ror(uint8_t *target, nes_t *nes) {
    uint8_t new_carry = *target & 1;
    *target = (*target >> 1) | (get_carry(nes) << 7);
    change_negative(*target >> 7, nes);
    change_zero(*target == 0, nes);
    change_carry(new_carry, nes);
}

void rti(nes_t *nes) {
    nes->cpu.regfile.p = pull(nes);
    nes->cpu.regfile.pc = pull(nes) | (pull(nes) << 8);
}

void rts(nes_t *nes) {
    nes->cpu.regfile.pc = (pull(nes) | (pull(nes) << 8)) + 1;
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

void sta(uint8_t *target, nes_t *nes) {
    *target = nes->cpu.regfile.a;
}

void stx(uint8_t *target, nes_t *nes) {
    *target = nes->cpu.regfile.x;
}

void sty(uint8_t *target, nes_t *nes) {
    *target = nes->cpu.regfile.y;
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
        uint8_t opcode = mem_read(nes.cpu.regfile.pc, &nes);
        
        if ((opcode & 0x3) == 1) {
            switch (opcode >> 5) {
                case 0:
                                 
            }
        }
    }
}

