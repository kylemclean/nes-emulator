#include <stdint.h>
#include <stdio.h>

#include "system.h"

uint8_t *mapper0(uint8_t addr, nes_t *nes) {
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

typedef uint8_t *(*mapper_func_t)(uint8_t, nes_t *);

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

uint8_t *ppu_reg(uint8_t addr, ppu_t *ppu) {
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
	fprintf(stderr, "Invalid PPU register address %04X\n", addr);
	return NULL;
}

uint8_t *apu-reg(uint8_t addr, apu_t *apu) {
	if (addr >= 0x4000 && addr <= 0x4013) {
		// TODO...

uint8_t *mem(uint8_t addr, nes_t *nes, int write) {
	if (addr < 0x2000) {
		return &nes->cpu.ram[addr % 0x800];
	} else if (addr < 0x4000) {
		return ppu_reg(addr, &nes->ppu);
	} else if (addr < 0x4014) {
		return apu_reg(addr, &nes->apu);
	} else if (addr == 0x4014) {
		return ppu_reg(addr, &nes->ppu);
	} else if (addr == 0x4015) {
		return &nes->apu.regfile.status;
	} else if (addr == 0x4016) {
		// TODO JOY1
		return NULL;
	} else if (addr == 0x4017) {
		if (write) {
			return &nes->apu.regfile.frame;
		} else {
			// TODO JOY2
			return NULL;
		}
	} else if (addr < 0x4020) {
		// TODO cpu test mode
		return NULL;
	} else if (addr < 0x6000) {
		// TODO cartridge expansion ROM
		return NULL;
	} else {
		mapper_t m = mapper(nes->rom);
		mapper_func_t mf = mapper_func(m);
		return mf(addr, nes);
	}
}

uint8_t mem_read(uint8_t addr, nes_t *nes) {
	return *mem(addr, nes, 0);
}

void mem_write(uint8_t addr, uint8_t value, nes_t *nes) {
	*mem(addr, nes, 1) = value;
}

void run_instruction(cpu_t *cpu, rom_t *rom) {
	uint8_t opcode = 
}

void run(rom_t *rom) {
	cpu_t cpu;


}

