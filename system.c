#include <stdint.h>

#define OAMDATA 0x2004
#define OAMDMA 0x4014

typedef struct {
	uint8_t a;
	uint8_t x;
	uint8_t y;
	uint8_t p;
	uint8_t s;
	uint16_t pc;
} cpu_regfile_t;

typedef struct {
	cpu_regfile_t regfile;
	uint8_t ram[2048];
} cpu_t;

typedef struct {
	uint8_t ctrl;
	uint8_t mask;
	uint8_t status;
	uint8_t oamaddr;
	uint8_t oamdata;
	uint8_t scroll;
	uint8_t addr;
	uint8_t data;
	uint8_t oamdma;
} ppu_regfile_t;

typedef struct {
	ppu_regfile_t regfile;
	uint8_t ram[2048];
} ppu_t;

typedef struct {
	uint8_t pulse[2][4];
	uint8_t triangle[3];
	uint8_t noise[4];
	uint8_t dmc[4];
	uint8_t status;
	uint8_t frame;
} apu_regfile_t;

typedef struct {
	apu_regfile_t regfile;
} apu_t;

typedef struct {
	size_t size;
	uint8_t *mem;
} sram_t;

typedef struct {
	cpu_t cpu;
	ppu_t ppu;
	apu_t apu;
	rom_t *rom;
	sram_t sram;
} nes_t;

uint8_t *mapper0(uint8_t addr, nes_t *nes) {
	if (addr >= 0x6000 && addr < 0x8000) {
		if (nes.sram.size < 1) {
			fprintf(stderr, "no sram\n");
			return NULL;
		}
		return nes.sram.mem[(addr - 0x6000) % nes.sram.size];
	} else {
		if (nes.rom->prg_size != 16384 && nes.rom->prg_size != 32768) {
			fprintf(stderr, "prg_size %d, not 16K or 32K\n", nes.rom->prg_size);
			return NULL;
		}
		return nes.rom->prg[(addr - 0x8000) % nes.rom->prg_size];
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

uint8_t *mem(uint8_t addr, nes_t *nes, int write) {
	if (addr < 0x2000) {
		return &nes.cpu.ram[addr % 0x800];
	} else if (addr < 0x4000) {
		return &nes.ppu.regfile[addr % 0x8];
	} else if (addr < 0x4014) {
		return &nes.apu.regfile[addr - 0x4000];
	} else if (addr == 0x4014) {
		return &nes.ppu.regfile.oamdma;
	} else if (addr == 0x4015) {
		return &nes.apu.regfile.status;
	} else if (addr == 0x4016) {
		// TODO JOY1
		return NULL;
	} else if (addr == 0x4017) {
		if (write) {
			return &nes.apu.regfile.frame;
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
		mapper_t m = mapper(nes.rom);
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

