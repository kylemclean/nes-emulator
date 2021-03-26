#ifndef _SYSTEM_H
#define _SYSTEM_H

#include <stddef.h>

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
	uint8_t header[16];
	uint8_t *trainer;
	size_t prg_size;
	uint8_t *prg;
	size_t chr_size;
	uint8_t *chr;
    size_t prg_ram_size;
} rom_t;

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

#endif // _SYSTEM_H
