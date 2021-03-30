#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <SDL.h>

#include "system.h"

int create_rom(FILE *file, rom_t *rom) {
  if (fseek(file, 0, SEEK_SET) != 0) {
    perror("fseek");
    return -1;
  }

  size_t bytes_read = 0;

  uint8_t *header = rom->header;
  bytes_read = fread(header, 1, sizeof(rom->header), file);
  if (bytes_read != sizeof(rom->header)) {
    fprintf(stderr, "Failed to read header\n");
    return -1;
  }

  if (header[0] != 0x4E || header[1] != 0x45 || header[2] != 0x53 || header[3] != 0x1A) {
    fprintf(stderr, "Invalid magic number %02X %02X %02X %02X\n", header[0], header[1], header[2], header[3]);
    return -1;
  }

  bool is_nes_2_0 = (header[7] & 0xC) == 0x8;
  if (is_nes_2_0) {
    fprintf(stderr, "NES 2.0 ROMs are not currently supported\n");
    return -1;
  }

  bool has_trainer = header[6] & 0x4;
  if (has_trainer) {
    size_t trainer_size = 512;
    rom->trainer = malloc(trainer_size);
    bytes_read = fread(rom->trainer, 1, trainer_size, file);
    if (bytes_read != trainer_size) {
      fprintf(stderr, "Failed to read trainer\n");
      return -1;
    }
  } else {
    rom->trainer = NULL;
  }

  rom->prg_size = header[4] * 16384;
  rom->prg = malloc(rom->prg_size);
  bytes_read = fread(rom->prg, 1, rom->prg_size, file);
  if (bytes_read != rom->prg_size) {
    fprintf(stderr, "Failed to read PRG ROM data\n");
    return -1;
  }

  rom->chr_size = header[5] * 8192;
  rom->chr = malloc(rom->chr_size);
  bytes_read = fread(rom->chr, 1, rom->chr_size, file);
  if (bytes_read != rom->chr_size) {
    fprintf(stderr, "Failed to read CHR ROM data\n");
    return -1;
  }

  rom->prg_ram_size = header[8] * 8192;
  if (rom->prg_ram_size == 0) {
    rom->prg_ram_size = 8192;
  }

  bool is_playchoice_10 = header[7] & 0x2;
  if (is_playchoice_10) {
    printf("Ignoring Playchoice 10 data\n");
    if (fseek(file, 8192 + 16 + 16, SEEK_CUR) != 0) {
      perror("fseek");
      return -1;
    }
  }

  long end_pos = ftell(file);
  if (end_pos == -1L) {
    perror("ftell");
    return -1;
  }

  if (fseek(file, 0, SEEK_END) != 0) {
    perror("fseek");
    return -1;
  }

  long eof_pos = ftell(file);
  if (eof_pos != end_pos) {
    fprintf(stderr, "Warning: %ld extra bytes at end of file\n", eof_pos - end_pos);
    return 0;
  }

  return 0;
}

void destroy_rom(rom_t *rom) {
  if (rom->trainer) {
    free(rom->trainer);
  }
  if (rom->prg) {
    free(rom->prg);
  }
  if (rom->chr) {
    free(rom->chr);
  }
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s <rom-file>\n", argv[0]);
    return 1;
  }

  const char *path = argv[1];
  FILE *file = fopen(path, "rb");
  if (file == NULL) {
    perror("fopen");
    fprintf(stderr, "Failed to open file\n");
    return -1;
  }

  rom_t rom;
  if (create_rom(file, &rom) != 0) {
    fprintf(stderr, "Failed to read ROM\n");
    return 1;
  }

  printf("trainer: %d\nprg size: %zu\nchr size: %zu\n", rom.trainer != NULL, rom.prg_size, rom.chr_size);

  init_nes(&rom);
  return 0;

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) {
    fprintf(stderr, "Failed to initialize SDL: %s\n", SDL_GetError());
    return 1;
  }

  SDL_Window *window = SDL_CreateWindow("NES Emulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 640, 480, 0);
  if (window == NULL) {
    fprintf(stderr, "Failed to create SDL window: %s\n", SDL_GetError());
    return 1;
  }

  SDL_Surface *surface = SDL_GetWindowSurface(window);

  if (SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0x00, 0x40, 0xFF)) < 0) {
    fprintf(stderr, "Failed to fillrect: %s\n", SDL_GetError());
    return 1;
  }

  SDL_UpdateWindowSurface(window);

  for (;;)
    SDL_Delay(1000);

  SDL_DestroyWindow(window);

  SDL_Quit();

  return 0;
}
