# NES Emulator
This is a work-in-progress NES emulator written in C.  

So far, only a 6502 emulator is implemented.

## Dependencies
Requires SDL2 (not currently used for anything).

## Building
CMake is used for building.
```
$ git clone https://github.com/kylemclean/nes-emulator.git
$ cd nes-emulator
$ mkdir build
$ cd build
$ cmake ..
$ make
```
The executable `NESEmulator` will be created in the build directory.
