# Printed Curcuit Board Projects

This repository contains various printed circuit board projects. Also includes any custom firmware where applicable.

## parts

This is a git submodule pointing to https://github.com/jrseti/parts.

The contains pcb related symbols, footprints and 3d files for KiCad PCB projects. All KiCAD projects in this repository will use parts that are in these libraries. 

During development if a part is not in this library, add it. See directions in the parts/READM.md file.

Make sure it is always updated to the latest:

- git submodule update --init --recursive

# mcu_test_board

This is a PCB project to create a board that has a STM32 MCU that runs a PID loop. Once this is debugged and working the design will be inproved in a more encompaing board.

