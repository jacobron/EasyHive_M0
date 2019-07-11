#!/bin/bash

openocd -f sodaq_sff.cfg -c "program progHiveM0-Sketch.ino.sodaq_sff.bin verify reset exit 0x00000000"

