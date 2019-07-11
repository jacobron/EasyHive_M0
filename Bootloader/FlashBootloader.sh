#!/bin/bash

openocd -f sodaq_sff.cfg -c "program samd21_sa_one.bin verify reset exit 0x00000000"

