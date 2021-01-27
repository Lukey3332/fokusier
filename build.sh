#!/bin/bash
gcc -Os -I ../cwiid/libcwiid -L ../cwiid/libcwiid/ -lcwiid -lm main.c -o fokusier
