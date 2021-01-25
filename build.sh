#!/bin/bash
gcc -I ../cwiid/libcwiid -L ../cwiid/libcwiid/ -lcwiid -lm main.c -o fokusier
