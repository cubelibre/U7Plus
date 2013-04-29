#!/bin/sh


pngtopnm ./logo.png > logo.pnm
pnmquant 224 logo.pnm > logo_clut224.pnm
pnmtoplainpnm logo_clut224.pnm > logo_clut224.ppm
rm -rf *.pnm

#	rm -rf *.pnm
#	./png/pnmtologo -t clut224 -n logo_charge${var}_clut224 -o logo_charge${var}_clut224.c logo_charge${var}_clut224.ppm
#	rm -rf *.ppm
#	cat logo_charge${var}_clut224.c >> logo_charge_clut224.c
#	rm -rf logo_charge${var}_clut224.c

