##
## Main Makefile for the AutoBahn test driver
##
## (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
##          Toni Giorgino      (toni@pcape2.pi.infn.it)
##
## Makefile,v 1.2 1997/10/27 16:15:39 acister Exp

## Flags

CC = gcc

WARN = -Winline -Wno-format
DEFINES = -DMODULE
CCOPT = -O -m486

## Use the following for a less verbose and faster module
## DEFINES = -DMODULE
## COPT = -O2 -m486 -fomit-frame-pointer

CFLAGS = $(CCOPT) $(WARN) $(DEFINES)

MODULES = ab.o

## Main target

all: $(MODULES)

## Generic Rules

%.o: %.c
	$(CC) -c $(CFLAGS) $<

## AutoBahn driver

ab.o: ab.c ab.h s5933.h

## Tags

.PHONY: tags
tags:
	ctags -dT *.c *.h
	etags *.c *.h

## Misc

.PHONY: clean
clean:
	rm -f *.o *~ ~* *.bck core tags TAGS
