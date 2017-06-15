#
# Makefile for the device driver test programs.
#
# (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
#
# $Id: Makefile,v 2.5 1997/10/01 08:27:52 acister Exp $

# Paths

VPATH = lib:..

# Flags

CC = gcc

WARN = -Wall -Wno-format
COPTS = -g -O -m486
DEFINES = -DDEBUG -D__ISA__
INCDIR = -I..

CFLAGS = $(WARN) $(COPTS) $(DEFINES) $(INCDIR)

# Rules

%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

# Variables

BINS = pfifo ififo pmb imb # ppt
SRCS = pfifo.c ififo.c pmb.c imb.c ppt.c strtol_e.c
OBJS = $(SRCS:%.c=%.o)

# Main target

all: $(BINS)

# Dependancies

$(OBJS): amcc.h
$(BINS): lib/strtol_e.o

pfifo.o: testpci.h
pfifo: pfifo.o

ififo.o: testisa.h
ififo: ififo.o

pmb.o: testpci.h main.h
pmb: pmb.o

imb.o: testisa.h main.h
imb: imb.o

ppt.o: ppt.c main.h
	$(CC) $(WARN) -g -m486 $(DEFINES) $(INCDIR) -c $<
ppt: ppt.o

# Misc

.PHONY: clean
clean:
	rm -f *.o *~ ~* *.bck core
	rm -f $(BINS)
	cd lib && rm -f *.o *~ ~* *.bck core
