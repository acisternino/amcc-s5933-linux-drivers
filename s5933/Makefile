#
# Main Makefile for the AMCC S5933 PCI Matchmaker test driver
#
# (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
#
# $Id: Makefile,v 2.8 1997/09/23 10:54:40 acister Exp $

CC = gcc

WARN = -Wall -Winline -Wno-format

DEFINES = -DMODULE -DDEBUG
COPT = -O -m486

# Use the following for a less verbose and faster module
# DEFINES = -DMODULE
# COPT = -O2 -m486 -fomit-frame-pointer

CFLAGS = $(COPT) $(WARN) $(DEFINES)

# Files

SRCS = main.c
HEADERS = amcc.h main.h
MODULE = amcc.o

# Main target

all: $(MODULE) test tags

# Generic Rules

%.o: %.c
	$(CC) -c $(CFLAGS) $<

# Driver

$(MODULE): $(SRCS)
	$(CC) $(CFLAGS) -o $@ -c $<

# Dependancies

$(MODULE): $(HEADERS)

# Test

.PHONY: test
test:
	$(MAKE) -C test

# tags

tags: $(SRCS) $(HEADERS)
	ctags -dT $^

# Misc

.PHONY: clean
clean:
	rm -f *.o *~ ~* *.bck core tags
	$(MAKE) -C test clean
