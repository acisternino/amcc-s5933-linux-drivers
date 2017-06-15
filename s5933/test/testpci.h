/*
 *  testpci.h -- Main header file for the PCI support library
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *
 *  $Id: testpci.h,v 2.1 1997/04/21 15:14:52 acister Exp $
 */

#ifndef _TESTPCI_H
#define _TESTPCI_H

/*
 *  Defines & macros
 */

/*
 *  Prototypes
 */

extern int  strtol_e (char *, char, int);

/*
 *  Inline functions
 */

/**
 *  Read function for test programs.
 *  It reads a single DWORD from the FIFO.
 */

static inline __u32
read_fifo (int fd, __u32 *data)
{
    return (read (fd, data, sizeof (__u32)));
}

#endif  /* _TESTPCI_H */
