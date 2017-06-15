/*
 *  isalib.h -- Main header file for ISA board support library
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *
 *  $Id: testisa.h,v 2.3 1997/05/06 14:00:42 acister Exp $
 */

#ifndef _TESTISA_H
#define _TESTISA_H

/*
 *  Defines & macros
 */

#define AMCC_ISA_AIMB1   (AMCC_ISA_BASE + AMCC_OP_REG_AIMB1)
#define AMCC_ISA_AIMB2   (AMCC_ISA_BASE + AMCC_OP_REG_AIMB2)
#define AMCC_ISA_AIMB3   (AMCC_ISA_BASE + AMCC_OP_REG_AIMB3)
#define AMCC_ISA_AIMB4   (AMCC_ISA_BASE + AMCC_OP_REG_AIMB4)
#define AMCC_ISA_AOMB1   (AMCC_ISA_BASE + AMCC_OP_REG_AOMB1)
#define AMCC_ISA_AOMB2   (AMCC_ISA_BASE + AMCC_OP_REG_AOMB2)
#define AMCC_ISA_AOMB3   (AMCC_ISA_BASE + AMCC_OP_REG_AOMB3)
#define AMCC_ISA_AOMB4   (AMCC_ISA_BASE + AMCC_OP_REG_AOMB4)

#define AMCC_ISA_AFIFO   (AMCC_ISA_BASE + AMCC_OP_REG_AFIFO)
#define AMCC_ISA_AGCSTS  (AMCC_ISA_BASE + AMCC_OP_REG_AGCSTS)

#define BUSY_WAIT()     { while ((inl (AMCC_ISA_AGCSTS)) & AGCSTS_FS_P2A_EMPTY) ; }
#define FIFO_EMPTY()    ((inl (AMCC_ISA_AGCSTS)) & AGCSTS_FS_P2A_EMPTY)

/*
 *  Variables
 */

/*
 *  Prototypes
 */

extern int strtol_e (char *, char, int);

/*
 *  Inline functions
 */

/*
 *  read_fifo ()
 *  Read a single DWORD from the Add-on To PCI FIFO.
 */

extern inline __u32
read_fifo (void)
{
    /* this hack is necessary because the ISA bus is 16 bit wide */

    __u32 data;

    data = ((__u32) inw (AMCC_ISA_AFIFO + 2)) << 16;
    data |= (__u32) inw (AMCC_ISA_AFIFO);

    return data;
}

/*
 *  write_fifo ()
 *  Write in the Add-on To PCI FIFO. Size is in __u32 units.
 */

extern inline void
write_fifo (__u32 *buf, int size)
{
    int index = 0L;
    __u16 data;

    while (size) {

        data = (buf[index] & 0xffff0000) >> 16;
        outw (data, (AMCC_ISA_AFIFO + 2));

        data = (__u16) (buf[index] & 0x0000ffff);
        outw (data, AMCC_ISA_AFIFO);

        index++;
        size--;
    }
}

#endif  /* _TESTISA_H */
