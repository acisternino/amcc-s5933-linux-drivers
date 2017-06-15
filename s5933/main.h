/*
 *  main.h -- Simple AMCC S5933 PCI Matchmaker driver
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *
 *  $Id: main.h,v 2.11 1997/09/23 10:52:28 acister Exp $
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 2, or (at
 *  your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the
 *  Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139,
 *  USA.
 */

/*--------------------------------------------------------------------
 *
 *  Main Include file
 *  This file contains all the macros and defines needed by the driver
 *  and user programs using it.
 *
 *--------------------------------------------------------------------*/

#ifndef _MAIN_H
#define _MAIN_H

/*-- Support macros --------------------------------------------------*/

/* for AMCCIOSETOMBIRQ and AMCCIOSETIMBIRQ ioctls */

#define OMBIRQ(mb,byte)  (((byte) & 0x00000003) | ((((mb) - 1) << 2) & 0x0000000C))
#define IMBIRQ(mb,byte)  ((((byte) << 8) & 0x00000300) | ((((mb) - 1) << 10) & 0x00000C00))

/*-- Defines ---------------------------------------------------------*/

#define PCI_DEVICE_ID_AMCC_S5933    0x4750

/*-- Ioctls ----------------------------------------------------------*/

#define AMCC_IOCTL_BASE         0x00
#define AMCC_FIFO_IOCTL_BASE    0x40
#define AMCC_MB_IOCTL_BASE      0x80
#define AMCC_PT_IOCTL_BASE      0xC0

/* general (from 0x00) */

#define AMCCIORESET         _IO('z', (AMCC_IOCTL_BASE + 0))
#define AMCCIORESETADDON    _IO('z', (AMCC_IOCTL_BASE + 1))
#define AMCCIOGETICSR       _IOR('z', (AMCC_IOCTL_BASE + 2), __u32)
#define AMCCIOGETMCSR       _IOR('z', (AMCC_IOCTL_BASE + 3), __u32)
#define AMCCIOGETIRQTIME    _IOR('z', (AMCC_IOCTL_BASE + 4), long)
#define AMCCIOTEST          _IOWR('z', (AMCC_IOCTL_BASE + 5), long)

/* FIFOs (from 0x40) */

#define AMCCIOFIFORESET     _IO('z', (AMCC_FIFO_IOCTL_BASE + 0))
#define AMCCIOSETP2ABUF     _IOW('z', (AMCC_FIFO_IOCTL_BASE + 1), u_long)
#define AMCCIOSETA2PBUF     _IOW('z', (AMCC_FIFO_IOCTL_BASE + 2), u_long)
#define AMCCIOGETMRTC       _IOR('z', (AMCC_FIFO_IOCTL_BASE + 3), __u32)
#define AMCCIOGETMWTC       _IOR('z', (AMCC_FIFO_IOCTL_BASE + 4), __u32)

/* Mailboxes (from 0x80) */

#define AMCCIOREADMB        _IOWR('z', (AMCC_MB_IOCTL_BASE + 0), struct amcc_mbox)
#define AMCCIOWRITEMB       _IOWR('z', (AMCC_MB_IOCTL_BASE + 1), struct amcc_mbox)
#define AMCCIOSETOMBIRQ     _IOW('z', (AMCC_MB_IOCTL_BASE + 2), __u32)
#define AMCCIOSETIMBIRQ     _IOW('z', (AMCC_MB_IOCTL_BASE + 3), __u32)
#define AMCCIOCLEARMBIRQ    _IO('z', (AMCC_MB_IOCTL_BASE + 4))

/* Pass-thru (from 0xC0) */

/* NONE */

/*-- Structures ------------------------------------------------------*/

/**
 *  struct amcc_timings.
 *  This structure holds user-level IRQ timing information.
 */

struct amcc_timings {
    volatile long start;
    volatile long irq;
    volatile long bh;
    volatile long ioctl;
};

/**
 *  struct amcc_mailbox.
 *  This structure holds data for one mailbox.
 */

struct amcc_mbox {
    int         num;
    __u32       data;
};

/*--------------------------------------------------------------------*
 *
 *  KERNEL ONLY Section
 *
 *--------------------------------------------------------------------*/

#ifdef __KERNEL__

/*-- Support macros --------------------------------------------------*/

/* Compatibility Linux 2.0.X <-> Linux 2.1.X */

#ifndef LINUX_VERSION_CODE
# include <linux/version.h>
#endif

#if (LINUX_VERSION_CODE < 0x020100)
# include <linux/mm.h>
# define copy_from_user memcpy_fromfs
# define copy_to_user   memcpy_tofs
# define ioremap        vremap
# define iounmap        vfree
#else
# include <asm/uaccess.h>
#endif

#define MIN(a,b)    (((a) < (b)) ? (a) : (b))

#define LONG_ALIGN(a)   (((a) + (sizeof (long) - 1)) & ~(sizeof (long) - 1))

#define BUFFER_SIZE(ord)    (1UL << ((ord) + PAGE_SHIFT))

/*-- Defines ---------------------------------------------------------*/

#define AMCC_NAME           "amcc"

#define AMCC_MAGIC          0x35393333  /* "5933" */
#define AMCC_RW_TIMEOUT     (HZ * 5)    /* 5 sec. */

/* lock bits for concurrent access */

#define AMCC_P2A_LOCK_BIT       0
#define AMCC_A2P_LOCK_BIT       1
#define AMCC_TEST_IRQ_BIT       2

/* misce */

#define AMCC_FIFO_BUFFER_ORDER  2       /* 2^order = num. of pages */
#define AMCC_DMA_READ           0
#define AMCC_DMA_WRITE          1
#define AMCC_DMA_IRQ_OFF        0
#define AMCC_DMA_IRQ_ON         1

/* FIFO polled I/O constants */

#define AMCC_FIFO_IO_DELAY      20UL                    /* in usecs */
#define AMCC_FIFO_POLL_SIZE     (200 * sizeof (__u32))  /* in bytes */

/* return codes for interrupts */

#define AMCC_IRQ_READ_OK        1
#define AMCC_IRQ_WRITE_OK       2
#define AMCC_IRQ_MASTER_ABORT   3
#define AMCC_IRQ_TARGET_ABORT   4
#define AMCC_IRQ_OMB_EMPTY      5
#define AMCC_IRQ_IMB_FULL       6

/*-- Operation Registers Flags ---------------------------------------*/

/* Bus Master Control/Status Register (4.10) */

#define MCSR_CONTROL_MASK       0xFFFFFF00
#define   MCSR_NV_ACC_MASK      0xE0000000
#define   MCSR_RESET_MASK       0x0F000000
#define   MCSR_NV_DA_MASK       0x00FF0000
#define   MCSR_RTC_MASK         0x00007000
#define   MCSR_WTC_MASK         0x00000700
#define MCSR_STATUS_MASK        0x000000FF
#define   MCSR_TCZERO_MASK      0x000000C0
#define   MCSR_FIFO_ST_MASK     0x0000003F

#define MCSR_RESET_MBFLAGS      0x08000000
#define MCSR_RESET_A2P_FIFO     0x04000000
#define MCSR_RESET_P2A_FIFO     0x02000000
#define MCSR_RESET_FIFOS        (MCSR_RESET_A2P_FIFO | MCSR_RESET_P2A_FIFO)
#define MCSR_RESET_ADDON        0x01000000

#define MCSR_RTC_ENABLE         0x00004000
#define MCSR_RTC_FIFO_MNG       0x00002000
#define MCSR_RTC_RW_PRIO        0x00001000

#define MCSR_WTC_ENABLE         0x00000400
#define MCSR_WTC_FIFO_MNG       0x00000200
#define MCSR_WTC_RW_PRIO        0x00000100

#define MCSR_ALT_FIFO_PRIO      (MCSR_RTC_RW_PRIO | MCSR_WTC_RW_PRIO)

#define MCSR_A2P_TCOUNT         0x00000080
#define MCSR_P2A_TCOUNT         0x00000040

#define MCSR_FS_A2P_EMPTY       0x00000020
#define MCSR_FS_A2P_HALF        0x00000010
#define MCSR_FS_A2P_FULL        0x00000008

#define MCSR_FS_P2A_EMPTY       0x00000004
#define MCSR_FS_P2A_HALF        0x00000002
#define MCSR_FS_P2A_FULL        0x00000001

/* Interrupt Control/Status Register (4.9) */

#define ICSR_FMEC_MASK          0xFF000000
#define ICSR_INT_MASK           0x00FF0000
#define ICSR_SEL_MASK           0x0000FFFF
#define   ICSR_IS_ENSEL_MASK    0x00001F1F

#define ICSR_INT_ASSERTED       0x00800000
#define ICSR_TARGET_ABORT       0x00200000
#define ICSR_MASTER_ABORT       0x00100000

#define ICSR_RT_COMPL_INT       0x00080000
#define ICSR_WT_COMPL_INT       0x00040000

#define ICSR_IN_MB_INT          0x00020000
#define ICSR_OUT_MB_INT         0x00010000

#define ICSR_READ_COMPL         0x00008000
#define ICSR_WRITE_COMPL        0x00004000
#define   ICSR_FIFO_INT_MASK    0x0000C000

#define ICSR_IMB_ENABLE         0x00001000
#define ICSR_IMB_SELECT         0x00000C00
#define ICSR_IMB_BYTE           0x00000300

#define ICSR_OMB_ENABLE         0x00000010
#define ICSR_OMB_SELECT         0x0000000C
#define ICSR_OMB_BYTE           0x00000003
#define   ICSR_MB_INT_MASK      0x00001F1F

/* FIFO Management and Endian Control Byte */

#define ICSR_FMEC_PCI_ADV       0x30000000
#define ICSR_FMEC_ADDON_ADV     0x0C000000
#define ICSR_FMEC_CONV          0x03000000

/* Mailbox Empty/Full Status Register (4.8) */

#define MBEF_IN_ST_MASK         0xFFFF0000
#define MBEF_OUT_ST_MASK        0x0000FFFF

/* PCI Controlled Bus Master Write Address Register (4.4) */

#define MWAR_ADDR_MASK          0xFFFFFFFC
#define MWAR_DW_ADDR_MASK       0x00000003

/* PCI Controlled Bus Master Write Transfer Count Register (4.5) */

#define MWTC_COUNT_MASK         0x03FFFFFF

/* PCI Controlled Bus Master Read Address Register (4.6) */

#define MRAR_ADDR_MASK          0xFFFFFFFC
#define MRAR_DW_ADDR_MASK       0x00000003

/* PCI Controlled Bus Master Read Transfer Count Register (4.7) */

#define MRTC_COUNT_MASK         0x03FFFFFF

/*-- Structures ------------------------------------------------------*/

/*
 *  Pass-thru info.
 *  This struct holds information about a single pass-thru region.
 */

struct s5933_pt {
    u_int       size;       /* dimensions */
    u_int       phys_addr;  /* real address */
    char       *virt_addr;  /* after vremap/ioremap call */
};

/*
 *  struct amcc_s5933.
 *  This structure holds global data for a single PCI board.
 */

struct amcc_s5933 {
    int         major;
    u_long      op_regs;        /* this is BADDR[0] */
    int         op_regs_type;   /* MEM or I/O operation registers */
    u_char      bus;
    u_char      function;
    u_char      irq;            /* interrupt number */
    u_char      ptnum;          /* number of pass-thru regions */
    struct s5933_pt pts[4];     /* pass-thru regions */
    int         timeout;        /* timeout for async waits */
};

/*
 *  struct amcc_fifo.
 *  This structure holds data for FIFO handling.
 */

struct amcc_fifo {
    volatile int irq_code;
    volatile u_int copy_size;
    volatile u_int bytes_done;
    struct wait_queue *wq;
    char       *buf;
    u_long      order;
    u_int       async;
};

/*
 *  struct amcc_timeinfo.
 *  This structure holds IRQ timing information.
 */

struct amcc_timeinfo {
    struct timeval op;
    struct timeval irq;
    struct timeval bh;
};

/*-- Inline Functions ------------------------------------------------*/

extern inline __u32
amcc_read_opreg (int reg)
{
    extern struct amcc_s5933 amcc_dev;

    if (amcc_dev.op_regs_type == PCI_BASE_ADDRESS_SPACE_IO)
        return ((__u32) inl ((unsigned short) (amcc_dev.op_regs + reg)));
    else
        return (readl ((amcc_dev.op_regs + reg)));
}

extern inline void
amcc_write_opreg (__u32 value, int reg)
{
    extern struct amcc_s5933 amcc_dev;

    if (amcc_dev.op_regs_type == PCI_BASE_ADDRESS_SPACE_IO)
        outl (value, (unsigned short) (amcc_dev.op_regs + reg));
    else
        writel (value, (amcc_dev.op_regs + reg));
}

#endif __KERNEL__
#endif _MAIN_H
