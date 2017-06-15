/*
 *  ab.h -- Memory mapped AutoBahn driver main include file
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *           Toni Giorgino      (toni@pcape2.pi.infn.it)
 *
 *  ab.h,v 2.3 1997/11/10 09:26:54 acister Exp
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

#ifndef _AB_H
#define _AB_H

/*-- Defines & types -------------------------------------------------*/

#define AB_NAME             "ab"        /* driver name */
#define AB_DEV_NAME         "/dev/ab"   /* device node */

#define AB_MAX_DMA          60000000    /* max. dma size in bytes <= 67108864 (64MB) */

#define AB_LOCK_P2AFIFO     0       /* bit numbers used for locking */
#define AB_LOCK_A2PFIFO     1

#define AB_FLAG_A2PDONE     16      /* these are set by IRQ handler */
#define AB_FLAG_P2ADONE     17      /* on transfer complete */

#define AB_WAIT_P2A         1       /* wait DMA flags */
#define AB_WAIT_A2P         2

#define AB_OPREGS_OFFSET    0                   /* PCI opregs offset for mmap() */
#define AB_PT1_OFFSET       (1 * PAGE_SIZE)     /* pass-thru 1 offset for mmap() */
#define AB_DRV_OFFSET       (2 * PAGE_SIZE)     /* driver status offset for mmap() */
#define AB_BUF_OFFSET       (3 * PAGE_SIZE)     /* buffer offset for mmap() */

typedef unsigned long long u_llong;     /* for the rdtsc op */
typedef long long          llong;

/*-- Data structures -------------------------------------------------*/

/*
 *  struct ab_driver
 *  Holds driver status information available to applications.
 *  It is physically located in a shared memory page.
 */

struct ab_driver {
    u_int   drv_lock;       /* device locking */
    __u32   drv_icsr;       /* copy of ICSR on interrupt */
    u_llong drv_time;       /* last interrupt time */
    size_t  drv_wr_remain;  /* remaining size to write during A2P transfer */
};

/*-- Ioctls ----------------------------------------------------------*/

#define AB_IOCTL_BASE       0x00
#define AB_FIFO_IOCTL_BASE  0x40
#define AB_MB_IOCTL_BASE    0x80
#define AB_PT_IOCTL_BASE    0xC0

/* general (from 0x00) */

#define ABIORESET           _IO('z', (AB_IOCTL_BASE + 0))
#define ABIORESETADDON      _IO('z', (AB_IOCTL_BASE + 1))
#define ABIOSETLATENCY      _IOW('z', (AB_IOCTL_BASE + 2), u_char)
#define ABIOSETBUFFER       _IOW('z', (AB_IOCTL_BASE + 3), size_t)
#define ABIOGETPADDR        _IOR('z', (AB_IOCTL_BASE + 4), caddr_t)
#define ABIOGETOFFSET       _IOR('z', (AB_IOCTL_BASE + 5), caddr_t)

/* FIFO related (from 0x40) */

#define ABIOWAITDMA         _IOW('z', (AB_FIFO_IOCTL_BASE + 0), u_int)
#define ABIOA2PDMA          _IOW('z', (AB_FIFO_IOCTL_BASE + 1), size_t)

/* Pass-thru related (from 0xC0) */

#define ABIOGETPTSIZE       _IOR('z', (AB_PT_IOCTL_BASE + 0), size_t)

/* misce */

#define ABDELAY             _IOW('z', (0xD0), unsigned long)

/*--------------------------------------------------------------------*
 *
 *  KERNEL ONLY Section
 *
 *--------------------------------------------------------------------*/

#ifdef __KERNEL__

/*-- Kernel compatibility --------------------------------------------*/

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

/*-- Support macros & functions --------------------------------------*/

extern inline u_llong
getclock(void)
{
    u_llong tsc;

    __asm__ __volatile__("rdtsc"
                         : "=A"(tsc)
                         : /* no input*/
                         : "eax", "edx");

    return tsc;
}

/*-- Defines ---------------------------------------------------------*/

#define AB_MAJOR    120                     /* device major number */

#define AB_PT_NUM   PCI_BASE_ADDRESS_1      /* pass-thru area */

/*-- Structures ------------------------------------------------------*/

/*
 *  struct autobahn.
 *  This structure holds global data for a single PCI board.
 */

struct autobahn {
    struct ab_driver*   ab_drv;         /* shared driver status info (logical) */
    caddr_t             ab_drv_p;       /* shared driver status info (physical) */
    struct s5933_opreg* ab_opregs;      /* PCI operation registers (logical) */
    caddr_t             ab_opregs_p;    /* PCI operation registers (physical) */
    char*               ab_ptaddr;      /* pass-thru logical address */
    caddr_t             ab_ptaddr_p;    /* pass-thru physical address */
    size_t              ab_ptsize;      /* pass-thru dimension */
    u_char              ab_irq;         /* interrupt number */
    u_char              ab_bus;         /* PCI bus number */
    u_char              ab_function;    /* PCI device and function */
};

/*
 *  struct ab_procdata
 *  Holds per-process information
 */

struct ab_procdata {
    caddr_t pd_phys;        /* buffer physical address */
    size_t  pd_size;        /* buffer size */
    u_int   pd_id;          /* process ID */
};

#endif __KERNEL__
#endif _AB_H
