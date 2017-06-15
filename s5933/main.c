/*
 *  main.c -- Simple AMCC S5933 PCI Matchmaker driver
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *
 *  $Id: main.c,v 2.16 1997/10/01 08:17:34 acister Exp $
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
 *  Driver main file
 *  This file contains the all the driver code.
 *
 *--------------------------------------------------------------------*/

#ifndef __KERNEL__
# define __KERNEL__
#endif

#include <linux/config.h>       /* for CONFIG_PCI */

#ifdef MODULE
# include <linux/module.h>
# include <linux/version.h>
#else
# define MOD_INC_USE_COUNT
# define MOD_DEC_USE_COUNT
#endif

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/malloc.h>
#include <linux/pci.h>
#include <linux/bios32.h>
#include <linux/ioport.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <asm/io.h>             /* for inb(), outb() etc. */
#include <asm/system.h>
#include <asm/pgtable.h>

#include "amcc.h"               /* S5933 definitions */
#include "main.h"

/*--------------------------------------------------------------------
 *
 *  Static variables.
 *  These can be seen by every process using the device.
 *
 *--------------------------------------------------------------------*/

struct amcc_s5933 amcc_dev;             /* General data */

static struct amcc_fifo p2a_fifo;       /* P2A fifo control struct */
static struct amcc_fifo a2p_fifo;       /* A2P fifo control struct */

static struct wait_queue *mbwq;         /* wait queue for MBs */

static u_long lock = 0UL;               /* generic lock dword */

static struct amcc_timeinfo amcc_ti;    /* timings */

static u32  badrs[6] =
{
    PCI_BASE_ADDRESS_0,
    PCI_BASE_ADDRESS_1,
    PCI_BASE_ADDRESS_2,
    PCI_BASE_ADDRESS_3,
    PCI_BASE_ADDRESS_4,
    PCI_BASE_ADDRESS_5
};

static u32  imbox[4] =      /* for simpler access */
{
    AMCC_OP_REG_IMB1,
    AMCC_OP_REG_IMB2,
    AMCC_OP_REG_IMB3,
    AMCC_OP_REG_IMB4
};

static u32  ombox[4] =      /* for simpler access */
{
    AMCC_OP_REG_OMB1,
    AMCC_OP_REG_OMB2,
    AMCC_OP_REG_OMB3,
    AMCC_OP_REG_OMB4
};

static volatile u32 icsr;   /* local copy of ICSR */

/*--------------------------------------------------------------------
 *
 *  Flags.
 *  These variables can be overridden at load time by the insmod
 *  command.
 *
 *--------------------------------------------------------------------*/

int         vid = PCI_VENDOR_ID_AMCC;
int         did = PCI_DEVICE_ID_AMCC_S5933;

/*--------------------------------------------------------------------
 *
 *  Function prototypes
 *
 *--------------------------------------------------------------------*/

long        amcc_init (long, long);
int         init_module (void);
void        cleanup_module (void);

static int  amcc_read (struct inode *, struct file *, char *, int);
static int  amcc_write (struct inode *, struct file *, const char *, int);
static int  amcc_ioctl (struct inode *, struct file *, unsigned int, unsigned long);
static int  amcc_mmap (struct inode *, struct file *, struct vm_area_struct *);
static int  amcc_open (struct inode *, struct file *);
static void amcc_release (struct inode *, struct file *);

static void amcc_pt_setup (struct amcc_s5933 *, u32, int);
static int  amcc_init_dev (struct amcc_s5933 *, u_char, u_char);
static void amcc_free_dev (void);
static void amcc_dma_start (int, char *, int, int);
static void amcc_irq_handler (int, void *, struct pt_regs *);

/*--------------------------------------------------------------------
 *
 *  Linux kernel main entry point.
 *  For a definition of its fields see <linux/fs.h>.
 *
 *--------------------------------------------------------------------*/

static struct file_operations amcc_fops =
{
    NULL,               /* lseek()   */
    amcc_read,          /* read()    */
    amcc_write,         /* write()   */
    NULL,               /* readdir() */
    NULL,               /* select()  */
    amcc_ioctl,         /* ioctl()   */
    amcc_mmap,          /* mmap()    */
    amcc_open,          /* open()    */
    amcc_release,       /* release() */
    NULL,               /* fsync()   */
    NULL,               /* fasync()  */
    NULL,               /* check_media_change() */
    NULL,               /* revalidate() */
};

/*--------------------------------------------------------------------
 *
 *  Functions
 *
 *--------------------------------------------------------------------*/

#ifndef MODULE

/*--------------------------------------------------------------------
 *
 *  Standard driver initialization function.
 *  This function is called at boot time. Support functions can be found
 *  in file support.c
 *
 *--------------------------------------------------------------------*/

long
amcc_init (long mem_start, long mem_end)
{

#else   /* MODULE */

/*--------------------------------------------------------------------
 *
 *  Standard module initialization function.
 *  This function scans the PCI bus looking for the right board. Support
 *  functions can be found in file support.c
 *
 *--------------------------------------------------------------------*/

int
init_module (void)
{

#endif  /* MODULE */

    extern struct amcc_s5933 amcc_dev;
    int         result = 0L;

    pr_debug ("amcc: >>> START <<<\n");

    memset (&amcc_dev, 0, sizeof (amcc_dev));
    memset (&p2a_fifo, 0, sizeof (p2a_fifo));
    memset (&a2p_fifo, 0, sizeof (a2p_fifo));

    if (pcibios_present ()) {

        /* PCI bus scan, only first device */

        u_char      bus;
        u_char      function;

        result = pcibios_find_device ((u_short) vid, (ushort) did, 0, &bus, &function);

        if (result != PCIBIOS_SUCCESSFUL) {
            printk (KERN_WARNING "amcc: pci error: %s\n",
            pcibios_strerror (result));
            return -ENODEV;     /* device not found */
        }

        /*
         *  Major number allocation. It must be done differently because
         *  of the different return code.
         */

#ifndef MODULE

        if (amcc_dev.major = register_chrdev (0, AMCC_NAME, &amcc_fops)) {
            printk (KERN_WARNING "amcc: unable to get major number\n");
            return mem_start;
        }

#else

        if ((amcc_dev.major = register_chrdev (0, AMCC_NAME, &amcc_fops)) == -EBUSY) {
            printk (KERN_WARNING "amcc: unable to get major number\n");
            return -EIO;
        }

#endif  /* MODULE */

        /*
         *  Device initialization
         */

        result = amcc_init_dev (&amcc_dev, bus, function);

        amcc_dev.timeout = AMCC_RW_TIMEOUT;
    }

    pr_info ("amcc: AMCC S5933 $Revision: 2.16 $; "
             "(c) 1997 by Andrea Cisternino (acister@pcape1.pi.infn.it)\n");

#ifndef MODULE

    return mem_start;
}

#else

    return result;
}

/*--------------------------------------------------------------------
 *
 *  Standard module release function.
 *
 *--------------------------------------------------------------------*/

void
cleanup_module (void)
{
    amcc_free_dev ();

    pr_debug ("amcc: >>> STOP <<<\n");
}

#endif  /* MODULE */

/*--------------------------------------------------------------------
 *
 *  Standard open() entry point.
 *  It simply increments the module usage count.
 *
 *--------------------------------------------------------------------*/

static int
amcc_open (struct inode *inode, struct file *file)
{
    pr_debug ("amcc: open\n");

    MOD_INC_USE_COUNT;

    return 0;
}

/*--------------------------------------------------------------------
 *
 *  Standard release() entry point.
 *  This function is called by the close() system call.
 *
 *--------------------------------------------------------------------*/

static void
amcc_release (struct inode *inode, struct file *file)
{
    pr_debug ("amcc: close\n");

    MOD_DEC_USE_COUNT;
}

/*--------------------------------------------------------------------
 *
 *  Standard read() entry point.
 *  It is used to read from the Add-on to PCI FIFO. It uses DMA to fill
 *  a kernel buffer with the specified number of bytes.
 *
 *  Non-Blocking I/O is handled only at entry time. A correct
 *  handling should also perform a polling read of the FIFO register
 *  until no data is available and then return the number of bytes
 *  read.
 *
 *--------------------------------------------------------------------*/

static int
amcc_read (struct inode *inode, struct file *file, char *buf, int count)
{
    int total_bytes_read = 0;

    if (count == 0)
        return 0;

    /* is the FIFO available? */

    if (set_bit (AMCC_A2P_LOCK_BIT, &lock)) {
        pr_debug ("amcc: read: FIFO already locked\n");
        return -EBUSY;
    }

    /* we first check the buffer */

    if (verify_area (VERIFY_WRITE, buf, count) == -EFAULT) {
        printk (KERN_WARNING "amcc: read: verify error, buf 0x%08lx, count %ld\n", buf, count);
        clear_bit (AMCC_A2P_LOCK_BIT, &lock);
        return -EFAULT;
    }

    /* this sets the flag correctly */

    if (file->f_flags & O_NONBLOCK)
        a2p_fifo.async = 1L;
    else
        a2p_fifo.async = 0L;

    /* now we check the FIFO */

    if ((amcc_read_opreg (AMCC_OP_REG_MCSR)) & MCSR_FS_A2P_EMPTY) {

        /*
         *  It is empty.
         *  We handle the condition accordingly to the flags passed.
         */

        if (a2p_fifo.async) {
            clear_bit (AMCC_A2P_LOCK_BIT, &lock);
            return -EAGAIN;
        }

        /* wait for the FIFO to become not empty */

        current->state = TASK_INTERRUPTIBLE;
        current->timeout = jiffies + amcc_dev.timeout;
        schedule ();

        /* we are awake now, check the reason why */

        if (current->signal & ~current->blocked) {
            clear_bit (AMCC_A2P_LOCK_BIT, &lock);
            return -ERESTARTSYS;
        }

        if ((amcc_read_opreg (AMCC_OP_REG_MCSR)) & MCSR_FS_A2P_EMPTY) {
            clear_bit (AMCC_A2P_LOCK_BIT, &lock);
            return -EIO;    /* still empty */
        }
    }

    /* start read */

    do {
        a2p_fifo.bytes_done = 0;
        a2p_fifo.copy_size = MIN (count, BUFFER_SIZE (a2p_fifo.order));

        while (a2p_fifo.copy_size) {

            /*
             *  If count is smaller than AMCC_FIFO_POLL_SIZE, we poll.
             *  WARNING: does not work with the ISA board
             */

            if (a2p_fifo.copy_size <= AMCC_FIFO_POLL_SIZE) {

                amcc_dma_start (AMCC_DMA_READ, a2p_fifo.buf, a2p_fifo.copy_size, AMCC_DMA_IRQ_OFF);

                /* we loop on transfer complete */

                do {
                    udelay (20);
                } while (!((amcc_read_opreg (AMCC_OP_REG_MCSR)) & MCSR_A2P_TCOUNT));

                /* fake interrupt */

                do_gettimeofday (&(amcc_ti.irq));
                a2p_fifo.irq_code = AMCC_IRQ_READ_OK;
                a2p_fifo.bytes_done += a2p_fifo.copy_size;
                a2p_fifo.copy_size = 0L;
            }
            else {
                /* start normal transfer and go to sleep */

                cli ();
                amcc_dma_start (AMCC_DMA_READ, a2p_fifo.buf, a2p_fifo.copy_size, AMCC_DMA_IRQ_ON);
                interruptible_sleep_on (&(a2p_fifo.wq));
            }

            /* we are awake now, copy the data */

            if (a2p_fifo.bytes_done)
                copy_to_user (buf, a2p_fifo.buf, a2p_fifo.bytes_done);

            /* we received a signal */

            if (current->signal & ~current->blocked) {

                int rc = total_bytes_read + a2p_fifo.bytes_done;

                clear_bit (AMCC_A2P_LOCK_BIT, &lock);
                return ((rc) ? (rc) : -EINTR);
            }

            /* we received an interrupt */

            switch (a2p_fifo.irq_code) {

                case AMCC_IRQ_READ_OK:
                    total_bytes_read += a2p_fifo.bytes_done;
                    buf += a2p_fifo.bytes_done;
                    count -= a2p_fifo.bytes_done;
                    break;

               case AMCC_IRQ_MASTER_ABORT:
               case AMCC_IRQ_TARGET_ABORT:
                    pr_debug ("amcc: write: pci error %d\n", a2p_fifo.irq_code);
                    clear_bit (AMCC_A2P_LOCK_BIT, &lock);
                    return -EIO;
                    break;

                default:
                    break;
            }
        }
    }
    while (count > 0);

    clear_bit (AMCC_A2P_LOCK_BIT, &lock);
    return total_bytes_read;
}

/*--------------------------------------------------------------------
 *
 *  Standard write() entry point.
 *  It is used to write to the PCI-to-Add-on FIFO. It copies the user
 *  buffer into our (smaller) kernel buffer and then performs a DMA
 *  transfer.
 *
 *--------------------------------------------------------------------*/

static int
amcc_write (struct inode *inode, struct file *file, const char *buf, int count)
{
    int total_bytes_written = 0;

    /* is the FIFO available? */

    if (set_bit (AMCC_P2A_LOCK_BIT, &lock)) {
        pr_debug ("amcc: write: FIFO already locked\n");
        return -EBUSY;
    }

    /* we first check the buffer */

    if (verify_area (VERIFY_READ, buf, count) == -EFAULT) {
        printk (KERN_WARNING "amcc: write: verify error, buf 0x%08lx, count %ld\n", buf, count);
        clear_bit (AMCC_P2A_LOCK_BIT, &lock);
        return -EFAULT;
    }

    /* this sets the flag correctly */

    if (file->f_flags & O_NONBLOCK)
        p2a_fifo.async = 1L;
    else
        p2a_fifo.async = 0L;

    /* now we check the FIFO */

    if (!((amcc_read_opreg (AMCC_OP_REG_MCSR)) & MCSR_FS_P2A_EMPTY)) {

        /*
         *  It is not empty.
         *  We handle the condition accordingly to the flags passed.
         */

        if (p2a_fifo.async) {
            clear_bit (AMCC_P2A_LOCK_BIT, &lock);
            return -EAGAIN;
        }

        /* wait for the FIFO to become empty */

        current->state = TASK_INTERRUPTIBLE;
        current->timeout = jiffies + amcc_dev.timeout;
        schedule ();

        /* we are awake now, check the reason why */

        if (current->signal & ~current->blocked) {
            clear_bit (AMCC_P2A_LOCK_BIT, &lock);
            return -ERESTARTSYS;
        }

        if (!((amcc_read_opreg (AMCC_OP_REG_MCSR)) & MCSR_FS_P2A_EMPTY)) {
            clear_bit (AMCC_P2A_LOCK_BIT, &lock);
            return -EIO;        /* still not empty */
        }
    }

    /* start write */

    if (p2a_fifo.async) {

        /* ASYNC write requested */

        if (count > BUFFER_SIZE (p2a_fifo.order)) {
            clear_bit (AMCC_P2A_LOCK_BIT, &lock);
            return -EAGAIN;     /* not enough space */
        }

        p2a_fifo.copy_size = count;
        p2a_fifo.bytes_done = 0;

        copy_from_user (p2a_fifo.buf, buf, p2a_fifo.copy_size);
        cli ();
        amcc_dma_start (AMCC_DMA_WRITE, p2a_fifo.buf, p2a_fifo.copy_size, AMCC_DMA_IRQ_ON);

        /* the lock bit is cleared by the IRQ handler */

        return count;
    }
    else {

        /* standard write requested */

        do {
            p2a_fifo.bytes_done = 0;
            p2a_fifo.copy_size = MIN (count, BUFFER_SIZE (p2a_fifo.order));

            copy_from_user (p2a_fifo.buf, buf, p2a_fifo.copy_size);

            while (p2a_fifo.copy_size) {

                /*
                 *  If count is smaller than AMCC_FIFO_POLL_SIZE, we poll.
                 *  WARNING: does not work with the ISA board
                 */

                if (p2a_fifo.copy_size <= AMCC_FIFO_POLL_SIZE) {

                    amcc_dma_start (AMCC_DMA_WRITE, p2a_fifo.buf, p2a_fifo.copy_size, AMCC_DMA_IRQ_OFF);

                    /* we loop on transfer complete */

                    do {
                        udelay (AMCC_FIFO_IO_DELAY);
                    } while (!((amcc_read_opreg (AMCC_OP_REG_MCSR)) & MCSR_P2A_TCOUNT));

                    /* fake interrupt */

                    p2a_fifo.irq_code = AMCC_IRQ_WRITE_OK;
                    p2a_fifo.bytes_done += p2a_fifo.copy_size;
                    p2a_fifo.copy_size = 0L;
                }
                else {
                    /* start transfer and go to sleep */

                    cli ();
                    amcc_dma_start (AMCC_DMA_WRITE, p2a_fifo.buf, p2a_fifo.copy_size, AMCC_DMA_IRQ_ON);
                    interruptible_sleep_on (&(p2a_fifo.wq));
                }

                /* we are awake now, check what happened meanwhile */

                /* we received a signal */

                if (current->signal & ~current->blocked) {

                    int rc = total_bytes_written + p2a_fifo.bytes_done;

                    clear_bit (AMCC_P2A_LOCK_BIT, &lock);
                    return ((rc) ? (rc) : -EINTR);
                }

                /* we received an interrupt */

                switch (p2a_fifo.irq_code) {

                    case AMCC_IRQ_WRITE_OK:
                        total_bytes_written += p2a_fifo.bytes_done;
                        buf += p2a_fifo.bytes_done;
                        count -= p2a_fifo.bytes_done;
                        break;

                    case AMCC_IRQ_MASTER_ABORT:
                    case AMCC_IRQ_TARGET_ABORT:
                        pr_debug ("amcc: write: pci error %d\n", p2a_fifo.irq_code);
                        clear_bit (AMCC_P2A_LOCK_BIT, &lock);
                        return -EIO;
                        break;

                    default:
                        break;
                }
            }
        } while (count > 0);
    }

    clear_bit (AMCC_P2A_LOCK_BIT, &lock);
    return total_bytes_written;
}

/*--------------------------------------------------------------------
 *
 *  Standard ioctl() entry point.
 *
 *--------------------------------------------------------------------*/

static int
amcc_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    extern struct amcc_s5933 amcc_dev;
    int ret = 0L;

    pr_debug ("amcc: ioctl: cmd: %08lx, arg: %08lx\n", cmd, arg);

    switch (cmd) {

        /*
         *  General
         */

        case AMCCIORESET:           /* reset the whole board */
            amcc_write_opreg ((u32) 0, AMCC_OP_REG_INTCSR);
            amcc_write_opreg ((MCSR_RESET_FIFOS | MCSR_RESET_MBFLAGS), AMCC_OP_REG_MCSR);
            amcc_write_opreg ((u32) 0, AMCC_OP_REG_MRTC);
            amcc_write_opreg ((u32) 0, AMCC_OP_REG_MWTC);

            lock = 0L;
            break;

        case AMCCIOGETICSR:         /* read ICSR register */
        {
            u32 *addr = (u32 *) arg;

            ret = verify_area (VERIFY_WRITE, (void *) addr, sizeof (u32));
            if (ret)
                return ret;

            put_user (amcc_read_opreg (AMCC_OP_REG_INTCSR), addr);
            break;
        }

        case AMCCIOGETMCSR:         /* read MCSR register */
        {
            u32 *addr = (u32 *) arg;

            ret = verify_area (VERIFY_WRITE, (void *) addr, sizeof (u32));
            if (ret)
                return ret;

            put_user (amcc_read_opreg (AMCC_OP_REG_MCSR), addr);
            break;
        }

        case AMCCIOGETIRQTIME:      /* IRQ handling time */
        {
            struct timeval ioctltv;
            struct amcc_timings ioctltim;

            if (set_bit (AMCC_TEST_IRQ_BIT, &lock))
                return -EBUSY;

            ret = verify_area (VERIFY_WRITE, (void *) arg, sizeof (struct amcc_timings));
            if (ret)
                return ret;

            /*
             *  We wait for IMB1 to became full.
             *  IRQs must be already enabled for that MB.
             */

            interruptible_sleep_on (&mbwq);
            do_gettimeofday (&ioctltv);

            ioctltim.start = amcc_read_opreg (AMCC_OP_REG_IMB1);
            ioctltim.irq = amcc_ti.irq.tv_usec;
            ioctltim.bh = amcc_ti.bh.tv_usec;
            ioctltim.ioctl = ioctltv.tv_usec;

            copy_to_user ((struct amcc_timings *) arg, &ioctltim, sizeof (struct amcc_timings));
            clear_bit (AMCC_TEST_IRQ_BIT, &lock);
            break;
        }

        case AMCCIOTEST:            /* miscellaneous tests */
        {
            u32 *addr = amcc_dev.pts[3].virt_addr;
            u32 data;
            data = addr[1];
            break;
        }

        /*
         *  FIFOs
         */

        case AMCCIOFIFORESET:       /* reset the two FIFO */
        {
            u32 l_icsr;

            amcc_write_opreg ((u32) 0, AMCC_OP_REG_MRTC);
            amcc_write_opreg ((u32) 0, AMCC_OP_REG_MWTC);

            l_icsr = amcc_read_opreg (AMCC_OP_REG_INTCSR);
            l_icsr &= ~ICSR_FIFO_INT_MASK;
            amcc_write_opreg (l_icsr, AMCC_OP_REG_INTCSR);

            amcc_write_opreg (MCSR_RESET_FIFOS, AMCC_OP_REG_MCSR);

            clear_bit (AMCC_A2P_LOCK_BIT, &lock);
            clear_bit (AMCC_P2A_LOCK_BIT, &lock);
            break;
        }

        case AMCCIOGETMRTC:         /* read MRTC register */
        case AMCCIOGETMWTC:         /* read MWTC register */
        {
            u32 *addr = (u32 *) arg;

            ret = verify_area (VERIFY_WRITE, (void *) addr, sizeof (u32));
            if (ret)
                return ret;

            if (cmd == AMCCIOGETMRTC)
                put_user (amcc_read_opreg (AMCC_OP_REG_MRTC), addr);
            else
                put_user (amcc_read_opreg (AMCC_OP_REG_MWTC), addr);

            break;
        }

        case AMCCIOSETP2ABUF:       /* change P2A FIFO buffer size */
        {
            char *tmpbuf = NULL;

            if (set_bit (AMCC_P2A_LOCK_BIT, &lock))
                return -EBUSY;

            /* allocate a new buffer */

            tmpbuf = (char *) __get_free_pages (GFP_ATOMIC, (u_long) arg, 0);
            if (tmpbuf == NULL) {
                clear_bit (AMCC_P2A_LOCK_BIT, &lock);
                return -ENOMEM;
            }

            /* free the old pages */

            free_pages ((unsigned long) p2a_fifo.buf, p2a_fifo.order);

            /* setup the new ones */

            p2a_fifo.buf = tmpbuf;
            p2a_fifo.order = (u_long) arg;

            clear_bit (AMCC_P2A_LOCK_BIT, &lock);
            break;
        }

        case AMCCIOSETA2PBUF:       /* change A2P FIFO buffer size */
        {
            char *tmpbuf = NULL;

            if (set_bit (AMCC_A2P_LOCK_BIT, &lock))
                return -EBUSY;

            /* allocate a new buffer */

            tmpbuf = (char *) __get_free_pages (GFP_ATOMIC, (u_long) arg, 0);
            if (tmpbuf == NULL) {
                clear_bit (AMCC_A2P_LOCK_BIT, &lock);
                return -ENOMEM;
            }

            /* free the old pages */

            free_pages ((unsigned long) a2p_fifo.buf, a2p_fifo.order);

            /* setup the new ones */

            a2p_fifo.buf = tmpbuf;
            a2p_fifo.order = (u_long) arg;

            clear_bit (AMCC_A2P_LOCK_BIT, &lock);
            break;
        }

        /*
         *  Mailboxes
         */

        case AMCCIOWRITEMB:         /* write to mailbox 1-4 */
        {
            struct amcc_mbox mb;

            ret = verify_area (VERIFY_READ, (void *) arg, sizeof (struct amcc_mbox));
            if (ret)
                return ret;

            copy_from_user (&mb, (struct amcc_mbox *) arg, sizeof (struct amcc_mbox));

            if ((mb.num < 1) || (mb.num > 4))
                ret = -EINVAL;
            else
                amcc_write_opreg (mb.data, ombox[mb.num - 1]);

            break;
        }

        case AMCCIOREADMB:          /* read from mailbox 1-4 */
        {
            u32 *addr = (u32 *) arg;
            u32 mbnum;

            if ((verify_area (VERIFY_WRITE, (void *) addr, sizeof (u32))) ||
                (verify_area (VERIFY_READ, (void *) addr, sizeof (u32))))
                return ret;

            mbnum = get_user (addr);

            if ((mbnum < 1) || (mbnum > 4))
                ret = -EINVAL;
            else
                put_user (amcc_read_opreg (imbox[mbnum - 1]), addr);

            break;
        }

        case AMCCIOSETOMBIRQ:       /* set IRQ on Outgoing Mailbox empty */
        case AMCCIOSETIMBIRQ:       /* set IRQ on Incoming Mailbox full */
        {
            u32 l_icsr = 0L;

            l_icsr = amcc_read_opreg (AMCC_OP_REG_INTCSR);

            if (cmd == AMCCIOSETOMBIRQ)
                l_icsr |= (arg | ICSR_OMB_ENABLE);
            else
                l_icsr |= (arg | ICSR_IMB_ENABLE);

            amcc_write_opreg (l_icsr, AMCC_OP_REG_INTCSR);
            break;
        }

        case AMCCIOCLEARMBIRQ:      /* clear MB IRQ flags */
        {
            u32 l_icsr = 0L;

            l_icsr = amcc_read_opreg (AMCC_OP_REG_INTCSR);
            l_icsr &= ~ICSR_MB_INT_MASK;
            amcc_write_opreg (l_icsr, AMCC_OP_REG_INTCSR);
            break;
        }

        /*
         *  Pass-thru
         */



        /*
         *  End of ioctls
         */

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

/*--------------------------------------------------------------------
 *
 *  Standard mmap() entry point.
 *  Very simple for the moment.
 *
 *--------------------------------------------------------------------*/

#define AREA 0

static int
amcc_mmap (struct inode *inode, struct file *file, struct vm_area_struct *vma)
{
    /* very simple for the moment - only pts[AREA] */

    pr_debug ("amcc: mmap: start: %08lx, end: %08lx, offset: %08lx, flags: %08lx\n",
              vma->vm_start, vma->vm_end, vma->vm_offset, vma->vm_flags);

    /* check dimensions */

    if ((vma->vm_end - vma->vm_start + vma->vm_offset) > amcc_dev.pts[AREA].size)
        return -EINVAL;

    /* get real address */

    vma->vm_offset += amcc_dev.pts[AREA].phys_addr;
    if (vma->vm_offset & ~PAGE_MASK)
        return -ENXIO;

    /* map pages */

#ifdef __i486__
    /* This disables high-memory caching (NEEDED?) */

    if (vma->vm_offset >= high_memory)
        pgprot_val (vma->vm_page_prot) |= _PAGE_PCD;
#endif

    if (remap_page_range (vma->vm_start, vma->vm_offset,
                          (vma->vm_end - vma->vm_start), vma->vm_page_prot))
        return -EAGAIN;

    vma->vm_inode = inode;
    inode->i_count++;

    return 0;
}

/*--------------------------------------------------------------------
 *
 *  Device initialization.
 *  This function initializes the board. It requests an interrupt and
 *  reserves an IO region for the operation registers. The Major number
 *  has already been allocated by the main init function.
 *
 *--------------------------------------------------------------------*/

static int
amcc_init_dev (struct amcc_s5933 *dev, u_char bus, u_char function)
{
    extern struct amcc_fifo p2a_fifo;
    extern struct amcc_fifo a2p_fifo;
    u_long ret;
    u32 l_icsr;
    int count;

    /* store the bus and functions values */

    dev->bus = bus;
    dev->function = function;

    /*
     *  Buffer allocation
     */

    p2a_fifo.buf = (char *) __get_free_pages (GFP_ATOMIC, AMCC_FIFO_BUFFER_ORDER, 0);
    p2a_fifo.order = AMCC_FIFO_BUFFER_ORDER;

    a2p_fifo.buf = (char *) __get_free_pages (GFP_ATOMIC, AMCC_FIFO_BUFFER_ORDER, 0);
    a2p_fifo.order = AMCC_FIFO_BUFFER_ORDER;

    if ((p2a_fifo.buf == NULL) || (a2p_fifo.buf == NULL)) {
        printk (KERN_WARNING "amcc: no memory for FIFO buffers\n");
        amcc_free_dev ();
        return -ENOMEM;
    }

    /*
     * IRQ facilities setup
     * register the irq with the system
     */

    ret = (u_long) pcibios_read_config_byte (bus, function, PCI_INTERRUPT_LINE, &(dev->irq));
    if (ret == PCIBIOS_SUCCESSFUL) {
        pr_debug ("amcc: PCI interrupt register = %d\n", dev->irq);

        if (request_irq (dev->irq, amcc_irq_handler, SA_INTERRUPT, AMCC_NAME, NULL)) {
            printk (KERN_WARNING "amcc: can't install handler for irq %d\n", dev->irq);
            dev->irq = 0;
            amcc_free_dev ();
            return -EIO;
        }
    }
    else {
        printk (KERN_WARNING "amcc: can't read PCI interrupt register\n");
        dev->irq = 0;
        amcc_free_dev ();
        return -EIO;
    }

    /*
     * the following section retrieve from PCI_BASE_ADDRESS_0 the address
     * of the S5933 operations registers. It then registers the
     * IO address range with the system.
     * We presume that its size is the standard one (16 DWORD).
     */

    ret = pcibios_read_config_dword (bus, function, badrs[0], (unsigned int *) &(dev->op_regs));

    if (ret != PCIBIOS_SUCCESSFUL) {
        printk (KERN_WARNING "amcc: reading BADR[0]: error %s\n", pcibios_strerror (ret));
        dev->op_regs = 0;
        amcc_free_dev ();
        return -EIO;
    }

    pr_debug ("amcc: init: BADR[0] = 0x%08lx\n", dev->op_regs);

    /* Check for I/O or Memory Mapped Operation Registers */

    if (dev->op_regs & PCI_BASE_ADDRESS_SPACE) {

        /* I/O mapped. We now register the region */

        dev->op_regs &= PCI_BASE_ADDRESS_IO_MASK;

        ret = check_region (dev->op_regs, AMCC_OP_REG_SIZE);
        if (ret == -EBUSY) {
            printk (KERN_WARNING "amcc: init: I/O space already requested at 0x%04lx\n", dev->op_regs);
            dev->op_regs = 0;
            amcc_free_dev ();
            return -EIO;
        }

        request_region (dev->op_regs, AMCC_OP_REG_SIZE, AMCC_NAME);
        dev->op_regs_type = PCI_BASE_ADDRESS_SPACE_IO;
    }
    else {

        /*
         * Memory mapped.
         * The address must be remapped in kernel-space.
         */

        dev->op_regs &= PCI_BASE_ADDRESS_MEM_MASK;
        dev->op_regs = (u_long) ioremap (dev->op_regs, AMCC_OP_REG_SIZE);
        dev->op_regs_type = PCI_BASE_ADDRESS_SPACE_MEMORY;
    }

    /*
     *  Pass-thru regions in following base addresses
     */

    dev->ptnum = 0;
    for (count = 1; count <= 5; count++) {
        u32 data;

        pcibios_read_config_dword (bus, function, badrs[count], &data);
        if (data == 0)
            break;

        amcc_pt_setup (dev, data, count);
    }

    /*
     * board reset
     */

    amcc_write_opreg ((MCSR_RESET_FIFOS | MCSR_RESET_MBFLAGS), AMCC_OP_REG_MCSR);

    l_icsr = amcc_read_opreg (AMCC_OP_REG_INTCSR);
    l_icsr &= ~ICSR_SEL_MASK;
    amcc_write_opreg (l_icsr, AMCC_OP_REG_INTCSR);

    amcc_write_opreg ((u32) 0, AMCC_OP_REG_MRTC);
    amcc_write_opreg ((u32) 0, AMCC_OP_REG_MWTC);

    /* some debug info */

    pr_debug ("amcc: init: MCSR: 0x%08lx\n", amcc_read_opreg (AMCC_OP_REG_MCSR));
    pr_debug ("amcc: init: ICSR: 0x%08lx\n", amcc_read_opreg (AMCC_OP_REG_INTCSR));
    pr_debug ("amcc: init: function terminated successfully\n");

    return 0;
}

/*--------------------------------------------------------------------
 *
 *  Device finalization.
 *  It selectively frees all the resources allocated by the driver.
 *
 *--------------------------------------------------------------------*/

static void
amcc_free_dev ()
{
    extern struct amcc_s5933 amcc_dev;
    extern struct amcc_fifo p2a_fifo;
    extern struct amcc_fifo a2p_fifo;

    int count = 0L;

    /* reset the ICSR register */

    amcc_write_opreg ((u32) 0, AMCC_OP_REG_INTCSR);

    if (amcc_dev.major)
        unregister_chrdev (amcc_dev.major, AMCC_NAME);

    if (amcc_dev.op_regs) {
        if (amcc_dev.op_regs_type == PCI_BASE_ADDRESS_SPACE_IO)
            release_region (amcc_dev.op_regs, AMCC_OP_REG_SIZE);
        else
            iounmap (amcc_dev.op_regs);
    }

    if (amcc_dev.irq)
        free_irq (amcc_dev.irq, NULL);

    /* pass-thru regions */

    while ((amcc_dev.pts[count]).virt_addr != NULL) {
        iounmap ((amcc_dev.pts[count]).virt_addr);
        count++;
    }

    /* FIFO buffers */

    if (p2a_fifo.buf)
        free_pages ((unsigned long) p2a_fifo.buf, p2a_fifo.order);

    if (a2p_fifo.buf)
        free_pages ((unsigned long) a2p_fifo.buf, a2p_fifo.order);
}

/*--------------------------------------------------------------------
 *
 *  Pass-thru regions setup.
 *  This function extract information from the PCI base addresses
 *  registers and remaps each area into the kernel virtual memory space
 *  using the ioremap() function.
 *
 *--------------------------------------------------------------------*/

static void
amcc_pt_setup (struct amcc_s5933 *dev, u32 badr, int number)
{
    u32 mask;

    /* we only support memory mapped PT regions */

    if (badr & PCI_BASE_ADDRESS_SPACE_IO)
        return;

    /* get configuration */

    cli ();
    pcibios_write_config_dword (dev->bus, dev->function, badrs[number], ~0);
    pcibios_read_config_dword (dev->bus, dev->function, badrs[number], &mask);
    pcibios_write_config_dword (dev->bus, dev->function, badrs[number], badr);
    sti ();

    pr_debug ("amcc: pt: %d - mask %08lx, current %08lx\n", number, mask, badr);

    /* extract data */

    mask &= PCI_BASE_ADDRESS_MEM_MASK;

    dev->ptnum++;

    (dev->pts[number - 1]).size = (~mask) + 1;
    (dev->pts[number - 1]).phys_addr = (badr & PCI_BASE_ADDRESS_MEM_MASK);
    (dev->pts[number - 1]).virt_addr = ioremap ((dev->pts[number - 1]).phys_addr,
                                                (dev->pts[number - 1]).size);

    pr_debug ("amcc: pt: PT[%d] - S: %d, P: %08lx, V: %08lx\n", (number - 1),
              (dev->pts[number - 1]).size,
              (dev->pts[number - 1]).phys_addr,
              (dev->pts[number - 1]).virt_addr);
}

/*--------------------------------------------------------------------
 *
 *  DMA setup function.
 *  It set up a DMA transfer on either FIFO.
 *
 *--------------------------------------------------------------------*/

static void
amcc_dma_start (int type, char *addr, int count, int irq_on)
{
    u32 l_icsr;

    switch (type) {

        case AMCC_DMA_WRITE:        /* write (PCI -> Add-on) */

            /* data setup */

            amcc_write_opreg ((int) virt_to_bus (addr), AMCC_OP_REG_MRAR);
            amcc_write_opreg (count, AMCC_OP_REG_MRTC);

            /* enable the TCOUNT == 0 interrupt if needed */

            if (irq_on) {
                l_icsr = amcc_read_opreg (AMCC_OP_REG_INTCSR);
                l_icsr |= ICSR_READ_COMPL;
                amcc_write_opreg (l_icsr, AMCC_OP_REG_INTCSR);
            }

            /* start transfer */

            do_gettimeofday (&(amcc_ti.op));
            amcc_write_opreg ((MCSR_RTC_FIFO_MNG | MCSR_ALT_FIFO_PRIO | MCSR_RTC_ENABLE),
                              AMCC_OP_REG_MCSR);
            break;

        case AMCC_DMA_READ:         /* read (Add-on -> PCI) */

            /* data setup */

            amcc_write_opreg ((int) virt_to_bus (addr), AMCC_OP_REG_MWAR);
            amcc_write_opreg (count, AMCC_OP_REG_MWTC);

            /* enable the TCOUNT == 0 interrupt if needed */

            if (irq_on) {
                l_icsr = amcc_read_opreg (AMCC_OP_REG_INTCSR);
                l_icsr |= ICSR_WRITE_COMPL;
                amcc_write_opreg (l_icsr, AMCC_OP_REG_INTCSR);
            }

            /* start transfer */

            do_gettimeofday (&(amcc_ti.op));
            amcc_write_opreg ((MCSR_WTC_FIFO_MNG | MCSR_ALT_FIFO_PRIO | MCSR_WTC_ENABLE),
                              AMCC_OP_REG_MCSR);
            break;

        default:                    /* shouldn't happen */
            break;
    }
}

/*--------------------------------------------------------------------
 *
 *  Interrupt handler.
 *  It stops a DMA transfer in case of an error on th PCI bus.
 *  If the interrupt is generated by a transfer complete condition
 *  the proper counters are updated and the sleeping process is
 *  wake up.
 *
 *--------------------------------------------------------------------*/

static void
amcc_irq_handler (int irq, void *dev, struct pt_regs *regs)
{
    extern volatile u32 icsr;
    extern struct amcc_timeinfo amcc_ti;
    u32 mcsr;

    /* record the timestamp */

    do_gettimeofday (&(amcc_ti.irq));

    /* first obvious check :-) */

    if (irq != amcc_dev.irq) {
        printk (KERN_WARNING "amcc: irq: spurious interrupt!\n");
        return;
    }

    /* read registers */

    mcsr = amcc_read_opreg (AMCC_OP_REG_MCSR);
    icsr = amcc_read_opreg (AMCC_OP_REG_INTCSR);

    pr_debug ("amcc: got irq: ICSR = 0x%08lx\n", icsr);

    if (!(icsr & ICSR_INT_ASSERTED))
        return;

    /*
     *  Disable bus-mastering if needed.
     *  We do it in a selective way.
     */

    if (icsr & (ICSR_TARGET_ABORT | ICSR_MASTER_ABORT)) {
        mcsr &= ~(MCSR_RTC_ENABLE | MCSR_WTC_ENABLE);
        icsr &= ~(ICSR_READ_COMPL | ICSR_WRITE_COMPL);
        amcc_write_opreg (mcsr, AMCC_OP_REG_MCSR);
    }
    else {
        if (icsr & ICSR_RT_COMPL_INT) {
            mcsr &= ~MCSR_RTC_ENABLE;
            icsr &= ~ICSR_READ_COMPL;
            amcc_write_opreg (mcsr, AMCC_OP_REG_MCSR);
        }
        if (icsr & ICSR_WT_COMPL_INT) {
            mcsr &= ~MCSR_WTC_ENABLE;
            icsr &= ~ICSR_WRITE_COMPL;
            amcc_write_opreg (mcsr, AMCC_OP_REG_MCSR);
        }
    }

    /* clear the interrupt source */

    amcc_write_opreg (icsr, AMCC_OP_REG_INTCSR);

    /* FIFO interrupts */

    if (icsr & (ICSR_TARGET_ABORT | AMCC_IRQ_MASTER_ABORT)) {

        /*
         *  Severe error during a PCI transaction. We stop every
         *  process using the FIFOs.
         */

        if (icsr & ICSR_MASTER_ABORT) {
            a2p_fifo.irq_code = AMCC_IRQ_MASTER_ABORT;
            p2a_fifo.irq_code = AMCC_IRQ_MASTER_ABORT;
        }
        else {
            a2p_fifo.irq_code = AMCC_IRQ_TARGET_ABORT;
            p2a_fifo.irq_code = AMCC_IRQ_TARGET_ABORT;
        }

        a2p_fifo.bytes_done = 0;
        p2a_fifo.bytes_done = 0;

        wake_up_interruptible (&(a2p_fifo.wq));
        wake_up_interruptible (&(p2a_fifo.wq));
    }

    if (icsr & ICSR_RT_COMPL_INT) {

        /* read (PCI -> Add-on) transfer complete. */

        int done = p2a_fifo.copy_size - amcc_read_opreg (AMCC_OP_REG_MRTC);

        /* this is by the OS point of view */

        p2a_fifo.irq_code = AMCC_IRQ_WRITE_OK;

        p2a_fifo.copy_size -= done;
        p2a_fifo.bytes_done += done;

        if (p2a_fifo.async) {

            /*
             *  ASYNC write terminated. The amount of data transferred
             *  is stored in p2a_fifo.bytes_done.
             */

            clear_bit (AMCC_P2A_LOCK_BIT, &lock);
            p2a_fifo.async = 0L;
        }
        else
            wake_up_interruptible (&(p2a_fifo.wq));
    }

    if (icsr & ICSR_WT_COMPL_INT) {

        /* write (Add-on -> PCI) transfer complete. */

        int done = a2p_fifo.copy_size - amcc_read_opreg (AMCC_OP_REG_MWTC);

        /* this is by the OS point of view */

        a2p_fifo.irq_code = AMCC_IRQ_READ_OK;

        a2p_fifo.copy_size -= done;
        a2p_fifo.bytes_done += done;

        wake_up_interruptible (&(a2p_fifo.wq));
    }

    /* mailbox interrupts */

    if (icsr & ICSR_IN_MB_INT) {

        /* handle incoming mailbox full */

        u16 imbef = (((amcc_read_opreg (AMCC_OP_REG_MBEF)) & MBEF_IN_ST_MASK) >> 16);

        pr_debug ("amcc: bh: IMB full\n");
        pr_debug ("amcc: bh: IMBEF = %04lx\n", imbef);

        /* are we testing IRQ speed? */

        if (test_bit (AMCC_TEST_IRQ_BIT, &lock)) {
            wake_up_interruptible (&mbwq);
            return;
        }
    }

    if (icsr & ICSR_OUT_MB_INT) {

        /* handle outgoing mailbox empty */

        pr_debug ("amcc: bh: OMB empty\n");
        pr_debug ("amcc: bh: MBEF = %08lx\n", amcc_read_opreg (AMCC_OP_REG_MBEF));
    }
}
