/*
 *  ab.c -- Memory mapped AutoBahn driver
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *           Toni Giorgino      (toni@pcape2.pi.infn.it)
 *
 *  ab.c,v 2.5 1997/11/10 09:26:25 acister Exp
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
 *  This file contains all the driver code.
 *
 *--------------------------------------------------------------------*/

#ifndef __KERNEL__
# define __KERNEL__
#endif

#include <linux/config.h>       /* for CONFIG_PCI */

#ifndef CONFIG_BIGPHYS_AREA
# error This module needs the kernel bigphysarea patch.
#endif

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
#include <linux/bigphysarea.h>  /* for buffer allocation */
#include <asm/io.h>             /* for inb(), outb() etc. */
#include <asm/system.h>
#include <asm/pgtable.h>

#include "s5933.h"              /* S5933 definitions */
#include "ab.h"

/*--------------------------------------------------------------------
 *
 *  Module flags.
 *  These variables can be overridden at load time by the insmod
 *  command.
 *
 *--------------------------------------------------------------------*/
/*
int vid = PCI_VENDOR_ID_AMCC;
int did = PCI_DEVICE_ID_AMCC_S5933;
*/
int vid = 0x10DC;
int did = 0x11;

/*--------------------------------------------------------------------
 *
 *  Static variables.
 *  These can only be seen by functions in this file.
 *
 *--------------------------------------------------------------------*/

static struct autobahn dev;     /* General data */

static u_short id_cnt = 0;      /* ID counter */

/* wait queues */

static struct wait_queue* p2awq;
static struct wait_queue* a2pwq;

/*--------------------------------------------------------------------
 *
 *  Function prototypes
 *
 *--------------------------------------------------------------------*/

int  init_module(void);
void cleanup_module(void);

static int  ab_select(struct inode*, struct file*, int, select_table*);
static int  ab_ioctl(struct inode*, struct file*, unsigned int, unsigned long);
static int  ab_mmap(struct inode*, struct file*, struct vm_area_struct*);
static int  ab_open(struct inode*, struct file*);
static void ab_release(struct inode*, struct file*);

static int  ab_init_dev(struct autobahn*, u_char, u_char);
static void ab_free_dev(void);
static void ab_irq_handler(int, void*, struct pt_regs*);

/*--------------------------------------------------------------------
 *
 *  Linux kernel main entry point.
 *  For a definition of its fields see <linux/fs.h>.
 *
 *--------------------------------------------------------------------*/

static struct file_operations ab_fops = {
    NULL,               /* lseek()   */
    NULL,               /* read()    */
    NULL,               /* write()   */
    NULL,               /* readdir() */
    ab_select,          /* select()  */
    ab_ioctl,           /* ioctl()   */
    ab_mmap,            /* mmap()    */
    ab_open,            /* open()    */
    ab_release,         /* release() */
    NULL,               /* fsync()   */
    NULL,               /* fasync()  */
    NULL,               /* check_media_change() */
    NULL,               /* revalidate() */
};

/*--------------------------------------------------------------------
 *
 *  Standard module initialization function.
 *  This function scans the PCI bus looking for the right board.
 *
 *--------------------------------------------------------------------*/

int
init_module(void)
{
    extern struct autobahn dev;
    extern int             vid, did;

    int result = 0;

    pr_debug(AB_NAME ": >>> START <<<\n");
    printk(KERN_WARNING AB_NAME ": init ab driver\n");

    /* shared data page allocation */

    dev.ab_drv_p = bigphysarea_alloc(sizeof(struct ab_driver));
    if (dev.ab_drv_p == NULL) {
        printk(KERN_WARNING AB_NAME ": init_mod: no memory for ab_driver structure\n");
        return -ENOMEM;
    }

    pr_debug(AB_NAME ": init_mod: drv: %08lx\n", dev.ab_drv_p);
    pr_debug(AB_NAME ": init_mod: high_memory: %08lx\n", high_memory);

    dev.ab_drv = (struct ab_driver*) ioremap((u_long) dev.ab_drv_p, sizeof(struct ab_driver));
    memset(dev.ab_drv, 0, sizeof(struct ab_driver));

    /* device initialization */

    if (pcibios_present()) {

        /* PCI bus scan, only first device reported */

        u_char bus;
        u_char function;

        result = pcibios_find_device((u_short) vid, (u_short) did, 0, &bus, &function);

        if (result != PCIBIOS_SUCCESSFUL) {
            printk(KERN_WARNING AB_NAME ": pci error: %s\n", pcibios_strerror(result));
            return -ENODEV; /* device not found */
        }

        /* store device and function numbers */

        dev.ab_bus = bus;
        dev.ab_function = function;

        pr_debug(AB_NAME ": init_mod: device found: BUS:%02x, FN:%02x\n", bus, function);

        /* major number allocation */

        result = register_chrdev(AB_MAJOR, AB_NAME, &ab_fops);
        if (result) {
            printk(KERN_WARNING AB_NAME ": unable to get major number\n");
            return -EIO;
        }

        /* device initialization */

        result = ab_init_dev(&dev, bus, function);
    }

    return result;
}

/*--------------------------------------------------------------------
 *
 *  Standard module release function.
 *
 *--------------------------------------------------------------------*/

void
cleanup_module(void)
{
    ab_free_dev();

    pr_debug(AB_NAME ": >>> STOP <<<\n");
}

/*--------------------------------------------------------------------
 *
 *  Standard open() entry point.
 *  Very simple for the moment.
 *
 *--------------------------------------------------------------------*/

static int
ab_open(struct inode* inode, struct file* file)
{
    struct ab_procdata* pd;

    pr_debug(AB_NAME ": open\n");

    /* allocate private data */

    pd = kmalloc(sizeof(struct ab_procdata), GFP_KERNEL);
    if (pd == NULL) {
        printk(KERN_WARNING AB_NAME ": open: no memory for private data\n");
        return -ENOMEM;
    }

    memset(pd, 0, sizeof(struct ab_procdata));

    /* generate unique ID */

    pd->pd_id = (id_cnt << 16) | (current->pid);
    pr_debug(AB_NAME ": open: ID: %d\n", pd->pd_id);

    file->private_data = pd;    /* save information */

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
ab_release(struct inode* inode, struct file* file)
{
    struct ab_procdata* pd = file->private_data;

    pr_debug(AB_NAME ": close\n");

    if (pd) {
        pr_debug(AB_NAME ": close: ID: %d\n", pd->pd_id);

        /* bigphys buffer */

        if (pd->pd_phys)
            bigphysarea_free(pd->pd_phys, pd->pd_size);

        kfree(pd);
        file->private_data = NULL;
    }

    MOD_DEC_USE_COUNT;
}

/*--------------------------------------------------------------------
 *
 *  Standard select() entry point.
 *
 *--------------------------------------------------------------------*/

static int
ab_select(struct inode* inode, struct file* file, int sel_type, select_table* wait)
{
    return 0;
}

/*--------------------------------------------------------------------
 *
 *  Standard ioctl() entry point.
 *
 *--------------------------------------------------------------------*/

static int
ab_ioctl(struct inode* inode, struct file* file, unsigned int cmd, unsigned long arg)
{
    extern struct autobahn dev;

    struct ab_driver* drv = dev.ab_drv;
    int               ret = 0;

    pr_debug(AB_NAME ": ioctl: cmd: %08lx, arg: %08lx\n", cmd, arg);

    switch (cmd) {

        /* general */

        case ABIORESET:             /* reset the whole board */
            (dev.ab_opregs)->icsr = 0;
            (dev.ab_opregs)->reset = ~0;    /* FIFO, MB and start of add-on */
            (dev.ab_opregs)->ao_reset = 0;  /* end of add-on */
            (dev.ab_opregs)->mwtc = 0;
            (dev.ab_opregs)->mrtc = 0;
            break;

        case ABIOGETOFFSET:         /* get the opreg offset relative to page boundary */
        {
            caddr_t*      addr = (caddr_t*) arg;
            unsigned long aboffs = ((unsigned long) dev.ab_opregs_p) & ~PAGE_MASK;

            ret = verify_area(VERIFY_WRITE, (void*) addr, sizeof(caddr_t));
            if (ret)
                return ret;
            put_user(aboffs, addr);
            break;
        }

        case ABIORESETADDON:        /* reset the add-on */
            (dev.ab_opregs)->ao_reset = 1;
            udelay(20UL);
            (dev.ab_opregs)->ao_reset = 0;
            break;

        case ABIOSETLATENCY:        /* set PCI latency timer */
            if (arg < 0x20 || arg > 0xF8) {
                printk(KERN_WARNING AB_NAME ": ioctl: invalid PCI latency %02x\n", arg);
                return -EINVAL;
            }

            pcibios_write_config_byte(dev.ab_bus, dev.ab_function, PCI_LATENCY_TIMER, arg);
            break;

        case ABIOSETBUFFER:         /* allocate buffer for calling process */
        {
            caddr_t             buf;
            struct ab_procdata* pd = file->private_data;

            /* buffer allocation */

            buf = bigphysarea_alloc((size_t) arg);
            if (buf == NULL) {
                printk(KERN_WARNING AB_NAME ": ioctl: no memory for buffer\n");
                return -ENOMEM;
            }

            pr_debug(AB_NAME ": ioctl: allocated %ld bytes at %08lx\n", (size_t) arg, buf);
            printk(KERN_WARNING AB_NAME ": ioctl: allocated %ld bytes at %08lx\n", (size_t) arg, buf);

            pd->pd_phys = buf;
            pd->pd_size = (size_t) arg;
            break;
        }

        case ABIOGETPADDR: {
            caddr_t*            addr = (caddr_t*) arg;
            struct ab_procdata* pd = file->private_data;

            ret = verify_area(VERIFY_WRITE, (void*) addr, sizeof(caddr_t));
            if (ret)
                return ret;

            put_user(pd->pd_phys, addr);
            break;
        }

        /* FIFO related */

        case ABIOWAITDMA:           /* wait for DMA completion */
        {
            struct ab_procdata* pd = file->private_data;
            struct wait_queue** wq;

            u_int bitnum;

            pr_debug(AB_NAME ": ioctl: ID waiting: %d\n", pd->pd_id);

            if (arg == AB_WAIT_P2A) {
                /* read (PCI -> Add-on) */

                bitnum = AB_FLAG_P2ADONE;
                wq = &p2awq;
            }

            if (arg == AB_WAIT_A2P) {
                /* write (Add-on -> PCI) */

                bitnum = AB_FLAG_A2PDONE;
                wq = &a2pwq;
            }

            /* already finished? */

            cli();      /* do not disturb */

            if (test_bit(bitnum, &(drv->drv_lock))) {
                sti();
                break;
            }

            /* no, go to sleep */

            interruptible_sleep_on(wq);
            clear_bit(bitnum, &(drv->drv_lock));
            break;
        }

        case ABIOA2PDMA:            /* setup Add-on -> PCI DMA transfer */
        {
            struct ab_procdata* pd = file->private_data;

            pr_debug(AB_NAME ": ioctl: ID start DMA: %d\n", pd->pd_id);

            (dev.ab_opregs)->mwar = (u_int) pd->pd_phys;

            if ((size_t) arg > AB_MAX_DMA) {
                /* split into several DMA blocks */

                (dev.ab_opregs)->mwtc = (size_t) AB_MAX_DMA;
                drv->drv_wr_remain = (size_t) arg - AB_MAX_DMA;
            }
            else {
                /* fits all in one DMA block */

                (dev.ab_opregs)->mwtc = (size_t) arg;
                drv->drv_wr_remain = 0;
            }

            /* printk (KERN_WARNING AB_NAME ": dmasetup remaining: %08lx\n", drv->drv_wr_remain); */

            (dev.ab_opregs)->mcsr = MCSR32_WRITE_TR_ENABLE | MCSR32_WRITE_PRIORITY;
            (dev.ab_opregs)->int_selection = ICSR32_INT_WRC_ENABLE;
            break;
        }

        /* Pass-thru related */

        case ABIOGETPTSIZE:         /* get pass-thru area 1 size */
        {
            u_int* addr = (u_int*) arg;

            ret = verify_area(VERIFY_WRITE, (void*) addr, sizeof(u_int));
            if (ret)
                return ret;

            put_user(dev.ab_ptsize, addr);
            break;
        }

        case ABDELAY: /* use kernel udelay */
        {
            unsigned long usec = (unsigned long) arg;
            udelay(usec);
            break;
        }

        /* End of ioctls */

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

/*--------------------------------------------------------------------
 *
 *  Standard mmap() entry point.
 *  We use the vma->vm_offset field to select the different address spaces
 *  that will be mapped to user space. The board memory map is divided in
 *  units of pages and is as follows:
 *
 *  0:  PCI operation registers
 *  1:  Pass-thru area 1
 *  3:  Shared page containing global driver status
 *  3:  DMA buffer
 *
 *--------------------------------------------------------------------*/

static int
ab_mmap(struct inode* inode, struct file* file, struct vm_area_struct* vma)
{
    extern struct autobahn dev;

    struct ab_procdata* pd = file->private_data;
    size_t              size, maxsize;
    caddr_t             addr = 0;
    u_int               ret;

    pr_debug(AB_NAME ": mmap: start: %08lx, end: %08lx, offset: %08lx, flags: %08lx\n",
             vma->vm_start, vma->vm_end, vma->vm_offset, vma->vm_flags);
    printk(KERN_WARNING AB_NAME ": mmap: start: %08lx, end: %08lx, offset: %08lx, flags: %08lx\n",
           vma->vm_start, vma->vm_end, vma->vm_offset, vma->vm_flags);

    /* check off(dev.ab_opregsset */

    switch (vma->vm_offset) {

        /* PCI operation registers */

        case AB_OPREGS_OFFSET:
            maxsize = PAGE_ALIGN(AMCC_OP_REG_SIZE);
            addr = (caddr_t)(((u_long) dev.ab_opregs_p) & PAGE_MASK);   /* address, page aligned */
            break;

        /* pass-thru area 1 */

        case AB_PT1_OFFSET:
            maxsize = PAGE_ALIGN(dev.ab_ptsize);
            addr = dev.ab_ptaddr_p;     /* address */
            break;

        /* global driver status */

        case AB_DRV_OFFSET:
            maxsize = PAGE_ALIGN(sizeof(struct ab_driver));
            addr = dev.ab_drv_p; /* address */
            break;

        /* main buffer (already allocated by ioctl(ABIOGETBUFFER)) */

        case AB_BUF_OFFSET:
            maxsize = pd->pd_size;      /* already page aligned */
            addr = pd->pd_phys;
            break;

        default:
            return -EINVAL;
    }

    /* required size */

    size = vma->vm_end - vma->vm_start;

    printk(KERN_WARNING AB_NAME ": mmap: req-size: %08lx, maxsize: %08lx\n", size, maxsize);

    /* actual mapping */

    if (size > maxsize) /* check dimensions */
        return -EINVAL;

    vma->vm_offset = (u_long) addr; /* set real address */
    pr_debug(AB_NAME "vma->vm_offset %lx\n", vma->vm_offset);
    if (vma->vm_offset & ~PAGE_MASK) /* page aligned ? */
        return -ENXIO;

#ifdef __i486__
    /* This disables high-memory caching (NEEDED?) */

    if (vma->vm_offset >= high_memory)
        pgprot_val(vma->vm_page_prot) |= _PAGE_PCD;
#endif

    /* map pages */

    ret = remap_page_range(vma->vm_start, vma->vm_offset, size, vma->vm_page_prot);

    if (ret)
        return -EAGAIN;

    vma->vm_inode = inode;
    inode->i_count++;

    return 0;
}

/*--------------------------------------------------------------------
 *
 *  Device initialization.
 *  This function initializes the board. It requests an interrupt and
 *  maps the operation registers in kernel space. The major number
 *  has already been allocated by the module init function.
 *
 *--------------------------------------------------------------------*/

static int
ab_init_dev(struct autobahn* dev, u_char bus, u_char function)
{
    u32    data;
    u_long ret;
    u_int  mask;

    /*
     * IRQ facilities setup.
     * Register the irq with the system.
     */

    /* to avoid plug'n'pray conflict we set the interrupt manually */
    /* ret = (u_long) pcibios_write_config_byte (bus, function, PCI_INTERRUPT_LINE, 11); */

    ret = (u_long) pcibios_read_config_byte(bus, function, PCI_INTERRUPT_LINE, &(dev->ab_irq));

    if (ret == PCIBIOS_SUCCESSFUL) {
        pr_debug(AB_NAME ": init: IRQ num: %d\n", dev->ab_irq);

        if (request_irq(dev->ab_irq, ab_irq_handler, SA_INTERRUPT, AB_NAME, NULL)) {
            printk(KERN_WARNING AB_NAME ": can't install handler for irq %d\n", dev->ab_irq);

            dev->ab_irq = 0;
            ab_free_dev();
            return -EIO;
        }
    }
    else {
        printk(KERN_WARNING AB_NAME ": can't read PCI interrupt register\n");
        dev->ab_irq = 0;
        ab_free_dev();
        return -EIO;
    }

    /*
     * The following section retrieve from PCI_BASE_ADDRESS_0 the address
     * of the S5933 operations registers.
     * We presume that their size is the standard one (16 DWORD).
     */

    ret = pcibios_read_config_dword(bus, function, PCI_BASE_ADDRESS_0, (unsigned int*) &data);

    if (ret != PCIBIOS_SUCCESSFUL) {
        printk(KERN_WARNING AB_NAME ": reading BADR[0]: error %s\n", pcibios_strerror(ret));

        ab_free_dev();
        return -EIO;
    }

    pr_debug(AB_NAME ": init: OPREGS: %08lx\n", data);

    /* Check for I/O or Memory Mapped Operation Registers */

    if (!(data & PCI_BASE_ADDRESS_SPACE)) {

        /* memory mapped */

        data &= PCI_BASE_ADDRESS_MEM_MASK;
        dev->ab_opregs_p = (caddr_t) data;
        dev->ab_opregs = (struct s5933_opreg*) ioremap(data, AMCC_OP_REG_SIZE);
    }
    else {

        /* I/O mapped -> error. We need a memory mapped chip. */

        printk(KERN_WARNING AB_NAME ": MCSR not memory mapped\n");

        ab_free_dev();
        return -EIO;
    }

    /* pass-thru area -- see ab.h for AB_PT_NUM */

    ret = pcibios_read_config_dword(bus, function, AB_PT_NUM, &data);

    if (ret != PCIBIOS_SUCCESSFUL) {
        printk(KERN_WARNING AB_NAME ": reading PT1 info: error %s\n", pcibios_strerror(ret));

        ab_free_dev();
        return -EIO;
    }

    /* we only support memory mapped PT regions */

    if (data & PCI_BASE_ADDRESS_SPACE_IO) {
        printk(KERN_WARNING AB_NAME ": PT1 not mapped in memory.\n");

        ab_free_dev();
        return -EIO;
    }

    /* get PT configuration */

    cli();
    pcibios_write_config_dword(bus, function, AB_PT_NUM, ~0);
    pcibios_read_config_dword(bus, function, AB_PT_NUM, &mask);
    pcibios_write_config_dword(bus, function, AB_PT_NUM, data);
    sti();

    pr_debug(AB_NAME ": init: PT1 - mask %08lx, current %08lx\n", mask, data);

    /* extract data */

    mask &= PCI_BASE_ADDRESS_MEM_MASK;
    data &= PCI_BASE_ADDRESS_MEM_MASK;

    dev->ab_ptsize = (~mask) + 1;
    dev->ab_ptaddr_p = (caddr_t) data;
    dev->ab_ptaddr = ioremap(data, dev->ab_ptsize);

    pr_debug(AB_NAME ": init: PT1 - dim: %d, phys: %08lx, virt: %08lx\n",
             dev->ab_ptsize, data, dev->ab_ptaddr);

    /* board reset */

    (dev->ab_opregs)->p2a_reset = 1; /* P2A FIFO */
    (dev->ab_opregs)->a2p_reset = 1; /* A2P FIFO */
    (dev->ab_opregs)->mb_reset = 1;  /* mailboxes */

    (dev->ab_opregs)->ao_reset = 1; /* Add-on */
    udelay(20UL);
    (dev->ab_opregs)->ao_reset = 0;

    (dev->ab_opregs)->int_selection = 0; /* no interrupts */

    (dev->ab_opregs)->mrtc = 0;
    (dev->ab_opregs)->mwtc = 0;

    /* some debug info */

    pr_debug(AB_NAME ": init: MCSR: %08lx\n", (dev->ab_opregs)->mcsr);
    pr_debug(AB_NAME ": init: ICSR: %08lx\n", (dev->ab_opregs)->icsr);

    return 0;
}

/*--------------------------------------------------------------------
 *
 *  Device finalization.
 *  It selectively frees all the resources allocated by the driver.
 *
 *--------------------------------------------------------------------*/

static void
ab_free_dev()
{
    extern struct autobahn dev;

    /* leave the board in a clean state */

    (dev.ab_opregs)->icsr = 0;
    (dev.ab_opregs)->mcsr = 0;

    (dev.ab_opregs)->mrtc = 0;
    (dev.ab_opregs)->mwtc = 0;

    (dev.ab_opregs)->p2a_reset = 1; /* P2A FIFO */
    (dev.ab_opregs)->a2p_reset = 1; /* A2P FIFO */
    (dev.ab_opregs)->mb_reset = 1;  /* mailboxes */

    (dev.ab_opregs)->ao_reset = 1; /* Add-on */
    udelay(20UL);
    (dev.ab_opregs)->ao_reset = 0;

    /* operation registers */

    if (dev.ab_opregs)
        iounmap(dev.ab_opregs);

    /* pass-thru region */

    if (dev.ab_ptaddr != NULL)
        iounmap(dev.ab_ptaddr);

    /* driver status  */

    if (dev.ab_drv_p) {
        iounmap(dev.ab_drv);
        bigphysarea_free(dev.ab_drv_p, sizeof(struct ab_driver));
    }

    /* major number & IRQ */

    if (dev.ab_irq)
        free_irq(dev.ab_irq, NULL);

    unregister_chrdev(AB_MAJOR, AB_NAME);
}

/*--------------------------------------------------------------------
 *
 *  Interrupt handler.
 *
 *--------------------------------------------------------------------*/

static void
ab_irq_handler(int irq, void* device, struct pt_regs* regs)
{
    extern struct autobahn dev;
    struct ab_driver*      drv = dev.ab_drv;

    /* first of all we get the clock */

    /* drv->drv_time = getclock (); */

    /* then we check :-) */

    if (irq != dev.ab_irq) {
        printk(KERN_WARNING AB_NAME ": irq: spurious interrupt!\n");
        return;
    }

    /* read interrupt code */

    if (!((dev.ab_opregs)->int_asserted))
        return;

    drv->drv_icsr = (dev.ab_opregs)->icsr;          /* local copy */

    /* (dev.ab_opregs)->int_selection = 0; */       /* clear interrupt source */

    pr_debug(AB_NAME ": irq: ICSR: %08lx\n", drv->drv_icsr);

    /*
     *  Disable bus-mastering if needed.
     *  We do it in a selective way.
     */

    if (drv->drv_icsr & ICSR32_DMA_ABORT) {

        /* PCI bus error */

        (dev.ab_opregs)->wtc_enable = 0;    /* stop transfer */
        (dev.ab_opregs)->rtc_enable = 0;

        (dev.ab_opregs)->wr_compl_en = 0;   /* disable interrupts */
        (dev.ab_opregs)->rd_compl_en = 0;
    }
    else {

        /* normal end of transfer */

        /* Add-on -> PCI bus DMA */

        if (drv->drv_icsr & ICSR32_WT_COMPL_INT) {

            /* clear interrupt source */

            (dev.ab_opregs)->int_code = ~0;

            if (drv->drv_wr_remain > 0) {

                /* quickly setup next dma transfer */

                if (drv->drv_wr_remain > AB_MAX_DMA) {

                    /* still more than 1 block? */

                    (dev.ab_opregs)->mwtc = (size_t) AB_MAX_DMA;
                    drv->drv_wr_remain -= AB_MAX_DMA;
                }
                else {

                    /* one more block left */

                    (dev.ab_opregs)->mwtc = drv->drv_wr_remain;
                    drv->drv_wr_remain = 0;
                }

                (dev.ab_opregs)->int_selection = ICSR32_INT_WRC_ENABLE;
            }
            else {

                /* done with dma */

                (dev.ab_opregs)->wtc_enable = 0;
                (dev.ab_opregs)->wr_compl_en = 0;

                set_bit(AB_FLAG_A2PDONE, &(drv->drv_lock));
                wake_up_interruptible(&a2pwq);
            }
        }

        /* PCI bus -> Add-on  DMA*/

        if (drv->drv_icsr & ICSR32_RT_COMPL_INT) {

            (dev.ab_opregs)->rtc_enable = 0;
            (dev.ab_opregs)->rd_compl_en = 0;
            (dev.ab_opregs)->int_code = ~0;

            set_bit(AB_FLAG_P2ADONE, &(drv->drv_lock));
            wake_up_interruptible(&p2awq);
        }

        /* printk (KERN_WARNING AB_NAME ": irq - remaining: %08lx\n", drv->drv_wr_remain); */
    }

    /*
     *  Handle interrupt
     */

    if (drv->drv_icsr & ICSR32_WT_COMPL_INT) {
        /* write (Add-on -> PCI) complete */
    }

    if (drv->drv_icsr & ICSR32_RT_COMPL_INT) {
        /* read (PCI -> Add-on) complete */
    }
}

/* end of code */
