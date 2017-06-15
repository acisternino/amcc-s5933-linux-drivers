/*
 *  ppt.c -- Test program for the S5933 pass-thru
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *
 *  $Id: ppt.c,v 2.3 1997/09/23 15:04:39 acister Exp $
 *
 *  WARNING: If simple loops are used to R/W from the pass-thru, this
 *           code should NOT be optimized. Otherwise gcc will not generate
 *           some operations.
 *
 *  WARNING: This code is not complete. It is a snapshot of my work on the
 *           pass-thru areas and probably will not even compile.
 *           Please read the source before using.
 */

/**
 *  Test program for the S5933 pass-thru.
 *  It uses the real driver ("/dev/amcc") and the PCI interface.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <linux/pci.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <asm/io.h>     /* for inl() and outl() */

#include "amcc.h"
#include "main.h"       /* for the IOCTLs */

/*
 *  Defines
 */

#define MAP_FAILED  ((void *) -1)
#define PT_SIZE     (128 * 1024)
#define PT_SIZE_DW  (PT_SIZE / sizeof (__u32))
#define DELAY(a)    ({ u_int d = (((((a) * 377) / 1033) + 27) / 67); })

/*
 *  Flags & globals
 */

char       *pname;          /* program name */

static int  amcc_fd = 0L;   /* device */

__u32      *pt = NULL;      /* pass-thru area */

volatile __u32 data[PT_SIZE_DW];

/*
 *  Prototypes
 */

static void usage (const char *);
static void int_handler (int);
static void cleanup (int, char *);

/*
 *  Main program
 */

int
main (int argc, char *argv[])
{
    extern char *optarg;
    extern int optind;
    int opt = 0L;

    char command = 'x';         /* command to be executed */
    int cont;

    volatile __u32 dummy;
    int START;
    int MAX;
    int ppp;

    /* program name setup */

    pname = argv[0];

    /* no arguments */

    if (argc == 1) {
        usage (pname);
        exit (EXIT_SUCCESS);
    }

    /* option handling */

    while ((opt = getopt (argc, argv, "h")) != EOF) {

        switch (opt) {

            case 'h':           /* help */
                usage (pname);
                exit (EXIT_SUCCESS);
                break;

        case '?':               /* unknown option */
            exit (EXIT_FAILURE);
            break;

        default:
            abort ();
            break;
        }
    }

    /* controls */

    /* open the device */

    amcc_fd = open ("/dev/amcc", O_RDWR);
    if (amcc_fd == -1)
        cleanup (EXIT_FAILURE, "error opening device");

    /* interrupt handler */

    if (signal (SIGINT, int_handler) == SIG_IGN)
        signal (SIGINT, SIG_IGN);

    /* mmap pass-thru */

    pt = (__u32 *) mmap (NULL, PT_SIZE, (PROT_READ | PROT_WRITE), MAP_SHARED, amcc_fd, (off_t) 0L);
    if (pt == MAP_FAILED)
        cleanup (EXIT_FAILURE, "error mmapping pass-thru area");

    printf ("%s: pt = %08lx (%lu)\n", pname, pt, PT_SIZE);

    /* fill and check PT area - DO NOT OPTIMIZE!! */

    cont = 0;

#if 0
    if (*argv[1] == 'w') {

        for (ppp = 0; ppp < PT_SIZE_DW; ppp += 0x1000) {

            START = ppp;
            MAX = 0x1000;
            printf ("W START: %08x (%08x)\n", START, (START * 4));

            for (cont = START; cont < (START + MAX); cont++) {
                pt[cont] = cont;
                dummy = pt[cont];
            }
        }
    }

    for (ppp = 0x1000; ppp < PT_SIZE_DW; ppp += 0x1000) {

        MAX = 0x6;
        printf ("R START: %08x (%08x)\n", ppp, (ppp * 4));

        for (cont = 0; cont < PT_SIZE_DW; cont++) {
            /*
            if (pt[cont] != cont)
                printf ("error!! [%08x] %08x\n", cont, pt[cont]);
            data[cont] = pt[cont];
            */

            ppp = ((int)(&(pt[cont])) - (int)pt) & 0x1FFFF;
            printf ("%04x  %05x%c  %08x\n", cont, ppp, ((ppp & (1<<15)) ? '*' : ' '), pt[cont]);
        }
    }

    pt[0] = 0xDEAD;
    dummy = pt[0];
    pt[1] = 0xDEAE;
    dummy = pt[1];
#endif

    pt[0x1FFE] = 0xBEEC;
    dummy = pt[0x1FFE];
    pt[0x1FFF] = 0xBEED;
    dummy = pt[0x1FFF];

    pt[0x2000] = 0xBEEE;
    dummy = pt[0x2000];
    pt[0x2001] = 0xBEEF;
    dummy = pt[0x2001];

    data[0] = pt[0];
    data[1] = pt[1];
    data[2] = pt[0x1FFE];
    data[3] = pt[0x1FFF];
    data[4] = pt[0x2000];
    data[5] = pt[0x2001];

    printf ("     pt[0]: %08X\n", data[0]);
    printf ("     pt[1]: %08X\n", data[1]);
    printf ("pt[0x1FFE]: %08X\n", data[2]);
    printf ("pt[0x1FFF]: %08X\n", data[3]);
    printf ("pt[0x2000]: %08X\n", data[4]);
    printf ("pt[0x2001]: %08X\n", data[5]);

    /* close down */

    cleanup (EXIT_SUCCESS, NULL);

    return EXIT_SUCCESS;        /* to make gcc happy */
}

/*
 *  usage ()
 *  Print some information about the program.
 */

static void
usage (const char *pname)
{
    fprintf (stderr, "Usage: %s [OPTION] [DATA]\n"
             "Perform operations on the Pass-thru from the PCI side.\n\n"
             "-h\t\tprint this help.\n",
             pname);
}

/*
 *  cleanup ()
 *  Cleanup function. It is called on every exit situation.
 *  When ^C is pressed it is called by a small interrupt handler.
 */

static void
cleanup (int exitcode, char *msg)
{
    if (pt != MAP_FAILED)
        munmap (pt, PT_SIZE);       /* pass-thru area */

    if (amcc_fd)
        close (amcc_fd);            /* real device */

    if (msg) {
        fprintf (stderr, "%s: %s\n", pname, msg);
        perror (pname);
    }
    exit (exitcode);
}

/*
 *  int_handler ()
 *  Small interrupt handler. It simply calls the cleanup() function
 *  with the correct arguments
 */

static void
int_handler (int signum)
{
    fprintf (stderr, "%s: Interrupt\n", pname);
    cleanup (EXIT_FAILURE, NULL);
}
