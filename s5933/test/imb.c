/*
 *  pmb.c -- Test program for the S5933 mailbox set (ISA)
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *
 *  $Id: imb.c,v 2.5 1997/10/01 08:20:20 acister Exp $
 */

/**
 *  Test program for the S5933 mailbox set.
 *  It uses the developers' kit ISA Board connected to the PCI Board.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <linux/pci.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <asm/io.h>     /* for inl() and outl() */

#include "amcc.h"
#include "main.h"       /* for the IOCTLs */
#include "testisa.h"

/*
 *  Defines
 */

/*
 *  Flags & globals
 */

char       *pname;          /* program name */

static short imbox[4] =     /* for simpler access */
{
    AMCC_ISA_AIMB1,
    AMCC_ISA_AIMB2,
    AMCC_ISA_AIMB3,
    AMCC_ISA_AIMB4
};

static short ombox[4] =     /* for simpler access */
{
    AMCC_ISA_AOMB1,
    AMCC_ISA_AOMB2,
    AMCC_ISA_AOMB3,
    AMCC_ISA_AOMB4
};

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

    char command = 'x';     /* command to be executed */
    int mbnum = 0L;         /* number of mailbox */
    int tmp;
    __u32 data = ~0L;

    /* program name setup */

    pname = argv[0];

    /* no arguments */

    if (argc == 1) {
        usage (pname);
        exit (EXIT_SUCCESS);
    }

    /* option handling */

    while ((opt = getopt (argc, argv, "hr:w:t")) != EOF) {

        switch (opt) {

            case 'h':           /* help */
                usage (pname);
                exit (EXIT_SUCCESS);
                break;

            case 'w':           /* write */
                command = 'w';
                if (optarg)
                    mbnum = strtol_e (optarg, 'w', 10);
                break;

            case 'r':           /* read */
                command = 'r';
                if (optarg)
                    mbnum = strtol_e (optarg, 'r', 10);
                break;

            case 't':           /* IRQ timings */
                command = 't';
                mbnum = 1;
                break;

            case '?':           /* unknown option */
                exit (EXIT_FAILURE);
                break;

            default:
                abort ();
                break;
        }
    }

    /* controls */

    if (command == 'x')
        cleanup (EXIT_FAILURE, "no operation specified!");

    if ((mbnum < 1) || (mbnum > 4))
        cleanup (EXIT_FAILURE, "wrong mailbox number!");

    if (command == 'w') {
        if (optind == argc)
            cleanup (EXIT_FAILURE, "no argument specified!");
        else
            data = strtol_e (argv[optind], command, 16);
    }

    /* I/O permission on the ports */

    tmp = ioperm (AMCC_ISA_AIMB1, (8 * sizeof (__u32)), 1);
    if (tmp == -1)
        cleanup (EXIT_FAILURE, "can't acquire privilege on Mailboxes");

    /* interrupt handler */

    if (signal (SIGINT, int_handler) == SIG_IGN)
        signal (SIGINT, SIG_IGN);

    /* main ops */

    switch (command) {

        case 'r':               /* read */
            data = inl (imbox[mbnum - 1]);
            printf ("MB%d (0x%04x) = 0x%08x\n", mbnum, imbox[mbnum - 1], data);
            break;

        case 'w':               /* write */
            printf ("MB%d (0x%04x) = 0x%08x\n", mbnum, ombox[mbnum - 1], data);
            outl (data, ombox[mbnum - 1]);
            break;

        case 't':               /* IRQ timings */
        {
            struct timeval tv;

            gettimeofday (&tv, NULL);
            outl (tv.tv_usec, ombox[mbnum - 1]);
            break;
        }

        default:
            break;
    }

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
             "Perform operations on the Mailbox from the ISA side.\n\n"
             "-h\t\tprint this help.\n"
             "-rN\t\tread Mailbox N.\n"
             "-wN DATA\twrite DATA to Mailbox N.\n"
             "-t\t\tIRQ handling time test.\n",
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
