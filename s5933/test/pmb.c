/*
 *  pmb.c -- Test program for the S5933 mailbox set
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *
 *  $Id: pmb.c,v 2.4 1997/05/16 14:17:32 acister Exp $
 */

/**
 *  Test program for the S5933 mailbox set.
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
#include <sys/types.h>
#include <sys/ioctl.h>
#include <asm/io.h>         /* for inl() and outl() */

#include "amcc.h"
#include "main.h"           /* for the IOCTLs */
#include "testpci.h"

/*
 *  Defines
 */

/*
 *  Flags & globals
 */

char       *pname;          /* program name */

static int  amcc_fd = 0L;   /* device */

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
    struct amcc_mbox mb;
    __u32 data;

    /* program name setup */

    pname = argv[0];

    /* no arguments */

    if (argc == 1) {
        usage (pname);
        exit (EXIT_SUCCESS);
    }

    /* option handling */

    while ((opt = getopt (argc, argv, "hr:w:tp")) != EOF) {

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

            case 't':           /* IRQ handling time measure */
                command = 't';
                mbnum = 1;
                break;

            case 'p':           /* miscellaneous tests */
                command = 'p';
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

    /* open the device */

    amcc_fd = open ("/dev/amcc", O_RDWR);
    if (amcc_fd == -1)
        cleanup (EXIT_FAILURE, "error opening device");

    /* interrupt handler */

    if (signal (SIGINT, int_handler) == SIG_IGN)
        signal (SIGINT, SIG_IGN);

    /* main ops */

    switch (command) {

        case 'r':               /* read */
            data = mbnum;

            if ((ioctl (amcc_fd, AMCCIOREADMB, &data)) == -1)
                cleanup (EXIT_FAILURE, "error reading mailbox");

            printf ("MB%d = 0x%08x\n", mbnum, data);
            break;

        case 'w':               /* write */
            mb.num = mbnum;
            mb.data = data;

            printf ("MB%d = 0x%08x\n", mbnum, data);
            if ((ioctl (amcc_fd, AMCCIOWRITEMB, &mb)) == -1)
                cleanup (EXIT_FAILURE, "error writing mailbox");
            break;

        case 't':               /* IRQ speed test */
        {
            struct timeval tv;
            struct amcc_timings tim;

            /* we want IRQs on incoming mailbox full */

            if ((ioctl (amcc_fd, AMCCIOSETIMBIRQ, IMBIRQ (1, 0))) == -1)
                cleanup (EXIT_FAILURE, "error setting IRQ for mailbox");

            /* OK, now we wait */

            ioctl (amcc_fd, AMCCIOGETIRQTIME, &tim);
            gettimeofday (&tv, NULL);

            /* print results */

           printf ("start: %6ld\n", tim.start);
           printf ("  irq: %6ld %6ld %6ld\n", tim.irq, (tim.irq - tim.start), (tim.irq - tim.start));
           printf ("   bh: %6ld %6ld %6ld\n", tim.bh, (tim.bh - tim.irq), (tim.bh - tim.start));
           printf ("ioctl: %6ld %6ld %6ld\n", tim.ioctl, (tim.ioctl - tim.bh), (tim.ioctl - tim.start));
           printf ("  end: %6ld %6ld %6ld\n", tv.tv_usec, (tv.tv_usec - tim.ioctl), (tv.tv_usec - tim.start));

            if ((ioctl (amcc_fd, AMCCIOCLEARMBIRQ)) == -1)
                cleanup (EXIT_FAILURE, "error clearing IRQ for mailbox");
            break;
        }

        case 'p':               /* miscellaneous tests */
        {
            __u32 data;

            if ((ioctl (amcc_fd, AMCCIOTEST, &data)) == -1)
                cleanup (EXIT_FAILURE, "error performing tests");

            printf ("data: %ld\n", data);
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
             "Perform operations on the Mailbox from the PCI side.\n\n"
             "-h\t\tprint this help.\n"
             "-rN\t\tread Mailbox N.\n"
             "-wN DATA\twrite DATA to Mailbox N.\n"
             "-t\t\tIRQ handling time measure\n",
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
    if (amcc_fd)
        close (amcc_fd);        /* real device */

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
