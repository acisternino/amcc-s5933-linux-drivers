/*
 *  ififo.c -- AMCC FIFO test program on the ISA bus
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *
 *  $Id: ififo.c,v 2.3 1997/05/07 09:07:34 acister Exp $
 */

/**
 *  FIFO test program on the ISA bus.
 *  This program can perform different operations on the PCI to Add-on FIFO.
 *  It uses the developers' kit ISA Board connected to the PCI Board.
 *  The exit phase needs cleanup.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <asm/io.h>     /* for inl() and outl() */

#include "amcc.h"
#include "testisa.h"

/*
 *  Defines
 */

#define BUF_SIZE    (64 * 1024)     /* buffer size in bytes */
#define MAP_FAILED  ((void *) -1)

/*
 *  Flags & globals
 */

static int  binary = 0L;
static int  randf = 0L;
static int  cont = 0L;

char       *pname;          /* program name */

static char *fname = NULL;  /* I/O file */
static int  file = 0L;

static __u32 *buf = NULL;
static int  fsize;

/*
 *  Prototypes
 */

static void usage (const char *);
static void int_handler (int);
static void cleanup (int, char *);
static void print_fifo_data (__u32, int);
static void write_fifo_from_file (int);
static void write_fifo_from_buf (int);

/*
 *  Main program
 */

int
main (int argc, char *argv[])
{
    extern char *optarg;
    extern int  optind;
    int         opt = 0L;

    char        command = 'x';  /* command to be executed */
    int         num = 0L;       /* number of DWORD for I/O */
    int         tmp;
    __u32       data;

    /* program name setup */

    pname = argv[0];

    /* no arguments */

    if (argc == 1) {
        usage (pname);
        exit (EXIT_SUCCESS);
    }

    /* random numbers generator intialization */

    srand ((unsigned int) time (NULL));

    /* option handling */

    while ((opt = getopt (argc, argv, "hw::r::epf:cba::")) != EOF) {

        switch (opt) {

            case 'h':           /* help */
                usage (pname);
                exit (EXIT_SUCCESS);
                break;

            case 'w':           /* write */
               command = 'w';
               if (optarg)
                    num = strtol_e (optarg, 'w', 10);
               break;

            case 'r':           /* read */
                command = 'r';
                if (optarg)
                    num = strtol_e (optarg, 'r', 10);
                break;

            case 'e':           /* empty */
                command = 'e';
                break;

            case 'p':           /* pipe */
                command = 'p';
                break;

            case 'f':           /* file */
                fname = optarg;
                break;

            case 'c':           /* continuous read */
                cont = 1;
                break;

            case 'b':           /* binary I/O */
                binary = 1;
                break;

            case 'a':
                randf = 1;
                if (optarg)
                    srand ((unsigned int) strtol_e (optarg, 'a', 10));
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

    if ((command == 'p') && (fname != NULL))
        cleanup (EXIT_FAILURE, "no file with pipe option");

    if (cont && command != 'r')
        cleanup (EXIT_FAILURE, "continuous flag without read option");

    if (command == 'w' && (num == 0L) && (fname == NULL))
        cleanup (EXIT_FAILURE, "nothing to write");

    if (command == 'r' && (num == 0L))
        num++;

    /* IO permission on the ports */

    tmp = ioperm (AMCC_ISA_AFIFO, sizeof (__u32), 1);
    if (tmp == -1)
        cleanup (EXIT_FAILURE, "can't acquire privilege on FIFO");

    tmp = ioperm (AMCC_ISA_AGCSTS, sizeof (__u32), 1);
    if (tmp == -1)
        cleanup (EXIT_FAILURE, "can't acquire privilege on AGCSTS");

    /* interrupt handler */

    if (signal (SIGINT, int_handler) == SIG_IGN)
        signal (SIGINT, SIG_IGN);

    /* I/O file management */

    if (fname != NULL) {

        /* normal file */

        int flags;

        if (fname[0] != '-') {

            if (command == 'w')
                flags = O_RDONLY;                       /* from file to fifo */
            else
                flags = O_RDWR | O_CREAT | O_TRUNC;     /* from fifo to file */

            file = open (fname, flags, (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH));
        }
        else {
            /* stdio */

            if ((command == 'r') || (command == 'e')) {
                if (binary && (isatty (STDOUT_FILENO)))
                    cleanup (EXIT_FAILURE, "error: binary output to a terminal");
                file = STDOUT_FILENO;
            }
            else
                file = STDIN_FILENO;
        }

        if (file == -1)
            cleanup (EXIT_FAILURE, "error opening file");
    }

    /* some checks on the file */

    if (file != 0L) {

        struct stat file_stat;
        char *msg = NULL;

        if ((fstat (file, &file_stat)) == -1)
            cleanup (EXIT_FAILURE, "error statting file");

        if (S_ISDIR (file_stat.st_mode))
            msg = "error: file is a directory";

        if (S_ISSOCK (file_stat.st_mode))
            msg = "error: file is a socket";

        if (msg)
            cleanup (EXIT_FAILURE, msg);

        fsize = (int) file_stat.st_size;
    }

    /* main ops */

    switch (command) {

        case 'r':           /* read */
            tmp = 1;
            do {
                BUSY_WAIT ();
                data = read_fifo ();
                print_fifo_data (data, tmp++);
            } while ((cont) ? 1 : (num--));
            break;

        case 'w':           /* write */
            if (file != 0L)
                write_fifo_from_file (file);
            else
                write_fifo_from_buf (num);
            break;

        case 'e':           /* empty fifo */
            tmp = 1;
            while (!(FIFO_EMPTY ())) {
                data = read_fifo ();
                print_fifo_data (data, tmp++);
            }
            break;

        case 'p':           /* pipe mode */
            while (1) {
                BUSY_WAIT ();
                data = read_fifo ();
                write_fifo (&data, 1);
            }
            break;

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
    fprintf (stderr, "Usage: %s [OPTION]\n"
             "Perform operations on the FIFO from the Add-on side.\n\n"
             "-h\t\tprint this help.\n"
             "-r[N]\t\tread N DWORDs from FIFO (default = 1).\n"
             "-w[N]\t\twrite N DWORDs to FIFO (default = 1).\n"
             "-b\t\tbinary output.\n"
             "-f{-|name}\tuse file for I/O ('-' for stdio).\n"
             "-a[seed]\tuse random numbers.\n"
             "-c\t\tread until ^C is hit.\n"
             "-p\t\tconnect the P2A with the A2P fifo (pipe option).\n"
             "-e\t\tdump the contents of the FIFO and exit.\n",
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
    if (file && (fname[0] != '-'))
        close (file);       /* I/O file */

    if (buf)                /* buffer for I/O operations */
        free (buf);

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

/*
 *  print_fifo_data ()
 *  Read a piece of data from the fifo and prints it.
 */

static void
print_fifo_data (__u32 data, int order)
{
    int         err;
    char        line[80];       /* easier this way :-) */

    if (binary) {
        err = write (file, &data, sizeof (__u32));
    }
    else {
        sprintf (line, "%5d: %08x\n", order, data);
        err = write (file, line, strlen (line));
    }

    if (err == -1)
        cleanup (EXIT_FAILURE, "error writing output");
}

/*
 *  write_fifo_from_file ()
 *  Read data from a file, filling the FIFO.
 */

static void
write_fifo_from_file (int fd)
{
    extern __u32 *buf;

    if (fd == STDIN_FILENO) {

        int count;

        buf = malloc (BUF_SIZE);
        if (buf == NULL)
            cleanup (EXIT_FAILURE, "error allocating buffer");

        while ((count = read (fd, buf, BUF_SIZE))) {
            write_fifo (buf, (count / sizeof (__u32)));
        }

        free (buf);
        buf = NULL;
    }
    else {

        /* mmap is simpler for regular files */

        buf = mmap (NULL, fsize, PROT_READ, MAP_PRIVATE, fd, (off_t) 0L);
        if (buf == MAP_FAILED)
            cleanup (EXIT_FAILURE, "error mmapping file");

        write_fifo (buf, (fsize / sizeof (__u32)));

        munmap (buf, fsize);
        buf = NULL;
    }
}

/*
 *  write_fifo_from_buf ()
 *  Fill the FIFO with data from a memory buffer. num is the number
 *  of DWORDs that must be written.
 */

static void
write_fifo_from_buf (int num)
{
    extern __u32 *buf;
    int count;

    buf = (__u32 *) malloc (num * sizeof (__u32));
    if (buf == NULL)
        cleanup (EXIT_FAILURE, "error allocating buffer");

    for (count = 0; count < num; count++)
        buf[count] = (randf) ? rand () : count;

    write_fifo (buf, num);

    free (buf);
    buf = NULL;
}
