/*
 *  strtol_e.c
 *
 *  (c) 1996 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/*
 *  int strtol_e (char *, char, int)
 *
 *  Convert a string to a long integer checking for errors.
 */

int
strtol_e (char *arg, char opt, int base)
{
    extern char *pname;

    int num;
    char *next;

    num = strtol (arg, &next, base);
    if ((num == 0) && (next == arg)) {
        fprintf (stderr, "%s: wrong argument for option %c\n", pname, opt);
        exit (EXIT_FAILURE);
    }

    return num;
}
