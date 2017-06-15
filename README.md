# AMCC S5933 Linux drivers

This repository contains two old projects of mine that I am publishing on
GitHub for preservation purposes.

Both projects contain Linux 2.0/2.1 device drivers for the AMCC S5933 PCI
controller chip developed in the late 1990s.

The drivers where developed by me and [Toni Giorgino](https://sites.google.com/site/tonigiorgino/)
in 1997-1999 as members of the research group that was developing the
[APEmille](http://apegate.roma1.infn.it/mediawiki/index.php/APEmille_project)
SIMD massively parallel computer.

An interesting article describing the status of the project in the fall of 1997
is available [here](https://arxiv.org/pdf/hep-lat/9710006).

Having graduated in 1996, I consider this my first professionally developed,
production ready code.

## The chip

The S5933 is a generic controller that board manufacturers could use to develop
PCI 2.1 compliant devices without the need to develop the complex logic needed
to interact with the PCI bus.

Once programmed at startup through the use of a nvRAM, the chip could provide
different levels of functionality to the application logic sitting behind it.

More info can be found in the [included datasheet](s5933.pdf).

## S5933 general driver

The `s5933` directory contains a generic Linux 2.0/2.1 device driver for the S5933.

This driver was mainly used with the S5933 developers kit to gain a better
knowledge of the chip while waiting for the development of our custom hardware.

Given the lack of a specialized hardware, the driver is completely generic and
allows C programs to use all the communications channel implemented by the S5933.

A detailed description of the functionality can be found in the project's
[README](s5933/README) file.

## Autobahn driver

The generic driver was greatly enhanced by Toni Giorgino to support our custom
hardware, the so-called _"Autobahn"_ serial communication boards specifically
developed for the APEmille project.

This driver can be found in the `autobahn` directory.

Again, a very good description of the Autobahn hardware and of the features of
this driver can be found in the [README](autobahn/README).
