/*
 *  s5933.h -- Definitions for the S5933 chip
 *
 *  (c) 1997 Andrea Cisternino  (acister@pcape1.pi.infn.it)
 *           Toni Giorgino      (toni@pcape2.pi.infn.it)
 *
 *  s5933.h,v 1.6 1997/11/10 09:25:26 acister Exp
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
 *  Include file for the S5933 operation registers
 *  This file contains the struct definition needed to directly access
 *  the operation registers.
 *
 *--------------------------------------------------------------------*/

#ifndef _S5933_H
#define _S5933_H

#ifndef _LINUX_TYPES_H
#include <linux/types.h>
#endif

/*-- Defines ---------------------------------------------------------*/

#define PCI_DEVICE_ID_AMCC_S5933        0x4750

#define AMCC_OP_REG_SIZE    (sizeof(struct s5933_opreg))

/* these definitions are for faster handling of critical situations */

#define MCSR_RTC_ENABLE         0x04    /* 3 bit */
#define MCSR_RTC_FIFO_MNG       0x02
#define MCSR_RTC_RD_PRIO        0x01

#define MCSR_WTC_ENABLE         0x04    /* 3 bit */
#define MCSR_WTC_FIFO_MNG       0x02
#define MCSR_WTC_WR_PRIO        0x01

#define ICSR_TARGET_ABORT       0x20    /* 6 bit */
#define ICSR_MASTER_ABORT       0x10
#define ICSR_RT_COMPL_INT       0x08
#define ICSR_WT_COMPL_INT       0x04

/* 32-bit handling of MCSR */

#define MCSR32_GLOBAL_RESET         0x0F000000L     /* reset */
#define MCSR32_MB_RESET             0x08000000L
#define MCSR32_A2P_FIFO_RESET       0x04000000L
#define MCSR32_P2A_FIFO_RESET       0x02000000L
#define MCSR32_ADD_ON_RESET         0x01000000L

#define MCSR32_READ_MULT_ENABLE     0x00008000L     /* read transfer control */
#define MCSR32_READ_TR_ENABLE       0x00004000L
#define MCSR32_READ_IF_4_EMPTY      0x00002000L
#define MCSR32_READ_PRIORITY        0x00001000L

#define MCSR32_WRITE_TR_ENABLE      0x00000400L     /* write transfer control */
#define MCSR32_WRITE_IF_4_FILLED    0x00000200L
#define MCSR32_WRITE_PRIORITY       0x00000100L

#define MCSR32_A2P_TC_ZERO          0x00000080L     /* status */
#define MCSR32_P2A_TC_ZERO          0x00000040L

#define MCSR32_A2P_FIFO_EMPTY       0x00000020L
#define MCSR32_A2P_FIFO_HALF        0x00000010L
#define MCSR32_A2P_FIFO_FULL        0x00000008L
#define MCSR32_P2A_FIFO_EMPTY       0x00000004L
#define MCSR32_P2A_FIFO_HALF        0x00000002L
#define MCSR32_P2A_FIFO_FULL        0x00000001L

/* 32-bit handling of ICSR */

#define ICSR32_INT_ASSERTED         0x00800000L
#define ICSR32_TARGET_ABORT         0x00200000L
#define ICSR32_MASTER_ABORT         0x00100000L
#define ICSR32_DMA_ABORT            (ICSR32_TARGET_ABORT | ICSR32_MASTER_ABORT)

#define ICSR32_RT_COMPL_INT         0x00080000L
#define ICSR32_WT_COMPL_INT         0x00040000L

#define ICSR32_IN_MB_INT            0x00020000L
#define ICSR32_OUT_MB_INT           0x00010000L

#define ICSR32_INT_WRC_ENABLE       0x00004000L
#define ICSR32_INT_RDC_ENABLE       0x00008000L

/*-- Main struct -----------------------------------------------------*/

struct s5933_opreg {

    __u32 omb[4];       /* mailboxes */
    __u32 imb[4];

    __u32 fifo;         /* FIFO */

    __u32 mwar;         /* Master Write Address Register */
    __u32 mwtc;         /* Master Write Transfer Count Register */

    __u32 mrar;         /* Master Read Address Register */
    __u32 mrtc;         /* Master Read Transfer Count Register */

    union {             /* Mailbox Empty/Full Status */
        struct {
            u_int _omb : 16;
            u_int _imb : 16;
        } _t;
        struct {
            u_int _om1 : 4;
            u_int _om2 : 4;
            u_int _om3 : 4;
            u_int _om4 : 4;
            u_int _im1 : 4;
            u_int _im2 : 4;
            u_int _im3 : 4;
            u_int _im4 : 4;
        } _s;
        __u32 _l;
    } _mbef;

#define mbef    _mbef._l
#define omb1    _mbef._s._om1
#define omb2    _mbef._s._om2
#define omb3    _mbef._s._om3
#define omb4    _mbef._s._om4
#define imb1    _mbef._s._im1
#define imb2    _mbef._s._im2
#define imb3    _mbef._s._im3
#define imb4    _mbef._s._im4

    union {             /* Interrupt Control/Status Register */
        struct {
            u_int _intsel : 16;         /* Interrupt selection */
            u_int _intcode : 6;         /* Interrupt code */
            u_int : 1;
            u_int _interrupt : 1;       /* Interrupt asserted */
            u_int _fectl : 8;           /* FIFO and Endian control */
        } _t;
        struct {
            /* selection */
            u_int _isel_omb_byte : 2;   /* OMB byte # */
            u_int _isel_omb_num : 2;    /* OMB # */
            u_int _isel_omb_en : 1;     /* Enable int. */
            u_int : 3;
            u_int _isel_imb_byte : 2;   /* IMB byte # */
            u_int _isel_imb_num : 2;    /* IMB # */
            u_int _isel_imb_en : 1;     /* Enable int. */
            u_int : 1;
            u_int _isel_wtc : 1;        /* Int. on write complete */
            u_int _isel_rtc : 1;        /* Int. on read complete */

            /* code */
            u_int _int_omb : 1;         /* OMB interrupt */
            u_int _int_imb : 1;         /* IMB interrupt */
            u_int _int_wtc : 1;         /* write compl. interrupt */
            u_int _int_rtc : 1;         /* read compl. interrupt */
            u_int _int_mabort : 1;      /* Master abort interrupt */
            u_int _int_tabort : 1;      /* Target abort interrupt */
            u_int : 1;
            u_int _int_on : 1;          /* Interrupt asserted */

            /* FIFO & endian control */
            u_int _endian_conv : 2;
            u_int _fctl_pci_adv : 2;
            u_int _fctl_ao_adv : 2;
            u_int _fctl_a2p_toggle : 1;
            u_int _fctl_p2a_toggle : 1;
        } _s;
        __u32 _l;
    } _icsr;

#define icsr            _icsr._l

#define int_asserted    _icsr._t._interrupt
#define int_code        _icsr._t._intcode
#define int_selection   _icsr._t._intsel

#define wr_compl_en     _icsr._s._isel_wtc
#define rd_compl_en     _icsr._s._isel_rtc
#define omb_en          _icsr._s._isel_omb_en
#define omb_num         _icsr._s._isel_omb_num
#define omb_byte        _icsr._s._isel_omb_byte
#define imb_en          _icsr._s._isel_imb_en
#define imb_num         _icsr._s._isel_imb_num
#define imb_byte        _icsr._s._isel_imb_byte
#define int_wr_compl    _icsr._s._int_wtc
#define int_rd_compl    _icsr._s._int_rtc
#define int_m_abort     _icsr._s._int_mabort
#define int_t_abort     _icsr._s._int_tabort

    union {             /* Bus Master Control Status register */
        struct {
            u_int _status : 8;          /* FIFO status */
            u_int _wtc : 3;             /* Write Transfer Control */
            u_int : 1;
            u_int _rtc : 3;             /* Read Transfer Control */
            u_int : 1;
            u_int _nv_bus : 8;          /* NV address/data */
            u_int _reset : 4;           /* Reset Controls */
            u_int : 1;
            u_int _nv_ctl : 3;          /* NV Control */
        } _t;
        struct {
            /* status */
            u_int _fs_p2a_full : 1;
            u_int _fs_p2a_half : 1;
            u_int _fs_p2a_empty : 1;
            u_int _fs_a2p_full : 1;
            u_int _fs_a2p_half : 1;
            u_int _fs_a2p_empty : 1;
            u_int _fs_p2a_tczero : 1;
            u_int _fs_a2p_tczero : 1;

            /* control */
            u_int _wtc_prio : 1;
            u_int _wtc_fifo_mng : 1;
            u_int _wtc_wte : 1;
            u_int : 1;
            u_int _rtc_prio : 1;
            u_int _rtc_fifo_mng : 1;
            u_int _rtc_rte : 1;
            u_int : 1;
            u_int _nv_bus : 8;
            u_int _reset_ao : 1;
            u_int _reset_p2a : 1;
            u_int _reset_a2p : 1;
            u_int _reset_mb : 1;
            u_int : 1;
            u_int _nv_ctl : 3;
        } _s;
        __u32 _l;
    } _mcsr;

#define mcsr        _mcsr._l

#define wtc         _mcsr._t._wtc
#define rtc         _mcsr._t._rtc
#define nv_ctl      _mcsr._t._nv_ctl
#define nv_bus      _mcsr._t._nv_bus
#define reset       _mcsr._t._reset

#define p2afull     _mcsr._s._fs_p2a_full
#define p2ahalf     _mcsr._s._fs_p2a_half
#define p2aempty    _mcsr._s._fs_p2a_empty
#define a2pfull     _mcsr._s._fs_a2p_full
#define a2phalf     _mcsr._s._fs_a2p_half
#define a2pempty    _mcsr._s._fs_a2p_empty
#define p2azero     _mcsr._s._fs_p2a_tczero
#define a2pzero     _mcsr._s._fs_a2p_tczero

#define wtc_w_prio  _mcsr._s._wtc_prio
#define wtc_enable  _mcsr._s._wtc_wte
#define rtc_r_prio  _mcsr._s._rtc_prio
#define rtc_enable  _mcsr._s._rtc_rte
#define ao_reset    _mcsr._s._reset_ao
#define p2a_reset   _mcsr._s._reset_p2a
#define a2p_reset   _mcsr._s._reset_a2p
#define mb_reset    _mcsr._s._reset_mb
};

#endif  /* _S5933_H */
