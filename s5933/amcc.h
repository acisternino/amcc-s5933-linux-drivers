/****************************************************************************/
/*                                                                          */
/* Module: AMCC.H                                                           */
/*                                                                          */
/* Purpose: Definitions for AMCC PCI Library                                */
/*                                                                          */
/* $Id: amcc.h,v 2.2 1997/05/06 14:00:19 acister Exp $                      */
/****************************************************************************/

#ifndef _AMCC_H_
#define _AMCC_H_

/****************************************************************************/
/* AMCC Operation Register Offsets - PCI                                    */
/****************************************************************************/

#define AMCC_OP_REG_OMB1            0x00
#define AMCC_OP_REG_OMB2            0x04
#define AMCC_OP_REG_OMB3            0x08
#define AMCC_OP_REG_OMB4            0x0C
#define AMCC_OP_REG_IMB1            0x10
#define AMCC_OP_REG_IMB2            0x14
#define AMCC_OP_REG_IMB3            0x18
#define AMCC_OP_REG_IMB4            0x1C
#define AMCC_OP_REG_FIFO            0x20
#define AMCC_OP_REG_MWAR            0x24
#define AMCC_OP_REG_MWTC            0x28
#define AMCC_OP_REG_MRAR            0x2C
#define AMCC_OP_REG_MRTC            0x30
#define AMCC_OP_REG_MBEF            0x34
#define AMCC_OP_REG_INTCSR          0x38
#define   AMCC_OP_REG_INTCSR_SRC    (AMCC_OP_REG_INTCSR + 2)    /* INT source */
#define   AMCC_OP_REG_INTCSR_FEC    (AMCC_OP_REG_INTCSR + 3)    /* FIFO ctrl */
#define AMCC_OP_REG_MCSR            0x3C
#define   AMCC_OP_REG_MCSR_NVDATA   (AMCC_OP_REG_MCSR + 2)      /* Data in byte 2 */
#define   AMCC_OP_REG_MCSR_NVCMD    (AMCC_OP_REG_MCSR + 3)      /* Command in byte 3 */

#define AMCC_FIFO_DEPTH_DWORD       8
#define AMCC_FIFO_DEPTH_BYTES       (8 * sizeof (u32))

/****************************************************************************/
/* AMCC Operation Registers Size - PCI                                      */
/****************************************************************************/

#define AMCC_OP_REG_SIZE            64          /* in bytes */

/****************************************************************************/
/* AMCC ISA Board                                                           */
/****************************************************************************/

#define AMCC_ISA_BASE               0x0300
#define AMCC_ISA_LEN                60          /* 0x300 -> 0x33f */

#define AMCC_ISA_AMWTC              0x0718
#define AMCC_ISA_AMRTC              0x071c

/****************************************************************************/
/* AMCC Operation Register Offsets - Add-on                                 */
/****************************************************************************/

#define AMCC_OP_REG_AIMB1           0x00
#define AMCC_OP_REG_AIMB2           0x04
#define AMCC_OP_REG_AIMB3           0x08
#define AMCC_OP_REG_AIMB4           0x0C
#define AMCC_OP_REG_AOMB1           0x10
#define AMCC_OP_REG_AOMB2           0x14
#define AMCC_OP_REG_AOMB3           0x18
#define AMCC_OP_REG_AOMB4           0x1C
#define AMCC_OP_REG_AFIFO           0x20
#define AMCC_OP_REG_AMWAR           0x24
#define AMCC_OP_REG_APTA            0x28
#define AMCC_OP_REG_APTD            0x2C
#define AMCC_OP_REG_AMRAR           0x30
#define AMCC_OP_REG_AMBEF           0x34
#define AMCC_OP_REG_AINT            0x38
#define AMCC_OP_REG_AGCSTS          0x3C
#define AMCC_OP_REG_AMWTC           0x58
#define AMCC_OP_REG_AMRTC           0x5C

/****************************************************************************/
/* AMCC - Add-on General Control/Status Register                            */
/****************************************************************************/

#define AGCSTS_CONTROL_MASK         0xFFFFF000
#define   AGCSTS_NV_ACC_MASK        0xE0000000
#define   AGCSTS_RESET_MASK         0x0E000000
#define   AGCSTS_NV_DA_MASK         0x00FF0000
#define   AGCSTS_BIST_MASK          0x0000F000
#define AGCSTS_STATUS_MASK          0x000000FF
#define   AGCSTS_TCZERO_MASK        0x000000C0
#define   AGCSTS_FIFO_ST_MASK       0x0000003F

#define AGCSTS_RESET_MBFLAGS        0x08000000
#define AGCSTS_RESET_P2A_FIFO       0x04000000
#define AGCSTS_RESET_A2P_FIFO       0x02000000
#define AGCSTS_RESET_FIFOS          (AGCSTS_RESET_A2P_FIFO | AGCSTS_RESET_P2A_FIFO)

#define AGCSTS_A2P_TCOUNT           0x00000080
#define AGCSTS_P2A_TCOUNT           0x00000040

#define AGCSTS_FS_P2A_EMPTY         0x00000020
#define AGCSTS_FS_P2A_HALF          0x00000010
#define AGCSTS_FS_P2A_FULL          0x00000008

#define AGCSTS_FS_A2P_EMPTY         0x00000004
#define AGCSTS_FS_A2P_HALF          0x00000002
#define AGCSTS_FS_A2P_FULL          0x00000001

/****************************************************************************/
/* AMCC - Add-on Interrupt Control/Status Register                          */
/****************************************************************************/

#define AINT_INT_MASK               0x00FF0000
#define AINT_SEL_MASK               0x0000FFFF
#define   AINT_IS_ENSEL_MASK        0x00001F1F

#define AINT_INT_ASSERTED           0x00800000
#define AINT_BM_ERROR               0x00200000
#define AINT_BIST_INT               0x00100000

#define AINT_RT_COMPLETE            0x00080000
#define AINT_WT_COMPLETE            0x00040000

#define AINT_OUT_MB_INT             0x00020000
#define AINT_IN_MB_INT              0x00010000

#define AINT_READ_COMPL             0x00008000
#define AINT_WRITE_COMPL            0x00004000

#define AINT_OMB_ENABLE             0x00001000
#define AINT_OMB_SELECT             0x00000C00
#define AINT_OMB_BYTE               0x00000300

#define AINT_IMB_ENABLE             0x00000010
#define AINT_IMB_SELECT             0x0000000C
#define AINT_IMB_BYTE               0x00000003

#endif  /* _AMCC_H_ */
