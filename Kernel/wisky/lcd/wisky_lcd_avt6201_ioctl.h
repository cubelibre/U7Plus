//-----------------------------------------------------------------------------
//      
// avt6201 controller frame buffer driver.
// 
// Copyright(c) Seiko Epson Corporation 2009.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//---------------------------------------------------------------------------- 

/* ioctls
    0x45 is 'E'                                                          */

#ifndef _AVT6201_IOCTL_H_
#define _AVT6201_IOCTL_H_

struct avt6201_ioctl_hwc
{
        unsigned addr;
        unsigned value;
        void* buffer;
};

#define AVT6201_REGREAD                0x4540
#define AVT6201_REGWRITE               0x4541
#define AVT6201_MEMBURSTREAD           0x4546
#define AVT6201_MEMBURSTWRITE          0x4547
#define AVT6201_VBUF_REFRESH           0x4548

// System commands
#define INIT_CMD_SET                    0x00
#define INIT_PLL_STANDBY                0x01
#define RUN_SYS                         0x02
#define STBY                            0x04
#define SLP                             0x05
#define INIT_SYS_RUN                    0x06
#define INIT_SYS_STBY                   0x07
#define INIT_SDRAM                      0x08
#define INIT_DSPE_CFG                   0x09
#define INIT_DSPE_TMG                   0x0A
#define INIT_ROTMODE                    0x0B

// Register and memory access commands
#define RD_REG                          0x10
#define WR_REG                          0x11
#define RD_SFM                          0x12
#define WR_SFM                          0x13
#define END_SFM                         0x14

// Burst access commands
#define BST_RD_SDR                      0x1C
#define BST_WR_SDR                      0x1D
#define BST_END_SDR                     0x1E

// Image loading commands
#define LD_IMG                          0x20
#define LD_IMG_AREA                     0x22
#define LD_IMG_END                      0x23
#define LD_IMG_WAIT                     0x24
#define LD_IMG_SETADR                   0x25
#define LD_IMG_DSPEADR                  0x26

// Polling commands
#define WAIT_DSPE_TRG                   0x28
#define WAIT_DSPE_FREND                 0x29
#define WAIT_DSPE_LUTFREE               0x2A
#define WAIT_DSPE_MLUTFREE              0x2B

// Waveform update commands
#define RD_WFM_INFO                     0x30
#define UPD_INIT                        0x32
#define UPD_FULL                        0x33
#define UPD_FULL_AREA                   0x34
#define UPD_PART                        0x35
#define UPD_PART_AREA                   0x36
#define UPD_GDRV_CLR                    0x37
#define UPD_SET_IMGADR                  0x38

#pragma pack(1)

typedef struct
{
        u16 param[5];
}avt6201_ioctl_cmd_params;

#pragma pack()

#define AVT6201_INIT_CMD_SET           (0x4500 | INIT_CMD_SET)
#define AVT6201_INIT_PLL_STANDBY       (0x4500 | INIT_PLL_STANDBY)
#define AVT6201_RUN_SYS                (0x4500 | RUN_SYS)
#define AVT6201_STBY                   (0x4500 | STBY)
#define AVT6201_SLP                    (0x4500 | SLP)
#define AVT6201_INIT_SYS_RUN           (0x4500 | INIT_SYS_RUN)
#define AVT6201_INIT_SYS_STBY          (0x4500 | INIT_SYS_STBY)
#define AVT6201_INIT_SDRAM             (0x4500 | INIT_SDRAM)
#define AVT6201_INIT_DSPE_CFG          (0x4500 | INIT_DSPE_CFG)
#define AVT6201_INIT_DSPE_TMG          (0x4500 | INIT_DSPE_TMG)
#define AVT6201_INIT_ROTMODE           (0x4500 | INIT_ROTMODE)
#define AVT6201_RD_REG                 (0x4500 | RD_REG)
#define AVT6201_WR_REG                 (0x4500 | WR_REG)
#define AVT6201_RD_SFM                 (0x4500 | RD_SFM)
#define AVT6201_WR_SFM                 (0x4500 | WR_SFM)
#define AVT6201_END_SFM                (0x4500 | END_SFM)

// Burst access commands
#define AVT6201_BST_RD_SDR             (0x4500 | BST_RD_SDR)
#define AVT6201_BST_WR_SDR             (0x4500 | BST_WR_SDR)
#define AVT6201_BST_END_SDR            (0x4500 | BST_END_SDR)

// Image loading IOCTL commands
#define AVT6201_LD_IMG                 (0x4500 | LD_IMG)
#define AVT6201_LD_IMG_AREA            (0x4500 | LD_IMG_AREA)
#define AVT6201_LD_IMG_END             (0x4500 | LD_IMG_END)
#define AVT6201_LD_IMG_WAIT            (0x4500 | LD_IMG_WAIT)
#define AVT6201_LD_IMG_SETADR          (0x4500 | LD_IMG_SETADR)
#define AVT6201_LD_IMG_DSPEADR         (0x4500 | LD_IMG_DSPEADR)

// Polling commands
#define AVT6201_WAIT_DSPE_TRG          (0x4500 | WAIT_DSPE_TRG)
#define AVT6201_WAIT_DSPE_FREND        (0x4500 | WAIT_DSPE_FREND)
#define AVT6201_WAIT_DSPE_LUTFREE      (0x4500 | WAIT_DSPE_LUTFREE)
#define AVT6201_WAIT_DSPE_MLUTFREE     (0x4500 | WAIT_DSPE_MLUTFREE)

// Waveform update IOCTL commands
#define AVT6201_RD_WFM_INFO            (0x4500 | RD_WFM_INFO)
#define AVT6201_UPD_INIT               (0x4500 | UPD_INIT)
#define AVT6201_UPD_FULL               (0x4500 | UPD_FULL)
#define AVT6201_UPD_FULL_AREA          (0x4500 | UPD_FULL_AREA)
#define AVT6201_UPD_PART               (0x4500 | UPD_PART)
#define AVT6201_UPD_PART_AREA          (0x4500 | UPD_PART_AREA)
#define AVT6201_UPD_GDRV_CLR           (0x4500 | UPD_GDRV_CLR)
#define AVT6201_UPD_SET_IMGADR         (0x4500 | UPD_SET_IMGADR)

#endif

