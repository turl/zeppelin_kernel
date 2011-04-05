/* drivers/video/msm/mddi_tpo_nv.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h> /* for vreg.h */
#include <linux/delay.h>
#include <mach/vreg.h>
#include <mach/board.h>

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/leds.h>
#include <linux/gpio.h>
#include "mddi_tpo_nv.h"
#include <mot/esd_poll.h>
#include "board-mot.h" //GPIO/I2C addresses


/*=============================================================================
                        LOCAL CONSTANTS
=============================================================================*/
#define TPO_NV_HVGA_PRIM      1
#define MDDI_SLEEP_CMD        0xFFFFFFFF
#define TABLE_END             0xEFFFFFFF 

#define LCD_WIDTH             320
#define LCD_HEIGHT            480
#define V_SYNC_WIDTH          0
#define V_BACK_PORCH          0
#define V_FRONT_PORCH         0

#define TPO_NV_VER_F_USERID   0x05
#define TPO_NV_VER_G_USERID   0x06
#define TPO_NV_CLK_RATE       160000000

#define TPO_PLAT_DISP         0xB9F65395
#define TMD_PLAT_DISP         0xB9F65451

/*=============================================================================
                        LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
=============================================================================*/
struct init_table {
    unsigned int reg;
    unsigned int val;
};

/*=============================================================================
                        LOCAL FUNCTION PROTOTYPES
=============================================================================*/

/*=============================================================================
                                 LOCAL MACROS
=============================================================================*/

/*=============================================================================
                                LOCAL VARIABLES
=============================================================================*/
static int mddi_tpo_nv_cabc_en = 0;
static struct platform_device *als_pdev = NULL;

static uint32 mddi_tpo_nv_rows_per_second = LCD_HEIGHT * 100; // 90Hz
static uint32 mddi_tpo_nv_rows_per_refresh = LCD_HEIGHT;
static boolean tpo_nv_powered_on = TRUE;
static uint32 plat_disp_type = 0;

static struct init_table const tpo_nv_plat_esd_recovery[] =
{
   { MM_SET_RAM_ADDR0,  0x0000 },
   { MM_SET_RAM_ADDR1,  0x0000 },
   { MM_SET_RAM_ADDR2,  0x0000 },
   { MM_SET_RAM_ADDR3,  0x0000 },
   { TABLE_END,         0x0000 }, /* terminate table */
};

static struct init_table const tpo_nv_plat_init[] =
{
#ifdef CONFIG_MACH_PITTSBURGH
	   { MDDI_SLEEP_CMD, 10     }, /* wait 10ms */
	   { 0x1100,	     0x0000 }, /* sleep out command */
	   { MDDI_SLEEP_CMD, 10     }, /* wait 10ms */
	   { 0xB100,	     0x0065 }, /* fix horizontal scanning, plus inverted scan */
	   { 0x3A00,	     0x0066 }, /* 18 bits/pixel */
	   { 0x3600,	     0x0000 }, /* normal scan */
	   { 0x5100,	     0x00FF }, /* Display brightness */
	   { 0x5300,	     0x0016 }, /* CABC */
	   { 0x2900,	     0x0001 }, /* display turn ON command */
	   { 0x5500,	     0x0000 }, /* CABC off */
	   { TABLE_END,      0x0000 }, /* terminate table */
#endif
   { MDDI_SLEEP_CMD,       10     }, /* wait 10ms */
   { MM_SLEEPOUT,          0x0000 }, /* sleep out command */
   { MDDI_SLEEP_CMD,       10     }, /* wait 10ms */
   { MM_DIMFUNC1,          0x0000 }, /* dimming control 1 */
   { MM_DIMFUNC2,          0x0044 }, /* dimming control 2 */

   // Begin CABC
   { MM_DISP_BRIGHTNESS,   0x00FF }, /* Display brightness */
   { MM_WRCTRLD,           0x0013 }, /* CABC */
   { MM_WRCABC,            0x0002 }, /* CABC on */
   { 0x6d0d,               0x0030 },
   { 0x6e02,               0x009F },
   { 0x6e03,               0x000F },
   { 0x6e06,               0x0004 },

   // Still mode PWM
   {0x6D00,                0x00E6 },
   {0x6D01,                0x00D9 },
   {0x6D02,                0x00CC },
   {0x6D03,                0x00BF },
   {0x6D04,                0x00B3 },
   {0x6D05,                0x00A6 },
   {0x6D06,                0x0099 },
   {0x6D07,                0x0099 },
   {0x6D08,                0x0099 },
   {0x6D09,                0x0099 },

   // Moving mode PWM
   {0x6C02,                0x00E6 },
   {0x6C03,                0x00D9 },
   {0x6C04,                0x00CC },
   {0x6C05,                0x00BF },
   {0x6C06,                0x00B3 },
   {0x6C07,                0x00A6 },
   {0x6C08,                0x0099 },
   {0x6C09,                0x0099 },
   {0x6C0A,                0x0099 },
   {0x6C0B,                0x0099 },

   { MM_SET_TEAR_ON,       0x0000 }, /* set tear on */
   { MM_SET_TEAR_LINE,     0x0190 }, /* emit FTE on 400th line */
   { MM_DDC_SAYS_SO,       0x00A5 }, /* fix horizontal scanning */
   { MM_DISPON,            0x0001 }, /* display turn ON command */
   { TABLE_END,            0x0000 }, /* terminate table */

};

static struct init_table const tmd_nv_plat_init[] =
{
#ifdef CONFIG_MACH_PITTSBURGH
   { MDDI_SLEEP_CMD, 10     }, /* wait 10ms */
   { 0x1100,         0x0000 }, /* sleep out command */
   { MDDI_SLEEP_CMD, 10     }, /* wait 10ms */
   { 0xB100,         0x00A5 }, /* fix horizontal scanning */
   { 0x3A00,         0x0066 }, /* 18 bits/pixel */
   { 0x3600,         0x00C0 }, /* inverted scan */
   { 0x5100,         0x00FF }, /* Display brightness */
   { 0x5300,         0x002C }, /* CABC */
   { 0x2900,         0x0001 }, /* display turn ON command */
   { 0x5500,         0x0000 }, /* CABC off */
   { TABLE_END,      0x0000 }, /* terminate table */
#else
   { MDDI_SLEEP_CMD,       10     }, /* wait 10ms */
   { MM_SLEEPOUT,          0x0000 }, /* sleep out command */
   { MDDI_SLEEP_CMD,       15     }, /* wait 15ms */

   // Begin CABC
   { MM_DISP_BRIGHTNESS,   0x00FF }, /* Display brightness */
   { MM_WRCTRLD,           0x0024 }, /* it was 0x002C, but having dimming on during wake up caused flicker
                                        when lm3535 driver resumes. 0x002C, enabling CABC dimming will kick
                                        in via msm_fb_led_resumed called after init */
   { MM_WRCABC,            0x0002 }, /* CABC on */
   { 0xF400,               0x0055 },
   { 0x83C0,               0x0018 },
   { 0x81C1,               0x0004 },
   { 0x82C0,               0x00B0 },
   { 0x83C1,               0x0040 },
   { 0x83C2,               0x0000 },
   { 0xF5C0,               0x0055 },
   { 0xF300,               0x00AA },
   { 0x0080,               0x0001 },
   { 0x20C0,               0x00E6 },
   { 0x21C0,               0x00D9 },
   { 0x22C0,               0x00CC },
   { 0x23C0,               0x00BF },
   { 0x24C0,               0x00B3 },
   { 0x25C0,               0x00A6 },
   { 0x26C0,               0x0099 },
   { 0x27C0,               0x0099 },
   { 0x28C0,               0x0099 },
   { 0x29C0,               0x0099 },
   { 0x2AC0,               0x00E6 },
   { 0x2BC0,               0x00D9 },
   { 0x2CC0,               0x00CC },
   { 0x2DC0,               0x00BF },
   { 0x2EC0,               0x00B3 },
   { 0x2FC0,               0x00A6 },
   { 0x30C0,               0x0099 },
   { 0x31C0,               0x0099 },
   { 0x32C0,               0x0099 },
   { 0x33C0,               0x0099 },
   { 0x34C0,               0x001F },
   { 0x35C0,               0x000F },
   { 0x06C0,               0x0024 },
   { 0x07C0,               0x0041 },
   { 0x0BC0,               0x0004 },
   { 0x15C0,               0x00AA },
    
   { MM_SET_TEAR_ON,       0x0000 },
   { MM_SET_TEAR_LINE,     0x0090 },
   { MM_SET_PIX_FORMAT,    0x0066 },
   { MM_SET_MDDI_I,        0x0001 },
   { MM_SET_ADDR_MODE_PARAM2, 1   }, /* enable High speed mode */
   { MM_DISPON,            0x0001 }, /* display turn ON command */
   { TABLE_END,            0x0000 }, /* terminate table */

#endif
};

static struct init_table const tpo_nv_plat_uninit[] =
{
   { MM_DISPOFF,     0x0000 }, /* turn display off */
   { MDDI_SLEEP_CMD, 5      }, /* wait 5ms */
   { MM_SLEEPIN,     0x0000 }, /* put panel to sleep */
   { MDDI_SLEEP_CMD, 20     }, /* wait for a couple frames */
   { TABLE_END,      0x0000 }, /* terminate table */
};

static struct init_table const tpo_nv_version_E_init[] = 
{
    {MDDI_SLEEP_CMD,10             }, // wait 5ms

//Set Sleep-Out command    - Set Register access packet Address = 110h, data = 0h
    {SLEPCTRL ,0x0000             },
 
    {MDDI_SLEEP_CMD,10             }, // wait 5ms

    {PANELCTRL1 ,0x0044           },
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x0087           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x007B           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x000F           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },

    {ENTRYMODE1 ,0x0030           }, //ENTRYMODE1: Entry Mode 1 (360h)
#if defined(CONFIG_MACH_CALGARY)
    {ENTRYMODE2 ,0x00C5           },
#elif defined(CONFIG_MACH_PITTSBURGH)
    {ENTRYMODE2 ,0x0005           },
#else
    {ENTRYMODE2 ,0x00C5           },
#endif

    {DRVCTRL ,0x0012              },
    {DRV_OUT_CTRL1 ,0x00EF        },
    {DRV_OUT_CTRL2 ,0x0001        },

    {SPECTRL ,0x0080              },

    {WHRAMLOWST ,0x0000           }, //Window Horizontal RAM Low Address Start
    {WHRAMHIGHST ,0x0000          }, //Window Horizontal RAM High Address Start (2A1h)
    {WHRAMLOWEND ,0x003F          }, //Window Horizontal RAM Low Address End (2A2h) 
    {WHRAMHIGHEND ,0x0001         }, //Window Horizontal RAM High Address End (2A3h)
    {WVRAMLOWST ,0x0000           }, //Window Vertical RAM Low Address Start (2B0h)
    {WVRAMHIGHST ,0x0000          }, //Window Vertical RAM High Address Start (2B1h) 
    {WVRAMLOWEND ,0x00DF          }, //Window Vertical RAM Low Address End (2B2h)
    {WVRAMHIGHEND ,0x0001         }, //Window Vertical RAM High Address End (2B3h) 
    {RAMHSETL ,0x0000             }, //RAMHSETL: RAM Low Address Set (Horizontal Address) (2D0h)    
    {RAMHSETH ,0x0000             }, //RAMHSETH: RAM High Address Set (Horizontal Address) (2D1h)
    {RAMVSETL ,0x0000             }, //RAMVSETL: RAM Low Address Set (Vertical Address) (2D2h) 
    {RAMVSETH ,0x0000             }, //RAMVSETH: RAM High Address Set (Vertical Address) (2D3h) 
    {FPOSITION ,0                 }, //FPOSITION: Frame Tearing Effect Position (350h)
    {FCYCLE ,0x0000               }, //FCYCLE: Frame Tearing Effect Position and Output cycle(351h)

    {PWRCTRL2 ,0x0010             }, //Power Control 2 (0xC31)

    {PUMPCTRL1, 0x0000            }, //DC/DC on Glass Control 1 (0xBA0)
    {PUMPCTRL2, 0x0086            }, //DC/DC on Glass Control 2 (0xBA1)

    {VCOMHCTRL, 0x0033            }, //VCOM_H Voltage Control (C50h)
    {VCOMDCCTRL, 0x3A             }, //VCOM_DC Voltage Control (C52h)

    {PWRCTRL5 ,0x0038             }, //Power Control 5 (C40h)

    {SOUTCTRL1 ,0x0001            }, //Source Output Control 1 (BE0h)
    {SOUTCTRL2 ,0x0000            }, //Source Output Control 2 (BE1h)

//#ifdef THESE_ARE_THE_CABC_PARAMS
    {WRCTRLD, 0x13                }, //CABC: Write CTRL Display (530h)
    {WRCABC, 0x00                 }, //CABC: Content Adaptive Brightness Control (550h)
//    {WRCABCMB, 0xFE               }, //CABC: Minimum Brightness Level (5E0h)
//#endif

    // Red Gamma positive polarity
    {GMACTRL1_0 ,0x0001           },
    {GMACTRL1_1 ,0x0008           },
    {GMACTRL1_2 ,0x000E           },
    {GMACTRL1_3 ,0x000F           },
    {GMACTRL1_4 ,0x002A           },
    {GMACTRL1_5 ,0x0025           },
    {GMACTRL1_6 ,0x0016           },
    {GMACTRL1_7 ,0x0016           },
    {GMACTRL1_8 ,0x0010           },
    {GMACTRL1_9 ,0x003A           },
    {GMACTRL1_A ,0x0016           },
    {GMACTRL1_B ,0x002B           },
    {GMACTRL1_C ,0x0041           },
    {GMACTRL1_D ,0x0031           },
    {GMACTRL1_E ,0x0046           },
    {GMACTRL1_F ,0x0067           },

    // Red Gamma negative polarity
    {GMACTRL2_0 ,0x0000           },
    {GMACTRL2_1 ,0x0022           },
    {GMACTRL2_2 ,0x0036           },
    {GMACTRL2_3 ,0x001E           },
    {GMACTRL2_4 ,0x0033           },
    {GMACTRL2_5 ,0x0029           },
    {GMACTRL2_6 ,0x002C           },
    {GMACTRL2_7 ,0x000D           },
    {GMACTRL2_8 ,0x0008           },
    {GMACTRL2_9 ,0x0050           },
    {GMACTRL2_A ,0x001A           },
    {GMACTRL2_B ,0x0034           },
    {GMACTRL2_C ,0x004F           },
    {GMACTRL2_D ,0x005A           },
    {GMACTRL2_E ,0x005F           },
    {GMACTRL2_F ,0x0066           },

    // Green Gamma positive polarity
    {GMACTRL3_0 ,0x0016           },
    {GMACTRL3_1 ,0x001D           },
    {GMACTRL3_2 ,0x0023           },
    {GMACTRL3_3 ,0x0012           },
    {GMACTRL3_4 ,0x002B           },
    {GMACTRL3_5 ,0x0026           },
    {GMACTRL3_6 ,0x0020           },
    {GMACTRL3_7 ,0x0015           },
    {GMACTRL3_8 ,0x0010           },
    {GMACTRL3_9 ,0x003F           },
    {GMACTRL3_A ,0x0015           },
    {GMACTRL3_B ,0x0029           },
    {GMACTRL3_C ,0x0041           },
    {GMACTRL3_D ,0x0033           },
    {GMACTRL3_E ,0x003A           },
    {GMACTRL3_F ,0x0067           },

    // Green Gamma negative polarity
    {GMACTRL4_0 ,0x0000           },
    {GMACTRL4_1 ,0x002E           },
    {GMACTRL4_2 ,0x003A           },
    {GMACTRL4_3 ,0x001C           },
    {GMACTRL4_4 ,0x0034           },
    {GMACTRL4_5 ,0x002A           },
    {GMACTRL4_6 ,0x0027           },
    {GMACTRL4_7 ,0x000E           },
    {GMACTRL4_8 ,0x0009           },
    {GMACTRL4_9 ,0x0046           },
    {GMACTRL4_A ,0x001A           },
    {GMACTRL4_B ,0x0034           },
    {GMACTRL4_C ,0x004E           },
    {GMACTRL4_D ,0x0044           },
    {GMACTRL4_E ,0x004A           },
    {GMACTRL4_F ,0x0051           },

    // Blue Gamma positive polarity
    {GMACTRL5_0 ,0x0008           },
    {GMACTRL5_1 ,0x0017           },
    {GMACTRL5_2 ,0x0022           },
    {GMACTRL5_3 ,0x0015           },
    {GMACTRL5_4 ,0x0031           },
    {GMACTRL5_5 ,0x0029           },
    {GMACTRL5_6 ,0x0028           },
    {GMACTRL5_7 ,0x0015           },
    {GMACTRL5_8 ,0x0010           },
    {GMACTRL5_9 ,0x0043           },
    {GMACTRL5_A ,0x0015           },
    {GMACTRL5_B ,0x0029           },
    {GMACTRL5_C ,0x0040           },
    {GMACTRL5_D ,0x0036           },
    {GMACTRL5_E ,0x004A           },
    {GMACTRL5_F ,0x0067           },

    // Blue Gamma negative polarity
    {GMACTRL6_0 ,0x0000           },
    {GMACTRL6_1 ,0x001D           },
    {GMACTRL6_2 ,0x0031           },
    {GMACTRL6_3 ,0x001E           },
    {GMACTRL6_4 ,0x0036           },
    {GMACTRL6_5 ,0x002B           },
    {GMACTRL6_6 ,0x0023           },
    {GMACTRL6_7 ,0x000D           },
    {GMACTRL6_8 ,0x0009           },
    {GMACTRL6_9 ,0x003E           },
    {GMACTRL6_A ,0x0016           },
    {GMACTRL6_B ,0x002D           },
    {GMACTRL6_C ,0x0048           },
    {GMACTRL6_D ,0x0046           },
    {GMACTRL6_E ,0x0051           },
    {GMACTRL6_F ,0x005F           },

    {DISPON, 1                    }, //display on
    {TABLE_END  ,0x0000           }, /* terminate table */
};

static struct init_table const tpo_nv_version_F_init[] = 
{
    {MDDI_SLEEP_CMD,10             }, // wait 5ms

//Set Sleep-Out command    - Set Register access packet Address = 110h, data = 0h
    {SLEPCTRL ,0x0000             },
 
    {MDDI_SLEEP_CMD,10		  }, // wait 5ms

    {PANELCTRL1 ,0x0044           },
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x0087           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x007B           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x000F           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },

    {ENTRYMODE1 ,0x0030           }, //ENTRYMODE1: Entry Mode 1 (360h)
#if defined(CONFIG_MACH_CALGARY)
    {ENTRYMODE2 ,0x00C5           },
#elif defined(CONFIG_MACH_PITTSBURGH)
    {ENTRYMODE2 ,0x0005           },
#else
    {ENTRYMODE2 ,0x00C5           },
#endif

    {DRVCTRL ,0x0012              },
    {DRV_OUT_CTRL1 ,0x00EF        },
    {DRV_OUT_CTRL2 ,0x0003        },

    {SPECTRL ,0x0002              },

    {WHRAMLOWST ,0x0000           }, //Window Horizontal RAM Low Address Start
    {WHRAMHIGHST ,0x0000          }, //Window Horizontal RAM High Address Start (2A1h)
    {WHRAMLOWEND ,0x003F          }, //Window Horizontal RAM Low Address End (2A2h) 
    {WHRAMHIGHEND ,0x0001         }, //Window Horizontal RAM High Address End (2A3h)
    {WVRAMLOWST ,0x0000           }, //Window Vertical RAM Low Address Start (2B0h)
    {WVRAMHIGHST ,0x0000          }, //Window Vertical RAM High Address Start (2B1h) 
    {WVRAMLOWEND ,0x00DF          }, //Window Vertical RAM Low Address End (2B2h)
    {WVRAMHIGHEND ,0x0001         }, //Window Vertical RAM High Address End (2B3h) 
    {RAMHSETL ,0x0000             }, //RAMHSETL: RAM Low Address Set (Horizontal Address) (2D0h)    
    {RAMHSETH ,0x0000             }, //RAMHSETH: RAM High Address Set (Horizontal Address) (2D1h)
    {RAMVSETL ,0x0000             }, //RAMVSETL: RAM Low Address Set (Vertical Address) (2D2h) 
    {RAMVSETH ,0x0000             }, //RAMVSETH: RAM High Address Set (Vertical Address) (2D3h) 

    {FPOSITION ,0                 }, //FPOSITION: Frame Tearing Effect Position (350h)
    {FCYCLE ,0x0000               }, //FCYCLE: Frame Tearing Effect Position and Output cycle(351h)

    {PWRCTRL2 ,0x0010             }, //Power Control 2 (0xC31)
    {PWRCTRL4 ,0x0010             }, //Power Control 4 (C31h)

    {PUMPCTRL1, 0x0000            }, //DC/DC on Glass Control 1 (0xBA0)
    {PUMPCTRL2, 0x0086            }, //DC/DC on Glass Control 2 (0xBA1)

//#ifdef THESE_ARE_THE_CABC_PARAMS
    {WRCTRLD, 0x13                }, //CABC: Write CTRL Display (530h)
    {WRCABC, 0x00                 }, //CABC: Content Adaptive Brightness Control (550h)
//    {WRCABCMB, 0xFE               }, //CABC: Minimum Brightness Level (5E0h)
//#endif

    {VCOMHCTRL, 0x0033            }, //VCOM_H Voltage Control (C50h)
    {VCOMDCCTRL, 0x0032           }, //VCOM_DC Voltage Control (C52h)

    {PWRCTRL5 ,0x0034             }, //Power Control 5 (C40h)

    {SOUTCTRL1 ,0x0001            }, //Source Output Control 1 (BE0h)
    {SOUTCTRL2 ,0x0000            }, //Source Output Control 2 (BE1h)

    // Red Gamma positive polarity
    {GMACTRL1_0 ,0x0000           },
    {GMACTRL1_1 ,0x0007           },
    {GMACTRL1_2 ,0x000C           },
    {GMACTRL1_3 ,0x0012           },
    {GMACTRL1_4 ,0x002C           },
    {GMACTRL1_5 ,0x0026           },
    {GMACTRL1_6 ,0x0012           },
    {GMACTRL1_7 ,0x0013           },
    {GMACTRL1_8 ,0x0011           },
    {GMACTRL1_9 ,0x0039           },
    {GMACTRL1_A ,0x0015           },
    {GMACTRL1_B ,0x0025           },
    {GMACTRL1_C ,0x003B           },
    {GMACTRL1_D ,0x0032           },
    {GMACTRL1_E ,0x0054           },
    {GMACTRL1_F ,0x006D           },

    // Red Gamma negative polarity
    {GMACTRL2_0 ,0x0001           },
    {GMACTRL2_1 ,0x0019           },
    {GMACTRL2_2 ,0x003B           },
    {GMACTRL2_3 ,0x0025           },
    {GMACTRL2_4 ,0x003B           },
    {GMACTRL2_5 ,0x002C           },
    {GMACTRL2_6 ,0x002E           },
    {GMACTRL2_7 ,0x000A           },
    {GMACTRL2_8 ,0x000A           },
    {GMACTRL2_9 ,0x004E           },
    {GMACTRL2_A ,0x0019           },
    {GMACTRL2_B ,0x0032           },
    {GMACTRL2_C ,0x004C           },
    {GMACTRL2_D ,0x0051           },
    {GMACTRL2_E ,0x0056           },
    {GMACTRL2_F ,0x005C           },

    // Green Gamma positive polarity
    {GMACTRL3_0 ,0x0000           },
    {GMACTRL3_1 ,0x0009           },
    {GMACTRL3_2 ,0x0011           },
    {GMACTRL3_3 ,0x0013           },
    {GMACTRL3_4 ,0x002E           },
    {GMACTRL3_5 ,0x0027           },
    {GMACTRL3_6 ,0x001B           },
    {GMACTRL3_7 ,0x0015           },
    {GMACTRL3_8 ,0x0011           },
    {GMACTRL3_9 ,0x003C           },
    {GMACTRL3_A ,0x0014           },
    {GMACTRL3_B ,0x0022           },
    {GMACTRL3_C ,0x0039           },
    {GMACTRL3_D ,0x0039           },
    {GMACTRL3_E ,0x0058           },
    {GMACTRL3_F ,0x006D           },

    // Green Gamma negative polarity
    {GMACTRL4_0 ,0x0001           },
    {GMACTRL4_1 ,0x0015           },
    {GMACTRL4_2 ,0x0034           },
    {GMACTRL4_3 ,0x0028           },
    {GMACTRL4_4 ,0x003F           },
    {GMACTRL4_5 ,0x002D           },
    {GMACTRL4_6 ,0x002B           },
    {GMACTRL4_7 ,0x000C           },
    {GMACTRL4_8 ,0x0006           },
    {GMACTRL4_9 ,0x0046           },
    {GMACTRL4_A ,0x0017           },
    {GMACTRL4_B ,0x002F           },
    {GMACTRL4_C ,0x004B           },
    {GMACTRL4_D ,0x004C           },
    {GMACTRL4_E ,0x0053           },
    {GMACTRL4_F ,0x005C           },

    // Blue Gamma positive polarity
    {GMACTRL5_0 ,0x0000           },
    {GMACTRL5_1 ,0x0005           },
    {GMACTRL5_2 ,0x0017           },
    {GMACTRL5_3 ,0x0013           },
    {GMACTRL5_4 ,0x0030           },
    {GMACTRL5_5 ,0x0027           },
    {GMACTRL5_6 ,0x0021           },
    {GMACTRL5_7 ,0x0013           },
    {GMACTRL5_8 ,0x000F           },
    {GMACTRL5_9 ,0x0040           },
    {GMACTRL5_A ,0x0016           },
    {GMACTRL5_B ,0x0027           },
    {GMACTRL5_C ,0x0042           },
    {GMACTRL5_D ,0x0037           },
    {GMACTRL5_E ,0x006C           },
    {GMACTRL5_F ,0x006D           },

    // Blue Gamma negative polarity
    {GMACTRL6_0 ,0x0001           },
    {GMACTRL6_1 ,0x0005           },
    {GMACTRL6_2 ,0x0035           },
    {GMACTRL6_3 ,0x001F           },
    {GMACTRL6_4 ,0x003A           },
    {GMACTRL6_5 ,0x002A           },
    {GMACTRL6_6 ,0x0027           },
    {GMACTRL6_7 ,0x000D           },
    {GMACTRL6_8 ,0x0009           },
    {GMACTRL6_9 ,0x0040           },
    {GMACTRL6_A ,0x0017           },
    {GMACTRL6_B ,0x002D           },
    {GMACTRL6_C ,0x004A           },
    {GMACTRL6_D ,0x0046           },
    {GMACTRL6_E ,0x0058           },
    {GMACTRL6_F ,0x005C           },
    {DISPON, 1                    }, //display on
    {TABLE_END  ,0x0000           }, /* terminate table */
};

static struct init_table const tpo_nv_version_G_init[] =
{
//Wait 5 msec or more after HW reset
    {MDDI_SLEEP_CMD,10            }, // wait 5ms

//Set Sleep-Out command    - Set Register access packet Address = 110h, data = 0h
    {SLEPCTRL ,0x0000             },

//Wait 5 msec or more
    {MDDI_SLEEP_CMD,10            }, // wait 5ms

#ifdef NINETY_HZ
    {PANELCTRL1 ,0x002E           }, // 90Hz
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x005B           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x004F           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x0006           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },

    {PANELCTRL1 ,0x0044           }, // 60Hz
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x0087           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x007B           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x000F           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },
#endif

    {PANELCTRL1 ,0x0037           }, // 75Hz
    {PANELCTRL2 ,0x0000           },
    {PANELCTRL3 ,0x006D           },
    {PANELCTRL4 ,0x0006           },
    {PANELCTRL5 ,0x0061           },
    {PANELCTRL6 ,0x000E           },
    {PANELCTRL7 ,0x0009           },
    {PANELCTRL8 ,0x0003           },
    {PANELCTRL9 ,0x0000           },

    {ENTRYMODE1 ,0x0030           }, //ENTRYMODE1: Entry Mode 1 (360h)
#if defined(CONFIG_MACH_CALGARY)
    {ENTRYMODE2 ,0x00C5           },
#elif defined(CONFIG_MACH_PITTSBURGH)
    {ENTRYMODE2 ,0x0005           },
#else
    {ENTRYMODE2 ,0x00C5           },
#endif

    {DRVCTRL ,0x0012              },
    {DRV_OUT_CTRL1 ,0x00EF        },

    {PWRCTRL4 ,0x0010             }, //Power Control 4 (C31h)

    {PUMPCTRL1, 0x0000            }, //DC/DC on Glass Control 1 (0xBA0)
    {PUMPCTRL2, 0x0086            }, //DC/DC on Glass Control 2 (0xBA1)

    {FPOSITION, 0x1               }, //FPOSITION: Frame Tearing Effect Position (350h)
    {FCYCLE ,0x0000               }, //FCYCLE: Frame Tearing Effect Position and Output cycle(351h)

    {WRDISBV, 0xff                }, //CABC: Write Display Brightness (510h)
    {WRCTRLD, 0x13                }, //CABC: Write CTRL Display (530h) Disable dimming. It causes a 4..5
                                     //second delay in backlight enable when resumed
    {WRCABC, 0x02                 }, //CABC: Content Adaptive Brightness Control (550h)
    {0x6d9, 0                     }, //CABC:
    {0x6e4, 31                    }, //CABC:
    {0x6e5, 15                    }, //CABC:
    {0x6e6, 11                    }, //CABC: dimming time (fastest)
    {0x6e7, 0                     }, //CABC:

    {PWRCTRL5 ,0x0034             }, //Power Control 5 (C40h)

    {SOUTCTRL1 ,0x0001            }, //Source Output Control 1 (BE0h)
    {SOUTCTRL2 ,0x0000            }, //Source Output Control 2 (BE1h)

    {WHRAMLOWST ,0x0000           }, //Window Horizontal RAM Low Address Start
    {WHRAMHIGHST ,0x0000          }, //Window Horizontal RAM High Address Start (2A1h)
    {WHRAMLOWEND ,0x003F          }, //Window Horizontal RAM Low Address End (2A2h) 
    {WHRAMHIGHEND ,0x0001         }, //Window Horizontal RAM High Address End (2A3h)
    {WVRAMLOWST ,0x0000           }, //Window Vertical RAM Low Address Start (2B0h)
    {WVRAMHIGHST ,0x0000          }, //Window Vertical RAM High Address Start (2B1h) 
    {WVRAMLOWEND ,0x00DF          }, //Window Vertical RAM Low Address End (2B2h)
    {WVRAMHIGHEND ,0x0001         }, //Window Vertical RAM High Address End (2B3h) 
    {RAMHSETL ,0x0000             }, //RAMHSETL: RAM Low Address Set (Horizontal Address) (2D0h)    
    {RAMHSETH ,0x0000             }, //RAMHSETH: RAM High Address Set (Horizontal Address) (2D1h)
    {RAMVSETL ,0x0000             }, //RAMVSETL: RAM Low Address Set (Vertical Address) (2D2h) 
    {RAMVSETH ,0x0000             }, //RAMVSETH: RAM High Address Set (Vertical Address) (2D3h) 

    {DISPON, 1                    }, //display on
    {TABLE_END, 0x0000            }, /* terminate table */
};

static struct init_table const tpo_nv_uninit_defines[] =
{
   { DISPON, 0          }, //display off
   { MDDI_SLEEP_CMD, 5  }, // wait 5ms
   /*
    * Set Sleep-In command
    * Set Register access packet Address = 110h, data = 1h
    */
   { SLEPCTRL,  0x0001  },
   { TABLE_END, 0x0000  }, /* terminate table */
};

static struct init_table const *disp_init = NULL;
static struct init_table const *disp_uninit = NULL;
static struct init_table const *disp_esd_recovery = NULL;

/*=============================================================================
                              GLOBAL VARIABLES
=============================================================================*/
#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
extern mddi_client_capability_type mddi_client_capability_pkt;
#endif
/*=============================================================================
                              LOCAL FUNCTIONS
=============================================================================*/
static void xmit_panel_seq(struct init_table const *init_table)
{
   unsigned n;

   for (n = 0;; n++)
   {
      if (init_table[n].reg == TABLE_END)
      {
         break;
      }
      else if (init_table[n].reg == MDDI_SLEEP_CMD)
      {
         mddi_wait(init_table[n].val);
      }
      else
      {
         mddi_queue_register_write(init_table[n].reg, init_table[n].val, 0, 0);
      }
   }
}

static void mddi_tpo_nv_esd_poll(void* arg)
{
#ifdef MDDI_TPO_NV_ESD_DEBUG
   uint32_t val;
#endif
   struct platform_device *pdev = (struct platform_device *)arg;

   msm_fb_down(pdev);
#ifdef MDDI_TPO_NV_ESD_DEBUG
   printk(KERN_ERR "TPO NV - ESD register check START\n");
   mddi_queue_register_write(ENTRYMODE2 ,0x00C4, 0, 0);
   mddi_wait(20);
   for (i = 0;; i++)
   {
      if (disp_init[i].reg == TABLE_END)
      {
         break;
      }
      else if (disp_init[i].reg != MDDI_SLEEP_CMD)
      {
         mddi_queue_register_read(disp_init[i].reg, &val, 1, 0);
         if (disp_init[i].val != val)
         {
            printk(KERN_ERR "TPO NV reg 0x%X should be 0x%X, found 0x%X\n",
               disp_init[i].reg, disp_init[i].val, val); 
         }
      }
   }
   mddi_queue_register_write(ENTRYMODE2 ,0x00C5, 0, 0);
   mddi_wait(30);
   printk(KERN_ERR "TPO NV - ESD register check END\n");
#endif /* MDDI_TPO_NV_ESD_DEBUG */
   if (disp_esd_recovery)
   {
      xmit_panel_seq(disp_esd_recovery);
   }
   msm_fb_up(pdev);
}

static int mddi_tpo_nv_seq_init(void)
{
   if (plat_disp_type)
   {
      if (plat_disp_type == TPO_PLAT_DISP)
      {
         printk("%s: Found TPO Platform Display\n", __FUNCTION__);
         disp_init = tpo_nv_plat_init;
      }
      else if (plat_disp_type == TMD_PLAT_DISP)
      {
         printk("%s: Found TMD Platform Display\n", __FUNCTION__);
         disp_init = tmd_nv_plat_init;
      }
      mddi_tpo_nv_cabc_en = 1;
      disp_uninit = tpo_nv_plat_uninit;
      disp_esd_recovery = tpo_nv_plat_esd_recovery;
   }
   else
   {
      unsigned userid = 0;
      unsigned revision = 0;

      mddi_queue_register_read(USERID, &userid, TRUE, 0);
      mddi_queue_register_read(REVISIONID, &revision, TRUE, 0);

      switch(userid)
      {
      case TPO_NV_VER_F_USERID:
         printk("%s: Found TPO vF Display\n", __FUNCTION__);
         disp_init = tpo_nv_version_F_init;
         break;
#ifdef CONFIG_MACH_MOT
	case TPO_NV_VER_G_USERID:
	 printk("%s: Found TPO vG Display\n", __FUNCTION__);
	 disp_init = tpo_nv_version_G_init;
         break;
#endif
      default:
         printk("%s: Found TPO vE Display\n", __FUNCTION__);
         disp_init = tpo_nv_version_E_init;
         break;
      }
      disp_uninit = tpo_nv_uninit_defines;
   }
   if (!disp_init)
   {
      return -EINVAL;
   }
   return 0;
}

static int mddi_tpo_nv_lcd_on(struct platform_device *pdev)
{
   struct msm_fb_data_type *mfd;

   mfd = platform_get_drvdata(pdev);

   if (!mfd || mfd->panel.id != TPO_NV_HVGA_PRIM)
      return -ENODEV;

   if (mfd->key != MFD_KEY)
      return -EINVAL;

   if (!disp_init)
   {
      if (mddi_tpo_nv_seq_init())
      {
         return -EINVAL;
      }
      esd_poll_start(mddi_tpo_nv_esd_poll, pdev);
   }

   if (tpo_nv_powered_on == FALSE)
   {
      xmit_panel_seq(disp_init);
      esd_poll_start(mddi_tpo_nv_esd_poll, pdev);
      tpo_nv_powered_on = TRUE;
   }
   
   als_pdev = mfd->pdev;
   
   return 0;
}

static int mddi_tpo_nv_lcd_off(struct platform_device *pdev)
{
   if (!disp_init)
   {
      if (mddi_tpo_nv_seq_init())
      {
         return -EINVAL;
      }
   }
   
   if (tpo_nv_powered_on == TRUE)
   {
      esd_poll_stop(mddi_tpo_nv_esd_poll);
      xmit_panel_seq(disp_uninit);
      tpo_nv_powered_on = FALSE;
   }
   return 0;
}

static void enable_cabc(struct platform_device *pdev, int en)
{
   unsigned reg_addr = 0;

   printk("enable_cabc(%d)\n", en);

   if(tpo_nv_version_G_init == disp_init)
       reg_addr = WRCABC; // interim cabc command
   else if((plat_disp_type == TPO_PLAT_DISP) || (plat_disp_type == TMD_PLAT_DISP))
            reg_addr = MM_WRCABC; // platform display cabc command

   if(reg_addr)
   {
     msm_fb_down(pdev); // serialize access to the panel & MDDI while communicating
     mddi_queue_register_write(reg_addr, 2*en, TRUE, 0); 
     msm_fb_up(pdev);
     mddi_tpo_nv_cabc_en = en;
   }
}

static int mddi_tpo_nv_als_cb(unsigned old_zone, unsigned new_zone, uint32_t cookie)
{
    printk("mddi_tpo_nv_als_cb( 0x%x , 0x%x ) - powered_on: 0x%x - als_pdev: 0x%x \n",
	   old_zone, new_zone, tpo_nv_powered_on, (unsigned int) als_pdev);

    if (tpo_nv_powered_on == TRUE && als_pdev)
    {
        if(new_zone > 1 && mddi_tpo_nv_cabc_en) enable_cabc((struct platform_device *)als_pdev, 0); // moved from indoor to outdoor
        else if(new_zone < 2 && !mddi_tpo_nv_cabc_en) enable_cabc((struct platform_device *)als_pdev, 1); // moved from outdoor to indoor
    }

    return 0;
}

static int __init mddi_tpo_nv_probe(struct platform_device *pdev)
{
   extern int lm3535_register_als_callback (int (*cb)(unsigned old_zone, unsigned new_zone, uint32_t cookie), uint32_t cookie);

   msm_fb_add_device(pdev);

   lm3535_register_als_callback(mddi_tpo_nv_als_cb, 0);

   return 0;
}

static struct platform_driver mddi_tpo_nv_driver = {
   .probe = mddi_tpo_nv_probe,
   .driver = {
      .name = "mddi_tpo_nv"
   },
};

static struct msm_fb_panel_data mddi_tpo_nv_panel_data = {
   .on = mddi_tpo_nv_lcd_on,
   .off = mddi_tpo_nv_lcd_off,
};

static struct platform_device mddi_tpo_nv_device = {
   .name = "mddi_tpo_nv",
   .id = TPO_NV_HVGA_PRIM,
   .dev = {
      .platform_data = &mddi_tpo_nv_panel_data,
   }
};

static int __init mddi_tpo_nv_init(void)
{
   int ret;
   struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
   mddi_wait(10); // wait to come completely out of reset
   plat_disp_type = mddi_get_client_id();

   printk("%s: client_id=0x%X, crc=0x%X, width=%d, height=%d\n",
      __FUNCTION__, plat_disp_type, mddi_client_capability_pkt.parameter_CRC,
      mddi_client_capability_pkt.Bitmap_Width,
      mddi_client_capability_pkt.Bitmap_Height);
   if (mddi_client_capability_pkt.Bitmap_Width == 0x00 ||
       mddi_client_capability_pkt.Bitmap_Height == 0x00)
   {
      printk("%s: panel appears to be missing. Exiting\n", __FUNCTION__);
      return 0;
   }
#endif

   ret = platform_driver_register(&mddi_tpo_nv_driver);
   if (!ret) {
      pinfo = &mddi_tpo_nv_panel_data.panel_info;
      pinfo->xres = LCD_WIDTH;
      pinfo->yres = LCD_HEIGHT;
      pinfo->type = MDDI_PANEL;
      pinfo->pdest = DISPLAY_1;
      pinfo->wait_cycle = 0;
      pinfo->bpp = 18;
      pinfo->fb_num = 2;
      pinfo->clk_min = TPO_NV_CLK_RATE;
      pinfo->clk_max = TPO_NV_CLK_RATE;
      pinfo->clk_rate = TPO_NV_CLK_RATE;
      pinfo->lcd.vsync_enable = TRUE;
      pinfo->lcd.refx100 =
         (mddi_tpo_nv_rows_per_second * 100) /
         mddi_tpo_nv_rows_per_refresh;
      pinfo->lcd.v_back_porch = V_BACK_PORCH;
      pinfo->lcd.v_front_porch = V_FRONT_PORCH;
      pinfo->lcd.v_pulse_width = V_SYNC_WIDTH;
      pinfo->lcd.hw_vsync_mode = TRUE;
      pinfo->lcd.vsync_notifier_period = (1 * HZ);
      pinfo->bl_max = 7;
      pinfo->bl_min = 1;

      ret = platform_device_register(&mddi_tpo_nv_device);
      if (ret)
      {
         platform_driver_unregister(&mddi_tpo_nv_driver);
      }
   }
   return ret;
}

unsigned msm_fb_mddi_nv_get_manprodid (void)
{
    return plat_disp_type;
}
EXPORT_SYMBOL(msm_fb_mddi_nv_get_manprodid);

unsigned get_tpo_defvalue(void)
{
   return TPO_PLAT_DISP;
}
EXPORT_SYMBOL(get_tpo_defvalue);

void msm_fb_led_resumed(void)
{
    printk("msm_fb_led_resumed - powered_on: 0x%x - als_pdev: 0x%x \n",
           tpo_nv_powered_on, (unsigned int) als_pdev);

    if(tpo_nv_powered_on == TRUE && als_pdev)
    {
       unsigned reg_addr = 0;
       unsigned reg_val = 0x0016;

       if(tpo_nv_version_G_init == disp_init)
           reg_addr = WRCTRLD; // interim cabc command
       else if(plat_disp_type == TPO_PLAT_DISP)
           reg_addr = MM_WRCTRLD; // platform display cabc command
       else if(plat_disp_type == TMD_PLAT_DISP)
       {
	 reg_addr = MM_WRCTRLD;
	 reg_val  = 0x002C;
       }

       if(reg_addr)
       {
            msm_fb_down(als_pdev); // serialize access to the panel & MDDI while communicating
            mddi_queue_register_write(reg_addr, reg_val, TRUE, 0); // enable dimming bit
            msm_fb_up(als_pdev);
       }
    }
}
EXPORT_SYMBOL(msm_fb_led_resumed);

module_init(mddi_tpo_nv_init);

