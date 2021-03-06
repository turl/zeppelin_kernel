/* linux/arch/arm/mach-msm/board-mot-7x27.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * Copyright (c) 2008-2009 QUALCOMM Incorporated.
 *
 * Copyright (C) 2009 Motorola, Inc.
 * Author: Adam Zajac <Adam.Zajac@motorola.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#if defined(CONFIG_TOUCHSCREEN_QUANTUM_OBP)
#include <linux/qtouch_obp_ts.h>
#endif /* CONFIG_TOUCHSCREEN_QUANTUM_OBP */
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/power_supply.h>
//#include <linux/timed_gpio.h>
#include <../../../drivers/staging/android/timed_gpio.h> //header got moved for 2.6.29
#include <linux/sfh7743.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/mach/mmc.h>


#include <mach/msm_fb.h>
#include <linux/device.h> /* for vreg.h */
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_hs.h>
#include <mach/system.h>
#include <mach/mpp.h>
#include <asm/setup.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_serial_hs.h>
#include <mach/memory.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <mach/crucialtec_oj.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <linux/proc_fs.h>
#include <mach/memory.h>
#include <mach/camera.h>
#include <mach/msm_battery.h>
#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#endif

#include "devices.h"
#include "clock.h"
#include "proc_comm.h"
#include "socinfo.h"
#include "msm-keypad-devices.h"
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif
#ifndef CONFIG_MACH_PITTSBURGH
#include "timed_vibrator.h"
#endif /*CONFIG_MACH_PITTSBURGH*/
#include "board-mot.h"
#include "pm.h"
#include <mot/mot_handover.h>

unsigned long long calg_hw_serial = 0;

static struct kobject *mot_kobj;
static spinlock_t g_list_lock = SPIN_LOCK_UNLOCKED;

#if defined (CONFIG_MACH_PITTSBURGH)
extern void msm_init_pmic_vibrator(void);
#endif /*CONFIG_MACH_PITTSBURGH*/

static struct resource stmicro_resources[] = {
	{
		.start	= LIS331DLH_INT1,
		.end	= LIS331DLH_INT2,
		.flags	= IORESOURCE_IRQ,
	},
};

#if defined(CONFIG_TOUCHSCREEN_KEY08)
static struct resource key08_resources[]  =  {
	{
		.start	= KEY08_I2C_ADDR,
		.end	= KEY08_BL_I2C_ADDR,
		.flags	= IORESOURCE_MEM,
	},
};
#endif /* CONFIG_TOUCHSCREEN_KEY08 */

static struct platform_device mot_adp8860_device = {
	.name		= "adp8860",
};

static struct platform_device mot_adp8870_device = {
	.name		= "adp8870",
};

static struct platform_device mot_akm8973_device = {
    .name       = "akm8973",
    .id         = 0
};

static struct platform_device mot_adp5588_device = {
	.name		= "adp5588_keypad", 
	.id             = 0,
};

#if defined(CONFIG_TOUCHSCREEN_KEY08)
static struct platform_device mot_key08_device = {
	.name		= "key08",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(key08_resources),
	.resource	= key08_resources,
};
#endif /* CONFIG_TOUCHSCREEN_KEY08 */

static struct platform_device mot_stmicro_device = {
	.name		= "stmicro",
	.id             = 11,
	.num_resources	= ARRAY_SIZE(stmicro_resources),
	.resource	= stmicro_resources,
};

#if defined(CONFIG_TOUCHSCREEN_QUANTUM_OBP) 
static int qtouch_touch_reset(void)
{
	gpio_direction_output(TOUCH_RST_N, 1);
	msleep(1);
	gpio_set_value(TOUCH_RST_N, 0);
	msleep(20);
	gpio_set_value(TOUCH_RST_N, 1);
	msleep(20);

	return 0;
}

static struct qtm_touch_keyarray_cfg qtouch_key_array_data[] = {
   {
      .ctrl = 0,
      .x_origin = 0,
      .y_origin = 0,
      .x_size = 0,
      .y_size = 0,
      .aks_cfg = 0,
      .burst_len = 0,
      .tch_det_thr = 0,
      .tch_det_int = 0,
      .rsvd1 = 0,
      .rsvd2 = 0,
   },
   {
      .ctrl = 0,
      .x_origin = 0,
      .y_origin = 0,
      .x_size = 0,
      .y_size = 0,
      .aks_cfg = 0,
      .burst_len = 0,
      .tch_det_thr = 0,
      .tch_det_int = 0,
      .rsvd1 = 0,
      .rsvd2 = 0,
   },
};

static struct vkey qtouch_touch_vkeys[] = {
   {
      .code    = KEY_BACK,
   },
   {
      .code    = KEY_MENU,
   },
   {
      .code    = KEY_HOME,
   },
   {
      .code    = KEY_SEARCH,
   },
};

static struct qtouch_ts_platform_data qtouch_platform_data = {
   .irqflags   = IRQF_TRIGGER_LOW,
   .flags      = (QTOUCH_SWAP_XY |
            QTOUCH_FLIP_Y |
            QTOUCH_USE_MULTITOUCH |
            QTOUCH_CFG_BACKUPNV |
            QTOUCH_EEPROM_CHECKSUM),
   .abs_min_x  = 0,
   .abs_max_x  = 1023,
   .abs_min_y  = 0,
   .abs_max_y  = 1023,
   .abs_min_p  = 0,
   .abs_max_p  = 255,
   .abs_min_w  = 0,
   .abs_max_w  = 15,
   .nv_checksum   = 0xfaf5,
   .fuzz_x     = 0,
   .fuzz_y     = 0,
   .fuzz_p     = 2,
   .fuzz_w     = 2,
   .hw_reset   = qtouch_touch_reset,
   .power_cfg  = {
      .idle_acq_int     = 0xff,
      .active_acq_int   = 0xff,
      .active_idle_to   = 50,
   },
   .acquire_cfg   = {
      .charge_time   = 7,
      .atouch_drift  = 5,
      .touch_drift   = 40,
      .drift_susp    = 5,
      .touch_autocal = 0,
      .sync          = 0x06,
      .atch_cal_suspend_time  = 0,
      .atch_cal_suspend_thres = 0,
   },
   .multi_touch_cfg  = {
      .ctrl    = 3,
      .x_origin   = 0,
      .y_origin   = 0,
      .x_size     = 13,
      .y_size     = 9,
      .aks_cfg    = 0,
      .burst_len  = 0x30,
      .tch_det_thr   = 0x2F,
      .tch_det_int   = 0x2,
      .orient        = 0,
      .mrg_to        = 50,
      .mov_hyst_init = 9,
      .mov_hyst_next = 5,
      .mov_filter = 0,
      .num_touch  = 10,
      .merge_hyst = 5,
      .merge_thresh  = 10,
      .amp_hyst   = 10,
      .x_res   = 0x0000,
      .y_res   = 0x0000,
      .x_low_clip = 0x00,
      .x_high_clip   = 0x00,
      .y_low_clip = 0x00,
      .y_high_clip   = 0x00,
      .x_edge_ctrl   = 0,
      .x_edge_dist   = 0,
      .y_edge_ctrl   = 0,
      .y_edge_dist   = 0,
   },
   .linear_tbl_cfg   = {
      .ctrl = 0x01,
      .x_offset   = 0x0000,
      .x_segment  = {
         0x48, 0x3f, 0x3c, 0x3E,
         0x3f, 0x3e, 0x3e, 0x3e,
         0x3f, 0x42, 0x41, 0x3f,
         0x41, 0x40, 0x41, 0x46
      },
      .y_offset   = 0x0000,
      .y_segment  = {
         0x44, 0x38, 0x37, 0x3e,
         0x3e, 0x41, 0x41, 0x3f,
         0x42, 0x41, 0x42, 0x42,
         0x41, 0x3f, 0x41, 0x45
      },
   },
   .grip_suppression_cfg   = {
      .ctrl    = 0x0C,
      .xlogrip = 0x00,
      .xhigrip = 0x00,
      .ylogrip = 0x00,
      .yhigrip = 0x00,
      .maxtchs = 0x00,
      .reserve0   = 0x00,
      .szthr1  = 0x00,
      .szthr2  = 0x00,
      .shpthr1 = 0x00,
      .shpthr2 = 0x00,
   },
   .touch_proximity_cfg = {
      .ctrl = 0xE0,
   },
   .noise_suppression_cfg = {
      .ctrl = 0x00,
   },
   .gpio_pwm_cfg = {
      .report_mask = 0x3C,
   },
   .self_test_cfg = {
      .ctrl = 0x3,
      .command = 0xFE,
   },
   .cte_config_cfg = {
      .idle_gcaf_depth = 0x4,
   },
   .key_array  = {
      .cfg        = qtouch_key_array_data,
      .num_keys   = ARRAY_SIZE(qtouch_key_array_data),
   },
   .vkeys= {
      .keys    = qtouch_touch_vkeys,
      .count   = ARRAY_SIZE(qtouch_touch_vkeys),
   },
};
#endif /* CONFIG_TOUCHSCREEN_QUANTUM_OBP */

#ifndef CONFIG_MACH_PITTSBURGH
static struct vibrator_platform_data vibrator_data = {
	.name = "vibrator",
	.gpio_en = HAPTICS_AMP_EN,
	.gpio_pwm = HAPTICS_MSM_PWM,
	.max_timeout = 15000,
	.active_low  = 0,

};

static struct platform_device mot_vibrator = {
	.name		= "vibrator",
	.id		= -1,
	.dev		= {
		.platform_data	= &vibrator_data,
	},
};
#endif /*CONFIG_MACH_PITTSBURGH*/


#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x01,
	.buf_size       = 16384,
	.vendor         = "Motorola",
	.product        = "Mass Storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif


#ifdef CONFIG_USB_ANDROID
/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = MOT_PID,
		/* DIAG + ADB + GENERIC MODEM + GENERIC NMEA + MSC*/
		.functions	    = 0x27614,
	}
};
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= MOT_VID,
	.product_id	= MOT_PID,
	.adb_product_id	= MOT_PID,
	.functions	= 0x3F,
	.version	= 0x0100,
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.product_name	= "Motorola A555",
	.manufacturer_name = "Motorola Inc.",
	.nluns = 1,
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
    .wakeup_irq = MSM_GPIO_TO_INT(45),
    .inject_rx_on_wakeup = 0, // 1; We have BT wakeup signal 
    .rx_to_inject = 0x32,
};


#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 			0},
	{"modem", 			1},
	{"ethernet", 		2},
	{"usb_mass_storage", 			3},
	{"adb",4},
	{"nmea", 			5},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
#ifdef CONFIG_EXPOSE_NMEA
	{
		.product_id     = MOT_PID,
		.functions	    = 0x3F, 	/* 00111111 ether/ums/modem/nmea/adb/diag */
		.config         = "Motorola Config 35",
	},
#else
	{
               .product_id     = MOT_PID1,
               .functions      = 0x0F, /* 00001111 ums/ether/modem/diag */
               .config         = "Motorola Config 34", 
                                  /* Motorola Android Composite Device */
       },
      {
               .product_id     = MOT_PID2,
               .functions      = 0x08,     /* 00001000 ums */
               .config         = "Motorola Config 14", 
                                 /* Motorola Mass Storage Class */
       },
       {
               .product_id     = MOT_PID3,
               .functions      = 0x08,     /* 00001000 ums */
               .config         = "Motorola Config 15", 
                                 /* Motorola Mass Storage Class MTD*/
       },
       {
                .product_id     = MOT_PID4,
               .functions      = 0x1f,     /* 00011111 adb/ums/ether/modem/diag */
               .config         = "Motorola Config 35", 
                                 /* Motorola Android Composite Device */
       },
       {
               .product_id     = MOT_PID5,
               .functions      = 0x19,     /* 00011001 adb/ums/diag */
               .config         = "Motorola Config 36", 
                                 /* Motorola Mass Storage Class */
       },
       {
               .product_id     = MOT_PID6,
               .functions      = 0x19,     /* 00011001 adb/ums/diag */
               .config         = "Motorola Config 37", 
                                 /* Motorola Mass Storage Class */
       },
       {
               .product_id     = MOT_PID7,
               .functions      = 0x08,     /* 00001000 ums */
               .config         = "Motorola Mass Storage Class",
       },
       {
               .product_id     = MOT_PID8,
               .functions      = 0x08,     /* 00001000 ums */
               .config         = "Motorola Config 14", 
                                 /* Motorola Mass Storage Class CD-ROM */
        },
       {
                .product_id     = MOT_PID9,
               .functions      = 0x05,     /* 00000101 ether/diag */
               .config         = "Motorola Config 47", 
       },
#endif /* CONFIG_EXPOSE_NMEA */
};
#endif

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
	.vendor_id          = MOT_VID,
	.product_id         = MOT_PID,
 	.product_name       = "Motorola A555",
	.serial_number      = MOT_SERIAL_NUM,
	.manufacturer_name  = "Motorola Inc.",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,
#endif
};

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
#ifdef CONFIG_MACH_PITTSBURGH
   SND(HANDSET, 0),      /* Pittsburgh: use primary mic */
#else
	SND(HANDSET, 16),
#endif
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(HEADPHONE, 4),    // Stereo headphone    , no mic    
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
#ifndef CONFIG_MACH_PITTSBURGH
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
#endif
	SND(CURRENT, 31),
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP)| \
	(1<<MSM_ADSP_CODEC_MP3))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP)| \
	(1<<MSM_ADSP_CODEC_MP3))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 5),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 5),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 4),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.start = 0,
	.size = MSM_PMEM_MDP_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.start = 0,
	.size = MSM_PMEM_ADSP_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.start = 0,
	.size = MSM_PMEM_GPU1_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_camera_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static char *msm_fb_vreg[] = {
	"gp3"
};

#define MSM_FB_VREG_OP(name, op)					\
do { \
	vreg = vreg_get(0, name); \
	if (vreg_##op(vreg))						\
		printk(KERN_ERR "%s: %s vreg operation failed \n",	\
			(vreg_##op == vreg_enable) ? "vreg_enable" \
				: "vreg_disable", name); \
} while (0)

static void msm_fb_mddi_power_save(int on)
{
	struct vreg *vreg;
	int i;

	if (!on)
	{
      gpio_set_value(LCD_RST_N_SIGNAL, 0);
      msleep_interruptible(TPO_NV_RST_VREG_DELAY);
	}

	for (i = 0; i < ARRAY_SIZE(msm_fb_vreg); i++) {
		if (on)
			MSM_FB_VREG_OP(msm_fb_vreg[i], enable);
		else
			MSM_FB_VREG_OP(msm_fb_vreg[i], disable);
	}
	
	if (on)
	{
      msleep_interruptible(TPO_NV_RST_VREG_DELAY);
      gpio_set_value(LCD_RST_N_SIGNAL, 1);
	}
}


static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;
	/* Note: since we're not detecting different panels yet, just return 0 here */
	ret = 0;

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "kgsl_phys_memory",
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
};

#ifdef CONFIG_MACH_PITTSBURGH
static struct platform_device msm_pmic_device_leds = {                                       
   .name   = "pmic-leds",
   .id     = -1,
};
#else
static struct platform_device mot_adp5588_device_leds = {                                       
   .name   = "adp5588-leds",
   .id     = -1,
};
#endif /* CONFIG_MACH_PITTSBURGH */

static struct sfh7743_platform_data sfh7743_data = {
	.name = "sfh7743",
	.gpio_en = -1,
	.vreg_en = "gp6",
	.gpio_intr = SFH7743_GPIO_INTR,
};

static struct platform_device mot_sfh7743_device = {
	.name		= "sfh7743",
	.id		= -2,
	.dev		= {
		.platform_data = &sfh7743_data,
	},
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = "7k_handset",
	},
};


static struct crucial_oj_platform_data mot_crucial_oj_pdata = {
   .name            = CRUCIALTEC_OJ_NAME,
   .gpio_motion_irq = CRUCIALTEC_OJ_MOTION_SIGNAL,
   .gpio_reset      = CRUCIALTEC_OJ_NRST_SIGNAL,
   .gpio_shutdown   = CRUCIALTEC_OJ_SHUTDOWN_SIGNAL,
};

static struct platform_device mot_crucialtec_oj_device = {
	.name    = CRUCIALTEC_OJ_NAME, 
	.id      = 0,
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO("adp8860", ADP8860_I2C_ADDR),
		.platform_data = &mot_adp8860_device         
	},
	{
		I2C_BOARD_INFO("adp8870", ADP8870_I2C_ADDR),
		.platform_data = &mot_adp8870_device         
	},
	{
		I2C_BOARD_INFO("adp5588_keypad", ADP5588_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(QWERTY_INT_N_SIGNAL),
		.platform_data = &mot_adp5588_device
	},
#if defined(CONFIG_TOUCHSCREEN_KEY08)
	{
		I2C_BOARD_INFO("key08", KEY08_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(TOUCH_INT_N),
		.platform_data = &mot_key08_device
	},
#endif /* CONFIG_TOUCHSCREEN_KEY08 */
#if defined(CONFIG_TOUCHSCREEN_QUANTUM_OBP)
	{
		I2C_BOARD_INFO(QTOUCH_TS_NAME, QTOUCH_I2C_ADDR),
		.platform_data = &qtouch_platform_data,
		.irq = MSM_GPIO_TO_INT(TOUCH_INT_N),
	},
#endif /* CONFIG_TOUCHSCREEN_QUANTUM_OBP */
	{
		I2C_BOARD_INFO("lis331dlh", LIS331DLH_I2C_ADDR),		
		.platform_data = &mot_stmicro_device
	},
	{
		I2C_BOARD_INFO("akm8973", AKM8973_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(DIG_COMP_INT_N),
		.platform_data = &mot_akm8973_device
	},
#if defined(CONFIG_CRUCIALTEC_OJ)
	{
		I2C_BOARD_INFO(CRUCIALTEC_OJ_NAME, CRUCIALTEC_OJ_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(CRUCIALTEC_OJ_MOTION_SIGNAL),
		.platform_data = &mot_crucial_oj_pdata
	},
#endif /* CONFIG_CRUCIALTEC_OJ */
};

static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
        GPIO_CFG(0,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* POWER DOWN */
        GPIO_CFG(1,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* RESET */
	GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
   /* parallel CAMERA interfaces */
   /* GPIO_0 and GPIO_1 need to be set to function 0 for the regular GPIO_OUT */
   GPIO_CFG(0,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* POWER DOWN */
   GPIO_CFG(1,  0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* RESET */
   GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
   GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
   GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
   GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
   GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
   GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
   GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
   GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
   GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
   GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
   GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
   GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
   GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
   GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
};

#if defined(CONFIG_INPUT_TOUCHSCREEN)
static void touch_init(void)
{
	gpio_request(TOUCH_RST_N, "touch reset");
	gpio_direction_output(TOUCH_RST_N, 1);

	gpio_request(TOUCH_INT_N, "touch irq");
	gpio_direction_input(TOUCH_INT_N);
}
#endif /* CONFIG_INPUT_TOUCHSCREEN */

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

#define MSM_PROBE_INIT(name) name##_probe_init
static struct msm_camera_sensor_info msm_camera_sensor[] = {
        {
           .sensor_reset = CAM_RST_N,
           .sensor_pwd   = CAM_PWRDN,
           .vcm_pwd      = 0,
           .sensor_name  = "mot_camera",
           .flash_type   = MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_MSM_CAMERA
           .sensor_probe = MSM_PROBE_INIT(mot_camera),
#endif
        },
};
#undef MSM_PROBE_INIT

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.snum = ARRAY_SIZE(msm_camera_sensor),
	.sinfo = &msm_camera_sensor[0],
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static void __init msm_camera_add_device(void)
{
	msm_camera_register_device(NULL, 0, &msm_camera_device_data);
	config_camera_off_gpios();
}

/* 
    Bluetooth stuff 
    ---------------------------------------------------------------- 
 */
#ifdef CONFIG_BT

static struct resource bluesleep_resources[] = {
    {
        .name    = "gpio_host_wake",
        .start    = BT_HOST_WAKE_SIGNAL,
        .end    = BT_HOST_WAKE_SIGNAL,
        .flags    = IORESOURCE_IO,
    },
    {
        .name    = "gpio_ext_wake",
        .start    = BT_EXT_WAKE_SIGNAL,
        .end    = BT_EXT_WAKE_SIGNAL,
        .flags    = IORESOURCE_IO,
    },
    {
        .name    = "host_wake",
        .start    = MSM_GPIO_TO_INT(BT_HOST_WAKE_SIGNAL),
        .end    = MSM_GPIO_TO_INT(BT_HOST_WAKE_SIGNAL),
        .flags    = IORESOURCE_IRQ,
    },
};

static struct platform_device msm_bluesleep_device = {
    .name = "bluesleep",
    .id        = -1,
    .num_resources    = ARRAY_SIZE(bluesleep_resources),
    .resource    = bluesleep_resources,
};

#else
#define bt_power_init(x) do {} while (0)
#endif /* CONFIG_BT */

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
    .voltage_min_design     = 3200,
    .voltage_max_design = 4200,
    .avail_chg_sources      = AC_CHG | USB_CHG ,
    .batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
    .calculate_capacity = &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
    u32 current_percent = 0;
    u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
    u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

    current_percent = ( (current_voltage - low_voltage) * 100
        / (high_voltage - low_voltage) );

    if(current_percent > 100)
    {
       current_percent =100;
    }

   return current_percent;
}

static struct platform_device msm_batt_device = {
    .name           = "msm-battery",
    .id         = -1,
    .dev.platform_data  = &msm_psy_batt_data,
};




static struct platform_device *devices[] __initdata = {
	&msm_device_uart1,
#if defined(CONFIG_SERIAL_MSM_CONSOLE)
	&msm_device_uart3,
#endif
	&msm_device_smd,
	&msm_device_nand,
	&msm_device_hsusb_otg,
	&msm_device_hsusb_host,
#if defined(CONFIG_USB_FUNCTION) || defined(CONFIG_USB_ANDROID)
	&msm_device_hsusb_peripheral,
#endif
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#endif
	&msm_device_i2c,
	&msm_device_tssc,
	&android_pmem_kernel_ebi1_device,
	&android_pmem_device,
	&android_pmem_camera_device,
	&android_pmem_adsp_device,
	&android_pmem_gpu1_device, 
	&msm_fb_device,
#ifdef CONFIG_MACH_PITTSBURGH
        &msm_pmic_device_leds,          /* Keypad backlight */
#else
	&mot_adp5588_device_leds,	/* Keypad backlight */
#endif CONFIG_MACH_PITTSBURGH	
	&mot_adp8860_device,
   &mot_crucialtec_oj_device,	/* Optical Finger Navigation */
	&mot_sfh7743_device,	/* Proximity sensor */
#ifndef CONFIG_MACH_PITTSBURGH
	&mot_vibrator,			/* Vibrator */
#endif /* CONFIG_MACH_PITTSBURGH */
	&msm_device_uart_dm1,
	&msm_device_snd,
#ifdef CONFIG_BT
	&msm_bluesleep_device,
#endif
	&msm_device_adspdec,
	&msm_device_kgsl,
	&hs_device,
	&msm_batt_device,
};



int get_factory_cable_status(void)
{
	return gpio_get_value(EMU_ID_PROT);
}


static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = LCD_HW_VSYNC,
};

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("ebi2", 0);
	msm_fb_register_device("pmdh", &mddi_pdata);
}

extern struct sys_timer msm_timer;

static void __init mot_7x27_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data mot_7x27_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
	.max_axi_khz = 128000,
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

static void sdcc_gpio_init(void)
{
    int rc = 0;

    if (gpio_request(SD_DETECT_N_SIGNAL, "sdc1_status_irq"))
        pr_err("failed to request gpio sdc1_status_irq\n");
    rc = gpio_tlmm_config(GPIO_CFG(SD_DETECT_N_SIGNAL, 0, GPIO_INPUT, GPIO_NO_PULL,
                GPIO_2MA), GPIO_ENABLE);
    if (rc)
        printk(KERN_ERR "%s: Failed to configure GPIO %d\n",
            __func__, rc);
			
	/* SDC1 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (gpio_request(51, "sdc1_data_3"))
		pr_err("failed to request gpio sdc1_data_3\n");
	if (gpio_request(52, "sdc1_data_2"))
		pr_err("failed to request gpio sdc1_data_2\n");
	if (gpio_request(53, "sdc1_data_1"))
		pr_err("failed to request gpio sdc1_data_1\n");
	if (gpio_request(54, "sdc1_data_0"))
		pr_err("failed to request gpio sdc1_data_0\n");
	if (gpio_request(55, "sdc1_cmd"))
		pr_err("failed to request gpio sdc1_cmd\n");
	if (gpio_request(56, "sdc1_clk"))
		pr_err("failed to request gpio sdc1_clk\n");
#endif

	/* SDC2 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (gpio_request(62, "sdc2_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(63, "sdc2_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(64, "sdc2_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(65, "sdc2_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(66, "sdc2_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(67, "sdc2_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
	if (gpio_request(WLAN_HOST_IRQ, "host_wlan_irq "))
		pr_err("failed to request gpio host_wlan_irq\n");		
#endif
}

static unsigned sdcc_cfg_data[][7] = {
    /* SDCC_CMD should be set to 8mA (7627 GPIO 55 and 63)*/
	/* SDC1 configs */
	{
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* 7th item in array */
	},
	/* SDC2 configs */
	{
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
	GPIO_CFG(WLAN_HOST_IRQ , 0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),    /* WLAN_HOST_IRQ */
	},
};

static unsigned long vreg_sdcc1_sts, vreg_sdcc2_sts, gpio_sts;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int i, rc;

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable)
		set_bit(dev_id, &gpio_sts);
	else
		clear_bit(dev_id, &gpio_sts);

	for (i = 0; i < ARRAY_SIZE(sdcc_cfg_data[dev_id - 1]); i++) {
		rc = gpio_tlmm_config(sdcc_cfg_data[dev_id - 1][i],
			enable ? GPIO_ENABLE : GPIO_DISABLE);
		if (rc)
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, sdcc_cfg_data[dev_id - 1][i], rc);
	}
}

uint32_t msm_sdcc1_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;
	struct vreg *vreg;

	pdev = container_of(dv, struct platform_device, dev);

	if (pdev->id == 1) //SD
		vreg = vreg_get(0, "wlan");
	else
	{
		printk(KERN_ERR "%s: unsupported sdcc port (%d) selected!\n", __func__, pdev->id);
		return -1;
	}

	if (vdd == 0) {
        /* Reconfigure GPIO's before disabling VREG */
        msm_sdcc_setup_gpio(pdev->id, !!vdd);

		if (!vreg_sdcc1_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sdcc1_sts);

		if (!vreg_sdcc1_sts) {
			if ((rc = vreg_disable(vreg)))
				printk(KERN_ERR "%s: vreg(wlan) for MMC disable failed (%d)\n", __func__, rc);
            msleep(7); // SanDisk requirement to allow VDD to get to 0.3v for 1 msec
		}
		return 0;
	}

	if (!vreg_sdcc1_sts) {
	    rc = vreg_set_level(vreg, 2850);
		if (rc)
 	   {
 	       printk(KERN_ERR "%s: vreg set level failed (%d)\n",
			       __func__, rc);
			return -1;
 	   }

		if ((rc = vreg_enable(vreg)))
			printk(KERN_ERR "%s: vreg(wlan) for MMC enable failed (%d)\n", __func__, rc);
	}
	set_bit(pdev->id, &vreg_sdcc1_sts);
    /* Reconfigure GPIO's only after enabling VREG */
    msm_sdcc_setup_gpio(pdev->id, !!vdd);

	return 0;
}

// WiFi static items
static unsigned int wlan_vreg_enabled = 0;
static void (*wlan_status_notify)(int card_present, void* dev_id) = NULL;
static unsigned long *wlan_host_ptr = NULL;


uint32_t msm_sdcc2_setup_power(struct device *dv, unsigned int vdd)
{
        int rc = 0;
        //struct platform_device *pdev;
        struct vreg *vreg;

        //pdev = container_of(dv, struct platform_device, dev);
        //msm_sdcc_setup_gpio(pdev->id, !!vdd);
        msm_sdcc_setup_gpio(2, !!vdd);

        if ((vdd && wlan_vreg_enabled) || (!vdd && !wlan_vreg_enabled))
                return 0;

        if (vdd == 0) {
                if (!vreg_sdcc2_sts)
                        return 0;

                //clear_bit(pdev->id, &vreg_sdcc2_sts);
                clear_bit(2, &vreg_sdcc2_sts);

                if (!vreg_sdcc2_sts) {
                        // WiFi DEBUG INFO:
                        printk(KERN_ERR "msm_sdcc_setup_power - DISABLING...WIFI\n");
                        gpio_request(WLAN_RST_N, "wlan_reset_n"); // put the WLAN in reset
                        gpio_direction_output(WLAN_RST_N, 0);
                        msleep_interruptible(100);              // do we really need this much?
                        gpio_request(WLAN_REG_ON_SIGNAL, "wlan_reg_on"); // turn the WLAN internal VREG off
                        gpio_direction_output(WLAN_REG_ON_SIGNAL, 0);
                        msleep_interruptible(100);  // do we really need this much?
                        wlan_vreg_enabled = 0;
                }
                return 0;
        }

        if (!vreg_sdcc2_sts) {
                // WiFi DEBUG INFO:
                printk(KERN_ERR "msm_sdcc_setup_power - ENABLING...WIFI\n");
                gpio_request(WLAN_REG_ON_SIGNAL, "wlan_reg_on"); // turn on WLAN internal VREG
                gpio_direction_output(WLAN_REG_ON_SIGNAL, 1);
                msleep_interruptible(100);              // BCM4325 powerup requirement
                gpio_request(WLAN_RST_N, "wlan_reset_n"); // take WLAN out of reset
                gpio_direction_output(WLAN_RST_N, 1);
                msleep_interruptible(100);              // BCM4325 powerup requirement
                wlan_vreg_enabled = 1;
        }
        //set_bit(pdev->id, &vreg_sdcc2_sts);
        set_bit(2, &vreg_sdcc2_sts);
        return 0;
}



#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int mot_7x27_sdcc_slot1_status(struct device *dev)
{
	unsigned val;
	val = gpio_get_value(SD_DETECT_N_SIGNAL);
	return (1 - val);
}

static unsigned int mot_7x27_sdcc_slot2_status(struct device *dev)
{
	/*
	 * This is hooked up to BT/WIFI chip. 
	 */
	return wlan_vreg_enabled;
}
#endif

// WiFi 
static int mot_sdcc_wifi_status_notify_register( void (*notify_callback)(int card_present, void *dev_id), void *dev)
{
    if((NULL != notify_callback) && (NULL == wlan_status_notify)) {
        wlan_status_notify = notify_callback;
        wlan_host_ptr = dev;
             printk ("%s:  status_notify callback registered dev_ptr %p\n", __FUNCTION__, dev);
    } else {
             printk ("%s:  status_notify callback registration skipped dev_ptr %p\n", __FUNCTION__, dev);
    }
    return 0;
}

static struct mmc_platform_data mot_7x27_sdcc1_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc1_setup_power,
};

static struct mmc_platform_data mot_7x27_sdcc2_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc2_setup_power,
};

static void __init mot_7x27_init_mmc(void)
{
    unsigned on_off = 1;  /* Turn on pulldown when VREG is disabled */
    unsigned id = 15; /*vreg_mmc->id*/;
	msm_add_sdcc(1, &mot_7x27_sdcc1_data, 0, 0); // SD slot

    /* Setup VREG to pulldown when off */
    msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

    set_irq_wake(MSM_GPIO_TO_INT(SD_DETECT_N_SIGNAL), 1); 
}

static void __init mot_7x27_init_wlan(void)
{
	msm_add_sdcc(2, &mot_7x27_sdcc2_data, 0, 0); // BRCM 4325
}

/* WLAN POWER MGMT - EXPORTED FUNCTIONS - e11370 */
void bcm_wlan_power_off(unsigned power_mode)
{
     struct device *dv;

     printk ("%s: power_mode %d\n", __FUNCTION__, power_mode);
     
     switch(power_mode){
	case 1:
		/* Unload driver */
		msm_sdcc2_setup_power(dv, 0);
		wlan_status_notify(0, wlan_host_ptr);
		msleep_interruptible(100);  // do we really need this much?
		break;
	case 2:
		/* Stop driver */
		msm_sdcc2_setup_power(dv, 0);
		break;
	default:
		printk ("%s: ERROR unsupported power_mode %d\n", __FUNCTION__, power_mode);
		break;
     }
}
EXPORT_SYMBOL(bcm_wlan_power_off);

void bcm_wlan_power_on(unsigned power_mode)
{
     struct device *dv;

     printk ("%s: power_mode %d\n", __FUNCTION__, power_mode);
     
     switch(power_mode){
        case 1:
        /* Load driver */
		msm_sdcc2_setup_power(dv, 1);
		wlan_status_notify(1, wlan_host_ptr);
                msleep_interruptible(100);
                break;
        case 2:
        /* Start driver */
                msm_sdcc2_setup_power(dv, 1);
		break;
        default:
                printk ("%s: ERROR unsupported power_mode %d\n", __FUNCTION__, power_mode);
                break;
        }
}
EXPORT_SYMBOL(bcm_wlan_power_on);
/* END  WLAN POWER MGMT */


static ssize_t mot_hw_type_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    return sprintf(buf, "0x%04X\n", mot_handover_get_upid());
}

static ssize_t mot_hw_type_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    return -EINVAL;
}

int mot_get_hw_type(void)
{
    return mot_handover_get_upid();
}

#define mot_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0664,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

#define mot_ro_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0444,			\
	},					\
	.show	= _name##_show,			\
	.store	= NULL,		\
}

mot_attr(mot_hw_type);

static struct attribute * g[] = {
	&mot_hw_type_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#ifdef CONFIG_PROC_FS
int hwrev_readproc(char* page, char** start, off_t offset, int count, int *eof, void* data)
{
	char *p = page;
	int len;

	p += sprintf(page, "0x%04X\n", mot_handover_get_upid());

	len = (p - page) - offset;
	if (len < 0)
		len = 0;

	*eof = (len <= count) ? 1 : 0;
	*start = page + offset;

	return len;
}
#endif

static void msm_i2c_gpio_config(int iface, int config_type)
{
	if (iface) {
		/* Note: Mot phones don't use GPIO[95,96] for AUX_I2C */
	} else {
		if (config_type) {
			/* Configure GPIO[I2C_SCL,I2C_SDA] for I2C */
			gpio_tlmm_config(GPIO_CFG(I2C_SCL, 1, GPIO_OUTPUT,
						GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
			gpio_tlmm_config(GPIO_CFG(I2C_SDA, 1, GPIO_OUTPUT,
						GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
		} else {
			gpio_tlmm_config(GPIO_CFG(I2C_SCL, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
			gpio_tlmm_config(GPIO_CFG(I2C_SDA, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		}
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.msm_i2c_config_gpio = msm_i2c_gpio_config
};

static void __init msm_device_i2c_init(void)
{
	/* Note: Mot phones don't use GPIO[95,96] for AUX_I2C */
	if (gpio_request(I2C_SCL, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(I2C_SDA, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};

static void __init mot_7x27_init(void)
{
    int ret = 0;

	if (socinfo_init() < 0)
		BUG();

	mot_kobj = kobject_create_and_add("mothw", NULL);
	if (mot_kobj == NULL) {
		printk("mot_7x27_init: subsystem_register failed\n");
		ret = -ENOMEM;
		goto err2;
	}
	ret = sysfs_create_group(mot_kobj, &attr_group);
	if(ret) {
		printk("mot_7x27_init: sysfs_create_group failed\n");
		goto err1;
	}
#ifdef CONFIG_PROC_FS	
	create_proc_read_entry("mot_hw", 0, NULL, hwrev_readproc, NULL);
#endif
#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			&msm_device_uart3.dev, 1);
#endif

	if (cpu_is_msm7x27())
		mot_7x27_clock_data.max_axi_khz = 200000;

	msm_acpu_clock_init(&mot_7x27_clock_data);
#ifdef TEMP_76XX_ANDROID	
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif /*TEMP_76XX_ANDROID*/

#if defined(CONFIG_USB_FUNCTION)
	/*
	 * If we powered-up with a Factory Mode cable attached,
	 * override the default USB configuration.
	 */
	if ( mot_handover_chk_powerup_reason(PU_MAIN_EXTERNAL_PWR_PU) )
	{
		msm_hsusb_pdata.product_id = MOT_PID9;
	}
#endif /* defined(CONFIG_USB_FUNCTION) */

	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
	msm_device_hsusb_host.dev.platform_data = &msm_hsusb_pdata;
    /* Note, gp3 is SLIDE and it's already enabled,
             gp5 is used for the QFUSE,
             gp6 will be used for the proximity sensor on P1 */
	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_camera_add_device();
#ifdef CONFIG_MACH_PITTSBURGH
	msm_init_pmic_vibrator();
#endif /*CONFIG_MACH_PITTSBURGH*/	
#if defined(CONFIG_INPUT_TOUCHSCREEN)
	touch_init();
#endif /* CONFIG_INPUT_TOUCHSCREEN */
	msm_device_i2c_init();
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
    
	gpio_request(LCD_RST_N_SIGNAL, "lcd_rst_n");
	gpio_set_value(LCD_RST_N_SIGNAL, 0);
	msleep_interruptible(100);
	gpio_set_value(LCD_RST_N_SIGNAL, 1);
	gpio_request(QWRTY_RST_N, "qwerty_rst_n");
	gpio_direction_output(QWRTY_RST_N, 1);

	gpio_request(DIG_COMP_RST_N, "ecompass_rst_n");	
	gpio_direction_output(DIG_COMP_RST_N, 1);

	msm_fb_add_devices();
	sdcc_gpio_init();
	mot_7x27_init_mmc();
	mot_7x27_init_wlan();
	msm_pm_set_platform_data(msm7x27_pm_data);

	return;
err1:
	kobject_del(mot_kobj);
err2:
	return;
}

static void __init msm_mot_7x27_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = PMEM_KERNEL_EBI1_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	android_pmem_kernel_ebi1_pdata.start = __pa(addr);
	android_pmem_kernel_ebi1_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for pmem kernel ebi1 arena\n", size, addr, __pa(addr));

	size = MSM_PMEM_MDP_SIZE;
	addr = alloc_bootmem(size);
	android_pmem_pdata.start = __pa(addr);
	android_pmem_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for pmem\n", size, addr, __pa(addr));

	size = MSM_PMEM_CAMERA_SIZE;
	addr = alloc_bootmem(size);
	android_pmem_camera_pdata.start = __pa(addr);
	android_pmem_camera_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for camera pmem\n", size, addr, __pa(addr));

	size = MSM_PMEM_ADSP_SIZE;
	addr = alloc_bootmem(size);
	android_pmem_adsp_pdata.start = __pa(addr);
	android_pmem_adsp_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for adsp pmem\n", size, addr, __pa(addr));

	size = MSM_PMEM_GPU1_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	android_pmem_gpu1_pdata.start = __pa(addr);
	android_pmem_gpu1_pdata.size = size;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
	       "for gpu1 pmem\n", size, addr, __pa(addr));

	size = MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = MSM_GPU_PHYS_SIZE;
	addr = alloc_bootmem_aligned(size, 0x100000);
	kgsl_resources[1].start = __pa(addr);
	kgsl_resources[1].end = kgsl_resources[1].start + size - 1;
	printk(KERN_INFO "allocating %lu bytes at %p (%lx physical)"
		"for KGSL pmem\n", size, addr, __pa(addr));
}

static void __init mot_7x27_map_io(void)
{
	msm_map_common_io();
	msm_clock_init(msm_clocks_7x27, msm_num_clocks_7x27);
	msm_mot_7x27_allocate_memory_regions();

#ifdef CONFIG_CACHE_L2X0
		/* 7x27 has 256KB L2 cache:
			64Kb/Way and 4-Way Associativity;
			R/W latency: 3 cycles;
			evmon/parity/share disabled. */
		l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
#endif
}

static void __init mot_7x27_fixup(struct machine_desc *desc, struct tag *tags,
                                 char **cmdline, struct meminfo *mi)
{
    struct tag *t;

    for (t=tags; t->hdr.size; t = tag_next(t))
    {
        if (t->hdr.tag == ATAG_CMDLINE)
        {
            printk("%s: cmdline=\"%s\"\n", __func__, t->u.cmdline.cmdline);
        }
        if (t->hdr.tag == ATAG_REVISION)
        {
            printk("%s: hw_rev=%x\n", __func__, t->u.revision.rev);
        }
        if (t->hdr.tag == ATAG_MEM)
        {
            printk("%s: mem_start=%d, mem_size=%d\n", __func__, t->u.mem.start, t->u.mem.size);
        }
        if (t->hdr.tag == ATAG_SERIAL)
        {
            calg_hw_serial = t->u.serialnr.high;
            calg_hw_serial = (calg_hw_serial << 32) | t->u.serialnr.low;
#ifdef CONFIG_USB_FUNCTION
            sprintf(msm_hsusb_pdata.serial_number, "%llx", calg_hw_serial);
#endif
#ifdef CONFIG_USB_ANDROID
            sprintf(android_usb_pdata.serial_number, "%llx", calg_hw_serial);
#endif
            printk("%s: calg_hw_serial=%llx\n", __func__, calg_hw_serial);
        }
    }
    if (mi)
    {
        mi->nr_banks = 1;
        mi->bank[0].start = PHYS_OFFSET;
        mi->bank[0].node  = PHYS_TO_NID(PHYS_OFFSET);
        mi->bank[0].size  = SCL_APPS_TOTAL_SIZE + SCL_MM_HEAP2_SIZE;
        printk("%s: mi->nr_banks=%x\n", __func__, mi->nr_banks);
        printk("%s: mi->bank[0].start=%lx\n", __func__, mi->bank[0].start);
        printk("%s: mi->bank[0].node=%x\n", __func__, mi->bank[0].node);
        printk("%s: mi->bank[0].size=%lx\n", __func__, mi->bank[0].size);
    }
    else
    {
           printk("%s: mi is NULL\n", __func__);
    }
}

MACHINE_START(CALGARY, MOT_HW_REV)
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.fixup      = mot_7x27_fixup,
	.boot_params	= 0x00200100,
	.map_io		= mot_7x27_map_io,
	.init_irq	= mot_7x27_init_irq,
	.init_machine	= mot_7x27_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(PITTSBURGH, MOT_HW_REV)
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.fixup      = mot_7x27_fixup,
	.boot_params	= 0x00200100,
	.map_io		= mot_7x27_map_io,
	.init_irq	= mot_7x27_init_irq,
	.init_machine	= mot_7x27_init,
	.timer		= &msm_timer,
MACHINE_END
