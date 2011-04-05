/* Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9p012_vcm.h"
#include <linux/wakelock.h>
#ifdef CONFIG_MACH_MOT
#include <asm/mach-types.h>
#include <mach/vreg.h>
#endif
#include <linux/sched.h>
#include <asm/param.h>
#include <linux/pm_qos_params.h>

/*=============================================================
    SENSOR REGISTER DEFINES
==============================================================*/
#define MT9P012_VCM_REG_MODEL_ID         0x0000
#define MT9P012_VCM_MODEL_ID             0x2801
#define MT9P012_VCM_MODEL_ID_5131        0x2803
#define REG_GROUPED_PARAMETER_HOLD   0x0104
#define GROUPED_PARAMETER_HOLD       0x0100
#define GROUPED_PARAMETER_UPDATE     0x0000
#define REG_COARSE_INT_TIME          0x3012
#define REG_VT_PIX_CLK_DIV           0x0300
#define REG_VT_SYS_CLK_DIV           0x0302
#define REG_PRE_PLL_CLK_DIV          0x0304
#define REG_PLL_MULTIPLIER           0x0306
#define REG_OP_PIX_CLK_DIV           0x0308
#define REG_OP_SYS_CLK_DIV           0x030A
#define REG_SCALE_M                  0x0404
#define REG_FRAME_LENGTH_LINES       0x300A
#define REG_LINE_LENGTH_PCK          0x300C
#define REG_X_ADDR_START             0x3004
#define REG_Y_ADDR_START             0x3002
#define REG_X_ADDR_END               0x3008
#define REG_Y_ADDR_END               0x3006
#define REG_X_OUTPUT_SIZE            0x034C
#define REG_Y_OUTPUT_SIZE            0x034E
#define REG_FINE_INTEGRATION_TIME    0x3014
#define REG_ROW_SPEED                0x3016
#define REG_SKEW                     0x309E
#define MT9P012_VCM_REG_RESET_REGISTER   0x301A
#define MT9P012_VCM_RESET_REGISTER_PWON  0x10CC
#define MT9P012_VCM_RESET_REGISTER_PWOFF 0x10C8
#define REG_READ_MODE                0x3040
#define REG_GLOBAL_GAIN              0x305E
#define REG_TEST_PATTERN_MODE        0x3070

#define MT9P012_VCM_REV_7
#define MT9P012_VCM_5131

#ifndef DIFF
#define DIFF(a,b) ( ((a) > (b)) ? ((a) - (b)) : ((b) - (a)) )
#endif

#define MOT_HW
#define MOT_SUPPRESS_READ_ERROR_MSGS
//#define VPT_AF

#define CAMERA_CONFIG_ATTEMPTS 5

enum mt9p012_vcm_test_mode_t {
	TEST_OFF,
	TEST_1,
	TEST_2,
	TEST_3,
	TEST_W1_12B = 256,
	TEST_W1_10B,
	TEST_W1_8B 
};

enum mt9p012_vcm_resolution_t {
	QTR_SIZE,
	FULL_SIZE,
	INVALID_SIZE
};

enum mt9p012_vcm_reg_update_t {
	/* Sensor egisters that need to be updated during initialization */
	REG_INIT,
	/* Sensor egisters that needs periodic I2C writes */
	UPDATE_PERIODIC,
	/* All the sensor Registers will be updated */
	UPDATE_ALL,
	/* Not valid update */
	UPDATE_INVALID
};

enum mt9p012_vcm_setting_t {
	RES_PREVIEW,
	RES_CAPTURE
};


#define VCM_STEP_POSITION_TABLE_CONSTRUCT 1 // Using this Macro to use Motorola's logic to populate the VCM AF lookup table. Uses the calibrated macro.
//#define VCM_STEP_POSITION_TABLE_CONSTRUCT 2 // Using this Macro to use QCOM logic to populate the VCM AF lookup table. DO NOT use calibrated macro.


/* AF Total steps parameters */
#define MT9P012_VCM_STEPS_NEAR_TO_CLOSEST_INF  40
#define MT9P012_VCM_TOTAL_STEPS_NEAR_TO_FAR    40


#define MT9P012_VCM_MU5M0_PREVIEW_DUMMY_PIXELS 0
#define MT9P012_VCM_MU5M0_PREVIEW_DUMMY_LINES  0

/* Time in milisecs for waiting for the sensor to reset.*/
#define MT9P012_VCM_RESET_DELAY_MSECS   66

/* for 20 fps preview */
#ifdef NEW_MCLK_PCLK
  #define MT9P012_VCM_DEFAULT_CLOCK_RATE  48000000
#else
  #define MT9P012_VCM_DEFAULT_CLOCK_RATE  24000000
#endif

#define MT9P012_VCM_DEFAULT_MAX_FPS     26 /* ???? */

#define S_WAIT(n) \
	mdelay(n)

struct mt9p012_vcm_work_t {
	struct work_struct work;
};
static struct mt9p012_vcm_work_t *mt9p012_vcm_sensorw;
static struct i2c_client *mt9p012_vcm_client;

struct mt9p012_vcm_ctrl_t {
	struct  msm_camera_sensor_info *sensordata;

	int sensormode;
	uint32_t fps_divider; /* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider; /* init to 1 * 0x00000400 */

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t init_curr_lens_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;

	enum mt9p012_vcm_resolution_t prev_res;
	enum mt9p012_vcm_resolution_t pict_res;
	enum mt9p012_vcm_resolution_t curr_res;
	enum mt9p012_vcm_test_mode_t  set_test;
};

#ifdef MOT_HW

// EEPROM DATA
#define EEPROM_BLOCK_SZ 4096
#define EEPROM_SIZE (16*1024)

typedef enum
{
    EEP_DATA_UNKNOWN,
    EEP_DATA_READ,
    EEP_DATA_VALID,
    EEP_DATA_INVALID
} eep_data_state;

static int8_t awb_decision = WB_FLORESCENT;
static module_data_t module_data;
static eep_data_state eep_state = EEP_DATA_UNKNOWN;
static uint8_t * pEpromImage = NULL;
static bool load_snapshot_lsc = false;

// AUTO FOCUS
static int32_t vcm_af_calculate_target_position(int16_t step_position, int32_t num_steps);

#endif /* MOT_HW */

extern uint8_t disable_lens_move;

static char af_addr = 0;

#define MT9P012_VCM_AF_ADDR_BAM1  0x05 // SEMCO 5MP Sensor with OLD AF module - Bit Bang.
#define MT9P012_VCM_AF_ADDR_BAM2  0x45 // SEMCO 5MP Sensor with new BAM module
#define MT9P012_VCM_AF_ADDR_VCM   (0x0C >> 1) //VPT 5MP Sensor with VCM module. 
//#define MT9P012_VCM_AF_I2C_ADDR  0x18 // 7 bit address for 0x0C; //

static struct wake_lock mt9p012_vcm_wake_lock;
static uint16_t update_type = UPDATE_PERIODIC;

static uint16_t step_position_table[MT9P012_VCM_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t debug_MT9P012_VCM_MU5M0_TOTAL_STEPS_NEAR_TO_FAR = 40;

uint16_t ee_addr = 0;
static uint16_t calibrated_macro, calibrated_liftoff_curr;
static uint16_t adjusted_macro, adjusted_liftoff_curr ; 

#ifndef MOT_HW
static uint8_t ic_cont_1 = 0x34;
static uint8_t ic_cont_7 = 0x44;
#endif
static uint16_t mt9p012_vcm_current_lens_position;



static struct mt9p012_vcm_ctrl_t *mt9p012_vcm_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9p012_vcm_wait_queue);
DECLARE_MUTEX(mt9p012_vcm_sem);

/*=============================================================*/

static int mt9p012_vcm_i2c_rxdata(unsigned short saddr, int slength,
			      unsigned char *rxdata, int rxlength, bool polling)
{
  struct i2c_msg msgs[] = {
	{   .addr   = saddr << 1, // Platform device is registered with  7 bit addr >> 2 . 
		.flags = 0 | (polling ? I2C_M_NAK_LIKELY : 0),
		.len   = slength,
		.buf   = rxdata,
	},
	{   .addr   = saddr << 1, // Platform device is registered with  7 bit addr >> 2 . 
		.flags = I2C_M_RD | (polling ? I2C_M_NAK_LIKELY : 0),
		.len   = rxlength,
		.buf   = rxdata,
	},
    };
    int rc = 0;
    int attempt = 0;

    do {
#ifndef MOT_SUPPRESS_READ_ERROR_MSGS
        if (rc < 0)
            CDBG("mt9p012_vcm: mt9p012_vcm_i2c_rxdata: ATTEMPT %d FAILED: rc=%d\n", attempt, rc);
#endif
        attempt++;

	    rc = i2c_transfer(mt9p012_vcm_client->adapter, msgs, 2);
	} while (!polling && rc < 0 && attempt < CAMERA_CONFIG_ATTEMPTS);

    if (rc < 0) {
#ifndef MOT_SUPPRESS_READ_ERROR_MSGS
	    CDBG("mt9p012_vcm_i2c_rxdata failed after %d attempts!\n", attempt);
#endif
	    return -EIO;
    }
	return 0;
}

static int32_t mt9p012_vcm_i2c_do_read_b(unsigned short saddr, uint8_t raddr,
	uint8_t *rdata, bool polling)
{
    int32_t rc = 0;
    unsigned char buf[2];

    if (!rdata)
        return -EIO;

    buf[0] = raddr;

    rc = mt9p012_vcm_i2c_rxdata(saddr, 1, buf, 1, polling);
    if (rc < 0) {
        if (!polling)
            CDBG("%s: failed! saddr = 0x%x, raddr = %d\n",
                    __func__, saddr, raddr);

        return rc;
    }

    *rdata = buf[0];

    return rc;
}


static int32_t mt9p012_vcm_i2c_poll_b(unsigned short saddr, uint8_t raddr,
    uint8_t *rdata)
{
    return mt9p012_vcm_i2c_do_read_b(saddr, raddr, rdata, true);
}

static int32_t mt9p012_vcm_i2c_read_b(unsigned short saddr, uint8_t raddr,
	uint8_t *rdata)
{
    return mt9p012_vcm_i2c_do_read_b(saddr, raddr, rdata, false);
}

static int32_t mt9p012_vcm_i2c_read_w(unsigned short saddr, unsigned short raddr,
	unsigned short *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];

	if (!rdata)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = mt9p012_vcm_i2c_rxdata(saddr, 2, buf, 2, false);
	if (rc < 0)
		return rc;

	*rdata = buf[0] << 8 | buf[1];

	if (rc < 0)
		CDBG("mt9p012_vcm_i2c_read_w failed!\n");

	return rc;
}

static int32_t mt9p012_vcm_i2c_txdata(unsigned short saddr,	unsigned char *txdata,
	int length)
{
	struct i2c_msg msg[] = {
		{
		.addr  = saddr << 1, // Platform device is registered with  7 bit addr >> 2 .
		.flags = 0,
		.len = length,
		.buf = txdata,
		},
	};
    int rc = 0;
    int attempt = 0;
	
    do {
        if (rc < 0)
            CDBG("mt9p012_vcm: mt9p012_vcm_i2c_txdata: ATTEMPT %d FAILED: rc=%d\n", attempt, rc);
        attempt++;

        rc = i2c_transfer(mt9p012_vcm_client->adapter, msg, 1);
	} while (rc < 0 && attempt < CAMERA_CONFIG_ATTEMPTS);

    if (rc < 0) {
        CDBG("mt9p012_vcm_i2c_txdata failed after %d attempts\n", attempt);
		return -EIO;
    }
    
	return 0;
}

static int32_t mt9p012_vcm_i2c_write_b(unsigned short saddr, unsigned short baddr,
	unsigned short bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	//CDBG("%s: IN \n", __func__);	
    memset(buf, 0, sizeof(buf));
    //CDBG("%s:baddr=%d bdata=%d\n", __func__, baddr, bdata);
	buf[0] = baddr;
	buf[1] = bdata;
	rc = mt9p012_vcm_i2c_txdata(saddr, buf, 2);

	if (rc < 0)
		CDBG("i2c_write failed, saddr = 0x%x addr = 0x%x, val =0x%x!\n",
		saddr, baddr, bdata);

	//CDBG("%s: OUT \n", __func__);
	return rc;
}

static int32_t mt9p012_vcm_i2c_write_w(unsigned short saddr, unsigned short waddr,
	unsigned short wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];

	memset(buf, 0, sizeof(buf));
    //CDBG("%s:waddr=0x%x wdata=0x%x\n", __func__, waddr, wdata);
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00)>>8;
	buf[3] = (wdata & 0x00FF);

	rc = mt9p012_vcm_i2c_txdata(saddr, buf, 4);

	if (rc < 0)
		CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);

	return rc;
}

static int32_t mt9p012_vcm_i2c_write_w_table(
	struct mt9p012_vcm_i2c_reg_conf const *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EFAULT;
        CVBS("%s Reg address : %d, Reg Value: %d \n", __func__, reg_conf_tbl->waddr, reg_conf_tbl->wdata);
	for (i = 0; i < num; i++) {
		rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			reg_conf_tbl->waddr, reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}

	return rc;
}

static bool axi_qos_requested = false;

static int request_axi_qos(void)
{
    CVBS("%s\n",__func__);
    if (! axi_qos_requested) {
        if ( pm_qos_add_requirement(PM_QOS_SYSTEM_BUS_FREQ, "msm_camera", 160000 ) < 0 ) {
            printk(KERN_ERR "unable to request AXI bus QOS\n");
            return -1;
        }
        else {
            CVBS("%s: request successful\n", __func__);
            axi_qos_requested = true;
            mdelay(5);
        }
    }
    return 0;
}

static int release_axi_qos(void)
{
    CVBS("%s\n",__func__);
    if ( axi_qos_requested ) {
        pm_qos_remove_requirement(PM_QOS_SYSTEM_BUS_FREQ, "msm_camera");
        CVBS("%s: release successful\n", __func__);
        axi_qos_requested = false;
        mdelay(5);
    }
    return 0;
}


static int32_t mt9p012_vcm_test(enum mt9p012_vcm_test_mode_t mo)
{
	int32_t rc = 0;

	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
		REG_GROUPED_PARAMETER_HOLD,
		GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	if (mo == TEST_OFF)
		return 0;
	else {
		rc = mt9p012_vcm_i2c_write_w_table(mt9p012_vcm_regs.ttbl,
					       mt9p012_vcm_regs.ttbl_size);
		if (rc < 0)
			return rc;

		rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_TEST_PATTERN_MODE, (uint16_t)mo);
		if (rc < 0)
			return rc;
	}

	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
		REG_GROUPED_PARAMETER_HOLD,
		GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		return rc;

	return rc;
}

static int32_t mt9p012_vcm_lens_shading_enable(uint8_t is_enable)
{
	int32_t rc = 0;

	CDBG("%s: entered. enable = %d\n", __func__, is_enable);

	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
		REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return rc;

	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr, 0x3780,
		((uint16_t) is_enable) << 15);
	if (rc < 0)
		return rc;

	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
		REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);

	CDBG("%s: exiting. rc = %d\n", __func__, rc);

	return rc;
}

static int32_t mt9p012_vcm_set_lc(void)
{
    int32_t rc;
#ifdef MOT_HW
    uint32_t index, i;

    if (eep_state == EEP_DATA_VALID)
    {
        rc = -EFAULT;

        CDBG("mt9p012_vcm lsc_count = %d\n", module_data.lsc_count);

	if (module_data.lsc_count == 1)
	{
	  index = 0;
	}
        else if (load_snapshot_lsc == true) 
        {
          index = awb_decision*LSC_DATA_ENTRIES;
          load_snapshot_lsc = false;
        }
	else
        {
	  index = LSC_DATA_ENTRIES;
	}

        CDBG("Starting LSC with AWB %d\n", index/LSC_DATA_ENTRIES);

        for (i = 0; i < LSC_DATA_ENTRIES; i++) {
            rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
                    module_data.lsc[i+index].lscAddr, 
                    module_data.lsc[i+index].lscVal);
            if (rc < 0)
            {
                CDBG ("LSC data application failed\n");
                break;
            }
        }

        if (rc >= 0)
            rc = mt9p012_vcm_i2c_write_w (mt9p012_vcm_client->addr, 0x3780, 0x8000);

        CDBG ("LSC data applied to sensor successfully\n");
    }
    else
    {
        rc = mt9p012_vcm_i2c_write_w_table(mt9p012_vcm_regs.lctbl,
                mt9p012_vcm_regs.lctbl_size);
    }
#else
    rc = mt9p012_vcm_i2c_write_w_table(mt9p012_vcm_regs.lctbl,
            mt9p012_vcm_regs.lctbl_size);

#if 0 /* Jignesh */
    if (rc < 0)
        return rc;
    rc = mt9p012_vcm_i2c_write_w_table(mt9p012_vcm_regs.rftbl,
                mt9p012_vcm_regs.rftbl_size);
#endif /* #if 0 */
#endif /* MOT_HW */
    return rc;
}

static void mt9p012_vcm_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider;   /*Q10 */
	//uint32_t pclk_mult; /*Q10 */
    uint32_t d1, d2;

	d1 = (uint32_t) ((mt9p012_vcm_regs.reg_pat[RES_PREVIEW].frame_length_lines *
      0x00000400) / mt9p012_vcm_regs.reg_pat[RES_CAPTURE].frame_length_lines);

	d2 = (uint32_t) ((mt9p012_vcm_regs.reg_pat[RES_PREVIEW].line_length_pck * 
      0x00000400) / mt9p012_vcm_regs.reg_pat[RES_CAPTURE].line_length_pck);

    divider = (uint32_t)(d1 * d2) / 0x00000400;

    /* Verify PCLK settings and frame sizes. */
	*pfps = (uint16_t) (fps * divider / 0x00000400);
 }

static uint16_t mt9p012_vcm_get_prev_lines_pf(void)
{
	if (mt9p012_vcm_ctrl->prev_res == QTR_SIZE)
		return mt9p012_vcm_regs.reg_pat[RES_PREVIEW].frame_length_lines;
	else
		return mt9p012_vcm_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9p012_vcm_get_prev_pixels_pl(void)
{
	if (mt9p012_vcm_ctrl->prev_res == QTR_SIZE)
		return mt9p012_vcm_regs.reg_pat[RES_PREVIEW].line_length_pck;
	else
		return mt9p012_vcm_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint16_t mt9p012_vcm_get_pict_lines_pf(void)
{
	return mt9p012_vcm_regs.reg_pat[RES_CAPTURE].frame_length_lines;
}

static uint16_t mt9p012_vcm_get_pict_pixels_pl(void)
{
	return mt9p012_vcm_regs.reg_pat[RES_CAPTURE].line_length_pck;
}

static uint32_t mt9p012_vcm_get_pict_max_exp_lc(void)
{
	uint16_t snapshot_lines_per_frame;

	if (mt9p012_vcm_ctrl->pict_res == QTR_SIZE)
		snapshot_lines_per_frame =
		mt9p012_vcm_regs.reg_pat[RES_PREVIEW].frame_length_lines - 1;
	else
		snapshot_lines_per_frame =
		mt9p012_vcm_regs.reg_pat[RES_CAPTURE].frame_length_lines - 1;

	return snapshot_lines_per_frame * 24;
}

static int32_t mt9p012_vcm_set_fps(struct fps_cfg *fps)
{
	/* input is new fps in Q10 format */
	int32_t rc = 0;

	mt9p012_vcm_ctrl->fps_divider = fps->fps_div;
	mt9p012_vcm_ctrl->pict_fps_divider = fps->pict_fps_div;

	rc =
		mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_HOLD);
	if (rc < 0)
		return -EBUSY;

	rc =
		mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_LINE_LENGTH_PCK,
			(mt9p012_vcm_regs.reg_pat[RES_PREVIEW].line_length_pck *
			fps->f_mult / 0x00000400));
	if (rc < 0)
		return rc;

	rc =
		mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);

	return rc;
}

static int32_t mt9p012_vcm_write_exp_gain(
        uint16_t gain, uint32_t line, int8_t is_outdoor)
{
	uint16_t max_legal_gain = 0x01FF;
	uint32_t line_length_ratio = 0x00000400;
	enum mt9p012_vcm_setting_t setting;
	int32_t rc = 0;
	int8_t multiplier = 1;
	CVBS("Line:%d mt9p012_vcm_write_exp_gain line=%d sensormode=%d\n",
		__LINE__, line, mt9p012_vcm_ctrl->sensormode);

	if (mt9p012_vcm_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
		mt9p012_vcm_ctrl->my_reg_gain = gain;
		mt9p012_vcm_ctrl->my_reg_line_count = (uint16_t)line;
	}

	if (gain > max_legal_gain)
		gain = max_legal_gain;

	/* Verify no overflow */
	if ((mt9p012_vcm_ctrl->sensormode != SENSOR_SNAPSHOT_MODE)&& 
	    (mt9p012_vcm_ctrl->sensormode != SENSOR_RAW_SNAPSHOT_MODE))
 {
		line = (uint32_t)(line * mt9p012_vcm_ctrl->fps_divider /
			0x00000400);
		setting = RES_PREVIEW;
        multiplier = 1;
    } else {
        line = (uint32_t)(line * mt9p012_vcm_ctrl->pict_fps_divider /
            0x00000400);
        setting = RES_CAPTURE;
        multiplier = 1;
        if (gain > max_legal_gain) {
			line = line >> 1;/*  divide by 2 */
            multiplier = 2;
        }
    }

	/* Set digital gain to 1 */
#ifdef MT9P012_VCM_REV_7
	gain |= 0x1000;
#else
	gain |= 0x0200;
#endif

	if ((mt9p012_vcm_regs.reg_pat[setting].frame_length_lines - 1) < line) {
		line_length_ratio = (uint32_t) (line * 0x00000400) /
		(mt9p012_vcm_regs.reg_pat[setting].frame_length_lines - 1);
	} else
		line_length_ratio = 0x00000400;

#if 1
	rc =
		mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
	if (rc < 0) {
		CDBG("mt9p012_vcm_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}
#endif
    if (mt9p012_vcm_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
#if 1
        if(is_outdoor == 1) {
            CVBS("%s:%d, outdoor\n", __func__, __LINE__);
            rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
                    0x316c, 0xA4F0); 
            if (rc < 0) {
                CDBG("%s:failed... Line:%d \n",
                        __func__, __LINE__);
                return rc;
            }
        } else if (is_outdoor == 0){
            CVBS("%s:%d, indoor\n", __func__, __LINE__);
            rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
                    0x316c, 0x4410); 
            if (rc < 0) {
                CDBG("%s:failed... Line:%d \n",
                        __func__, __LINE__);
                return rc;
            }
        }
#endif
    }
    if (mt9p012_vcm_ctrl->sensormode == SENSOR_PREVIEW_MODE) {
        CVBS("%s: value = %d\n", __func__,
                (mt9p012_vcm_regs.reg_pat[RES_PREVIEW]. \
                 line_length_pck * line_length_ratio / 0x00000400));
        rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
                REG_LINE_LENGTH_PCK,
                (uint16_t) (mt9p012_vcm_regs.reg_pat[RES_PREVIEW].\
                    line_length_pck * line_length_ratio / 0x00000400)); 
        if (rc < 0) {
            CDBG("%s:failed... Line:%d \n", __func__, __LINE__);
            return rc;
        }
    } else {
        CVBS("snapshot gain=%d\n",gain);
        if(gain > 0x10C0) {
            rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
                    REG_LINE_LENGTH_PCK, 
                    (uint16_t)mt9p012_vcm_regs.reg_pat[RES_CAPTURE]. \
                    line_length_pck*2);
            CDBG("line_length_pck=%d\n", 
                    (mt9p012_vcm_regs.reg_pat[RES_CAPTURE].line_length_pck*2));
            if (rc < 0) {
                CDBG("%s:failed,Line:%d\n", __func__, __LINE__);
                return rc;
            }
        }
    }
	rc =
		mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_GLOBAL_GAIN, gain);
	if (rc < 0) {
		CDBG("mt9p012_vcm_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

	rc =
		mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_COARSE_INT_TIME,
			(uint16_t)((uint32_t) line * 0x00000400 /
			line_length_ratio));
	if (rc < 0) {
		CDBG("mt9p012_vcm_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}
#if 1
	rc =
		mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_GROUPED_PARAMETER_HOLD,
			GROUPED_PARAMETER_UPDATE);
	if (rc < 0)
		CDBG("mt9p012_vcm_i2c_write_w failed... Line:%d \n", __LINE__);
#endif
	CVBS("mt9p012_vcm_write_exp_gain: gain = %d, line = %d\n", gain, line);

	return rc;
}

static int32_t mt9p012_vcm_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;

	CVBS("Line:%d mt9p012_vcm_set_pict_exp_gain \n", __LINE__);

	rc =
		mt9p012_vcm_write_exp_gain(gain, line, -1);
	if (rc < 0) {
		CDBG("Line:%d mt9p012_vcm_set_pict_exp_gain failed... \n",
			__LINE__);
		return rc;
	}

	rc =
	mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
		MT9P012_VCM_REG_RESET_REGISTER,
		0x10CC | 0x0002);
	if (rc < 0) {
		CDBG("mt9p012_vcm_i2c_write_w failed... Line:%d \n", __LINE__);
		return rc;
	}

	S_WAIT(5);

	return rc;
}

static int32_t mt9p012_vcm_setting(enum mt9p012_vcm_reg_update_t rupdate,
	enum mt9p012_vcm_setting_t rt)
{
	int32_t rc = 0;
        uint16_t readmode;

	switch (rupdate) {
	case UPDATE_PERIODIC: {
	  if (rt == RES_PREVIEW || rt == RES_CAPTURE) {

		struct mt9p012_vcm_i2c_reg_conf ppc_tbl[] = {
		{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD},
		{REG_ROW_SPEED, mt9p012_vcm_regs.reg_pat[rt].row_speed},
		{REG_X_ADDR_START, mt9p012_vcm_regs.reg_pat[rt].x_addr_start},
		{REG_X_ADDR_END, mt9p012_vcm_regs.reg_pat[rt].x_addr_end},
		{REG_Y_ADDR_START, mt9p012_vcm_regs.reg_pat[rt].y_addr_start},
		{REG_Y_ADDR_END, mt9p012_vcm_regs.reg_pat[rt].y_addr_end},
                {REG_READ_MODE, readmode =  (!(af_addr == MT9P012_VCM_AF_ADDR_VCM))?
                                               mt9p012_vcm_regs.reg_pat[rt].read_mode:
                                               (mt9p012_vcm_regs.reg_pat[rt].read_mode & 0xFFFF) |0xC000 },
		{REG_SCALE_M, mt9p012_vcm_regs.reg_pat[rt].scale_m},
		{REG_X_OUTPUT_SIZE, mt9p012_vcm_regs.reg_pat[rt].x_output_size},
		{REG_Y_OUTPUT_SIZE, mt9p012_vcm_regs.reg_pat[rt].y_output_size},

		{REG_LINE_LENGTH_PCK, mt9p012_vcm_regs.reg_pat[rt].line_length_pck},
		{REG_FRAME_LENGTH_LINES,
			(mt9p012_vcm_regs.reg_pat[rt].frame_length_lines *
			mt9p012_vcm_ctrl->fps_divider / 0x00000400)},
		{REG_COARSE_INT_TIME, mt9p012_vcm_regs.reg_pat[rt].coarse_int_time},
		{REG_FINE_INTEGRATION_TIME,
		  mt9p012_vcm_regs.reg_pat[rt].fine_int_time},
		{REG_SKEW, mt9p012_vcm_regs.reg_pat[rt].skew},
		{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE},
		};
		if (update_type == REG_INIT) {
			update_type = rupdate;
			return rc;
		}
                CVBS("%s : In mode : %d, new readmode for REG 3040 is %d \n", __func__, rt, readmode);
		rc = mt9p012_vcm_i2c_write_w_table(&ppc_tbl[0],
			ARRAY_SIZE(ppc_tbl));
		if (rc < 0)
			return rc;

		rc = mt9p012_vcm_test(mt9p012_vcm_ctrl->set_test);
		if (rc < 0)
			return rc;

		rc =
			mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			MT9P012_VCM_REG_RESET_REGISTER,
			MT9P012_VCM_RESET_REGISTER_PWON | 0x0002);
		if (rc < 0)
			return rc;

		S_WAIT(15);
		return rc;
	  }
	}
    break; /* UPDATE_PERIODIC */

	case REG_INIT: {
	    if (rt == RES_PREVIEW || rt == RES_CAPTURE) {
		struct mt9p012_vcm_i2c_reg_conf ipc_tbl1[] = {
		{MT9P012_VCM_REG_RESET_REGISTER, MT9P012_VCM_RESET_REGISTER_PWOFF},
		{REG_VT_PIX_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].vt_pix_clk_div},
		{REG_VT_SYS_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].vt_sys_clk_div},
		{REG_PRE_PLL_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].pre_pll_clk_div},
		{REG_PLL_MULTIPLIER, mt9p012_vcm_regs.reg_pat[rt].pll_multiplier},
		{REG_OP_PIX_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].op_pix_clk_div},
		{REG_OP_SYS_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].op_sys_clk_div},
#ifndef MT9P013
#ifdef MT9P012_VCM_REV_7
		{0x30B0, 0x0001},
		/* Jeff: {0x308E, 0xE060},
		{0x3092, 0x0A52}, */
		{0x3094, 0x4656},
		/* Jeff: {0x3096, 0x5652},
		{0x30CA, 0x8006},
		{0x312A, 0xDD02},
		{0x312C, 0x00E4},
		{0x3170, 0x299A}, */
#endif
		{0x0204, 0x0010},
		{0x0206, 0x0010},
		{0x0208, 0x0010},
		{0x020A, 0x0010},
		{0x020C, 0x0010},
#endif
		{0x316C, 0xA4F0},

#ifdef MT9P012_VCM_5131
		{0x3088, 0x6FFF}, //0x6FF6
		{0x3086, 0x2468},
#endif
		{MT9P012_VCM_REG_RESET_REGISTER, MT9P012_VCM_RESET_REGISTER_PWON},
		};

#ifndef MOT_HW // unused
		struct mt9p012_vcm_i2c_reg_conf ipc_tbl2[] = {
		{MT9P012_VCM_REG_RESET_REGISTER, MT9P012_VCM_RESET_REGISTER_PWOFF},
		{REG_VT_PIX_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].vt_pix_clk_div},
		{REG_VT_SYS_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].vt_sys_clk_div},
		{REG_PRE_PLL_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].pre_pll_clk_div},
		{REG_PLL_MULTIPLIER, mt9p012_vcm_regs.reg_pat[rt].pll_multiplier},
		{REG_OP_PIX_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].op_pix_clk_div},
		{REG_OP_SYS_CLK_DIV, mt9p012_vcm_regs.reg_pat[rt].op_sys_clk_div},
#ifndef MT9P013
#ifdef MT9P012_VCM_REV_7
		{0x30B0, 0x0001},
		{0x308E, 0xE060},
		{0x3092, 0x0A52},
		{0x3094, 0x4656},
		{0x3096, 0x5652},
		{0x30CA, 0x8006},
		{0x312A, 0xDD02},
		{0x312C, 0x00E4},
		{0x3170, 0x299A},
#endif
		{0x3088, 0x6FF6},
		{0x3154, 0x0282},
		{0x3156, 0x0381},
		{0x3162, 0x04CE},
		{0x0204, 0x0010},
		{0x316C, 0xA4F0},
#endif
		{MT9P012_VCM_REG_RESET_REGISTER, MT9P012_VCM_RESET_REGISTER_PWON},
		};
#endif

		struct mt9p012_vcm_i2c_reg_conf ipc_tbl3[] = {
		{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD},
		/* Set preview or snapshot mode */
		{REG_ROW_SPEED, mt9p012_vcm_regs.reg_pat[rt].row_speed},
		{REG_X_ADDR_START, mt9p012_vcm_regs.reg_pat[rt].x_addr_start},
		{REG_X_ADDR_END, mt9p012_vcm_regs.reg_pat[rt].x_addr_end},
		{REG_Y_ADDR_START, mt9p012_vcm_regs.reg_pat[rt].y_addr_start},
		{REG_Y_ADDR_END, mt9p012_vcm_regs.reg_pat[rt].y_addr_end},
                {REG_READ_MODE, readmode =  (!(af_addr == MT9P012_VCM_AF_ADDR_VCM))?
                                               mt9p012_vcm_regs.reg_pat[rt].read_mode:
                                               (mt9p012_vcm_regs.reg_pat[rt].read_mode & 0xFFFF) |0xC000 },
		{REG_SCALE_M, mt9p012_vcm_regs.reg_pat[rt].scale_m},
		{REG_X_OUTPUT_SIZE, mt9p012_vcm_regs.reg_pat[rt].x_output_size},
		{REG_Y_OUTPUT_SIZE, mt9p012_vcm_regs.reg_pat[rt].y_output_size},
		{REG_LINE_LENGTH_PCK, mt9p012_vcm_regs.reg_pat[rt].line_length_pck},
		{REG_FRAME_LENGTH_LINES,
			mt9p012_vcm_regs.reg_pat[rt].frame_length_lines},
		{REG_COARSE_INT_TIME, mt9p012_vcm_regs.reg_pat[rt].coarse_int_time},
		{REG_FINE_INTEGRATION_TIME,
		  mt9p012_vcm_regs.reg_pat[rt].fine_int_time},
		{REG_SKEW, mt9p012_vcm_regs.reg_pat[rt].skew},
		{REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE},
		};

		/* reset fps_divider */
		mt9p012_vcm_ctrl->fps_divider = 1 * 0x0400;
                CVBS("%s : In mode : %d, new readmode for REG 3040 is %d \n", __func__, rt, readmode);
		rc = mt9p012_vcm_i2c_write_w_table(&ipc_tbl1[0],
			ARRAY_SIZE(ipc_tbl1));
		if (rc < 0)
			return rc;
		S_WAIT(5);

		rc = mt9p012_vcm_i2c_write_w_table(&ipc_tbl3[0],
			ARRAY_SIZE(ipc_tbl3));
		if (rc < 0)
			return rc;

		rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_HOLD);
		if (rc < 0)
			return rc;

		rc = mt9p012_vcm_set_lc();
		if (rc < 0)
			return rc;

		rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			REG_GROUPED_PARAMETER_HOLD, GROUPED_PARAMETER_UPDATE);

		if (rc < 0)
			return rc;
		}
	}
		update_type = rupdate;
		break; /* case REG_INIT: */

	default:
		rc = -EFAULT;
		break;
	} /* switch (rupdate) */

	return rc;
}

static int32_t mt9p012_vcm_video_config(int mode, int res)
{
	int32_t rc;

	switch (res) {
	case QTR_SIZE:
		rc = mt9p012_vcm_setting(REG_INIT, RES_PREVIEW);
		if (rc < 0)
			return rc;

		rc = mt9p012_vcm_setting(UPDATE_PERIODIC, RES_PREVIEW);
		if (rc < 0)
			return rc;

		CDBG("mt9p012_vcm sensor configuration done!\n");
		break;

	case FULL_SIZE:
		rc = mt9p012_vcm_setting(REG_INIT, RES_CAPTURE);
		if (rc < 0)
			return rc;

		rc =
		mt9p012_vcm_setting(UPDATE_PERIODIC, RES_CAPTURE);
		if (rc < 0)
			return rc;

		break;

	default:
		return 0;
	} /* switch */

	mt9p012_vcm_ctrl->prev_res = res;
	mt9p012_vcm_ctrl->curr_res = res;
	mt9p012_vcm_ctrl->sensormode = mode;

	rc =
		mt9p012_vcm_write_exp_gain(mt9p012_vcm_ctrl->my_reg_gain,
			mt9p012_vcm_ctrl->my_reg_line_count, -1);

	rc =
		mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
			MT9P012_VCM_REG_RESET_REGISTER,
			0x10cc|0x0002);

	return rc;
}

static int32_t mt9p012_vcm_snapshot_config(int mode)
{
	int32_t rc = 0;

#ifdef MOT_HW
        load_snapshot_lsc = true;
#endif
	rc = mt9p012_vcm_setting(REG_INIT, RES_CAPTURE);      
	if (rc < 0)
		return rc;

	rc = mt9p012_vcm_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9p012_vcm_ctrl->curr_res = mt9p012_vcm_ctrl->pict_res;

	mt9p012_vcm_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9p012_vcm_raw_snapshot_config(int mode)
{
	int32_t rc = 0;

	rc = mt9p012_vcm_setting(REG_INIT, RES_CAPTURE);
	if (rc < 0)
		return rc;

	rc = mt9p012_vcm_setting(UPDATE_PERIODIC, RES_CAPTURE);
	if (rc < 0)
		return rc;

	mt9p012_vcm_ctrl->curr_res = mt9p012_vcm_ctrl->pict_res;

	mt9p012_vcm_ctrl->sensormode = mode;

	return rc;
}

static int32_t mt9p012_vcm_power_down(void)
{
	int32_t rc = 0;

	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
		MT9P012_VCM_REG_RESET_REGISTER,
		MT9P012_VCM_RESET_REGISTER_PWOFF);

	S_WAIT(5);

	return rc;
}

#ifdef MOT_HW

/****************************** EEPROM CODE ******************************/

static int32_t eeprom_i2c_read_block(unsigned short raddr,
        uint8_t *rdata, uint16_t length)
{
    struct i2c_msg msg[2];

    // NOTE: SEMCO and VPT have the same I2C address for the EEPROM
    uint16_t ee_addr = (0xa0>>1);

	int32_t rc = 0;
    unsigned char buf[4];
    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00)>>8;
    buf[1] = (raddr & 0x00FF);


    msg[0].addr = ee_addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = buf;

    msg[1].addr = ee_addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = length;
    msg[1].buf = rdata;

    rc = i2c_transfer(mt9p012_vcm_client->adapter, msg, 2);

    if (rc < 0)
        CDBG("eeprom i2c_read_block failed!\n");

    return rc;
}

static bool read_eep (void)
{
    uint16_t i;
    uint16_t data_pending;
    bool ret_val = true;


    pEpromImage = kmalloc (EEPROM_SIZE, GFP_ATOMIC);
    if (pEpromImage == NULL)
    {
        CDBG("CAL memory not allocated");
        return false;
    }

    data_pending = EEPROM_SIZE;
    for (i = 0; data_pending > EEPROM_BLOCK_SZ; i++)
    {
       if ((eeprom_i2c_read_block (i*EEPROM_BLOCK_SZ, 
                    &pEpromImage[i*EEPROM_BLOCK_SZ], 
				  EEPROM_BLOCK_SZ))< 0)
        {
            CDBG("CAL block %d read fail\n", i);
            eep_state = EEP_DATA_INVALID;
            ret_val= false;
            break;
        }

        data_pending -= EEPROM_BLOCK_SZ;
    }

    if (!eeprom_i2c_read_block (i*EEPROM_BLOCK_SZ, 
                &pEpromImage[i*EEPROM_BLOCK_SZ], 
                data_pending))
    {
        CDBG("CAL block %d read fail\n", i);
        eep_state = EEP_DATA_INVALID;
        ret_val= false;
    }

    if (ret_val == false)
    {
        eep_state = EEP_DATA_INVALID;
        kfree(pEpromImage);
        pEpromImage=NULL;
    }
    return ret_val;
}
#endif 

static int32_t vcm_af_move_lens(uint16_t data)
{
	int32_t rc = -EFAULT;
	uint8_t code_val_msb, code_val_lsb;
	CDBG("%s : lens move to %d \n", __func__, data);
	code_val_msb = (data >> 8) | 0xC0 ;
	code_val_lsb = (data & 0x00FF);
	rc = mt9p012_vcm_i2c_write_b(af_addr, code_val_msb, code_val_lsb);
	return rc;
}

static int32_t vcm_af_calculate_target_position(int16_t step_direction, int32_t num_steps)
{
	int32_t rc = 0;
	int16_t dest_lens_position, dest_step_position;
	int16_t target_dist;
	int16_t position;
	int16_t dist;
	int16_t step;
	uint16_t mt9p012_vcm_damp_threshold = 20; 
	uint16_t mt9p012_vcm_damping_step = 5;   
	uint16_t mt9p012_vcm_damping_time = 1;
	uint16_t  mt9p012_vcm_damping_course_step = 4;

	CDBG("Enter %s \n", __func__);

	dest_step_position = mt9p012_vcm_ctrl->curr_step_pos + (step_direction * num_steps);

	if (dest_step_position < 0)
	{
		dest_step_position = 0;
	}
	else if (dest_step_position > debug_MT9P012_VCM_MU5M0_TOTAL_STEPS_NEAR_TO_FAR)
	{
		dest_step_position = debug_MT9P012_VCM_MU5M0_TOTAL_STEPS_NEAR_TO_FAR;
	}

	if(dest_step_position == mt9p012_vcm_ctrl->curr_step_pos)
	{
		return rc;
	}

	dest_lens_position = step_position_table[dest_step_position];
	target_dist = step_direction * (dest_lens_position - mt9p012_vcm_ctrl->curr_lens_pos);



	step = target_dist/mt9p012_vcm_damping_course_step + (5 /10);
	if (step == 0)
	{
		step = 1; 
	}

	position = mt9p012_vcm_ctrl->curr_lens_pos;

	CDBG("%s : destination lens position: %d, target_distance %d, step value : %d, position %d \n ", __func__, dest_lens_position, target_dist, step, position);

	for (dist = 0; dist < target_dist; dist += step)
	{
		if (step_direction > 0)
		{
			position += step;
			/* 
			special case : position beyond the target distance 
			- liekely due to division of distance has remainder
			- write whatever we can without going over target
			- no need for delay since this would be the last write
			*/
			if (position > dest_lens_position)
			{
				position = dest_lens_position;
				if ( vcm_af_move_lens(position) < 0 )
				return rc; 
				break;
			}
		}
		else
		{
			position -= step;
			 /* 
			special case : position beyond the target distance 
			- liekely due to division of distance has remainder
			 - write whatever we can without going over target
			- no need for delay since this would be the last write
			*/
			if (position < dest_lens_position)
			{
				position = dest_lens_position;
				if (vcm_af_move_lens(position) < 0 )
				return rc; 
				break;
			}
 		}

		CDBG("%s Step_direction % d \n", __func__, step_direction);
		/* destination is inside damping zone and movement is toward infinity side */
		/* mt9p012_vcm_current_step_position is unknown at this point */ 
		if ((step_direction < 0) && (position < step_position_table[mt9p012_vcm_damp_threshold]) && (target_dist >= (4*mt9p012_vcm_damping_step))) 
		{

			CDBG("Inside the IF loop of damping zone \n");
			position = mt9p012_vcm_ctrl->curr_lens_pos;
			/* first, move lens to threshold if lens is outside of damping zone */
			if(mt9p012_vcm_ctrl->curr_lens_pos > step_position_table[mt9p012_vcm_damp_threshold])
			{      
				position = step_position_table[mt9p012_vcm_damp_threshold];
				if ( vcm_af_move_lens(position) < 0 )
					return rc;
				mt9p012_vcm_ctrl->curr_lens_pos = position;
				mdelay(mt9p012_vcm_damping_time);
			}
	 
			/* then, move 1 mt9p012_vcm_damping_step at a time untill the lens is at next_step_position */
			while (mt9p012_vcm_ctrl->curr_lens_pos > dest_lens_position)
			{
				position -= mt9p012_vcm_damping_step;
				if(position < dest_lens_position)
				{
					position = dest_lens_position;
				}
				if ( vcm_af_move_lens(position) < 0 )
				 	return rc;
				mt9p012_vcm_ctrl->curr_lens_pos = position;
				mdelay(mt9p012_vcm_damping_time);
			} 	
			break; 
		}
		/* Writing the digital code for current to the actuator */

		CDBG("%s About to move lens to Position : %d \n", __func__, position);
		if ( vcm_af_move_lens(position) < 0 )
			return rc; 
		mt9p012_vcm_ctrl->curr_lens_pos = position;
		if (position != dest_lens_position)
		{
			mdelay(5);
		 }
	} // end of inner for loop

	mt9p012_vcm_ctrl->curr_lens_pos = dest_lens_position;
	mt9p012_vcm_ctrl->curr_step_pos = dest_step_position;
	CDBG("done: current position %d\n", mt9p012_vcm_ctrl->curr_step_pos);
	return rc;

}


static int32_t mt9p012_vcm_move_focus(int direction, int32_t num_steps)
{
	int32_t rc = 0;
	int16_t step_direction;
	int16_t actual_step;
	int16_t next_position;
	uint8_t code_val;
	uint8_t time_out;
	uint16_t actual_position_target;
	uint16_t current_position_pi;
	uint16_t pi_gap;
#ifndef MOT_HW
	uint8_t ret_status = 0;
#endif

#ifdef MOT_HW 
    if(disable_lens_move)
    {
        printk("CAM_TCMD: lens move disabled.");
        return 0;
    }

    if (af_addr == 0) {
        return 0;
    }
#endif

    CVBS("%s: IN, num_steps = %d\n", __func__, num_steps);

	if (num_steps > MT9P012_VCM_TOTAL_STEPS_NEAR_TO_FAR)
		num_steps = MT9P012_VCM_TOTAL_STEPS_NEAR_TO_FAR;
	else if (num_steps == 0) {
		CDBG("mt9p012_vcm_move_focus failed at line %d ...\n", __LINE__);
		return -EINVAL;
	}

	if (direction == MOVE_NEAR)
			step_direction = 1;
	else if (direction == MOVE_FAR)
			step_direction = -1;
	else {
		CDBG("mt9p012_vcm_move_focus failed at line %d ...\n", __LINE__);
		return -EINVAL;
	}

	CDBG("%s : Time to calculate VCM target position , step_direction : %d , num_steps : %d \n", __func__, step_direction, num_steps);
	rc = vcm_af_calculate_target_position(step_direction, num_steps);
	return rc;

}

static int32_t mt9p012_vcm_set_default_focus(void)
{
	int32_t rc = 0;

#ifdef MOT_HW
    if(disable_lens_move)
    {
        printk("CAM_TCMD: lens move disabled.");
        return 0;
    }

    if (af_addr == 0) {
        return 0;
    }
#endif


	CDBG("%s : Time to Set VPT default focus \n",__func__);
	if( mt9p012_vcm_ctrl->curr_step_pos != 0)
	{
		if(mt9p012_vcm_move_focus(MOVE_FAR,  mt9p012_vcm_ctrl->curr_step_pos) < 0)
			return rc;
	}
	else
	{
		/* rewrite in case sensor is changed */
		if (vcm_af_move_lens(0) < 0)
			return rc;
	}
	mt9p012_vcm_ctrl->curr_step_pos = 0; 
	mt9p012_vcm_ctrl->curr_lens_pos = 0;

	return rc;
}


static int mt9p012_vcm_sensor_power_on(struct msm_camera_sensor_info *data)
{
    int32_t rc;
    uint8_t temp_pos;

    CDBG("%s\n", __func__);

#ifdef CONFIG_MACH_MOT
    if (machine_is_motus()) {
        struct vreg *vreg;
        vreg = vreg_get(0,"gp1");
        if (!vreg) {
            CDBG("%s: vreg_get failed\n", __func__);
            return -EIO;
        }

        if ((rc = vreg_set_level(vreg, 2800))) {
            CDBG("%s: failed to set GP1 level\n", __func__);
            return rc;
        }

        if ((rc = vreg_enable(vreg))) {
            CDBG("%s: failed to enable GP1\n", __func__);
            return rc;
        }
    }
#endif

    rc = gpio_request(data->sensor_pwd, "mt9p012_vcm");
    if (!rc) {
        gpio_direction_output(data->sensor_pwd, 0);
        S_WAIT(20);
        gpio_direction_output(data->sensor_pwd, 1);
    }
    else {
        CDBG("%s gpio request %d failed\n", __func__, data->sensor_pwd);
        return rc;
    }

    rc = gpio_request(data->sensor_reset, "mt9p012_vcm");
    if (!rc) {
        gpio_direction_output(data->sensor_reset, 0);
        S_WAIT(20);
        gpio_direction_output(data->sensor_reset, 1);
    }
    else {
        CDBG("%s gpio request %d failed\n", __func__, data->sensor_reset);
        return rc;
    }

    /* enable AF actuator */
#if 0
    rc = gpio_request(data->vcm_pwd, "mt9p012_vcm");
    if (!rc)
        gpio_direction_output(data->vcm_pwd, 1);
    else {
        CDBG("mt9p012_vcm_ctrl gpio request failed!\n");
        return rc;
    }
#endif

#if 0
#ifdef MOT_HW

 /* determine AF actuator by i2c address */
    af_addr = MT9P012_VCM_AF_ADDR_BAM2;
    if ( mt9p012_vcm_i2c_read_b(af_addr, 0x12, &temp_pos) < 0 ) {
        af_addr = MT9P012_VCM_AF_ADDR_BAM1;
        if ( mt9p012_vcm_i2c_read_b(af_addr, 0x12, &temp_pos) < 0 ) {
            af_addr = MT9P012_VCM_AF_ADDR_VCM;
	    //af_addr = MT9P012_VCM_AF_I2C_ADDR >> 1;            
	    if ( mt9p012_vcm_i2c_read_b(af_addr, 0x12, &temp_pos) < 0 ) {
                af_addr = 0;
                CDBG("* unable to determine AF actuator\n");
            }
        }
    }

    CDBG("mt9p012_vcm AF addr = 0x%02x\n", af_addr);

#endif /*MOT_HW */
#endif

    // request max AXI bus for camera
    rc = request_axi_qos();
    if (rc < 0) {
        CDBG("* request of axi qos failed\n");
        return rc;
    }

    return 0;
}

static int mt9p012_vcm_sensor_power_off(struct msm_camera_sensor_info *data)
{
    int32_t rc;

    CDBG("%s\n", __func__);

    // make sure we released the AXI QOS
    release_axi_qos();

	gpio_direction_output(data->sensor_pwd, 0);
	gpio_free(data->sensor_pwd);

	gpio_direction_output(data->sensor_reset, 0);
	gpio_free(data->sensor_reset);

#if 0
	gpio_direction_output(data->vcm_pwd, 0);
	gpio_free(data->vcm_pwd);
#endif

#ifdef CONFIG_MACH_MOT
    if (machine_is_motus()) {
        struct vreg *vreg;
        vreg = vreg_get(0,"gp1");
        if (!vreg) {
            CDBG("%s: vreg_get failed\n", __func__);
            return -EIO;
        }

        if ((rc = vreg_disable(vreg))) {
            CDBG("%s: failed to disable GP1\n", __func__);
            return rc;
        }
    }
#endif

   return 0;
}

static int mt9p012_vcm_probe_init_done(struct msm_camera_sensor_info *data)
{
    mt9p012_vcm_sensor_power_off(data);
	return 0;
}

static int mt9p012_vcm_probe_init_sensor(struct msm_camera_sensor_info *data)
{
	int32_t  rc;
	uint16_t chipid;
    uint16_t revnum;
	int actuator_count;
	uint8_t temp_pos;

    mt9p012_vcm_sensor_power_on(data);

#ifdef MOT_HW
	S_WAIT(20);

	/* enable mclk first */
    CDBG("mt9p012_vcm mclk = %d\n", MT9P012_VCM_DEFAULT_CLOCK_RATE);
	msm_camio_clk_rate_set(MT9P012_VCM_DEFAULT_CLOCK_RATE);

	S_WAIT(20);

#endif
	/* RESET the sensor image part via I2C command */
	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
		MT9P012_VCM_REG_RESET_REGISTER, 0x10CC|0x0001|1<<5);
	if (rc < 0) {
		CDBG("sensor reset failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

	S_WAIT(MT9P012_VCM_RESET_DELAY_MSECS);

	/* 3. Read sensor Model ID and Revision: */
	rc = mt9p012_vcm_i2c_read_w(mt9p012_vcm_client->addr,
		MT9P012_VCM_REG_MODEL_ID, &chipid);
	if (rc < 0)
		goto init_probe_fail;

	CDBG("mt9p012_vcm model_id = 0x%x\n", chipid);

    rc = mt9p012_vcm_i2c_read_w(mt9p012_vcm_client->addr,
        0x31FE, &revnum);
    if (rc < 0)
        goto init_probe_fail;

    CDBG("mt9p012_vcm revision = %d\n", revnum);

	/* 4. Verify sensor ID: */
	if (chipid != MT9P012_VCM_MODEL_ID
	&&
	chipid != MT9P012_VCM_MODEL_ID_5131) {
		rc = -ENODEV;
		goto init_probe_fail;
	}

//Probing the Actuator address to register the respective driver. 
	#ifdef MOT_HW

	if(data->actuator_probe_needed)
	{
		for (actuator_count = 0; actuator_count < data->num_af_actuators; actuator_count++)
		{
			af_addr = data->act_addr[actuator_count] >> 1;  // act_addr[0] = 0x0C. Right shifting the AF addr since we are doing << 1 in the Rx data.. 
			rc = mt9p012_vcm_i2c_read_b(af_addr, 0x12, &temp_pos);
			if (  rc >= 0 ) 
			{
				break;
				
			}
		}
		if (rc < 0)
		{
			goto init_probe_fail;
		}
		CDBG("%s: Probe succeeded - This is a VPT module (VCM AF) \n", __func__);
	}
    

    CDBG("mt9p012_vcm AF addr = 0x%02x\n", af_addr);

	#endif /*MOT_HW */
	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr, 0x306E, 0x9000);
	if (rc < 0) {
		CDBG("REV_7 write failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

	/* RESET_REGISTER, enable parallel interface and disable serialiser */
	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr, 0x301A, 0x10CC);
	if (rc < 0) {
		CDBG("enable parallel interface failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

	/* To disable the 2 extra lines */
	rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
		0x3064, 0x0805);

	if (rc < 0) {
		CDBG("disable the 2 extra lines failed. rc = %d\n", rc);
		goto init_probe_fail;
	}

#ifdef MOT_HW
	S_WAIT(MT9P012_VCM_RESET_DELAY_MSECS);

#endif
	goto init_probe_done;

init_probe_fail:
	mt9p012_vcm_probe_init_done(data);
init_probe_done:
	return rc;
}

static int mt9p012_vcm_sensor_open_init(struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint8_t i;
    int attempt = 0;
#ifndef MOT_HW
	uint8_t temp_pos;
	uint16_t temp;
#endif


	uint16_t mt9p012_vcm_nl_region_boundary1 = 3;
	uint16_t mt9p012_vcm_nl_region_code_per_step1 = 45; 
	uint16_t mt9p012_vcm_nl_region_boundary2 = 4;  
	uint16_t mt9p012_vcm_nl_region_code_per_step2 = 16;
	uint16_t mt9p012_vcm_l_region_code_per_step = 9;  


	mt9p012_vcm_ctrl = kzalloc(sizeof(struct mt9p012_vcm_ctrl_t), GFP_KERNEL);
	if (!mt9p012_vcm_ctrl) {
		CDBG("mt9p012_vcm_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	mt9p012_vcm_ctrl->fps_divider = 1 * 0x00000400;
	mt9p012_vcm_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9p012_vcm_ctrl->set_test = TEST_OFF;
	mt9p012_vcm_ctrl->prev_res = QTR_SIZE;
	mt9p012_vcm_ctrl->pict_res = FULL_SIZE;

	mt9p012_vcm_ctrl->curr_lens_pos = 0;


	if (data)
		mt9p012_vcm_ctrl->sensordata = data;

    wake_lock_init(&mt9p012_vcm_wake_lock, WAKE_LOCK_IDLE, "mt9p012_vcm");
    wake_lock(&mt9p012_vcm_wake_lock);

    do {
        if (rc < 0)
            CDBG("mt9p012_vcm: mt9p012_vcm_sensor_open_init: ATTEMPT FAILED: rc=%d\n", rc);
        attempt++;
        
	    /* enable mclk first */
	    msm_camio_clk_rate_set(MT9P012_VCM_DEFAULT_CLOCK_RATE);
	    S_WAIT(20);

        msm_camio_camif_pad_reg_reset();
        S_WAIT(20);

        rc = mt9p012_vcm_probe_init_sensor(data);
        if (rc < 0)
            continue;

        if (mt9p012_vcm_ctrl->prev_res == QTR_SIZE)
            rc = mt9p012_vcm_setting(REG_INIT, RES_PREVIEW);
        else
            rc = mt9p012_vcm_setting(REG_INIT, RES_CAPTURE);

        if (rc < 0) {
            CDBG("mt9p012_vcm_setting failed. rc = %d\n", rc);
            continue;
        }

        /* sensor : output enable */
        rc = mt9p012_vcm_i2c_write_w(mt9p012_vcm_client->addr,
                MT9P012_VCM_REG_RESET_REGISTER, MT9P012_VCM_RESET_REGISTER_PWON);
        if (rc < 0) {
            CDBG("sensor output enable failed. rc = %d\n", rc);
            continue;
        }

        S_WAIT(20);

		step_position_table[0] = 0;
	    if(VCM_STEP_POSITION_TABLE_CONSTRUCT == 2)
		{
			for(i=1; i <= debug_MT9P012_VCM_MU5M0_TOTAL_STEPS_NEAR_TO_FAR; i++)
			{
				if ( i <= mt9p012_vcm_nl_region_boundary1)
				{
					step_position_table[i] = step_position_table[i-1] + mt9p012_vcm_nl_region_code_per_step1;
				}
				else if ( i <= mt9p012_vcm_nl_region_boundary2)
				{
					step_position_table[i] = step_position_table[i-1] + mt9p012_vcm_nl_region_code_per_step2;
				}
				else
				{
					step_position_table[i] = step_position_table[i-1] + mt9p012_vcm_l_region_code_per_step;
				}
				CDBG("%s : Step_position_table[%d] value : %d \n", __func__, i, step_position_table[i]);
			}
		}

        rc = mt9p012_vcm_set_default_focus();
	} while (rc < 0 && attempt < CAMERA_CONFIG_ATTEMPTS);
    
	if (rc >= 0)
		goto init_done;

	mt9p012_vcm_probe_init_done(data);
	kfree(mt9p012_vcm_ctrl);
init_done:
	return rc;
}

static int mt9p012_vcm_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9p012_vcm_wait_queue);
	return 0;
}

static int32_t mt9p012_vcm_set_sensor_mode(int mode, int res)
{
	int attempt = 0;
    int32_t rc = 0;

	do {
        if (rc < 0)
            CDBG("mt9p012_vcm: set_sensor_mode: ATTEMPT FAILED: rc=%d\n", rc);
        attempt++;
        
        switch (mode) {
	    case SENSOR_PREVIEW_MODE:
            CDBG("mt9p012_vcm: set_sensor_mode: PREVIEW_MODE res=%d\n", res);
		    rc = mt9p012_vcm_video_config(mode, res);
		    break;

	    case SENSOR_SNAPSHOT_MODE:
            CDBG("mt9p012_vcm: set_sensor_mode: SNAPSHOT_MODE\n");
		    rc = mt9p012_vcm_snapshot_config(mode);
		    break;

	    case SENSOR_RAW_SNAPSHOT_MODE:
            CDBG("mt9p012_vcm: set_sensor_mode: RAW_SNAPSHOT_MODE\n");
		    rc = mt9p012_vcm_raw_snapshot_config(mode);
		    break;

	    default:
		    rc = -EINVAL;
		    break;
	    }
    } while (rc < 0 && rc != -EINVAL && attempt < CAMERA_CONFIG_ATTEMPTS);

	return rc;
}

int mt9p012_vcm_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	if (copy_from_user(&cdata,
			(void *)argp,
			sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	down(&mt9p012_vcm_sem);

    CVBS("%s: cfgtype = %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_PICT_FPS:
		mt9p012_vcm_get_pict_fps(cdata.cfg.gfps.prevfps,
			&(cdata.cfg.gfps.pictfps));

		if (copy_to_user((void *)argp, &cdata,
				sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_L_PF:
		cdata.cfg.prevl_pf = mt9p012_vcm_get_prev_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PREV_P_PL:
		cdata.cfg.prevp_pl = mt9p012_vcm_get_prev_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_L_PF:
		cdata.cfg.pictl_pf = mt9p012_vcm_get_pict_lines_pf();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_P_PL:
		cdata.cfg.pictp_pl = mt9p012_vcm_get_pict_pixels_pl();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_GET_PICT_MAX_EXP_LC:
		cdata.cfg.pict_max_exp_lc =
			mt9p012_vcm_get_pict_max_exp_lc();

		if (copy_to_user((void *)argp,
			&cdata,
			sizeof(struct sensor_cfg_data)))
			rc = -EFAULT;
		break;

	case CFG_SET_FPS:
	case CFG_SET_PICT_FPS:
		rc = mt9p012_vcm_set_fps(&(cdata.cfg.fps));
		break;

  	case CFG_SET_EXP_GAIN:
 		CVBS("Line:%d CFG_SET_EXP_GAIN gain=%d line=%d\n",
                         __LINE__, cdata.cfg.exp_gain.gain, 
                         cdata.cfg.exp_gain.line);
  		rc =
  			mt9p012_vcm_write_exp_gain(cdata.cfg.exp_gain.gain,
 				cdata.cfg.exp_gain.line, 
 				cdata.cfg.exp_gain.is_outdoor);
  		break;

	case CFG_SET_PICT_EXP_GAIN:
		CVBS("Line:%d CFG_SET_PICT_EXP_GAIN \n", __LINE__);
		rc =
			mt9p012_vcm_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
		break;

	case CFG_SET_MODE:
		rc =
			mt9p012_vcm_set_sensor_mode(
			cdata.mode, cdata.rs);
		break;

	case CFG_PWR_DOWN:
		rc = mt9p012_vcm_power_down();
		break;

	case CFG_MOVE_FOCUS:
		CVBS("mt9p012_vcm_ioctl: CFG_MOVE_FOCUS: dir=%d steps=%d\n",
			cdata.cfg.focus.dir, cdata.cfg.focus.steps);
		rc =
			mt9p012_vcm_move_focus(
			cdata.cfg.focus.dir,
			cdata.cfg.focus.steps);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc =
			mt9p012_vcm_set_default_focus();

		break;

	case CFG_SET_EFFECT:
		rc =
			mt9p012_vcm_set_default_focus();
		break;

	case CFG_SET_LENS_SHADING:
		CVBS("%s: CFG_SET_LENS_SHADING\n", __func__);
		rc = mt9p012_vcm_lens_shading_enable(
			cdata.cfg.lens_shading);
		break;

        case CFG_LOAD_TEST_PATTERN_DLI:
            mt9p012_vcm_ctrl->set_test = TEST_W1_10B;
            rc = mt9p012_vcm_setting(UPDATE_PERIODIC, RES_CAPTURE);
            break;

#ifdef MOT_HW
        case CFG_GET_EEPROM_DATA:
            if (pEpromImage != NULL)
            {
              memcpy (cdata.cfg.eeprom_data_ptr, pEpromImage, EEPROM_SIZE);
              if (copy_to_user ((void *)argp, &cdata, sizeof (cdata)))
                rc = -EFAULT;
              else
                CDBG ("EEPROM data provided by kernel \n");
            }
            else
              rc= -EFAULT;
            break;
          
        case CFG_SET_MODULE_DATA:
			CDBG("%s: CFG_SET_MODULE_DATA ; Module valid data: %d, \n", __func__,cdata.cfg.module_data.valid_data );
            if (cdata.cfg.module_data.valid_data == true)
            {
                memcpy (&module_data, &cdata.cfg.module_data, 
                      sizeof(module_data));

                eep_state = EEP_DATA_VALID;

				if (af_addr = MT9P012_VCM_AF_ADDR_VCM)//MT9P012_VCM_AF_I2C_ADDR >> 1 ) 
				{
					int i = 0;
					calibrated_macro = module_data.cal_macro_pos;
					calibrated_liftoff_curr = module_data.cal_liftoff_curr;

					CDBG("%s: calibrated af_macro : %d, calibrated af_liftoff_curr : %d \n", __func__, calibrated_macro, calibrated_liftoff_curr);
					
					adjusted_macro = module_data.adjusted_macro_pos;
					adjusted_liftoff_curr = module_data.adjusted_liftoff_curr; 

				    CDBG("%s: New af_macro : %d, New af_liftoff_curr : %d \n", __func__, adjusted_macro, adjusted_liftoff_curr);
					
					if (VCM_STEP_POSITION_TABLE_CONSTRUCT == 1)
					{
						for (i = 0; i < debug_MT9P012_VCM_MU5M0_TOTAL_STEPS_NEAR_TO_FAR; i++)
						{
							step_position_table[i] = adjusted_liftoff_curr + i*(adjusted_macro-adjusted_liftoff_curr)/debug_MT9P012_VCM_MU5M0_TOTAL_STEPS_NEAR_TO_FAR; 
							CDBG("%s : Step_position_table[%d] value : %d \n", __func__, i, step_position_table[i]);
						}
					}
	
				}

            }
            else
            {
                CDBG ("EEP- INVALID parsed data passed in \n");
                module_data.valid_data = false;
                eep_state = EEP_DATA_INVALID;
            }

            if (pEpromImage != NULL)
            {
              kfree(pEpromImage);
              pEpromImage=NULL;
            }
            break;

        case CFG_GET_AF_DEVICE_ADDRESS: 
            cdata.cfg.af_device_address = af_addr; //mt9p012_vcm_get_af_actuator_address()
            if (copy_to_user((void *)argp,
                    &cdata,
                    sizeof(struct sensor_cfg_data)))
                    rc = -EFAULT;
            break;

#endif /* MOT_HW */          
        case CFG_GET_WB_GAINS:
            CVBS("%s: CFG_GET_WB_GAINS\n", __func__);
#ifdef MOT_HW
            if ((eep_state == EEP_DATA_VALID) && (module_data.lsc_count != 1))
              {
                cdata.cfg.wb_gains.g_gain = module_data.Ggain;
                cdata.cfg.wb_gains.r_gain = module_data.Rgain;
                cdata.cfg.wb_gains.b_gain = module_data.Bgain;
              }
#else 
		cdata.cfg.wb_gains.g_gain = 0x100;
		cdata.cfg.wb_gains.r_gain = 0x200;
		cdata.cfg.wb_gains.b_gain = 0x300;

#endif /* MOT_HW */
            if (copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
            break;

 	case CFG_SET_AWB_DECISION:
#ifdef MOT_HW
            awb_decision =  cdata.cfg.awb_decision;
#endif
            CVBS("awb_decision = %d\n", cdata.cfg.awb_decision);
            break;

        case CFG_GET_CAL_DATA_STATUS:
            CDBG("%s: CFG_GET_CAL_DATA_STATUS\n", __func__);
            memcpy (&cdata.cfg.module_data, &module_data,
                      sizeof(module_data));
           if (copy_to_user((void *)argp, &cdata, sizeof(struct sensor_cfg_data)))
                rc = -EFAULT;
            break;

        default:
		rc = -EFAULT;
		break;
	}

	up(&mt9p012_vcm_sem);
	return rc;
}

int mt9p012_vcm_sensor_release(void)
{
	int rc = -EBADF;

	down(&mt9p012_vcm_sem);

	mt9p012_vcm_power_down();

    mt9p012_vcm_sensor_power_off(mt9p012_vcm_ctrl->sensordata);

	kfree(mt9p012_vcm_ctrl);
	mt9p012_vcm_ctrl = NULL;

    wake_unlock(&mt9p012_vcm_wake_lock);
    wake_lock_destroy(&mt9p012_vcm_wake_lock);

	CDBG("mt9p012_vcm_release completed\n");

	up(&mt9p012_vcm_sem);
	return rc;
}

static int mt9p012_vcm_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("mt9p012_vcm_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	mt9p012_vcm_sensorw = kzalloc(sizeof(struct mt9p012_vcm_work_t), GFP_KERNEL);
	if (!mt9p012_vcm_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9p012_vcm_sensorw);
	mt9p012_vcm_init_client(client);
	mt9p012_vcm_client = client;

	S_WAIT(50);

	CDBG("mt9p012_vcm_probe succeeded!\n");
#ifdef MOT_HW
    if (read_eep())
    {
        CDBG("EEPROM read successfully \n");
        eep_state = EEP_DATA_READ;
    }
#endif
    return 0;

probe_failure:
	CDBG("mt9p012_vcm_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit mt9p012_vcm_remove(struct i2c_client *client)
{
	struct mt9p012_vcm_work_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	i2c_detach_client(client);
	mt9p012_vcm_client = NULL;
	kfree(sensorw);
	return 0;
}

static const struct i2c_device_id mt9p012_vcm_id[] = {
	{ "mt9p012_vcm", 0},
	{ }
};

static struct i2c_driver mt9p012_vcm_driver = {
	.id_table = mt9p012_vcm_id,
	.probe  = mt9p012_vcm_probe,
	.remove = __exit_p(mt9p012_vcm_remove),
	.driver = {
		.name = "mt9p012_vcm",
	},
};

static int32_t mt9p012_vcm_init(void)
{
	int32_t rc = 0;

	CDBG("mt9p012_vcm_init called\n");

	rc = i2c_add_driver(&mt9p012_vcm_driver);
	if (IS_ERR_VALUE(rc))
		goto init_fail;

	return rc;

init_fail:
	CDBG("mt9p012_vcm_init failed\n");
	return rc;
}

void mt9p012_vcm_exit(void)
{
	i2c_del_driver(&mt9p012_vcm_driver);
}

int mt9p012_vcm_probe_init(void *dev, void *ctrl)
{
	int rc = 0;
	struct msm_camera_sensor_info *info =
		(struct msm_camera_sensor_info *)dev;

	struct msm_sensor_ctrl_t *s = (struct msm_sensor_ctrl_t *)ctrl;

	rc = mt9p012_vcm_init();
	if (rc < 0)
		goto probe_done;

	msm_camio_clk_rate_set(MT9P012_VCM_DEFAULT_CLOCK_RATE);
	S_WAIT(20);

	rc = mt9p012_vcm_probe_init_sensor(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = mt9p012_vcm_sensor_open_init;
	s->s_release = mt9p012_vcm_sensor_release;
	s->s_config  = mt9p012_vcm_sensor_config;
	mt9p012_vcm_probe_init_done(info);

probe_done:
	CDBG("%s: rc = %d\n", __func__, rc);
	return rc;
}
