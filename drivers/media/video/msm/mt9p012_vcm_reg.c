/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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

#include "mt9p012_vcm.h"

/*Micron settings from Applications for lower power consumption.*/
struct reg_struct const mt9p012_vcm_reg_pat[2] = {
	{ /* Preview */
		/* vt_pix_clk_div          REG=0x0300 */
		7,  /* 5 */

		/* vt_sys_clk_div          REG=0x0302 */
		1,

		/* pre_pll_clk_div         REG=0x0304 */
	    5,

		/* pll_multiplier          REG=0x0306 */
		76,

		/* op_pix_clk_div          REG=0x0308 */
		8,  /* 10 */

		/* op_sys_clk_div          REG=0x030A */
		1,

		/* scale_m                 REG=0x0404 */
		16,

		/* row_speed               REG=0x3016 */
		0x0111,

		/* x_addr_start            REG=0x3004 */
		8,

		/* x_addr_end              REG=0x3008 */
		2597,

		/* y_addr_start            REG=0x3002 */
		8,

		/* y_addr_end              REG=0x3006 */
		1949,

		/* read_mode               REG=0x3040 */
		0x14C3,

		/* x_output_size           REG=0x034C */
		1296,

		/* y_output_size           REG=0x034E */
		972,

		/* line_length_pck         REG=0x300C */
		4062,

		/* frame_length_lines      REG=0x300A */
		1055,  /* 1057, */

		/* coarse_integration_time REG=0x3012 */
		1000,

		/* fine_integration_time   REG=0x3014 */
		1794,

		/* skew                    REG=0x309E */
		0x5D00
	},
	{ /*Snapshot*/
		/* vt_pix_clk_div          REG=0x0300 */
		7,

		/* vt_sys_clk_div          REG=0x0302 */
		1,

		/* pre_pll_clk_div         REG=0x0304 */
	    5,

		/* pll_multiplier          REG=0x0306
		 * 60 for 10fps snapshot */
		0x004C,

		/* op_pix_clk_div          REG=0x0308 */
		8,

		/* op_sys_clk_div          REG=0x030A */
		1,

		/* scale_m                 REG=0x0404 */
		16,

		/* row_speed               REG=0x3016 */
		0x0111,

		/* x_addr_start            REG=0x3004 */
		0,

		/* x_addr_end              REG=0x3008 */
		0x0A2F,

		/* y_addr_start            REG=0x3002 */
		0,

		/* y_addr_end              REG=0x3006 */
		0x07A7,

		/* read_mode               REG=0x3040 */
		0x0041,

		/* x_output_size           REG=0x034C */
		0x0A30,

		/* y_output_size           REG=0x034E */
		0x07A8,

		/* line_length_pck         REG=0x300C */
		0x0EE0, /* 15 FPS 4889: 12 FPS */

		/* frame_length_lines      REG=0x300A //10 fps snapshot */
		0x07FD,

		/* coarse_integration_time REG=0x3012 */
		0x0834,

		/* fine_integration_time   REG=0x3014 */
		0x0372,

		/* skew                    REG=0x309E */
		0x5D00
	}
};


struct mt9p012_vcm_i2c_reg_conf const mt9p012_vcm_test_tbl[] = {
	{0x3044, 0x0544 & 0xFBFF},
	{0x30CA, 0x0004 | 0x0001},
	{0x30D4, 0x9020 & 0x7FFF},
	{0x31E0, 0x0003 & 0xFFFE},
	{0x3180, 0x91FF & 0x7FFF},
	{0x301A, (0x10CC | 0x8000) & 0xFFF7},
	{0x301E, 0x0000},
	{0x3780, 0x0000},
};


struct mt9p012_vcm_i2c_reg_conf const mt9p012_vcm_lc_tbl[] = {
	{0x360A, 0x0190},
	{0x360C, 0x4CAC},
	{0x360E, 0x2891},
	{0x3610, 0xB24D},
	{0x3612, 0x80D1},
	{0x364A, 0x4D6A},
	{0x364C, 0x766C},
	{0x364E, 0xDDED},
	{0x3650, 0x13CA},
	{0x3652, 0x184D},
	{0x368A, 0x1D31},
	{0x368C, 0x8650},
	{0x368E, 0x7110},
	{0x3690, 0x2A71},
	{0x3692, 0x88F4},
	{0x36CA, 0xDECD},
	{0x36CC, 0xA96F},
	{0x36CE, 0x12D0},
	{0x36D0, 0x184C},
	{0x36D2, 0x560F},
	{0x370A, 0x7DEE},
	{0x370C, 0x0E92},
	{0x370E, 0x94F5},
	{0x3710, 0xEB52},
	{0x3712, 0x3896},
	{0x3600, 0x00D0},
	{0x3602, 0x86EC},
	{0x3604, 0x04B1},
	{0x3606, 0x672E},
	{0x3608, 0xDD70},
	{0x3640, 0xE7E8},
	{0x3642, 0x14CC},
	{0x3644, 0x890E},
	{0x3646, 0xD58C},
	{0x3648, 0x63CE},
	{0x3680, 0x6730},
	{0x3682, 0x7B6E},
	{0x3684, 0x764E},
	{0x3686, 0xEC90},
	{0x3688, 0xB1D3},
	{0x36C0, 0x822D},
	{0x36C2, 0xB24F},
	{0x36C4, 0x1C10},
	{0x36C6, 0x21AE},
	{0x36C8, 0xA06F},
	{0x3700, 0x5FEE},
	{0x3702, 0x51F0},
	{0x3704, 0xC934},
	{0x3706, 0x8150},
	{0x3708, 0x0276},
	{0x3614, 0x0110},
	{0x3616, 0xD72B},
	{0x3618, 0x4270},
	{0x361A, 0x1E0E},
	{0x361C, 0x85F0},
	{0x3654, 0x108B},
	{0x3656, 0x032D},
	{0x3658, 0x4A8E},
	{0x365A, 0xA1EC},
	{0x365C, 0xD16D},
	{0x3694, 0x4A10},
	{0x3696, 0x0C8C},
	{0x3698, 0x4C70},
	{0x369A, 0xB9AF},
	{0x369C, 0xBA73},
	{0x36D4, 0x4CEF},
	{0x36D6, 0xB04F},
	{0x36D8, 0x8990},
	{0x36DA, 0x69EE},
	{0x36DC, 0xDFD1},
	{0x3714, 0x498F},
	{0x3716, 0x4EB1},
	{0x3718, 0xCC94},
	{0x371A, 0xB792},
	{0x371C, 0x7A55},
	{0x361E, 0x0210},
	{0x3620, 0x0B8C},
	{0x3622, 0x0B91},
	{0x3624, 0x8E6D},
	{0x3626, 0xF750},
	{0x365E, 0x158B},
	{0x3660, 0x7748},
	{0x3662, 0x2ECF},
	{0x3664, 0x68AB},
	{0x3666, 0xEC0F},
	{0x369E, 0x7D30},
	{0x36A0, 0xAD0F},
	{0x36A2, 0xD5CD},
	{0x36A4, 0x29B0},
	{0x36A6, 0x9933},
	{0x36DE, 0x494F},
	{0x36E0, 0xA20E},
	{0x36E2, 0xF530},
	{0x36E4, 0xC2E9},
	{0x36E6, 0xF3EC},
	{0x371E, 0x3C8D},
	{0x3720, 0x2731},
	{0x3722, 0xCBB4},
	{0x3724, 0x8791},
	{0x3726, 0x7F95},
	{0x3782, 0x04E0},
	{0x3784, 0x03B4},
	{0x3780, 0x8000},
};


struct mt9p012_vcm_i2c_reg_conf const mt9p012_vcm_rolloff_tbl[] = {
	{0x360A, 0x7FEF},
	{0x360C, 0x232C},
	{0x360E, 0x7050},
	{0x3610, 0xF3CC},
	{0x3612, 0x89D1},
	{0x364A, 0xBE0D},
	{0x364C, 0x9ACB},
	{0x364E, 0x2150},
	{0x3650, 0xB26B},
	{0x3652, 0x9511},
	{0x368A, 0x2151},
	{0x368C, 0x00AD},
	{0x368E, 0x8334},
	{0x3690, 0x478E},
	{0x3692, 0x0515},
	{0x36CA, 0x0710},
	{0x36CC, 0x452D},
	{0x36CE, 0xF352},
	{0x36D0, 0x190F},
	{0x36D2, 0x4413},
	{0x370A, 0xD112},
	{0x370C, 0xF50F},
	{0x370E, 0x6375},
	{0x3710, 0xDC11},
	{0x3712, 0xD776},
	{0x3600, 0x1750},
	{0x3602, 0xF0AC},
	{0x3604, 0x4711},
	{0x3606, 0x07CE},
	{0x3608, 0x96B2},
	{0x3640, 0xA9AE},
	{0x3642, 0xF9AC},
	{0x3644, 0x39F1},
	{0x3646, 0x016F},
	{0x3648, 0x8AB2},
	{0x3680, 0x1752},
	{0x3682, 0x70F0},
	{0x3684, 0x83F5},
	{0x3686, 0x8392},
	{0x3688, 0x1FD6},
	{0x36C0, 0x1131},
	{0x36C2, 0x3DAF},
	{0x36C4, 0x89B4},
	{0x36C6, 0xA391},
	{0x36C8, 0x1334},
	{0x3700, 0xDC13},
	{0x3702, 0xD052},
	{0x3704, 0x5156},
	{0x3706, 0x1F13},
	{0x3708, 0x8C38},
	{0x3614, 0x0050},
	{0x3616, 0xBD4C},
	{0x3618, 0x41B0},
	{0x361A, 0x660D},
	{0x361C, 0xC590},
	{0x3654, 0x87EC},
	{0x3656, 0xE44C},
	{0x3658, 0x302E},
	{0x365A, 0x106E},
	{0x365C, 0xB58E},
	{0x3694, 0x0DD1},
	{0x3696, 0x2A50},
	{0x3698, 0xC793},
	{0x369A, 0xE8F1},
	{0x369C, 0x4174},
	{0x36D4, 0x01EF},
	{0x36D6, 0x06CF},
	{0x36D8, 0x8D91},
	{0x36DA, 0x91F0},
	{0x36DC, 0x52EF},
	{0x3714, 0xA6D2},
	{0x3716, 0xA312},
	{0x3718, 0x2695},
	{0x371A, 0x3953},
	{0x371C, 0x9356},
	{0x361E, 0x7EAF},
	{0x3620, 0x2A4C},
	{0x3622, 0x49F0},
	{0x3624, 0xF1EC},
	{0x3626, 0xC670},
	{0x365E, 0x8E0C},
	{0x3660, 0xC2A9},
	{0x3662, 0x274F},
	{0x3664, 0xADAB},
	{0x3666, 0x8EF0},
	{0x369E, 0x09B1},
	{0x36A0, 0xAA2E},
	{0x36A2, 0xC3D3},
	{0x36A4, 0x7FAF},
	{0x36A6, 0x3F34},
	{0x36DE, 0x4C8F},
	{0x36E0, 0x886E},
	{0x36E2, 0xE831},
	{0x36E4, 0x1FD0},
	{0x36E6, 0x1192},
	{0x371E, 0xB952},
	{0x3720, 0x6DCF},
	{0x3722, 0x1B55},
	{0x3724, 0xA112},
	{0x3726, 0x82F6},
	{0x3782, 0x0510},
	{0x3784, 0x0390},
	{0x3780, 0x8000},
};


struct mt9p012_vcm_reg_t mt9p012_vcm_regs = {
	.reg_pat = &mt9p012_vcm_reg_pat[0],
	.reg_pat_size = ARRAY_SIZE(mt9p012_vcm_reg_pat),
	.ttbl = &mt9p012_vcm_test_tbl[0],
	.ttbl_size = ARRAY_SIZE(mt9p012_vcm_test_tbl),
	.lctbl = &mt9p012_vcm_lc_tbl[0],
	.lctbl_size = ARRAY_SIZE(mt9p012_vcm_lc_tbl),
	.rftbl = &mt9p012_vcm_rolloff_tbl[0],
	.rftbl_size = ARRAY_SIZE(mt9p012_vcm_rolloff_tbl)
};


