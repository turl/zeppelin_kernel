/*
 * Copyright (C) 1997-2008 Alacritech, Inc. All rights reserved
 *
 * sxgphycode.h:
 *
 * This file PHY microcode and register initialization data.
 */

/**********************************************************************
 * PHY Microcode
 *
 * The following contains both PHY microcode and PHY register
 * initialization data.  It is specific to both the PHY and the
 * type of transceiver.
 *
 **********************************************************************/

/*
 * Download for AEL2005C PHY with SR/LR transceiver (10GBASE-SR or 10GBASE-LR)
 */
static struct PHY_UCODE PhyUcode[] = {
	/*
	 * NOTE:  An address of 0 is a special case.  When the download routine
	 * sees an address of 0, it does not write to the PHY.  Instead, it
	 * delays the download.  The length of the delay (in ms) is given in
	 * the data field.
	 *
	 * Delays are required at certain points.
	 */

	/*
	 * Platform-specific MDIO Patches:
	 * (include patches for 10G RX polarity flip, 50Mhz Synth, etc)
	 */
	/* Addr, Data */
	{0xc017, 0xfeb0},	/* flip RX_LOS polarity (mandatory */
	/*  patch for SFP+ applications) */
	{0xC001, 0x0428},	/* flip RX serial polarity */

	{0xc013, 0xf341},	/* invert lxmit clock (mandatory patch) */
	{0xc210, 0x8000},	/* reset datapath (mandatory patch) */
	{0xc210, 0x8100},	/* reset datapath (mandatory patch) */
	{0xc210, 0x8000},	/* reset datapath (mandatory patch) */
	{0xc210, 0x0000},	/* reset datapath (mandatory patch) */
	{0x0000, 0x0032},	/* wait for 50ms for datapath reset to */
	/* complete. (mandatory patch) */

	/* Configure the LED's */
	{0xc214, 0x0099},	/* configure the LED drivers */
	{0xc216, 0x5f5f},	/* configure the Activity LED */
	{0xc217, 0x33ff},	/* configure the Link LED */

	/* Transceiver-specific MDIO Patches: */
	{0xc010, 0x448a},	/* (bit 14) mask out high BER input from the */
	/* LOS signal in 1.000A */
	/* (mandatory patch for SR code) */
	{0xc003, 0x0181},	/* (bit 7) enable the CDR inc setting in */
	/* 1.C005 (mandatory patch for SR code) */

	/* Transceiver-specific Microcontroller Initialization: */
	{0xc04a, 0x5200},	/* activate microcontroller and pause */
	{0x0000, 0x0032},	/* wait 50ms for microcontroller before */
	/* writing in code. */

	/* code block starts here: */
	{0xcc00, 0x2009},
	{0xcc01, 0x3009},
	{0xcc02, 0x27ff},
	{0xcc03, 0x300f},
	{0xcc04, 0x200c},
	{0xcc05, 0x300c},
	{0xcc06, 0x20c4},
	{0xcc07, 0x3c04},
	{0xcc08, 0x6437},
	{0xcc09, 0x20c4},
	{0xcc0a, 0x3c04},
	{0xcc0b, 0x6437},
	{0xcc0c, 0x25c4},
	{0xcc0d, 0x3c54},
	{0xcc0e, 0x6724},
	{0xcc0f, 0x25c4},
	{0xcc10, 0x3c54},
	{0xcc11, 0x6724},
	{0xcc12, 0x2042},
	{0xcc13, 0x3012},
	{0xcc14, 0x1002},
	{0xcc15, 0x2482},
	{0xcc16, 0x3012},
	{0xcc17, 0x1002},
	{0xcc18, 0x2a32},
	{0xcc19, 0x3002},
	{0xcc1a, 0x1002},
	{0xcc1b, 0x200d},
	{0xcc1c, 0x304d},
	{0xcc1d, 0x2862},
	{0xcc1e, 0x3012},
	{0xcc1f, 0x1002},
	{0xcc20, 0x2982},
	{0xcc21, 0x3002},
	{0xcc22, 0x1002},
	{0xcc23, 0x628f},
	{0xcc24, 0x20a4},
	{0xcc25, 0x3004},
	{0xcc26, 0x6438},
	{0xcc27, 0x20a4},
	{0xcc28, 0x3004},
	{0xcc29, 0x6438},
	{0xcc2a, 0x2015},
	{0xcc2b, 0x3005},
	{0xcc2c, 0x5853},
	{0xcc2d, 0x2bd2},
	{0xcc2e, 0x3002},
	{0xcc2f, 0x1342},
	{0xcc30, 0x200c},
	{0xcc31, 0x300c},
	{0xcc32, 0x2ff7},
	{0xcc33, 0x30f7},
	{0xcc34, 0x20c4},
	{0xcc35, 0x3c04},
	{0xcc36, 0x6724},
	{0xcc37, 0x20c4},
	{0xcc38, 0x3c04},
	{0xcc39, 0x6724},
	{0xcc3a, 0x2d32},
	{0xcc3b, 0x3002},
	{0xcc3c, 0x1002},
	{0xcc3d, 0x2008},
	{0xcc3e, 0x3008},
	{0xcc3f, 0x5c83},
	{0xcc40, 0x2d52},
	{0xcc41, 0x3002},
	{0xcc42, 0x1352},
	{0xcc43, 0x2008},
	{0xcc44, 0x3008},
	{0xcc45, 0x5c83},
	{0xcc46, 0x2d32},
	{0xcc47, 0x3002},
	{0xcc48, 0x1352},
	{0xcc49, 0x201c},
	{0xcc4a, 0x300c},
	{0xcc4b, 0x200d},
	{0xcc4c, 0x310d},
	{0xcc4d, 0x2862},
	{0xcc4e, 0x3012},
	{0xcc4f, 0x1002},
	{0xcc50, 0x2ed2},
	{0xcc51, 0x3002},
	{0xcc52, 0x1342},
	{0xcc53, 0x6f72},
	{0xcc54, 0x1002},
	{0xcc55, 0x628f},
	{0xcc56, 0x2514},
	{0xcc57, 0x3c64},
	{0xcc58, 0x6436},
	{0xcc59, 0x2514},
	{0xcc5a, 0x3c64},
	{0xcc5b, 0x6436},
	{0xcc5c, 0x2fa4},
	{0xcc5d, 0x3cd4},
	{0xcc5e, 0x6624},
	{0xcc5f, 0x2fa4},
	{0xcc60, 0x3cd4},
	{0xcc61, 0x6624},
	{0xcc62, 0x2f45},
	{0xcc63, 0x3015},
	{0xcc64, 0x5653},
	{0xcc65, 0x2eb2},
	{0xcc66, 0x3002},
	{0xcc67, 0x13d2},
	{0xcc68, 0x2ed2},
	{0xcc69, 0x3002},
	{0xcc6a, 0x1002},
	{0xcc6b, 0x6f72},
	{0xcc6c, 0x1002},
	{0xcc6d, 0x628f},
	{0xcc6e, 0x2602},
	{0xcc6f, 0x3012},
	{0xcc70, 0x1002},
	{0xcc71, 0x200d},
	{0xcc72, 0x320d},
	{0xcc73, 0x2862},
	{0xcc74, 0x3012},
	{0xcc75, 0x1002},
	{0xcc76, 0x25c4},
	{0xcc77, 0x3c54},
	{0xcc78, 0x6437},
	{0xcc79, 0x25c4},
	{0xcc7a, 0x3c54},
	{0xcc7b, 0x6437},
	{0xcc7c, 0x20c4},
	{0xcc7d, 0x3c04},
	{0xcc7e, 0x6724},
	{0xcc7f, 0x20c4},
	{0xcc80, 0x3c04},
	{0xcc81, 0x6724},
	{0xcc82, 0x6f72},
	{0xcc83, 0x1002},
	{0xcc84, 0x628f},
	{0xcc85, 0x26f2},
	{0xcc86, 0x3012},
	{0xcc87, 0x1002},
	{0xcc88, 0xc503},
	{0xcc89, 0xd5d5},
	{0xcc8a, 0xc600},
	{0xcc8b, 0x2a6d},
	{0xcc8c, 0xc601},
	{0xcc8d, 0x2a4c},
	{0xcc8e, 0xc602},
	{0xcc8f, 0x0111},
	{0xcc90, 0xc60c},
	{0xcc91, 0x5900},
	{0xcc92, 0xc710},
	{0xcc93, 0x0700},
	{0xcc94, 0xc718},
	{0xcc95, 0x0700},
	{0xcc96, 0xc720},
	{0xcc97, 0x4700},
	{0xcc98, 0xc801},
	{0xcc99, 0x7f50},
	{0xcc9a, 0xc802},
	{0xcc9b, 0x7760},
	{0xcc9c, 0xc803},
	{0xcc9d, 0x7fce},
	{0xcc9e, 0xc804},
	{0xcc9f, 0x5700},
	{0xcca0, 0xc805},
	{0xcca1, 0x5f11},
	{0xcca2, 0xc806},
	{0xcca3, 0x4751},
	{0xcca4, 0xc807},
	{0xcca5, 0x57e1},
	{0xcca6, 0xc808},
	{0xcca7, 0x2700},
	{0xcca8, 0xc809},
	{0xcca9, 0x0000},
	{0xccaa, 0xc821},
	{0xccab, 0x0002},
	{0xccac, 0xc822},
	{0xccad, 0x0014},
	{0xccae, 0xc832},
	{0xccaf, 0x1186},
	{0xccb0, 0xc847},
	{0xccb1, 0x1e02},
	{0xccb2, 0xc013},
	{0xccb3, 0xf341},
	{0xccb4, 0xc01a},
	{0xccb5, 0x0446},
	{0xccb6, 0xc024},
	{0xccb7, 0x1000},
	{0xccb8, 0xc025},
	{0xccb9, 0x0a00},
	{0xccba, 0xc026},
	{0xccbb, 0x0c0c},
	{0xccbc, 0xc027},
	{0xccbd, 0x0c0c},
	{0xccbe, 0xc029},
	{0xccbf, 0x00a0},
	{0xccc0, 0xc030},
	{0xccc1, 0x0a00},
	{0xccc2, 0xc03c},
	{0xccc3, 0x001c},
	{0xccc4, 0xc005},
	{0xccc5, 0x7a06},
	{0xccc6, 0x0000},
	{0xccc7, 0x0000},
	{0xccc8, 0x628f},
	{0xccc9, 0x26f2},
	{0xccca, 0x3012},
	{0xcccb, 0x1002},
	{0xcccc, 0xc620},
	{0xcccd, 0x0000},
	{0xccce, 0xc621},
	{0xcccf, 0x003f},
	{0xccd0, 0xc622},
	{0xccd1, 0x0000},
	{0xccd2, 0xc623},
	{0xccd3, 0x0000},
	{0xccd4, 0xc624},
	{0xccd5, 0x0000},
	{0xccd6, 0xc625},
	{0xccd7, 0x0000},
	{0xccd8, 0xc627},
	{0xccd9, 0x0000},
	{0xccda, 0xc628},
	{0xccdb, 0x0000},
	{0xccdc, 0xc62c},
	{0xccdd, 0x0000},
	{0xccde, 0x0000},
	{0xccdf, 0x0000},
	{0xcce0, 0x628f},
	{0xcce1, 0xd019},
	{0xcce2, 0x26f2},
	{0xcce3, 0x3012},
	{0xcce4, 0x1002},
	{0xcce5, 0xc210},
	{0xcce6, 0x8000},
	{0xcce7, 0xc210},
	{0xcce8, 0x8010},
	{0xcce9, 0xc210},
	{0xccea, 0x8000},
	{0xcceb, 0xc210},
	{0xccec, 0x0000},
	{0xcced, 0x0000},
	{0xccee, 0x0000},
	{0xccef, 0x8221},
	{0xccf0, 0x2752},
	{0xccf1, 0x3012},
	{0xccf2, 0x1002},
	{0xccf3, 0x6f72},
	{0xccf4, 0x1002},
	{0xccf5, 0x2806},
	{0xccf6, 0x3006},
	{0xccf7, 0x2007},
	{0xccf8, 0x3cc7},
	{0xccf9, 0xe161},
	{0xccfa, 0xc171},
	{0xccfb, 0x6134},
	{0xccfc, 0x6135},
	{0xccfd, 0x5453},
	{0xccfe, 0x2858},
	{0xccff, 0x3018},
	{0xcd00, 0x1348},
	{0xcd01, 0x6524},
	{0xcd02, 0x27b8},
	{0xcd03, 0x3018},
	{0xcd04, 0x1008},
	{0xcd05, 0x1002},
	{0xcd06, 0x628f},
	{0xcd07, 0x5dd3},
	{0xcd08, 0x2906},
	{0xcd09, 0x3016},
	{0xcd0a, 0x1306},
	{0xcd0b, 0x2ff7},
	{0xcd0c, 0x30f7},
	{0xcd0d, 0x60b7},
	{0xcd0e, 0xdffd},
	{0xcd0f, 0x0008},
	{0xcd10, 0x6f72},
	{0xcd11, 0x1002},
	{0xcd12, 0x0000},
	{0xcdff, 0x0a01},
	/* end of code block */

	/* Unpause the microcontroller to start program */
	{0xca00, 0x0080},
	{0xca12, 0x0000},
	{0x0000, 0x000A},	/* wait 10ms just to be safe */
	{0xffff, 0xffff}	/* table terminator */
};
