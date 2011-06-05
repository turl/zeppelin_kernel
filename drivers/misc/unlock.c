/*
 * Unlock by Yakk
 * See http://forum.androidfan.ru/index.php?showtopic=5014
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <asm/io.h>
#include <asm/uaccess.h>

static int __init patcher_init(void)
{
	long *pa;

	pa=ioremap(0x032817C8,4);  // patches in check of security consistency
	pa[0]=0x2200E01F;
	iounmap(pa);
	pa=ioremap(0x032818B4,4);
	pa[0]=0x5D31E007;
	iounmap(pa);

	pa=ioremap(0x03281AA4,4);  //patch after key comparison
	pa[0]=0x20002000;
	iounmap(pa);

	pa=ioremap(0x03281B78, 4);  //patches in counters updates
	pa[0]=0xE01A2800;
	iounmap(pa);
	pa=ioremap(0x03281BF0, 4);
	pa[0]=0x210ABDF0;
	iounmap(pa);

	
	return 0;
}

static void __exit patcher_exit(void)
{
}

module_init(patcher_init);
module_exit(patcher_exit);

