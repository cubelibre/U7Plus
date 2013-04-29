
/*
 *  Linux logo to be displayed on boot
 *
 *  Copyright (C) 1996 Larry Ewing (lewing@isc.tamu.edu)
 *  Copyright (C) 1996,1998 Jakub Jelinek (jj@sunsite.mff.cuni.cz)
 *  Copyright (C) 2001 Greg Banks <gnb@alphalink.com.au>
 *  Copyright (C) 2001 Jan-Benedict Glaw <jbglaw@lug-owl.de>
 *  Copyright (C) 2003 Geert Uytterhoeven <geert@linux-m68k.org>
 */

#include <linux/wisky.h>
#include <linux/linux_logo.h>
#include <linux/stddef.h>
#include <linux/module.h>

#ifdef CONFIG_M68K
#include <asm/setup.h>
#endif

#ifdef CONFIG_MIPS
#include <asm/bootinfo.h>
#endif

#include "wisky_logo.h"

static int nologo;
module_param(nologo, bool, 0);
MODULE_PARM_DESC(nologo, "Disables startup logo");

//extern const struct linux_logo logo_cruz_clut224;

//固件工厂工具解包密码:rkdroid
const unsigned char password[32] = {
    0x52, 0x4b, 0x20, 0x6c,
    0x6f, 0x67, 0x6f, 0x20,
    0x70, 0x61, 0x73, 0x73,
    0x77, 0x6f, 0x72, 0x64,

    0x31, 0x57, 0x8d, 0xeb,
    0x18, 0x4b, 0xa9, 0x41,
    0xd9, 0x47, 0xea, 0x2f,
    0x7e, 0x60, 0xb1, 0x67
};

/* logo's are marked __initdata. Use __init_refok to tell
 * modpost that it is intended that this function uses data
 * marked __initdata.
 */
 extern struct linux_logo logo_charge4_clut224;
const struct linux_logo * __init_refok fb_find_logo(int depth)
{
        struct linux_logo *logo = NULL;
	const struct linux_logo *m_logo = NULL;
	static show_cnt = 0;

	if (nologo)
		return NULL;

	if (depth >= 1) {
#ifdef CONFIG_LOGO_LINUX_MONO
		/* Generic Linux logo */
		logo = &logo_linux_mono;
#endif
#ifdef CONFIG_LOGO_SUPERH_MONO
		/* SuperH Linux logo */
		logo = &logo_superh_mono;
#endif
	}
	
	if (depth >= 4) {
#ifdef CONFIG_LOGO_LINUX_VGA16
		/* Generic Linux logo */
		logo = &logo_linux_vga16;
#endif
#ifdef CONFIG_LOGO_BLACKFIN_VGA16
		/* Blackfin processor logo */
		logo = &logo_blackfin_vga16;
#endif
#ifdef CONFIG_LOGO_SUPERH_VGA16
		/* SuperH Linux logo */
		logo = &logo_superh_vga16;
#endif
	}
	
	if (depth >= 8) {
#ifdef CONFIG_LOGO_LINUX_CLUT224
		/* Generic Linux logo */
		logo = &logo_linux_clut224;
#endif
#ifdef CONFIG_LOGO_G3_CLUT224
		/* Generic Linux logo */
		logo = &logo_g3_clut224;
#endif
#ifdef CONFIG_LOGO_BLACKFIN_CLUT224
		/* Blackfin Linux logo */
		logo = &logo_blackfin_clut224;
#endif
#ifdef CONFIG_LOGO_DEC_CLUT224
		/* DEC Linux logo on MIPS/MIPS64 or ALPHA */
		logo = &logo_dec_clut224;
#endif
#ifdef CONFIG_LOGO_MAC_CLUT224
		/* Macintosh Linux logo on m68k */
		if (MACH_IS_MAC)
			logo = &logo_mac_clut224;
#endif
#ifdef CONFIG_LOGO_PARISC_CLUT224
		/* PA-RISC Linux logo */
		logo = &logo_parisc_clut224;
#endif
#ifdef CONFIG_LOGO_SGI_CLUT224
		/* SGI Linux logo on MIPS/MIPS64 and VISWS */
		logo = &logo_sgi_clut224;
#endif
#ifdef CONFIG_LOGO_SUN_CLUT224
		/* Sun Linux logo */
		logo = &logo_sun_clut224;
#endif
#ifdef CONFIG_LOGO_SUPERH_CLUT224
		/* SuperH Linux logo */
		logo = &logo_superh_clut224;
#endif
#ifdef CONFIG_LOGO_M32R_CLUT224
		/* M32R Linux logo */
		logo = &logo_m32r_clut224;
#endif
#ifdef CONFIG_LOGO_CRUZ_CLUT224
                logo = &logo_cruz_clut224;
#endif

#if defined(WISKY_LOGO_LINUX)
		logo = &logo_linux_clut224;
#elif defined(WISKY_LOGO_PLOYER_1024X768)
		logo = &logo_ployer_1024x768;
#elif defined(WISKY_LOGO_PLOYER_MOMO9_800X480)
		logo = &logo_ployer_momo9_800x480;		
#elif defined(WISKY_LOGO_TECLAST_1024X600_H)
		logo = &logo_teclast_1024x600_h;
#elif defined(WISKY_LOGO_TECLAST_768X1024)
		logo = &logo_teclast_768x1024;
#elif defined(WISKY_LOGO_DANEW_DSLIDE972)
		logo = &logo_danew_dslide972;
#elif defined(WISKY_LOGO_EXPLAY_768X1024)	
		logo = &logo_explay_768x1024;
#elif defined(WISKY_LOGO_EXPLAY_SURFER801_1024x768)	
		logo = &logo_explay_surfer801_1024x768;		
#elif defined(WISKY_LOGO_TIANJIAO_MID8500)
		logo = &logo_tianjiao_mid8500;
#elif defined(WISKY_LOGO_DNS_800x480)
		logo = &logo_dns_800x480;
#elif defined(WISKY_LOGO_DNS_1024x600)
		logo = &logo_dns_1024x600;		
#elif defined(WISKY_LOGO_DNS_1024x768)
		logo = &logo_dns_1024x768;	
#elif defined(WISKY_LOGO_TECLAST_480X800)
		logo = &logo_teclast_480x800;
#elif defined(WISKY_LOGO_MINGXUAN_1024x768)
		logo = &logo_mingxuan_1024x768;		
#elif defined(WISKY_LOGO_TECLAST_P98_768X1024)
		logo = &logo_teclast_p98_768x1024;		
#elif defined(WISKY_LOGO_POCKETBOOK_U7PLUS_800X1280)
		logo = &logo_pocketbook_u7plus_800x1280;		

#else
		logo = &logo_linux_clut224;
#endif

		if(0 == show_cnt){
			show_cnt++;
			logo->width = ((logo->data[0] << 8) + logo->data[1]);
			logo->height = ((logo->data[2] << 8) + logo->data[3]);
			logo->clutsize = logo->clut[0];
			logo->data += 4;
			logo->clut += 1;
		}

	}

	m_logo = logo;
	return m_logo;
	
}
EXPORT_SYMBOL_GPL(fb_find_logo);
