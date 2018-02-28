/*
 * arch/arm/mach-at91/at91sam9263_devices.c
 *
 *  Copyright (C) 2007 Atmel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/fb.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>

// NOR Flash
#include <linux/mtd/physmap.h>

#include <video/atmel_lcdc.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_shdwc.h>

#include "sam9_smc.h"
#include "generic.h"


static void __init ek_map_io(void)
{
	/* Initialize processor: 16.367 MHz crystal */
	at91sam9263_initialize(16367660);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);

	/* USART0 on ttyS1. (Rx, Tx, RTS, CTS) */
	//at91_register_uart(AT91SAM9263_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS);
//start
	/* USART0 on ttyAT2 */
	at91_register_uart(AT91SAM9263_ID_US0, 2, 0);
	/* USART1 on ttyAT1 */
	at91_register_uart(AT91SAM9263_ID_US1, 1,ATMEL_UART_RTS);
	/* USART2 on ttyAT3 */
#ifdef DDU
	at91_register_uart(AT91SAM9263_ID_US2, 3, ATMEL_UART_RTS);
#else //BCP
	at91_register_uart(AT91SAM9263_ID_US2, 3, 0);
#endif
//end
	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init ek_init_irq(void)
{
	at91sam9263_init_interrupts(NULL);
}


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
	.vbus_pin	= { AT91_PIN_PA24, AT91_PIN_PA21 },
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	.vbus_pin	= AT91_PIN_PA25,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};


/*
 * ADS7846 Touchscreen
 */
#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
static int ads7843_pendown_state(void)
{
	return !at91_get_gpio_value(AT91_PIN_PA15);	/* Touchscreen PENIRQ */
}

static struct ads7846_platform_data ads_info = {
	.model			= 7843,
	.x_min			= 150,
	.x_max			= 3830,
	.y_min			= 190,
	.y_max			= 3830,
	.vref_delay_usecs	= 100,
	.x_plate_ohms		= 450,
	.y_plate_ohms		= 250,
	.pressure_max		= 15000,
	.debounce_max		= 1,
	.debounce_rep		= 0,
	.debounce_tol		= (~0),
	.get_pendown_state	= ads7843_pendown_state,
};

static void __init ek_add_device_ts(void)
{
	at91_set_B_periph(AT91_PIN_PA15, 1);	/* External IRQ1, with pullup */
	at91_set_gpio_input(AT91_PIN_PA31, 1);	/* Touchscreen BUSY signal */
}
#else
static void __init ek_add_device_ts(void) {
	printk(KERN_INFO "Touch Screen disabled\n");
}
#endif

/*
 * SPI devices.
 */
static struct spi_board_info ek_spi_devices[] = {
#if defined(CONFIG_MTD_AT91_DATAFLASH_CARD)
	{	/* DataFlash card */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
	// dataflash 2 start
	{
		.modalias	= "mtd_dataflash",
		.chip_select	= 1,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
	// dataflash 2 end
#endif
#if defined(CONFIG_TOUCHSCREEN_ADS7846) || defined(CONFIG_TOUCHSCREEN_ADS7846_MODULE)
	{
		.modalias	= "ads7846",
		.chip_select	= 3,
		.max_speed_hz	= 125000 * 16,	/* max sample rate * clocks per sample */
		.bus_num	= 0,
		.platform_data	= &ads_info,
		.irq		= AT91SAM9263_ID_IRQ1,
	},
#endif
};


/*
 * MCI (SD/MMC)
 */
static struct at91_mmc_data __initdata ek_mmc_data = {
	.wire4		= 1,
	.det_pin	= AT91_PIN_PE18,
	.wp_pin		= AT91_PIN_PE19,
//	.vcc_pin	= ... not connected
};


/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PE31,
	.is_rmii	= 1,
};


/*
 * NAND flash
 * updated partition sizes
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
//	{
//		.name	= "Bootstrap",
//		.offset	= 0,
//		.size	= SZ_4M,
//	},
	{
		.name	= "Bootstrap",
		.offset	= 0,
		.size	= 0x20000,
	},
	{
		.name	= "UBoot",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 0x80000,
	},
	{
		.name	= "Env. Config",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 0x160000,
	},
	{
		.name	= "Kernel Backup",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 2 * SZ_1M,
	},
	{
		.name	= "RootFS",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 50 * SZ_1M,
	},
	{
		.name	= "Firmware",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 10 * SZ_1M,
	},
	{
		.name	= "Reserve",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 24 * SZ_1M,
	},
	{
		.name	= "Audio & Fonts",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
//	.det_pin	= ... not connected
	.rdy_pin	= AT91_PIN_PA22,
	.enable_pin	= AT91_PIN_PD15,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_ATMEL_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};

static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 1,
	.ncs_write_setup	= 0,
	.nwe_setup		= 1,

	.ncs_read_pulse		= 3,
	.nrd_pulse		= 3,
	.ncs_write_pulse	= 3,
	.nwe_pulse		= 3,

	.read_cycle		= 5,
	.write_cycle		= 5,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 2,
};

static void __init ek_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (ek_nand_data.bus_width_16)
		ek_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		ek_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &ek_nand_smc_config);

	at91_add_device_nand(&ek_nand_data);
}

/*
 * NOR Flash 
 */
#define URP2_FLASH_BASE AT91_CHIPSELECT_0
#define URP2_FLASH_SIZE 0x400000

static struct mtd_partition __initdata nor_partitions[] = {
//	{
//		.name       = "Parallel flash",
//		.offset     = 0,
//		.size       = MTDPART_SIZ_FULL,
//		//.mask_flags   = MTD_WRITEABLE,    /* read only */
//	}
	{
		.name       = "HW Config",
		.offset     = 0,
		.size       = SZ_1M,
		//.mask_flags   = MTD_WRITEABLE,    /* read only */
	},
	{
		.name	= "Kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_2M,
	},
	{
		.name	= "Store",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct physmap_flash_data urp2_flash_data = {
    .width      = 2,
    .parts      = nor_partitions,
    .nr_parts   = ARRAY_SIZE(nor_partitions),
};

static struct resource __initdata urp2_flash_resources[] = {
    {
        .start      = URP2_FLASH_BASE,
        .end        = URP2_FLASH_BASE + URP2_FLASH_SIZE - 1,
        .flags      = IORESOURCE_MEM,
    }
};

static struct platform_device urp2_flash = {
    .name       = "physmap-flash",
    .id     = 0,
    .dev        = {
                .platform_data  = &urp2_flash_data,
            },
    .resource   = urp2_flash_resources,
    .num_resources  = ARRAY_SIZE(urp2_flash_resources),
};

static void __init ek_add_device_norflash(void)
{
    int ret;
    /*
     * Configure Chip-Select 0 on SMC for NOR Flash AT49BV322D
     */
printk("ek_add_device_norflash()!\n");
/*    at91_sys_write(AT91_SMC_SETUP(0), AT91_SMC_NWESETUP_(1) | AT91_SMC_NCS_WRSETUP_(0) | AT91_SMC_NRDSETUP_(0) | AT91_SMC_NCS_RDSETUP_(0));
    at91_sys_write(AT91_SMC_PULSE(0), AT91_SMC_NWEPULSE_(5) | AT91_SMC_NCS_WRPULSE_(8) | AT91_SMC_NRDPULSE_(8) | AT91_SMC_NCS_RDPULSE_(8));
    at91_sys_write(AT91_SMC_CYCLE(0), AT91_SMC_NWECYCLE_(8) | AT91_SMC_NRDCYCLE_(8));
    at91_sys_write(AT91_SMC_MODE(0), AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_BAT_WRITE | AT91_SMC_DBW_16 | AT91_SMC_TDF_(1));
*/

    at91_sys_write(AT91_SMC_SETUP(0), AT91_SMC_NWESETUP_(1) | AT91_SMC_NCS_WRSETUP_(0) | AT91_SMC_NRDSETUP_(0) | AT91_SMC_NCS_RDSETUP_(0));
    at91_sys_write(AT91_SMC_PULSE(0), AT91_SMC_NWEPULSE_(7) | AT91_SMC_NCS_WRPULSE_(10) | AT91_SMC_NRDPULSE_(10) | AT91_SMC_NCS_RDPULSE_(10));
    at91_sys_write(AT91_SMC_CYCLE(0), AT91_SMC_NWECYCLE_(12) | AT91_SMC_NRDCYCLE_(12));
    at91_sys_write(AT91_SMC_MODE(0), AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_BAT_WRITE | AT91_SMC_DBW_16 | AT91_SMC_TDF_(1));


    // Set VPP on
    //at91_set_gpio_output(AT91_PIN_PA26, 1);

    // Register NOR Flash device
    ret = platform_device_register(&urp2_flash);
printk("ek_add_device_norflash() End: ret=%d\n", ret);

}
// End



/*
 * LCD Controller
 */
#if defined(CONFIG_FB_ATMEL) || defined(CONFIG_FB_ATMEL_MODULE)
// LCD VGA information updates
static struct fb_videomode at91_tft_vga_modes[] = {
	{
		.name		= "08TLM092(COM57T5136GLC)_E @ 60",
		.refresh	= 60,
		.xres		= 320,		.yres		= 240,
		.pixclock	= KHZ2PICOS(4965),

		.left_margin	= 21,		.right_margin	= 95,
		.upper_margin	= 6,		.lower_margin	= 0,
		.hsync_len	= 4,		.vsync_len	= 8,

		.sync		= FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};
// LCD End

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "HIT",
	.monitor	= "TX09D70VM1CCA",

	.modedb		= at91_tft_vga_modes,
	.modedb_len	= ARRAY_SIZE(at91_tft_vga_modes),
	.hfmin		= 15000,
	.hfmax		= 64000,
	.vfmin		= 50,
	.vfmax		= 150,
};

#define AT91SAM9263_DEFAULT_LCDCON2 	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE)

static void at91_lcdc_power_control(int on)
{
	at91_set_gpio_value(AT91_PIN_PA30, on);
}

/* Driver datas */
static struct atmel_lcdfb_info __initdata ek_lcdc_data = {
	.lcdcon_is_backlight		= true,
	.default_bpp			= 16,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9263_DEFAULT_LCDCON2,
	.default_monspecs		= &at91fb_default_monspecs,
	.atmel_lcdfb_power_control	= at91_lcdc_power_control,
	.guard_time			= 1,
};

#else
static struct atmel_lcdfb_info __initdata ek_lcdc_data;
#endif


/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#ifdef DDU //
static struct gpio_keys_button ek_buttons[] = {
	{	/* BP1 */
		.code		= KEY_1,
		.gpio		= AT91_PIN_PE0,
		.active_low	= 0,
		.desc		= "button 1",
		.wakeup		= 1,
		.debounce_interval	= 30,
	},
	{	/* BP2 */
		.code		= KEY_2,
		.gpio		= AT91_PIN_PE1,
		.active_low	= 0,
		.desc		= "button 2",
		.wakeup		= 1,
		.debounce_interval	= 30,
	},
	{	/* BP3 */
		.code		= KEY_3,
		.gpio		= AT91_PIN_PE2,
		.active_low	= 0,
		.desc		= "button 3",
		.wakeup		= 1,
		.debounce_interval	= 30,
	},
	{	/* BP4 */
		.code		= KEY_4, 
		.gpio		= AT91_PIN_PE3,
		.active_low	= 0,
		.desc		= "button 4",
		.wakeup		= 1,
		.debounce_interval	= 30,
	},
	{	/* BP5 */
		.code		= KEY_5,
		.gpio		= AT91_PIN_PE4,
		.active_low	= 0,
		.desc		= "button 5",
		.wakeup		= 0,
		.debounce_interval	= 30,
	},
	{	/* BP6 */
		.code		= KEY_6,
		.gpio		= AT91_PIN_PE5,
		.active_low	= 0,
		.desc		= "button 6",
		.wakeup		= 1,
		.debounce_interval	= 30,
	},
	{       /* BP6 */
                .code           = KEY_7,
                .gpio           = AT91_PIN_PD4,
                .active_low     = 0,
                .desc           = "check 5v",
                .wakeup         = 1,
                .debounce_interval = 0,
        },
};

static struct gpio_keys_platform_data ek_button_data = {
	.buttons	= ek_buttons,
	.nbuttons	= ARRAY_SIZE(ek_buttons),
};

static struct platform_device ek_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ek_button_data,
	}
};

static void __init ek_add_device_buttons(void)
{
	at91_set_GPIO_periph(AT91_PIN_PE0, 1);	/* button 1 */
	at91_set_deglitch(AT91_PIN_PE0, 1);
	at91_set_GPIO_periph(AT91_PIN_PE1, 1);	/* button 2 */
	at91_set_deglitch(AT91_PIN_PE1, 1);
	at91_set_GPIO_periph(AT91_PIN_PE2, 1);	/* button 3 */
	at91_set_deglitch(AT91_PIN_PE2, 1);
	at91_set_GPIO_periph(AT91_PIN_PE3, 1);	/* button 4 */
	at91_set_deglitch(AT91_PIN_PE3, 1);
	at91_set_GPIO_periph(AT91_PIN_PE4, 1);	/* button 5 */
	at91_set_deglitch(AT91_PIN_PE4, 1);
	at91_set_GPIO_periph(AT91_PIN_PE5, 1);	/* button 6 */
	at91_set_deglitch(AT91_PIN_PE5, 1);
        at91_set_GPIO_periph(AT91_PIN_PD4, 1);  /* check 5v */

	platform_device_register(&ek_button_device);
}
#else
//BCP 
static void __init ek_add_device_buttons(void) {}
#endif
#else
static void __init ek_add_device_buttons(void) {}
#endif


/*
 * AC97
 */
static struct atmel_ac97_data ek_ac97_data = {
	.reset_pin	= AT91_PIN_PA13,
};


/*
 * LEDs ... these could all be PWM-driven, for variable brightness
 */
static struct gpio_led ek_leds[] = {
	{	/* "right" led, green, userled2 (could be driven by pwm2) */
		.name			= "ds2",
		.gpio			= AT91_PIN_PC29,
		.active_low		= 1,
		.default_trigger	= "nand-disk",
	},
	{	/* "power" led, yellow (could be driven by pwm0) */
		.name			= "ds3",
		.gpio			= AT91_PIN_PB7,
		.default_trigger	= "heartbeat",
	}
};

/*
 * PWM Leds
 */
static struct gpio_led ek_pwm_led[] = {
	/* For now only DS1 is PWM-driven (by pwm1) */
	{
		.name			= "ds1",
		.gpio			= 1,	/* is PWM channel number */
		.active_low		= 1,
		.default_trigger	= "none",
	}
};

/*
 * additional part
 * RTC i2c
 * 
 */
static struct i2c_board_info __initdata urp2_i2c_devices[] = {
	{
		I2C_BOARD_INFO("s35390a", 0x30),
	},
};
// RTC END

static void __init ek_board_init(void)
{
#ifdef DDU
	printk(KERN_INFO "######## DDU Kernel ########\n");
#else
	printk(KERN_INFO "######## BCP Kernel ########\n");
#endif
	//Start
	urp2_add_ext_uart();
	//end
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
	/* USB Device */
	at91_add_device_udc(&ek_udc_data);
	/* SPI */
	//at91_set_gpio_output(AT91_PIN_PE20, 1);		/* select spi0 clock */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
	/* Touchscreen */
	ek_add_device_ts();
	/* MMC */
#ifndef DDU
	at91_add_device_mmc(1, &ek_mmc_data);
#endif
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* NAND */
	ek_add_device_nand();

	// add nor flash
	ek_add_device_norflash();

	/* I2C */
	// Add I2C Devices
	//at91_add_device_i2c(NULL, 0);
	at91_add_device_i2c(urp2_i2c_devices, ARRAY_SIZE(urp2_i2c_devices));
	/* LCD Controller */
	at91_add_device_lcdc(&ek_lcdc_data);
	/* Push Buttons */
	ek_add_device_buttons();
	/* AC97 */
	at91_add_device_ac97(&ek_ac97_data);
	/* LEDs */
	at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
	at91_pwm_leds(ek_pwm_led, ARRAY_SIZE(ek_pwm_led));
	/* shutdown controller, wakeup button (5 msec low) */
	at91_sys_write(AT91_SHDW_MR, AT91_SHDW_CPTWK0_(10) | AT91_SHDW_WKMODE0_LOW
				| AT91_SHDW_RTTWKEN);
}

MACHINE_START(AT91SAM9263EK, "Atmel AT91SAM9263-EK")
	/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
