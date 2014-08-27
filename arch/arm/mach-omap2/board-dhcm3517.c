/*
 * linux/arch/arm/mach-omap2/board-dhcm3517.c
 *
 * Copyright (C) 2011 dh-electronics
 *
 * Based on mach-omap2/board-am3517evm.c
 * Based on mach-omap2/board-omap3evm.c
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/davinci_emac.h>
#include <linux/mmc/host.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/export.h>

#include <linux/i2c/tsc2004.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/vram.h>
#include <plat/nand.h>
#include <plat/gpmc.h>
#include <plat/mcspi.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <linux/pwm_backlight.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "board-flash.h"
#include "common-board-devices.h"

#define DHCM3517_MDIO_FREQUENCY	(1000000)

static struct mdio_platform_data dhcm3517_mdio_pdata = {
	.bus_freq	= DHCM3517_MDIO_FREQUENCY,
};

static struct resource am3517_mdio_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end    = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET +
			  SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device am3517_mdio_device = {
	.name		= "davinci_mdio",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(am3517_mdio_resources),
	.resource	= am3517_mdio_resources,
	.dev.platform_data = &dhcm3517_mdio_pdata,
};

static struct emac_platform_data dhcm3517_emac_pdata = {
	.rmii_en	= 1,
};

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x2FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name		= "davinci_emac",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_emac_resources),
	.resource	= am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR |
		AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
		AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

#define ETH_VIO_GPIO 145
static struct gpio dhcm3517_eth_vio_gpios[] __initdata = {

	/* GPIO 145 */
	{ ETH_VIO_GPIO,	 GPIOF_OUT_INIT_HIGH, "eth vio" },
};

static void __init dhcm3517_ethernet_init(struct emac_platform_data *pdata)
{
	unsigned int regval;
	int r;

	/* setup ETH_VIO_GPIO  */
	omap_mux_init_gpio(ETH_VIO_GPIO, OMAP_PIN_OUTPUT);
	r = gpio_request_array(	dhcm3517_eth_vio_gpios,
				ARRAY_SIZE(dhcm3517_eth_vio_gpios));
	if (r) {
		printk(KERN_ERR "ETH: *** failed to get ETH_VIO_GPIO\n");
		return;
	}
	gpio_set_value(ETH_VIO_GPIO, 1);
	gpio_export(ETH_VIO_GPIO, 0);
	printk(KERN_ERR "ETH VIO GPIO ok\n");

	/* platform data init */
	pdata->ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset	= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version			= EMAC_VERSION_2;
	pdata->hw_ram_addr		= AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable		= am3517_enable_ethernet_int;
	pdata->interrupt_disable	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data	= pdata;
	platform_device_register(&am3517_emac_device);
	platform_device_register(&am3517_mdio_device);
	clk_add_alias(NULL, dev_name(&am3517_mdio_device.dev),
		      NULL, &am3517_emac_device.dev);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}

/* DH-Touch (Single-Touch) - configuration of Infrafit IWRK */

#define TMG120_TS_IRQGPIO 155

static struct i2c_board_info dhcm3517_i2c3_boardinfo[] = {
#ifdef CONFIG_TOUCHSCREEN_TMG120_TS
	{
		I2C_BOARD_INFO("tmg120_ts", 0x20),
		.type= "tmg120_ts",
		.irq = TMG120_TS_IRQGPIO,
	},
#endif
};


#ifdef CONFIG_TOUCHSCREEN_TSC2004
#define TSC2004_IRQGPIO		147
static int tsc2004_get_pendown_state(void)
{
	return !gpio_get_value(TSC2004_IRQGPIO);
}

static struct tsc2004_platform_data tsc2004_info = {
	.model = 2004,
	.x_plate_ohms	= 180,
	.get_pendown_state = tsc2004_get_pendown_state,
};
#endif

static struct i2c_board_info __initdata dhcm3517_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic23", 0x1A),
	},
#ifdef CONFIG_TOUCHSCREEN_TSC2004
	{
		I2C_BOARD_INFO("tsc2004", 0x4b),
		.type= "tsc2004",
		.platform_data= &tsc2004_info,
		.irq = OMAP_GPIO_IRQ(TSC2004_IRQGPIO),
	},
#endif
	{
		I2C_BOARD_INFO("rtc-ds1307", 0x6f),
		.type   = "mcp7941x",
	},
};

static int __init dhcm3517_i2c_init(void)
{
#ifdef CONFIG_TOUCHSCREEN_TSC2004
	omap_mux_init_gpio(TSC2004_IRQGPIO, OMAP_PIN_INPUT);
	if (gpio_request_one(TSC2004_IRQGPIO, GPIOF_IN, "TS IRQ") < 0) {
		printk(KERN_ERR "Error request Touchscreen IRQ #%d:\n",
		TSC2004_IRQGPIO);
	} else {
#endif
		omap_register_i2c_bus(1, 400, dhcm3517_i2c1_boardinfo,
				ARRAY_SIZE(dhcm3517_i2c1_boardinfo));
#ifdef CONFIG_TOUCHSCREEN_TSC2004
	}
#endif

	omap_register_i2c_bus(2, 400, NULL, 0);

#ifdef CONFIG_TOUCHSCREEN_TMG120_TS
	omap_mux_init_gpio(TMG120_TS_IRQGPIO, OMAP_PIN_INPUT);
	if (gpio_request_one(TMG120_TS_IRQGPIO, GPIOF_IN, "TS IRQ") < 0) {
		printk(KERN_ERR "Error request Touchscreen IRQ #%d:\n",
		TMG120_TS_IRQGPIO);
	} else {
	        dhcm3517_i2c3_boardinfo[0].irq = gpio_to_irq(TMG120_TS_IRQGPIO);
#endif
	        omap_register_i2c_bus(3, 400, dhcm3517_i2c3_boardinfo,
				ARRAY_SIZE(dhcm3517_i2c3_boardinfo));
#ifdef CONFIG_TOUCHSCREEN_TMG120_TS
	}
#endif
	return 0;
}



static struct platform_pwm_backlight_data backligh_led_pdata = {
	.pwm_id			= 11,
	.max_brightness		= 255,
	.dft_brightness		= 205, /* default brightness 80% */
};

static struct platform_device backlight_led_device = {
	.name		= "omap-pwm-backlight",
	.id		= -1,
	.dev.platform_data	= &backligh_led_pdata
};



#define LCD_PANEL_EN		141
static struct gpio dhcm3517_dss_gpios[] __initdata = {

	/* GPIO 141 = LCD Panel Power enable pin */
	{ LCD_PANEL_EN,	 GPIOF_OUT_INIT_LOW, "lcd en" },
};

static int dhcm3517_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PANEL_EN, 1);
	return 0;
}

static void dhcm3517_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PANEL_EN, 0);
}

static struct panel_generic_dpi_data lcd_panel = {
	.name			= "primeview_pm070wt3",
	.platform_enable	= dhcm3517_panel_enable_lcd,
	.platform_disable	= dhcm3517_panel_disable_lcd,
};

static struct omap_dss_device dhcm3517_lcd_device = {
	.name			= "lcd",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.driver_name		= "generic_dpi_panel",
	.data			= &lcd_panel,
	.phy.dpi.data_lines 	= 24,
};

static struct omap_dss_device *dhcm3517_dss_devices[] = {
	&dhcm3517_lcd_device,
};

static struct omap_dss_board_info dhcm3517_dss_data = {
	.num_devices	= ARRAY_SIZE(dhcm3517_dss_devices),
	.devices	= dhcm3517_dss_devices,
	.default_device	= &dhcm3517_lcd_device,
};

static void __init dhcm3517_display_init(void)
{
	int r;

	omap_mux_init_gpio(LCD_PANEL_EN, OMAP_PIN_OUTPUT);

	r = gpio_request_array(dhcm3517_dss_gpios,
			       ARRAY_SIZE(dhcm3517_dss_gpios));
	if (r) {
		printk(KERN_ERR "DSS: *** failed to get DSS panel control GPIOs\n");
		return;
	}

	/* first switch off */
	gpio_set_value(LCD_PANEL_EN, 0);
	gpio_export(LCD_PANEL_EN, 0);

	printk(KERN_ERR "DSS: Panel control GPIOs ok, init display\n");
	omap_display_init(&dhcm3517_dss_data);
}

static int otg_mode_host=1;

static int __init dhcm3517_otg_mode(char *options)
{
	if (!strcmp(options, "device"))
		otg_mode_host = 0;
	else if (!strcmp(options, "host"))
		otg_mode_host = 1;
	else
		pr_info("otg_mode neither \"host\" nor \"device\". "
			"Defaulting to host\n");
	return 0;
}
__setup("otg_mode=", dhcm3517_otg_mode);

static int otg_power_pin=0;

static int __init dheva_dhcm3517_otg_power_pin(char *options)
{
	if (!strcmp(options, "100"))
		otg_power_pin = 1;
	else if (!strcmp(options, "200"))
		otg_power_pin = 1;
	else {
		otg_power_pin = 0;
		pr_info("No otg power pin. \n");
	}
	return 0;
}
__setup("dheva01_version=", dheva_dhcm3517_otg_power_pin);


static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_UTMI,

	/* Switching DEVICE / HOST does not work !*/
	/* .mode                   = MUSB_OTG, */
	/* .mode                   = MUSB_HOST, */

	.mode                   = MUSB_HOST,

	.power                  = 500,
	.set_phy_power		= am35x_musb_phy_power,
	.clear_irq		= am35x_musb_clear_irq,
	.set_mode		= am35x_set_mode,
	.reset			= am35x_musb_reset,
};

#define USB_PWR_GPIO 117
static struct gpio dhcm3517_usb_gpios[] __initdata = {

	/* GPIO 117 = GPIO B Power pin , also on leds */
	{ USB_PWR_GPIO,	 GPIOF_OUT_INIT_HIGH, "usb pwr" },
};

static __init void dhcm3517_musb_init(void)
{
	int r;
	u32 devconf2;

	/* OTG Power Pin only if argument was set */
	if (otg_power_pin) {

		omap_mux_init_gpio(USB_PWR_GPIO, OMAP_PIN_OUTPUT);

		r = gpio_request_array(dhcm3517_usb_gpios,
				       ARRAY_SIZE(dhcm3517_usb_gpios));
		if (r) {
			printk(KERN_ERR "USB: *** failed to get USB PWR GPIO\n");
			return;
		}

		/* set required otg power modus */
		if (otg_mode_host) {
			gpio_set_value(USB_PWR_GPIO, 1);
		} else {
			gpio_set_value(USB_PWR_GPIO, 0);
		}

		gpio_export(USB_PWR_GPIO, 0);
		printk(KERN_ERR "USB: PWR GPIO ok\n");
	}

	/* set required otg port modus */
	if (otg_mode_host)
		musb_board_data.mode = MUSB_HOST;
	else
		musb_board_data.mode = MUSB_PERIPHERAL;


	/* Set up USB clock/mode in the DEVCONF2 register */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);

	devconf2	|= CONF2_REFFREQ_13MHZ
			| CONF2_SESENDEN
			| CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);
	usb_musb_init(&musb_board_data);

}

#define HSUSB1_RESET	156
static struct usbhs_omap_board_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = HSUSB1_RESET,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
};

static struct resource am3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_hecc_device = {
	.name		= "ti_hecc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_hecc_resources),
	.resource	= am3517_hecc_resources,
};

static struct ti_hecc_platform_data dhcm3517_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
};

static void dhcm3517_hecc_init(struct ti_hecc_platform_data *pdata)
{
	am3517_hecc_device.dev.platform_data = pdata;
	platform_device_register(&am3517_hecc_device);
}

#if defined(CONFIG_MTD_NAND_OMAP2) || defined(CONFIG_MTD_NAND_OMAP2_MODULE)
static struct mtd_partition dhcm3517_nand_partitions[] = {
	{
		.name           = "xloader",
		.offset         = 0,
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 4 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "uboot environment",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "uboot environment(red)",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "splash",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 34 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "dh",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "linux",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 48 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
	},
};
#else
static struct mtd_partition dhcm3517_nand_partitions[] = {};
#endif

static void __init dhcm3517_init_nand(void) {
	omap_nand_flash_init(0, dhcm3517_nand_partitions,
		 ARRAY_SIZE(dhcm3517_nand_partitions));
}

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd        = 127,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= 0x00100000,		// 3.3V
	},
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.caps2		= MMC_CAP2_CD_HIGH_ACTIVE,
		.gpio_cd	= 126,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= 0x00100000,		// 3.3V
	},
	{
		.mmc = 0,
	}
};

static inline void dhcm3517_init_hsmmc(void) {

	omap2_hsmmc_init(mmc);
}

/* #define USE_SPIFLASH_PARTITIONS */
static struct mtd_partition dhcm3517_spiflash_part[] = {
#ifdef USE_SPIFLASH_PARTITIONS
	[0] = {
		.name = "Partition",
		.offset = 0,
		.size = SZ_4M - SZ_64K,
		.mask_flags = MTD_WRITEABLE,
	},
	[1] = {
		.name = "MAC-Address",
		.offset = SZ_4M - SZ_64K,
		.size = SZ_64K,
		.mask_flags = MTD_WRITEABLE,
	},
#endif
};

static struct flash_platform_data dhcm3517_spiflash_data = {
	.name		= "m25p80",
	.parts		= dhcm3517_spiflash_part,
	.nr_parts	= ARRAY_SIZE(dhcm3517_spiflash_part),
	.type		= "sst25vf032b",
};

static struct omap2_mcspi_device_config dhcm3517_spiflash_cfg = {
	.turbo_mode = 1,
	.single_channel = 1,
};

static struct spi_board_info dhcm3517_spi_info[] = {
/*	{
		.modalias		= "m25p80",
		.platform_data		= &dhcm3517_spiflash_data,
		.controller_data	= &dhcm3517_spiflash_cfg,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 30000000,
		.bus_num		= 1,
		.chip_select		= 0,
	}, */
	{
		.modalias		= "spidev",
		.controller_data	= &dhcm3517_spiflash_cfg,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 30000000,
		.bus_num		= 1,
		.chip_select		= 1,
	},
	{
		.modalias		= "spidev",
		.controller_data	= &dhcm3517_spiflash_cfg,
		.mode			= SPI_MODE_0,
		.max_speed_hz		= 30000000,
		.bus_num		= 2,
		.chip_select		= 0,
	},
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {

	/* MMC1/2 DETECT */
	OMAP3_MUX(SDMMC1_DAT4,	OMAP_MUX_MODE4 ),			/* gpio 126 mmc2 detect */
	OMAP3_MUX(SDMMC1_DAT5,	OMAP_MUX_MODE4 ),			/* gpio 127 mmc1 detect */

	OMAP3_MUX(SDMMC1_DAT6,	OMAP_MUX_MODE4),			/* gpio 128 pmic int */
	OMAP3_MUX(SDMMC1_DAT7,	OMAP_MUX_MODE4),			/* gpio 129 rtc  mfp */

	/* LCD PWM */
	OMAP3_MUX(GPMC_NCS6,	OMAP_MUX_MODE3),			/* gpio  57 (SCM GPIO PWM) */

	/* USB_HOST2_SUSPEND - STANDARD GPIO */
	/* check GPMC_NBE1 */

	/* USB_HOST2_SPEED  - STANDARD GPIO */
	/* check GPMC_WAIT1 */

	/* USB_HOST2_SOFTCON  - STANDARD GPIO */
	/* check GPMC_WAIT2 */

	/* ETH_INT  -- check -- */
	/* check GPMC_WAIT3 */

	/* GPIO */
	OMAP3_MUX(UART2_CTS,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 144 (SPI FLASH HOLD */
	OMAP3_MUX(UART2_RTS,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 145 (SPI FLASH WP */
	OMAP3_MUX(UART2_TX,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 146 (ETH VIO GPIO) */
	OMAP3_MUX(UART2_RX,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 147 (TOUCH INT) */

	/* UART1 FF-Pins */
	OMAP3_MUX(SDMMC2_DAT7,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 139 (SCM DHCOM UART1_RI)  */
	OMAP3_MUX(SDMMC2_DAT6,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 138 (SCM DHCOM UART1_DSR) */
	OMAP3_MUX(SDMMC2_DAT5,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 137 (SCM DHCOM UART1_DCD) */
	OMAP3_MUX(SDMMC2_DAT4,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 136 (SCM DHCOM UART1_DTR) */

	/* USB */
	OMAP3_MUX(USB0_DRVVBUS, OMAP_MUX_MODE0),			/* gpio 125 (SCM USBOTG_DRVVBUS) */

	/* HECC */
	OMAP3_MUX(HECC1_TXD,	OMAP_MUX_MODE0),			/* (HECC TX) */
	OMAP3_MUX(HECC1_RXD,	OMAP_MUX_MODE0 | OMAP_PIN_INPUT),	/* (HECC RX) */

	/* CAM -> setup pads with alternative DHCOM GPIO function */
	OMAP3_MUX(CCDC_HD,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio  96 (GPIO J) */
	OMAP3_MUX(CCDC_PCLK,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio  94 (GPIO K) */
	OMAP3_MUX(GPMC_NCS7,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio  58 (GPIO L / CAM PWM) */
	OMAP3_MUX(CCDC_VD,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio  97 (GPIO M) */
	OMAP3_MUX(CCDC_WEN,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio  98 (GPIO N) */
	OMAP3_MUX(CCDC_FIELD,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio  95 (GPIO O) */
	OMAP3_MUX(CCDC_DATA7,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 106 (GPIO P) */
	OMAP3_MUX(CCDC_DATA6,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 105 (GPIO Q) */
	OMAP3_MUX(CCDC_DATA5,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 104 (GPIO R) */
	OMAP3_MUX(CCDC_DATA4,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 103 (GPIO S) */
	OMAP3_MUX(CCDC_DATA3,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 102 (GPIO T) */
	OMAP3_MUX(CCDC_DATA2,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 101 (GPIO U) */
	OMAP3_MUX(CCDC_DATA1,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 100 (GPIO V) */
	OMAP3_MUX(CCDC_DATA0,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio  99 (GPIO W) */


	OMAP3_MUX(MCSPI1_CS2,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),	/* gpio 176  (SCM CODE NAND)  */

	/* check */
	/* OMAP3_MUX(MCSPI1_CS3, ),					(SCM USB_HOST2_VP)  */

        /* SPI 2*/
	OMAP3_MUX(MCSPI2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),		/* (SCM SPI2_CLK) */
	OMAP3_MUX(MCSPI2_SIMO, OMAP_MUX_MODE0),				/* (SCM SPI2_SIMO) */
	OMAP3_MUX(MCSPI2_SOMI, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),	/* (SCM SPI2_SOMI) */
	OMAP3_MUX(MCSPI2_CS0, OMAP_MUX_MODE0),				/* (SCM SPI2_CS0) */

	/* check */
	/* OMAP3_MUX(MCSPI2_CS1, ),					(SCM USB_HOST2_#OE)  */

	/* GPIO - part 1 */
	OMAP3_MUX(MCBSP2_FSX,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 116 (SCM GPIO A) */
	OMAP3_MUX(MCBSP2_CLKX,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 117 (SCM GPIO B) */
	OMAP3_MUX(MCBSP2_DR,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 118 (SCM GPIO C) */
	OMAP3_MUX(MCBSP2_DX,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 119 (SCM GPIO D) */
	OMAP3_MUX(USB0_DRVVBUS,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 125 (SCM GPIO E) */
	OMAP3_MUX(MCBSP3_DX,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 140 (SCM GPIO F) */

	/* LCD */
	OMAP3_MUX(MCBSP3_DR,	OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT | OMAP_PIN_INPUT),
									/* gpio 141 (SCM GPIO G)   */
	/* GPIO - part 2 */
	OMAP3_MUX(MCBSP4_FSX,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 155 (SCM GPIO H) */
	OMAP3_MUX(MCBSP4_CLKX,	OMAP_MUX_MODE4 | OMAP_PIN_INPUT),	/* gpio 152 (SCM GPIO I) */

	/* UART 3 - DHCOM UART BT */
	OMAP3_MUX(UART3_RX_IRRX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(UART3_TX_IRTX, OMAP_MUX_MODE0),
	OMAP3_MUX(UART3_CTS_RCTX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
        OMAP3_MUX(UART3_RTS_SD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	
	/* UART 2 - DHCOM UART STD */
	OMAP3_MUX(MCBSP3_FSX,	OMAP_MUX_MODE1 | OMAP_PIN_INPUT),	/* RX */
	OMAP3_MUX(MCBSP3_CLKX,	OMAP_MUX_MODE1 | OMAP_PIN_INPUT),	/* TX */

	/* check */
	/* OMAP3_MUX(MCSPI4_DR, ),					(SCM USB_HOST2_#OE)  */
	/* OMAP3_MUX(MCSPI4_DX, ),					(SCM USB_HOST2_#OE)  */

	/* check */
	/* OMAP3_MUX(MCBSP1_CLKR,	OMAP_MUX_MODE4 ),*/		/* gpio 156 HSUSB1_RES */
	OMAP3_MUX(MCBSP1_FSR,	OMAP_MUX_MODE4 ),			/* gpio 157 USB PWR_STAT */

	/* check */
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* (CODE SDRAM_0)  */
	OMAP3_MUX(ETK_D10, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* (CODE SDRAM_1)  */
	OMAP3_MUX(ETK_D11, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* (CODE SDRAM_2)  */
	OMAP3_MUX(ETK_D12, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* (CODE HW 0)  */
	OMAP3_MUX(ETK_D13, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),		/* (CODE HW 1)  */

	/* check */
	/* OMAP3_MUX(ETK_D14, ),					(SCM USB_HOST2_RCV)  */
	/* OMAP3_MUX(ETK_D15, ),					(SCM USB_HOST2_VM)  */

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_board_config_kernel dhcm3517_config[] __initdata = {
};

static int backlight_invert=0;

static int __init dhcm3517_bl_invert(char *options)
{
	if (!strcmp(options, "normal"))
		backlight_invert = 0;
	else if (!strcmp(options, "inverted"))
		backlight_invert = 1;
	else
		pr_info("pwm_pol neither \"normal\" nor \"invert\". "
			"Defaulting to normal\n");
	return 0;
}
__setup("pwm_pol=", dhcm3517_bl_invert);

static int framebuffer_depth=0;

static int __init dhcm3517_fb_depth(char *options)
{
	if (!strcmp(options, "16"))
		framebuffer_depth = 16;
	else if (!strcmp(options, "24"))
		framebuffer_depth = 24;
	else
		pr_info("fb_depth neither \"16\" nor \"24\". "
			"Defaulting to u-boot\n");
	return 0;
}
__setup("fb_depth=", dhcm3517_fb_depth);

int dhcm3517_get_framebuffer_depth(void)
{
	return framebuffer_depth;
}
EXPORT_SYMBOL(dhcm3517_get_framebuffer_depth);

static void __init dhcm3517_init(void)
{
	omap_board_config = dhcm3517_config;
	omap_board_config_size = ARRAY_SIZE(dhcm3517_config);
	omap3_mux_init(board_mux, OMAP_PACKAGE_ZCN);

	omap_serial_init();

	dhcm3517_display_init();

	dhcm3517_i2c_init();

	dhcm3517_init_nand();

	dhcm3517_musb_init();

	usbhs_init(&usbhs_bdata);

	dhcm3517_init_hsmmc();

	dhcm3517_hecc_init(&dhcm3517_hecc_pdata);

	spi_register_board_info(dhcm3517_spi_info,
			ARRAY_SIZE(dhcm3517_spi_info));

	dhcm3517_ethernet_init(&dhcm3517_emac_pdata);

	if (backlight_invert)
		backligh_led_pdata.pwm_inverted = 1;

	platform_device_register(&backlight_led_device);
}

static void __init dhcm3517_reserve(void)
{
	omap_reserve();
}

MACHINE_START(DHCM3517, "DHCM3517")
	.atag_offset	= 0x100,
	.reserve	= dhcm3517_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap3_init_early,
	.init_irq	= omap3_init_irq,
	.init_machine	= dhcm3517_init,
	.timer		= &omap3_timer,
MACHINE_END
