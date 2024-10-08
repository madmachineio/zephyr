/*
 * Copyright (c) 2018, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <fsl_iomuxc.h>
#include <fsl_gpio.h>

#if DT_NODE_HAS_STATUS(DT_NODELABEL(enet), okay) && CONFIG_NET_L2_ETHERNET
static gpio_pin_config_t enet_gpio_config = {
	.direction = kGPIO_DigitalOutput,
	.outputLogic = 0,
	.interruptMode = kGPIO_NoIntmode
};
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usdhc1), okay) && CONFIG_DISK_DRIVER_SDMMC

/*Drive Strength Field: R0(260 Ohm @ 3.3V, 150 Ohm@1.8V, 240 Ohm for DDR)
 *Speed Field: medium(100MHz)
 *Open Drain Enable Field: Open Drain Disabled
 *Pull / Keep Enable Field: Pull/Keeper Enabled
 *Pull / Keep Select Field: Pull
 *Pull Up / Down Config. Field: 47K Ohm Pull Up
 *Hyst. Enable Field: Hysteresis Enabled.
 */

static void mimxrt1020_evk_usdhc_pinmux(
	uint16_t nusdhc, bool init,
	uint32_t speed, uint32_t strength)
{
	uint32_t cmd_data = IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) |
				IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
				IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
				IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
				IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
				IOMUXC_SW_PAD_CTL_PAD_PUS(1) |
				IOMUXC_SW_PAD_CTL_PAD_DSE(strength);
	uint32_t clk = IOMUXC_SW_PAD_CTL_PAD_SPEED(speed) |
				IOMUXC_SW_PAD_CTL_PAD_SRE_MASK |
				IOMUXC_SW_PAD_CTL_PAD_HYS_MASK |
				IOMUXC_SW_PAD_CTL_PAD_PUS(0) |
				IOMUXC_SW_PAD_CTL_PAD_DSE(strength);

	if (nusdhc == 0) {
		if (init) {
			IOMUXC_SetPinMux(/*SD_CD*/
				IOMUXC_GPIO_SD_B0_06_USDHC1_CD_B,
				0U);
			IOMUXC_SetPinMux(
				IOMUXC_GPIO_AD_B1_07_USDHC1_VSELECT,
				0U);
			IOMUXC_SetPinMux(
				IOMUXC_GPIO_SD_B0_02_USDHC1_CMD,
				0U);
			IOMUXC_SetPinMux(
				IOMUXC_GPIO_SD_B0_03_USDHC1_CLK,
				0U);
			IOMUXC_SetPinMux(
				IOMUXC_GPIO_SD_B0_04_USDHC1_DATA0,
				0U);
			IOMUXC_SetPinMux(
				IOMUXC_GPIO_SD_B0_05_USDHC1_DATA1,
				0U);
			IOMUXC_SetPinMux(
				IOMUXC_GPIO_SD_B0_00_USDHC1_DATA2,
				0U);
			IOMUXC_SetPinMux(
				IOMUXC_GPIO_SD_B0_01_USDHC1_DATA3,
				0U);

			IOMUXC_SetPinConfig(/*SD0_CD_SW*/
				IOMUXC_GPIO_SD_B0_06_USDHC1_CD_B,
				0x017089u);
			IOMUXC_SetPinConfig(/*SD0_VSELECT*/
				IOMUXC_GPIO_AD_B1_07_USDHC1_VSELECT,
				0x0170A1u);
		}

		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_02_USDHC1_CMD,
			cmd_data);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_03_USDHC1_CLK,
			clk);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_04_USDHC1_DATA0,
			cmd_data);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_05_USDHC1_DATA1,
			cmd_data);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_00_USDHC1_DATA2,
			cmd_data);
		IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B0_01_USDHC1_DATA3,
			cmd_data);
	}
}
#endif

static int mimxrt1020_evk_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	CLOCK_EnableClock(kCLOCK_Iomuxc);
	CLOCK_EnableClock(kCLOCK_IomuxcSnvs);

	/* LED */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	/* SW0 */
	IOMUXC_SetPinMux(IOMUXC_SNVS_WAKEUP_GPIO5_IO00, 0);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart1), okay) && CONFIG_SERIAL
	/* LPUART1 TX/RX */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_06_LPUART1_TX, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_07_LPUART1_RX, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_06_LPUART1_TX,
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_07_LPUART1_RX,
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpuart2), okay) && CONFIG_SERIAL
	/* LPUART2 TX/RX */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_LPUART2_TX, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_LPUART2_RX, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_08_LPUART2_TX,
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_09_LPUART2_RX,
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpi2c1), okay) && CONFIG_I2C
	/* LPI2C1 SCL, SDA */
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_14_LPI2C1_SCL, 1);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_15_LPI2C1_SDA, 1);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_14_LPI2C1_SCL,
			    IOMUXC_SW_PAD_CTL_PAD_PUS(3) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_15_LPI2C1_SDA,
			    IOMUXC_SW_PAD_CTL_PAD_PUS(3) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(lpi2c4), okay) && CONFIG_I2C
	/* LPI2C4 SCL, SDA */
	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_02_LPI2C4_SCL, 1);
	IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B1_03_LPI2C4_SDA, 1);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_02_LPI2C4_SCL,
			    IOMUXC_SW_PAD_CTL_PAD_PUS(3) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));

	IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B1_03_LPI2C4_SDA,
			    IOMUXC_SW_PAD_CTL_PAD_PUS(3) |
			    IOMUXC_SW_PAD_CTL_PAD_PKE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
			    IOMUXC_SW_PAD_CTL_PAD_SPEED(2) |
			    IOMUXC_SW_PAD_CTL_PAD_DSE(6));
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(enet), okay) && CONFIG_NET_L2_ETHERNET
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04, 0U);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_06_GPIO1_IO22, 0U);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_ENET_RDATA00, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_09_ENET_RDATA01, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_11_ENET_RX_EN, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_14_ENET_TDATA00, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_15_ENET_TDATA01, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_13_ENET_TX_EN, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_ENET_REF_CLK1, 1);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_12_ENET_RX_ER, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_41_ENET_MDC, 0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_40_ENET_MDIO, 0);

	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04, 0xB0A9u);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_06_GPIO1_IO22, 0xB0A9u);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_ENET_RDATA00, 0xB0E9);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_09_ENET_RDATA01, 0xB0E9);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_11_ENET_RX_EN, 0xB0E9);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_14_ENET_TDATA00, 0xB0E9);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_15_ENET_TDATA01, 0xB0E9);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_13_ENET_TX_EN, 0xB0E9);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_08_ENET_REF_CLK1, 0x31);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_12_ENET_RX_ER, 0xB0E9);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_41_ENET_MDC, 0xB0E9);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_40_ENET_MDIO, 0xB829);

	IOMUXC_EnableMode(IOMUXC_GPR, kIOMUXC_GPR_ENET1TxClkOutputDir, true);

	/* Initialize ENET_INT GPIO */
	GPIO_PinInit(GPIO1, 4, &enet_gpio_config);
	GPIO_PinInit(GPIO1, 22, &enet_gpio_config);

	/* pull up the ENET_INT before RESET. */
	GPIO_WritePinOutput(GPIO1, 22, 1);
	GPIO_WritePinOutput(GPIO1, 4, 0);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usdhc1), okay) && CONFIG_DISK_DRIVER_SDMMC
	mimxrt1020_evk_usdhc_pinmux(0, true, 2, 1);
	imxrt_usdhc_pinmux_cb_register(mimxrt1020_evk_usdhc_pinmux);
#endif

	return 0;
}

#if DT_NODE_HAS_STATUS(DT_NODELABEL(enet), okay) && CONFIG_NET_L2_ETHERNET
static int mimxrt1020_evk_phy_reset(const struct device *dev)
{
	/* RESET PHY chip. */
	k_busy_wait(USEC_PER_MSEC * 10U);
	GPIO_WritePinOutput(GPIO1, 4, 1);

	return 0;
}
#endif

SYS_INIT(mimxrt1020_evk_init, PRE_KERNEL_1, 0);
#if DT_NODE_HAS_STATUS(DT_NODELABEL(enet), okay) && CONFIG_NET_L2_ETHERNET
SYS_INIT(mimxrt1020_evk_phy_reset, POST_KERNEL, CONFIG_PHY_INIT_PRIORITY);
#endif
