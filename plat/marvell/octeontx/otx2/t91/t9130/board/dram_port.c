/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/mentor/mi2cv.h>
#include <lib/mmio.h>

#include <mv_ddr_if.h>
#include <mvebu_def.h>
#include <plat_marvell.h>

#define MVEBU_CP_MPP_CTRL37_OFFS		20
#define MVEBU_CP_MPP_CTRL38_OFFS		24
#define MVEBU_CP_MPP_CTRL37_I2C0_SCK_ENA	0x2
#define MVEBU_CP_MPP_CTRL38_I2C0_SDA_ENA	0x2

#define MVEBU_MPP_CTRL_MASK			0xf

/*
 * This struct provides the DRAM training code with
 * the appropriate board DRAM configuration
 */
struct mv_ddr_iface dram_iface_ap0 = {
	.ap_base = MVEBU_REGS_BASE_AP(0),
	.state = MV_DDR_IFACE_NRDY,
	.validation = MV_DDR_MEMORY_CHECK,
	.sscg = SSCG_EN,
	.id = 0,
	.iface_base_addr = 0,
	.tm = {
		DEBUG_LEVEL_INFO,
		0x1, /* active interfaces */
		/* cs_mask, mirror, dqs_swap, ck_swap X subphys */
		{ { { {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0} },
		   SPEED_BIN_DDR_2400R,		/* speed_bin */
		   MV_DDR_DEV_WIDTH_16BIT,	/* sdram device width */
		   MV_DDR_DIE_CAP_8GBIT,	/* die capacity */
		   MV_DDR_FREQ_SAR,		/* frequency */
		   0, 0,			/* cas_l, cas_wl */
		   MV_DDR_TEMP_HIGH} },		/* temperature */
#if DDR32
		MV_DDR_32BIT_ECC_PUP8_BUS_MASK,	/* subphys mask */
#else
		MV_DDR_64BIT_BUS_MASK,
#endif
		MV_DDR_CFG_SPD,			/* ddr configuration data src */
		NOT_COMBINED,			/* ddr twin-die combined*/
		{ {0} },			/* raw spd data */
		{0},				/* timing parameters */
		{	/* electrical configuration */
			{	/* memory electrical configuration */
				MV_DDR_RTT_NOM_PARK_RZQ_DISABLE,  /* rtt_nom */
				{	/* rtt_park 1cs */
					MV_DDR_RTT_NOM_PARK_RZQ_DIV4,
					/* rtt_park 2cs */
					MV_DDR_RTT_NOM_PARK_RZQ_DIV1
				},
				{	/* rtt_wr 1cs */
					MV_DDR_RTT_WR_DYN_ODT_OFF,
					/* rtt_wr 2cs */
					MV_DDR_RTT_WR_RZQ_DIV2
				},
				MV_DDR_DIC_RZQ_DIV7	/* dic */
			},
			{	/* phy electrical configuration */
				MV_DDR_OHM_30,	/* data_drv_p */
				MV_DDR_OHM_30,	/* data_drv_n */
				MV_DDR_OHM_30,	/* ctrl_drv_p */
				MV_DDR_OHM_30,	/* ctrl_drv_n */
				{
					MV_DDR_OHM_60,	/* odt_p 1cs */
					MV_DDR_OHM_120	/* odt_p 2cs */
				},
				{
					MV_DDR_OHM_60,	/* odt_n 1cs */
					MV_DDR_OHM_120	/* odt_n 2cs */
				},
			},
			{	/* mac electrical configuration */
				MV_DDR_ODT_CFG_NORMAL,	/* odtcfg_pattern */
				MV_DDR_ODT_CFG_ALWAYS_ON,/* odtcfg_write */
				MV_DDR_ODT_CFG_NORMAL	/* odtcfg_read */
			},
		},
	},
};

int spd_crc16_sr(u8 *ptr, int count) {
	/*crc16 calculation*/
	int crc, i;
	crc = 0;
	while (--count >= 0) {
		crc = crc ^ (int)*ptr++ << 8;
		for (i = 0; i < 8; ++i) {
			if (crc & 0x8000) {
				crc = crc << 1 ^ 0x1021;
			} else {
				crc = crc << 1;
			}
		}
	}
	return (crc & 0xFFFF);
}

/* this function verifies that the  the first 196 bytes to check if SPD has any DDR configuration */
u8 spd_verify_correction_sr(u8 *spd_buffer) {
	int data16;
	data16 = spd_crc16_sr(spd_buffer, 126);
	if ((spd_buffer[126] !=(u8)(data16&0xff)) || (spd_buffer[127]!=(u8)(data16>>8))){
		/* if bits 126 and 127 in the spd matches the crc calculation and the data is valid*/
		return 1;
	}

	return 0;
}

void define_gpio_input_sr (u32 mux_address, u32 direction_address, u8 bit_start_mux, u8 bit_dir_address)
{
	u32 mask=0;
	u32 val;

	/* define as gpio */
	val = mmio_read_32 (mux_address);
	mask |= (0xf << bit_start_mux);
	mask = ~mask;
	val &= mask;
	mmio_write_32(mux_address, val);
	/*define as input */

	val = mmio_read_32(direction_address);
	val |= (1 << bit_dir_address);
	mmio_write_32(direction_address, val);

	/*all is standard polarity  by defualt */
}

u8 read_gpio_strap_sr (u32 addr, u8 bit_loc)
{
	u32 val;

	val=mmio_read_32 (addr);
	val &= (0x1<<bit_loc);

	if (!val)
		return (u8) 0;

	return (u8) 1;

}

void set_param_based_on_som_strap (){
	/*	sample GPIOs 10, 11
	 *	MPP10: 1-ECC /0-noECC
	 *	MPP11: 1-4Gb /0-8Gb per component
	 */

	/*defining MPP[11:10] as gpio inputs*/
	define_gpio_input_sr (0xf2440004, 0xf2440100, 8, 10);
	define_gpio_input_sr (0xf2440004, 0xf2440100, 12,11);

	/*after SPD crc failed, defining as DEFAULT configuration*/
	dram_iface_ap0.tm.cfg_src = MV_DDR_CFG_DEFAULT;

	/* read gpio value to check if ECC exist or not */
	if (read_gpio_strap_sr ((u32) 0xf2440110, (u8) 10)){
		printf("w/ecc; strap value=1\n\n");
		dram_iface_ap0.tm.bus_act_mask=MV_DDR_64BIT_ECC_PUP8_BUS_MASK;
	}
	else {
		dram_iface_ap0.tm.bus_act_mask=MV_DDR_64BIT_BUS_MASK;
		printf("w/o ecc; strap value=0\n\n");
	}
	/* read gpio value to check 4GB or 8GB configuration */
	if (!read_gpio_strap_sr ((u32) 0xf2440110, (u8) 11)){
		dram_iface_ap0.tm.interface_params[0].memory_size=MV_DDR_DIE_CAP_16GBIT;
		printf("8GB capacity; strap value=1\n\n");
	}
	else{
		dram_iface_ap0.tm.interface_params[0].memory_size=MV_DDR_DIE_CAP_8GBIT;
		printf("4GB capacity; strap value=1\n\n");
	}
}

/* Pointer to the first DRAM interface in the system */
struct mv_ddr_iface *ptr_iface = &dram_iface_ap0;

struct mv_ddr_iface *mv_ddr_iface_get(void)
{
	/* Return current ddr interface */
	return ptr_iface;
}

struct mv_ddr_topology_map *mv_ddr_topology_map_get(void)
{
	/* Return the board topology as defined in the board code */
	return &ptr_iface->tm;
}

static void mpp_config(void)
{
	uintptr_t reg;
	uint32_t val;

	reg = MVEBU_CP_MPP_REGS(0, 4);
	/* configure CP0 MPP 37 and 38 to i2c */
	val = mmio_read_32(reg);
	val &= ~((MVEBU_MPP_CTRL_MASK << MVEBU_CP_MPP_CTRL37_OFFS) |
		(MVEBU_MPP_CTRL_MASK << MVEBU_CP_MPP_CTRL38_OFFS));
	val |= (MVEBU_CP_MPP_CTRL37_I2C0_SCK_ENA <<
			MVEBU_CP_MPP_CTRL37_OFFS) |
		(MVEBU_CP_MPP_CTRL38_I2C0_SDA_ENA <<
			MVEBU_CP_MPP_CTRL38_OFFS);
	mmio_write_32(reg, val);
}

/*
 * This function may modify the default DRAM parameters
 * based on information received from SPD or bootloader
 * configuration located on non volatile storage
 */
void plat_marvell_dram_update_topology(void)
{
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	INFO("Gathering DRAM information\n");

	if (tm->cfg_src == MV_DDR_CFG_SPD) {
		/* configure MPPs to enable i2c */
		mpp_config();

		/* initialize i2c */
		i2c_init((void *)MVEBU_CP0_I2C_BASE);

		/* select SPD memory page 0 to access DRAM configuration */
		i2c_write(I2C_SPD_P0_ADDR, 0x0, 1, tm->spd_data.all_bytes, 0);

		/* read data from spd */
		i2c_read(I2C_SPD_ADDR, 0x0, 1, tm->spd_data.all_bytes,
			 sizeof(tm->spd_data.all_bytes));

		/* if SPD is empty, then copy default configuration as SPD */
		if (spd_verify_correction_sr(tm->spd_data.all_bytes)) {
			set_param_based_on_som_strap();
		}
	}
}
