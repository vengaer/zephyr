/*
 * Copyright (c) 2026 Vilhelm Engström
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/dsa_core.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/phy.h>
#include <zephyr/toolchain.h>

#include "dsa_ksz8463.h"

LOG_MODULE_REGISTER(ksz8463, CONFIG_ETHERNET_LOG_LEVEL);

/* The driver strives to adhere to the following convetions
 *
 * - Parameters named dev *always* refer to the switch device.
 * - Parameters named ptdev *always* refer to a switch port.
 * - Devices that correspond to neither the switch nor one of its ports
 *   are named appropriately, e.g. gpio_dev.
 *
 * The above means that
 *
 *   dev->data *always* refers to a struct ksz8463_data
 *   dev->config *always* refers to a struct ksz8463_config
 *   ptdev->data *always* refers to a struct dsa_switch_context
 *   ptdev->config *always* refers to a struct dsa_port_config
 */

/* compatible = "microchip,ksz8463"; */
#define DT_DRV_COMPAT microchip_ksz8463

/* Switch data */
struct ksz8463_data {

	/* Chip id read from CIDER */
	uint8_t chip_id;

	/* Work scheduled on chip IRQ */
	struct k_work chip_isr_work;

	/* Mutex protection SPI */
	struct k_mutex spi_mutex;

	/* Callback invoked on chip IRQ */
	struct gpio_callback chip_cb;

	/* Array of port devices */
	const struct device **ptdevs;
};

/* Switch configuration */
struct ksz8463_config {

	/* Whether or not MLD snooping should be enabled */
	const bool mld_snoop_en;

	/* Whether or not IGMP snooping should be enabled */
	const bool igmp_snoop_en;

	/* Whether or not legal max packet size_check should be enabled */
	const bool pkt_sz_chk_en;

	/* Number of port devices in the data ptdevs array */
	const uint8_t num_ptdevs;

	/* Port LED mode */
	const uint8_t led_mode;

	/* Fixed speed to use on auto-negotiation failure */
	const uint8_t fixed_speed;

	/* SPI operational timeout */
	const uint16_t spi_oper_timeout;

	/* Reset GPIO, if any */
	struct gpio_dt_spec *rst_gpio;

	/* IRQ GPIO, if any */
	struct gpio_dt_spec *irq_gpio;

	/* SPI DT information */
	struct spi_dt_spec spi;

	/* Pin configuration */
	const struct pinctrl_dev_config *pincfg;

	/* Switch device */
	const struct device *dev;
};

/* DSA switch context private data */
struct ksz8463_prv_data {

	/* Switch device */
	const struct device *dev;
};

/* Per-port configuration */
struct ksz8463_port_config {

	/* Whether or not to disable EEE */
	const bool disable_eee;

	/* Whether or not EEE is currently enabled */
	bool eee_enabled;

	/* Whether or not auto-negotiation is enabled */
	bool autoneg_enabled;

	/* Whether or not the PHY link is up */
	bool phy_up;

	/* Whether or not the link is ready */
	bool link_ready;

	/* Auto-negotiation poll interval */
	const uint32_t autoneg_intvl;

	/* Auto-negotiation timeout */
	const uint32_t autoneg_timeout;

	/* Timepoint when auto-negotiation timeout expires */
	k_timepoint_t autoneg_expiry;

	/* Auto-negotiation work */
	struct k_work_delayable autoneg_dwork;

	/* Mutex protecting the link */
	struct k_mutex *link_mutex;

	/* Associated port device */
	const struct device *const ptdev;
};

/* The port's link_mutex should be held when invoking this function */
static int ksz8463_set_port_carrier(const struct device *ptdev, bool on)
{
	struct net_if *iface = net_if_lookup_by_dev(ptdev);

	if (unlikely(!iface)) {
		return -ENODEV;
	}

	if (on) {
		net_if_carrier_on(iface);
	} else {
		net_if_carrier_off(iface);
	}

	return 0;
}

static inline int ksz8463_port_carrier_on(const struct device *ptdev)
{
	return ksz8463_set_port_carrier(ptdev, true);
}

static inline int ksz8463_port_carrier_off(const struct device *ptdev)
{
	return ksz8463_set_port_carrier(ptdev, false);
}

static inline bool ksz8463_have_hard_reset(const struct device *dev)
{
	const struct ksz8463_config *cfg = dev->config;

	return cfg->rst_gpio;
}

static inline bool ksz8463_have_irq_gpio(const struct device *dev)
{
	const struct ksz8463_config *cfg = dev->config;

	return cfg->irq_gpio;
}

static bool ksz8463_is_cpu_port(const struct device *ptdev)
{
	struct net_if *iface = net_if_lookup_by_dev(ptdev);
	struct ethernet_context *eth_ctx;

	if (unlikely(!iface)) {
		LOG_ERR("Error on interface lookup");
		return false;
	}

	eth_ctx = net_if_l2_data(iface);
	return eth_ctx->dsa_port == DSA_CPU_PORT;
}

static int ksz8463_spi_lock(const struct spi_dt_spec *spi)
{
	const struct device *dev;
	struct ksz8463_data *data;
	const struct ksz8463_config *cfg = CONTAINER_OF(spi, struct ksz8463_config, spi);

	dev = cfg->dev;
	data = dev->data;

	return k_mutex_lock(&data->spi_mutex, K_MSEC(50));
}

static void ksz8463_spi_unlock(const struct spi_dt_spec *spi)
{
	const struct device *dev;
	struct ksz8463_data *data;
	const struct ksz8463_config *cfg = CONTAINER_OF(spi, struct ksz8463_config, spi);

	dev = cfg->dev;
	data = dev->data;

	k_mutex_unlock(&data->spi_mutex);
}

static uint16_t ksz8463_spi_cmd(uint16_t reg, size_t size)
{
	/* Chip employs a 4-byte aligned, 11 bit address. Accessing
	 * unaligned addresses requires a wider access at the nearest,
	 * aligned-down address.
	 *
	 * Address is sent in bits 6:0 of the first byte in the command phase
	 * and bits 7:5 of the second.
	 *
	 * See Table 3-14 in the data sheet.
	 */
	reg = (reg >> 2u) << 4u;
	switch (size) {
	case 1u:
		reg |= (1u << (reg & 0x03u));
		break;
	case 2u:
		reg |= (reg & 0x02 ? 0x0c : 0x03);
		break;
	default:
		reg |= 0x0fu;
		break;
	}

	return reg << 2u;
}

/* Read n bytes over SPI. The SPI mutex should be held */
static int ksz8463_spi_read_raw(const struct spi_dt_spec *spi, uint16_t addr, void *dst, size_t n)
{
	int ret;
	uint16_t reg;
	uint8_t cmd[KSZ8463_SPI_CMD_MAX_SIZE];

	const struct spi_buf_set tx = {
		.buffers = &(const struct spi_buf){.buf = cmd, .len = sizeof(reg)},
		.count = 1u,
	};

	const struct spi_buf_set rx = {
		.buffers = &(const struct spi_buf){.buf = cmd, .len = sizeof(reg) + n},
		.count = 1u,
	};

	if (unlikely(n > KSZ8463_SPI_CMD_MAX_DATA_PH_SIZE)) {
		return -ENOBUFS;
	}

	reg = ksz8463_spi_cmd(addr, n);
	sys_put_be16(reg, cmd);

	ret = spi_transceive_dt(spi, &tx, &rx);
	if (ret >= 0 && n) {
		memcpy(dst, cmd + sizeof(reg), n);
	}

	return ret;
}

/* Read 4 bytes over SPI. SPI lock should be held */
static int64_t ksz8463_spi_readl_raw(const struct spi_dt_spec *spi, uint16_t addr)
{
	int ret;
	uint32_t be32;

	/* Address must be 4-byte aligned */
	if (unlikely(addr & 0x03)) {
		return -EFAULT;
	}

	ret = ksz8463_spi_read_raw(spi, addr, &be32, sizeof(be32));
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("Read 0x%04" PRIx16 " (0x%04" PRIx16 "): 0x%08" PRIx32, addr,
		ksz8463_spi_cmd(addr, sizeof(be32)), be32);

	return (int64_t)be32;
}

/* Like ksz8463_spi_readl_raw but with automatic lock acquisition */
static inline int64_t ksz8463_spi_readl(const struct spi_dt_spec *spi, uint16_t addr)
{
	int64_t ret = ksz8463_spi_lock(spi);

	if (!ret) {
		ret = ksz8463_spi_readl_raw(spi, addr);
		ksz8463_spi_unlock(spi);
	}
	return ret;
}

/* Read 16 bits from register, aligning addr if required. The SPI mutex should
 * be held
 */
static int ksz8463_spi_readw_raw(const struct spi_dt_spec *spi, uint16_t addr)
{
	int ret;
	int64_t ret64;
	uint16_t be16;

	/* Must be 2-byte aligned */
	if (unlikely(addr & 0x01u)) {
		return -EFAULT;
	}

	/* Chip supports accesses only at 4 byte boundaries */
	if (addr & 0x02u) {
		ret64 = ksz8463_spi_readl_raw(spi, addr & ~0x02u);
		if (ret64 < 0) {
			return (int)ret64;
		}

		return (int)(ret64 >> 16u);
	}

	ret = ksz8463_spi_read_raw(spi, addr, &be16, sizeof(be16));
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("Read 0x%04" PRIx16 " (0x%04" PRIx16 "): 0x%04" PRIx16, addr,
		ksz8463_spi_cmd(addr, sizeof(be16)), be16);

	/* ISO/IEC 9899:1999 guarantees only INT_MAX >= 32767, USHRT_MAX >= 65535, i.e.
	 * not that INT_MAX >= UINT16_MAX. See Section 5.2.4.2.1.
	 */
	BUILD_ASSERT(INT_MAX >= UINT16_MAX, "Potential overflow");
	return (int)be16;
}

/* Like ksz8463_spi_readw_raw but with automatic lock acquisition */
static inline int ksz8463_spi_readw(const struct spi_dt_spec *spi, uint16_t addr)
{
	int ret = ksz8463_spi_lock(spi);

	if (!ret) {
		ret = ksz8463_spi_readw_raw(spi, addr);
		ksz8463_spi_unlock(spi);
	}
	return ret;
}

/* Read 8 bites from register, aligning addr if required. The SPI mutex should
 * be held
 */
static int ksz8463_spi_readb_raw(const struct spi_dt_spec *spi, uint16_t addr)
{
	int ret;
	uint8_t b;

	/* Device supports accesses only at 4 byte boundaries */
	if (addr & 0x03u) {
		ret = ksz8463_spi_readw_raw(spi, addr & ~0x01u);
		if (ret < 0) {
			return ret;
		}

		return (ret >> ((addr & 0x01) << 0x03u)) & 0xff;
	}

	ret = ksz8463_spi_read_raw(spi, addr, &b, sizeof(b));
	if (ret < 0) {
		return ret;
	}

	LOG_DBG("Read 0x%04" PRIx16 "(0x%04" PRIx16 "): 0x%02" PRIx8, addr,
		ksz8463_spi_cmd(addr, sizeof(b)), b);
	return (int)b;
}

/* Like ksz8463_spi_readb_raw but with automatic acquisition of the SPI mutex */
static inline int ksz8463_spi_readb(const struct spi_dt_spec *spi, uint16_t addr)
{
	int ret = ksz8463_spi_lock(spi);

	if (!ret) {
		ret = ksz8463_spi_readb_raw(spi, addr);
		ksz8463_spi_unlock(spi);
	}

	return ret;
}

/* Write n bytes over SPI. The SPI mutex should be held */
static int ksz8463_spi_write_raw(const struct spi_dt_spec *spi, uint16_t addr, const void *value,
				 size_t n)
{
	uint16_t reg;
	uint8_t cmd[KSZ8463_SPI_CMD_MAX_SIZE];

	struct spi_buf_set tx = {
		.buffers = &(const struct spi_buf){.buf = cmd, .len = sizeof(addr) + n},
		.count = 1u,
	};

	if (unlikely(n > KSZ8463_SPI_CMD_MAX_DATA_PH_SIZE)) {
		return -ENOTSUP;
	}

	reg = ksz8463_spi_cmd(addr, n);
	sys_put_be16(reg, cmd);
	cmd[0] |= KSZ8463_SPI_CMD_WR;
	if (n) {
		memcpy(cmd + sizeof(reg), value, n);
	}

	return spi_write_dt(spi, &tx);
}

/* Write 32 bits over SPI. The SPI mutex should be held */
static int ksz8463_spi_writel_raw(const struct spi_dt_spec *spi, uint16_t addr, uint32_t value)
{
	/* Must be 4-byte aligned */
	if (unlikely(addr & 0x03u)) {
		return -EFAULT;
	}

	LOG_DBG("Write 0x%04" PRIx16 "(0x%04" PRIx16 "): 0x%08" PRIx32, addr,
		ksz8463_spi_cmd(addr, sizeof(value)), value);
	return ksz8463_spi_write_raw(spi, addr, &value, sizeof(value));
}

/* Like ksz8463_spi_writel_raw but acquires the SPI mutex */
static inline int ksz8463_spi_writel(const struct spi_dt_spec *spi, uint16_t addr, uint32_t value)
{
	int ret = ksz8463_spi_lock(spi);

	if (!ret) {
		ret = ksz8463_spi_writel_raw(spi, addr, value);
		ksz8463_spi_unlock(spi);
	}
	return ret;
}

/* Update bits in 32 bit register. The new value is passed in the bits parameter
 * whereas the mask determines which bits are to be updated. The SPI mutex
 * should be held
 */
static int ksz8463_spi_updatel_raw(const struct spi_dt_spec *spi, uint16_t addr, uint32_t bits,
				   uint32_t mask)
{
	int ret;
	int64_t ret64;
	uint32_t be32;

	/* Mask must cover all set bits */
	if (unlikely(bits & ~mask)) {
		return -EINVAL;
	}

	ret64 = ksz8463_spi_readl_raw(spi, addr);
	if (ret64 < 0) {
		return (int)ret64;
	}
	be32 = (uint32_t)ret64 & ~mask;
	be32 |= bits;

	ret = 0;
	/* Skip write if possible */
	if (be32 != (uint32_t)ret64) {
		ret = ksz8463_spi_writel_raw(spi, addr, be32);
	}

	return ret;
}

/* Write 16 bit register over SPI, aligning accesses as required. The SPI mutex
 * should be held
 */
static int ksz8463_spi_writew_raw(const struct spi_dt_spec *spi, uint16_t addr, uint16_t value)
{
	/* Access supported only at 4 byte boundaries */
	if (addr & 0x02) {
		return ksz8463_spi_updatel_raw(spi, addr & ~0x02u, value << 16u, 0xffff0000u);
	}

	LOG_DBG("Write 0x%04x" PRIx16 "(0x%04" PRIx16 "): 0x%04" PRIx16, addr,
		ksz8463_spi_cmd(addr, sizeof(value)), value);
	return ksz8463_spi_write_raw(spi, addr, &value, sizeof(value));
}

/* Like ksz8463_spi_writew_raw but with acquisition of the SPI mutex */
static inline int ksz8463_spi_writew(const struct spi_dt_spec *spi, uint16_t addr, uint16_t value)
{
	int ret = ksz8463_spi_lock(spi);

	if (!ret) {
		ret = ksz8463_spi_writew_raw(spi, addr, value);
		ksz8463_spi_unlock(spi);
	}
	return ret;
}

/* Like ksz8463_spi_updatel_raw but with 16 bit registers */
static int ksz8463_spi_updatew_raw(const struct spi_dt_spec *spi, uint16_t addr, uint16_t bits,
				   uint16_t mask)
{
	int ret;
	uint16_t be16;

	if (unlikely(bits & ~mask)) {
		return -EINVAL;
	}

	ret = ksz8463_spi_readw_raw(spi, addr);
	if (ret < 0) {
		return ret;
	}

	be16 = (uint16_t)ret & ~mask;
	be16 |= bits;

	/* Skip write if possible */
	if (be16 != (uint16_t)ret) {
		ret = ksz8463_spi_writew_raw(spi, addr, be16);
	}

	return ret;
}

/* Write 8 bits over SPI, aligning access as required. The SPI mutex should be held */
static int ksz8463_spi_writeb_raw(const struct spi_dt_spec *spi, uint16_t addr, uint8_t value)
{
	if (addr & 0x03u) {
		return ksz8463_spi_updatew_raw(spi, addr & ~0x01u, value << ((addr & 0x01u) << 3u),
					       0xff << ((addr & 0x01u) << 3u));
	}

	LOG_DBG("Write 0x%04" PRIx16 "(0x%04" PRIx16 "): 0x%02" PRIx8, addr,
		ksz8463_spi_cmd(addr, sizeof(value)), value);
	return ksz8463_spi_write_raw(spi, addr, &value, sizeof(value));
}

/* Lock ksz8463_spi_writeb_raw but with automatic SPI lock acquisition */
static inline int ksz8463_spi_writeb(const struct spi_dt_spec *spi, uint16_t addr, uint8_t value)
{
	int ret = ksz8463_spi_lock(spi);

	if (!ret) {
		ret = ksz8463_spi_writeb_raw(spi, addr, value);
		ksz8463_spi_unlock(spi);
	}

	return ret;
}

/* Like ksz8463_spi_udpw_raw but with 8 bit registers */
static int ksz8463_spi_updateb_raw(const struct spi_dt_spec *spi, uint16_t addr, uint8_t bits,
				   uint8_t mask)
{
	int ret;
	uint8_t b;

	if (unlikely(bits & ~mask)) {
		return -EINVAL;
	}

	ret = ksz8463_spi_readb_raw(spi, addr);
	if (ret < 0) {
		return ret;
	}

	b = (uint8_t)ret & ~mask;
	b |= bits;

	if (b != (uint8_t)ret) {
		ret = ksz8463_spi_writeb_raw(spi, addr, b);
	}

	return ret;
}

/* Lock the SPI mutex and invoke ksz8463_spi_updateb_raw */
static inline int ksz8463_spi_updateb(const struct spi_dt_spec *spi, uint16_t addr, uint8_t bits,
				      uint8_t mask)
{
	int ret;

	ret = ksz8463_spi_lock(spi);
	if (!ret) {
		ret = ksz8463_spi_updateb_raw(spi, addr, bits, mask);
		ksz8463_spi_unlock(spi);
	}

	return ret;
}

static int ksz8463_eee_en_disen(const struct device *ptdev, bool enable, bool lock_spi)
{
	int ret;
	const struct device *dev;
	const struct ksz8463_config *cfg;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	struct ksz8463_port_config *prv_cfg = dsa_cfg->prv_config;
	struct dsa_switch_context *dsa_switch_ctx = ptdev->data;
	struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;

	if (prv_cfg->eee_enabled == enable) {
		return 0;
	}

	dev = prv_data->dev;
	cfg = dev->config;

	if (unlikely(dsa_cfg->port_idx >= KSZ8463_NUM_USER_PORTS)) {
		return -EINVAL;
	}

	if (lock_spi) {
		ret = ksz8463_spi_lock(&cfg->spi);
		if (ret) {
			return ret;
		}
	}

	BUILD_ASSERT(KSZ8463_PCSEEEC_P1_NEXT_PG_EN == BIT(0));
	BUILD_ASSERT(KSZ8463_PCSEEEC_P2_NEXT_PG_EN == BIT(1));

	ret = ksz8463_spi_updateb_raw(&cfg->spi, KSZ8463_REG_PCSEEEC,
				      enable ? BIT(dsa_cfg->port_idx) : 0, BIT(dsa_cfg->port_idx));
	if (!ret) {
		prv_cfg->eee_enabled = enable;
	}

	if (lock_spi) {
		ksz8463_spi_unlock(&cfg->spi);
	}

	return ret;
}

static inline int ksz8463_eee_disable(const struct device *ptdev)
{
	return ksz8463_eee_en_disen(ptdev, false, true);
}

static inline int ksz8463_eee_disable_raw(const struct device *ptdev)
{
	return ksz8463_eee_en_disen(ptdev, false, false);
}

static inline int ksz8463_eee_enable_raw(const struct device *ptdev)
{
	return ksz8463_eee_en_disen(ptdev, true, false);
}

static int ksz8463_link_ready(const struct device *ptdev)
{
	int ret;
	const struct device *dev;
	bool base100, full_duplex;
	enum phy_link_speed speed;
	const struct ksz8463_config *cfg;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	struct ksz8463_port_config *prv_cfg = dsa_cfg->prv_config;
	const struct dsa_switch_context *dsa_switch_ctx = ptdev->data;
	const struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;

	dev = prv_data->dev;
	cfg = dev->config;

	ret = ksz8463_spi_readb(&cfg->spi, KSZ8463_REG_PxSR_HI(dsa_cfg->port_idx));
	if (ret < 0) {
		return ret;
	}

	base100 = !!(ret & KSZ8463_PxSR_HI_OPER_SPEED_100MBPS);
	full_duplex = !!(ret & KSZ8463_PxSR_HI_OPER_FULL_DUPLEX);

	switch ((base100 << 8) | full_duplex) {
	case (1 << 8) | 1: /* 100Mbps, full-duplex */
		speed = LINK_FULL_100BASE;
		break;
	case (1 << 8) | 0: /* 100Mbps, half-duplex */
		speed = LINK_HALF_100BASE;
		break;
	case (0 << 8) | 1: /* 10Mbps, full-duplex */
		speed = LINK_FULL_10BASE;
		break;
	case (0 << 8) | 0: /* 10Mbps, half-duplex */
		speed = LINK_HALF_10BASE;
		break;
	default:
		CODE_UNREACHABLE;
	}

	LOG_INF("Speed 10%sMbps, %s-duplex", PHY_LINK_IS_SPEED_100M(speed) ? "0" : "",
		PHY_LINK_IS_FULL_DUPLEX(speed) ? "full" : "half");

	ret = k_mutex_lock(prv_cfg->link_mutex, K_FOREVER);
	if (!ret) {
		prv_cfg->link_ready = true;
		if (prv_cfg->phy_up) {
			ret = ksz8463_port_carrier_on(ptdev);
		}
		k_mutex_unlock(prv_cfg->link_mutex);
	}

	return ret;
}

static int ksz8463_autoneg_en_disen(const struct device *ptdev, bool enable)
{
	int ret, r;
	bool disable_eee;
	const struct device *dev;
	const struct ksz8463_config *cfg;
	const uint8_t bit = KSZ8463_PxCR4_LO_AUTONEG_EN;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	struct ksz8463_port_config *prv_cfg = dsa_cfg->prv_config;
	const struct dsa_switch_context *dsa_switch_ctx = ptdev->data;
	const struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;

	if (prv_cfg->autoneg_enabled == enable) {
		return 0;
	}

	dev = prv_data->dev;
	cfg = dev->config;

	ret = ksz8463_spi_lock(&cfg->spi);
	if (ret) {
		return ret;
	}

	/* Errata 4 from KSZ8463_MLI_FMLI_PROD_0.3_errata. Temporarily disable
	 * EEE when disabling auto-negotiation
	 */
	disable_eee = prv_cfg->eee_enabled && !enable;
	if (disable_eee) {
		ret = ksz8463_eee_disable_raw(ptdev);
	}

	if (!ret) {
		ret = ksz8463_spi_updateb_raw(&cfg->spi, KSZ8463_REG_PxCR4_LO(dsa_cfg->port_idx),
					      bit * enable, bit);

		if (disable_eee) {
			r = ksz8463_eee_enable_raw(ptdev);
			if (r) {
				LOG_ERR("Error reenabling EEE on port %d: %d", dsa_cfg->port_idx,
					-ret);
			}
		}
	}

	if (!ret) {
		prv_cfg->autoneg_enabled = enable;
	}
	ksz8463_spi_unlock(&cfg->spi);
	return ret;
}

static int ksz8463_enable_autoneg(const struct device *ptdev)
{
	return ksz8463_autoneg_en_disen(ptdev, true);
}

static int ksz8463_disable_autoneg(const struct device *ptdev)
{
	return ksz8463_autoneg_en_disen(ptdev, false);
}

static int ksz8463_configure_fix_link(const struct device *ptdev)
{
	int ret;
	uint8_t bits;
	const struct device *dev;
	const struct ksz8463_config *cfg;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	const struct dsa_switch_context *dsa_switch_ctx = ptdev->data;
	const struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;
	const uint8_t mask =
		KSZ8463_PxMBCR_HI_FORCE_100BASE_TX | KSZ8463_PxMBCR_HI_FORCE_FULL_DUPLEX;

	dev = prv_data->dev;
	cfg = dev->config;
	bits = 0;

	switch (cfg->fixed_speed) {
	case KSZ8463_SPEED_10BASE_HALF_DUPLEX:
		break;
	case KSZ8463_SPEED_10BASE_FULL_DUPLEX:
		bits |= KSZ8463_PxMBCR_HI_FORCE_FULL_DUPLEX;
		break;
	case KSZ8463_SPEED_100BASE_HALF_DUPLEX:
		bits |= KSZ8463_PxMBCR_HI_FORCE_100BASE_TX;
		break;
	case KSZ8463_SPEED_100BASE_FULL_DUPLEX:
		bits = mask;
		break;
	default:
		return -EINVAL;
	}

	ret = ksz8463_disable_autoneg(ptdev);
	if (!ret) {
		ret = ksz8463_spi_updateb(&cfg->spi, KSZ8463_REG_PxMBCR_HI(dsa_cfg->port_idx), bits,
					  mask);
	}
	if (!ret) {
		ret = ksz8463_link_ready(ptdev);
	}

	return ret;
}

/* 1 - link is up, 0 - link is not up, -errno - error */
static int ksz8463_link_good(const struct device *ptdev)
{
	int ret;
	const struct device *dev;
	const struct ksz8463_config *cfg;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	const struct dsa_switch_context *dsa_switch_ctx = ptdev->data;
	const struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;

	dev = prv_data->dev;
	cfg = dev->config;

	if (unlikely(ksz8463_is_cpu_port(ptdev))) {
		return -EINVAL;
	}

	ret = ksz8463_spi_readb(&cfg->spi, KSZ8463_REG_PxSR_LO(dsa_cfg->port_idx));

	return ret < 0 ? ret : !!(ret & KSZ8463_PxSR_LO_LINK_STATUS);
}

/* 1 - capable, 0 - incapable, -errno - error */
static int ksz8463_partner_is_autoneg_capable(const struct device *ptdev)
{
	int ret;
	const struct device *dev;
	const struct ksz8463_config *cfg;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	const struct dsa_switch_context *dsa_switch_ctx = ptdev->data;
	const struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;

	dev = prv_data->dev;
	cfg = dev->config;

	if (unlikely(ksz8463_is_cpu_port(ptdev))) {
		/* CPU port should use fix link */
		return 0;
	}

	ret = ksz8463_spi_readb(&cfg->spi, KSZ8463_REG_PxEEECS_LO(dsa_cfg->port_idx));

	return ret < 0 ? ret : !!(ret & KSZ8463_PxEEECS_LO_LNK_AUTONEG_CPBL);
}

static inline int ksz8463_autoneg_restart(const struct device *ptdev)
{
	const struct device *dev;
	const struct ksz8463_config *cfg;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	const struct dsa_switch_context *dsa_switch_ctx = ptdev->data;
	const struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;

	dev = prv_data->dev;
	cfg = dev->config;

	/* Restart bit is self-clearing */
	return ksz8463_spi_updateb(&cfg->spi, KSZ8463_REG_PxCR4_HI(dsa_cfg->port_idx),
				   KSZ8463_PxCR4_HI_AUTONEG_RESTART,
				   KSZ8463_PxCR4_HI_AUTONEG_RESTART);
}

static int ksz8463_start_autoneg(const struct device *ptdev)
{
	int ret;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	struct ksz8463_port_config *prv_cfg = dsa_cfg->prv_config;

	ret = ksz8463_enable_autoneg(ptdev);
	if (!ret) {
		ret = ksz8463_autoneg_restart(ptdev);
	}
	if (!ret) {
		prv_cfg->autoneg_expiry = sys_timepoint_calc(K_MSEC(prv_cfg->autoneg_timeout));
		k_work_reschedule(&prv_cfg->autoneg_dwork, K_MSEC(prv_cfg->autoneg_intvl));
	}

	return ret;
}

static int ksz8463_configure_link(const struct device *ptdev)
{
	int ret = ksz8463_partner_is_autoneg_capable(ptdev);

	switch (ret) {
	case 0:
		LOG_DBG("Link partner does not support auto-negotiation");
		ret = ksz8463_configure_fix_link(ptdev);
		break;
	case 1:
		ret = ksz8463_start_autoneg(ptdev);
		break;
	default:
		break;
	}

	return ret;
}

static inline void ksz8463_cancel_autoneg(const struct device *ptdev)
{
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	struct ksz8463_port_config *prv_cfg = dsa_cfg->prv_config;

	k_work_cancel_delayable(&prv_cfg->autoneg_dwork);
}

static int ksz8463_link_status_changed(const struct device *dev)
{
	int ret;
	const struct device *ptdev;
	const struct dsa_port_config *dsa_cfg;
	struct ksz8463_port_config *prv_cfg;
	struct ksz8463_data *data = dev->data;
	const struct ksz8463_config *cfg = dev->config;

	ret = 0;
	for (unsigned int i = 0u; !ret && i < cfg->num_ptdevs; ++i) {
		ptdev = data->ptdevs[i];

		/* CPU port's link status won't change */
		if (!ptdev || ksz8463_is_cpu_port(ptdev)) {
			continue;
		}

		dsa_cfg = ptdev->config;
		prv_cfg = dsa_cfg->prv_config;

		ret = ksz8463_link_good(ptdev);
		switch (ret) {
		case 0:
			ret = k_mutex_lock(prv_cfg->link_mutex, K_FOREVER);
			if (!ret) {
				if (prv_cfg->link_ready) {
					ret = ksz8463_port_carrier_off(ptdev);
					ksz8463_cancel_autoneg(ptdev);
					prv_cfg->link_ready = false;
				}
				k_mutex_unlock(prv_cfg->link_mutex);
			}
			break;
		case 1:
			if (!prv_cfg->link_ready) {
				ret = ksz8463_configure_link(ptdev);
			}
			break;
		default:
			LOG_ERR("Error determining link state: %d", -ret);
			break;
		}
	}

	return ret;
}

static void ksz8463_chip_isr(const struct device *gpio_dev, struct gpio_callback *cb,
			     gpio_port_pins_t pins)
{
	struct ksz8463_data *data = CONTAINER_OF(cb, struct ksz8463_data, chip_cb);

	k_work_submit(&data->chip_isr_work);
}

static void ksz8463_chip_isr_work(struct k_work *work)
{
	int ret;
	unsigned int key;
	bool lcis_cleared;
	const struct device *dev, *ptdev;
	const struct ksz8463_config *cfg;
	const struct ksz8463_prv_data *prv_data;
	const struct dsa_switch_context *dsa_switch_ctx;
	struct ksz8463_data *data = CONTAINER_OF(work, struct ksz8463_data, chip_isr_work);

	/* There is at least one non-null entry in the ptdevs array. Find it */
	for (ptdev = data->ptdevs[0]; !ptdev; ++ptdev) {
	}

	dsa_switch_ctx = ptdev->data;
	prv_data = dsa_switch_ctx->prv_data;
	dev = prv_data->dev;
	cfg = dev->config;

	key = irq_lock();

	/* Chip's internal interrupt state may change even though irq_lock guarantees
	 * the ISR ISR won't be invoked here. Repeatedly act on the interrupt state
	 * until the LCIS interrupt is low
	 */
	do {
		ret = ksz8463_spi_readb(&cfg->spi, KSZ8463_REG_ISR_HI);
		if (ret < 0) {
			LOG_ERR("Error reading interrupt status: %d", -ret);
			break;
		}

		if (unlikely(!(ret & KSZ8463_ISR_HI_LCIS))) {
			break;
		}

		/* Write 1 to clear */
		ret = ksz8463_spi_updateb(&cfg->spi, KSZ8463_REG_ISR_HI, KSZ8463_ISR_HI_LCIS,
					  KSZ8463_ISR_HI_LCIS);

		lcis_cleared = !ret;
		if (!lcis_cleared) {
			LOG_ERR("Could not clear LCIS: %d", -ret);
		}

		ret = ksz8463_link_status_changed(dev);
		if (ret) {
			LOG_ERR("Error acting on link change: %d", -ret);
			break;
		}
	} while (lcis_cleared);

	irq_unlock(key);
}

static void ksz8463_autoneg_work(struct k_work *work)
{
	int ret;
	const struct device *dev, *ptdev;
	const struct ksz8463_config *cfg;
	const struct dsa_port_config *dsa_cfg;
	const struct ksz8463_prv_data *prv_data;
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	const struct dsa_switch_context *dsa_switch_ctx;
	struct ksz8463_port_config *prv_cfg =
		CONTAINER_OF(dwork, struct ksz8463_port_config, autoneg_dwork);

	ptdev = prv_cfg->ptdev;
	dsa_cfg = ptdev->config;
	dsa_switch_ctx = ptdev->data;
	prv_data = dsa_switch_ctx->prv_data;
	dev = prv_data->dev;
	cfg = dev->config;

	ret = ksz8463_spi_readb(&cfg->spi, KSZ8463_REG_PxMBSR_LO(dsa_cfg->port_idx));
	if (ret < 0) {
		LOG_ERR("Error reading P%dMBSR: %d", dsa_cfg->port_idx, -ret);
		return;
	}

	if (ret & KSZ8463_PxMBSR_AUTONEG_CPLT) {
		LOG_DBG("Auto-negotiation completed on port %d", dsa_cfg->port_idx);
		ret = ksz8463_link_ready(ptdev);
	} else if (sys_timepoint_expired(prv_cfg->autoneg_expiry)) {
		LOG_INF("Auto-negotiation timed out, falling back on fix speed");
		ret = ksz8463_configure_fix_link(ptdev);
	} else {
		LOG_DBG("Auto-negotiation in process, rescheduling");
		ret = k_work_reschedule(&prv_cfg->autoneg_dwork, K_MSEC(prv_cfg->autoneg_intvl));
	}

	if (ret < 0) {
		LOG_ERR("Error polling for auto-negotiation completion: %d", -ret);
	}
}

static int ksz8463_validate_phy_mode(const struct device *ptdev)
{
	int ret;
	const struct device *dev;
	struct ksz8463_data *data;
	struct ksz8463_prv_data *prv_data;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	struct dsa_switch_context *dsa_switch_ctx = ptdev->data;

	prv_data = dsa_switch_ctx->prv_data;
	dev = prv_data->dev;
	data = dev->data;

	ret = -ENOTSUP;
	switch (data->chip_id) {
	case KSZ8463_MII_CHIP_ID:
		if (!strcmp(dsa_cfg->phy_mode, "mii")) {
			ret = 0;
		}
		break;
	case KSZ8463_RMII_CHIP_ID:
		if (!strcmp(dsa_cfg->phy_mode, "rmii")) {
			ret = 0;
		}
		break;
	default:
		break;
	}

	if (ret) {
		LOG_ERR("Invalid PHY mode %s on CPU port", dsa_cfg->phy_mode);
	}
	return ret;
}

static inline int ksz8463_enable_port(const struct device *ptdev)
{
	const struct device *dev;
	const struct ksz8463_config *cfg;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	const struct dsa_switch_context *dsa_switch_ctx = ptdev->data;
	const struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;
	const uint8_t bits = KSZ8463_PxCR2_HI_TX_EN | KSZ8463_PxCR2_HI_RX_EN;
	const uint8_t mask = bits | KSZ8463_PxCR2_HI_LEARN_DIS;

	dev = prv_data->dev;
	cfg = dev->config;

	return ksz8463_spi_updateb(&cfg->spi, KSZ8463_REG_PxCR2_HI(dsa_cfg->port_idx), bits, mask);
}

static int ksz8463_port_init(const struct device *ptdev)
{
	int ret;
	const struct device *dev;
	struct ksz8463_data *data;
	const struct ksz8463_config *cfg;
	struct ksz8463_port_config *prv_cfg;
	const struct ksz8463_prv_data *prv_data;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	struct dsa_switch_context *dsa_switch_ctx = ptdev->data;

	prv_data = dsa_switch_ctx->prv_data;
	dev = prv_data->dev;
	data = dev->data;
	cfg = dev->config;
	prv_cfg = dsa_cfg->prv_config;

	if (unlikely(dsa_cfg->port_idx >= cfg->num_ptdevs)) {
		return -EINVAL;
	}

	data->ptdevs[dsa_cfg->port_idx] = ptdev;

	k_work_init_delayable(&prv_cfg->autoneg_dwork, ksz8463_autoneg_work);

	ret = 0;
	if (ksz8463_is_cpu_port(ptdev)) {
		ret = ksz8463_validate_phy_mode(ptdev);
	} else if (prv_cfg->disable_eee) {
		ret = ksz8463_eee_disable(ptdev);
	}
	if (!ret) {
		ret = ksz8463_enable_port(ptdev);
	}

	return ret;
}

static int ksz8463_configure_irq(const struct device *dev)
{
	int ret;
	struct ksz8463_data *data = dev->data;
	const struct ksz8463_config *cfg = dev->config;

	if (unlikely(cfg->irq_gpio)) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(cfg->irq_gpio, GPIO_INPUT);
	if (!ret) {
		gpio_init_callback(&data->chip_cb, ksz8463_chip_isr, BIT(cfg->irq_gpio->pin));
		gpio_add_callback(cfg->irq_gpio->port, &data->chip_cb);
		ret = gpio_pin_interrupt_configure_dt(cfg->irq_gpio, GPIO_INT_EDGE_RISING);
	}

	if (!ret) {
		ret = ksz8463_spi_writeb(&cfg->spi, KSZ8463_REG_IER_HI, KSZ8463_IER_HI_LCIE);
	}
	if (!ret) {
		LOG_DBG("Link-change interrupt configured");
	}

	return ret;
}

static inline int ksz8463_configure_leds(const struct device *dev)
{
	const struct ksz8463_config *cfg = dev->config;

	return ksz8463_spi_updateb(&cfg->spi, KSZ8463_REG_SGCR7_HI, cfg->led_mode,
				   KSZ8463_SGCR7_HI_PORT_LED_MODE_MASK);
}

static inline int ksz8463_configure_legal_pkt_size_chk(const struct device *dev)
{
	const struct ksz8463_config *cfg = dev->config;
	const uint8_t bit = KSZ8463_SGCR2_LO_LEGAL_PKT_SZ_CHK_EN;

	return ksz8463_spi_updateb(&cfg->spi, KSZ8463_REG_SGCR2_LO, cfg->pkt_sz_chk_en ? bit : 0,
				   bit);
}

static inline int ksz8463_configure_snooping(const struct device *dev)
{
	const struct ksz8463_config *cfg = dev->config;
	const uint8_t bits = (KSZ8463_SGCR2_HI_MLD_SNOOP_EN * cfg->mld_snoop_en) |
			     (KSZ8463_SGCR2_HI_IGMP_SNOOP_EN * cfg->igmp_snoop_en);
	const uint8_t mask = KSZ8463_SGCR2_HI_MLD_SNOOP_EN | KSZ8463_SGCR2_HI_IGMP_SNOOP_EN;

	return ksz8463_spi_updateb(&cfg->spi, KSZ8463_REG_SGCR2_HI, bits, mask);
}

static int ksz8463_configure_autoneg(const struct device *dev)
{
	int ret;
	const struct device *ptdev;
	struct ksz8463_data *data = dev->data;
	const struct ksz8463_config *cfg = dev->config;

	ret = 0;
	for (unsigned int i = 0u; !ret && i < cfg->num_ptdevs; ++i) {
		ptdev = data->ptdevs[i];
		if (unlikely(!ptdev || ksz8463_is_cpu_port(ptdev))) {
			continue;
		}

		ret = ksz8463_link_good(ptdev);
		if (ret == 1) {
			ret = ksz8463_configure_link(ptdev);
		}
	}

	return ret;
}

static int ksz8463_switch_setup(const struct dsa_switch_context *dsa_switch_ctx)
{
	int ret;
	const struct device *dev;
	struct ksz8463_data *data;
	const struct ksz8463_prv_data *prv_data = dsa_switch_ctx->prv_data;

	dev = prv_data->dev;
	data = dev->data;

	ret = 0;
	if (ksz8463_have_irq_gpio(dev)) {
		k_work_init(&data->chip_isr_work, ksz8463_chip_isr_work);
		ret = ksz8463_configure_irq(dev);
	}

	if (!ret) {
		ret = ksz8463_configure_leds(dev);
	}
	if (!ret) {
		ret = ksz8463_configure_legal_pkt_size_chk(dev);
	}
	if (!ret) {
		ret = ksz8463_configure_snooping(dev);
	}
	if (!ret) {
		ret = ksz8463_configure_autoneg(dev);
	}

	return ret;
}

static void ksz8463_port_phylink_change(const struct device *phydev, struct phy_link_state *state,
					const struct device *ptdev)
{
	int ret;
	const struct dsa_port_config *dsa_cfg = ptdev->config;
	struct ksz8463_port_config *prv_cfg = dsa_cfg->prv_config;

	ret = k_mutex_lock(prv_cfg->link_mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("Could not lock link mutex: %d", -ret);
		return;
	}

	if (state->is_up && prv_cfg->link_ready) {
		ret = ksz8463_port_carrier_on(ptdev);
	} else {
		ret = ksz8463_port_carrier_off(ptdev);
		ksz8463_cancel_autoneg(ptdev);
	}

	if (ret) {
		LOG_ERR("Error acting on PHY link change: %d", -ret);
	}

	prv_cfg->phy_up = state->is_up;
	k_mutex_unlock(prv_cfg->link_mutex);
}

static enum ethernet_hw_caps ksz8463_get_capabilities(const struct device *ptdev)
{
	/* Leave ETHERNET_PTP unset until driver supports it */
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;
}

static int ksz8463_hard_reset(const struct device *dev)
{
	int ret;
	const struct ksz8463_config *cfg = dev->config;

	if (unlikely(&cfg->rst_gpio || !gpio_is_ready_dt(cfg->rst_gpio))) {
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(cfg->rst_gpio, GPIO_OUTPUT_ACTIVE);
	if (!ret) {
		k_sleep(K_MSEC(10));
		ret = gpio_pin_set_dt(cfg->rst_gpio, 0);
	}
	if (!ret) {
		k_sleep(K_MSEC(10));
		LOG_DBG("Reset complete");
	}

	return ret;
}

static int ksz8463_soft_reset(const struct device *dev)
{
	int ret;
	const struct ksz8463_config *cfg = dev->config;

	ret = ksz8463_spi_lock(&cfg->spi);
	if (ret) {
		return ret;
	}

	ret = ksz8463_spi_writeb_raw(&cfg->spi, KSZ8463_REG_GRR_LO, KSZ8463_GRR_GBL_SOFT_RST);
	if (!ret) {
		k_sleep(K_MSEC(10));
		ret = ksz8463_spi_writeb_raw(&cfg->spi, KSZ8463_REG_GRR_LO, 0);
	}

	ksz8463_spi_unlock(&cfg->spi);

	if (!ret) {
		LOG_DBG("Reset complete");
	}
	return ret;
}

static int ksz8463_read_chip_id(const struct device *dev)
{
	int ret;
	unsigned int intvl;
	k_timepoint_t expiry;
	struct ksz8463_data *data = dev->data;
	const struct ksz8463_config *cfg = dev->config;

	expiry = sys_timepoint_calc(K_MSEC(cfg->spi_oper_timeout));
	intvl = MIN(10, cfg->spi_oper_timeout >> 1u);

	do {
		ret = ksz8463_spi_readb(&cfg->spi, KSZ8463_REG_CIDER);
		if (ret < 0) {
			return ret;
		}

		if (ret >> 8 == KSZ8463_FAMILY_ID) {
			break;
		}

		if (sys_timepoint_expired(expiry)) {
			LOG_ERR("Timed out waiting for SPI");
			return -ETIMEDOUT;
		}

		k_sleep(K_MSEC(intvl));
	} while (1);

	data->chip_id = (uint8_t)(ret & 0xff);
	LOG_DBG("Chip ID 0x%02" PRIx8, data->chip_id);
	return 0;
}

static int ksz8463_init(const struct device *dev)
{
	int ret;
	struct ksz8463_data *data = dev->data;
	const struct ksz8463_config *cfg = dev->config;

	ret = k_mutex_init(&data->spi_mutex);
	if (!ret && cfg->pincfg) {
		ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	}
	if (!ret && ksz8463_have_hard_reset(dev)) {
		ret = ksz8463_hard_reset(dev);
	}
	if (!ret) {
		ret = ksz8463_read_chip_id(dev);
	}
	if (!ret && !ksz8463_have_hard_reset(dev)) {
		/* SPI is operational here, soft reset possible */
		ret = ksz8463_soft_reset(dev);
	}

	return ret;
}

static struct dsa_api ksz8463_dsa_api = {
	.port_init = ksz8463_port_init,
	.switch_setup = ksz8463_switch_setup,
	.port_phylink_change = ksz8463_port_phylink_change,
	.get_capabilities = ksz8463_get_capabilities,
};

#define KSZ8463_PINCTRL_DT_DEFINE(node_id)                                                         \
	COND_CODE_1(DT_NUM_PINCTRL_STATES(node_id),						   \
	(											   \
		LISTIFY(									   \
			DT_NUM_PINCTRL_STATES(node_id),						   \
			Z_PINCTRL_STATE_PINS_DEFINE,						   \
			(;),									   \
			node_id									   \
		);										   \
												   \
		static const struct pinctrl_state						   \
			Z_PINCTRL_STATES_NAME(node_id)[] = {					   \
			LISTIFY(								   \
				DT_NUM_PINCTRL_STATES(node_id),					   \
				Z_PINCTRL_STATE_INIT,						   \
				(,),								   \
				node_id								   \
			)									   \
		};										   \
												   \
		Z_PINCTRL_DEV_CONFIG_STATIC Z_PINCTRL_DEV_CONFIG_CONST				   \
			struct pinctrl_dev_config						   \
				Z_PINCTRL_DEV_CONFIG_NAME(node_id) =				   \
				Z_PINCTRL_DEV_CONFIG_INIT(node_id)				   \
	), (EMPTY))

#define KSZ8463_PINCTRL_DT_INST_DEFINE(n) KSZ8463_PINCTRL_DT_DEFINE(DT_DRV_INST(n))

#define KSZ8463_PINCTRL_DT_INST_CFG_OR_NULL(n)                                                     \
	COND_CODE_1(DT_INST_NUM_PINCTRL_STATES(n),						   \
		(PINCTRL_DT_INST_DEV_CONFIG_GET(n)),						   \
		(NULL))

#define KSZ8463_GPIO_DT_SPEC_OR_NULL(n, prop)                                                      \
	COND_CODE_1(DT_PROP_HAS_IDX(DT_DRV_INST(n), prop, 0),					   \
		(&(struct gpio_dt_spec) GPIO_DT_SPEC_GET_BY_IDX(				   \
			DT_DRV_INST(n), prop, 0							   \
		)), (NULL))

#define KSZ8463_IS_CPU_PORT(node_id) DT_NODE_HAS_PROP(node_id, ethernet)

#define KSZ8463_IS_USER_PORT(node_id) DT_NODE_HAS_PROP(node_id, phy_handle)

#define KSZ8463_PORT_INIT(pt, n)                                                                   \
	BUILD_ASSERT(DT_PROP(pt, microchip_autoneg_poll_interval) <                                \
			      DT_PROP(pt, microchip_autoneg_timeout),                              \
		      "Auto-negotiation poll must be less than timeout");                          \
                                                                                                   \
	/* The CPU port needs a phy-connection-type */                                             \
	BUILD_ASSERT(!DT_NODE_HAS_PROP(pt, ethernet) ||                                            \
			      DT_NODE_HAS_PROP(pt, phy_connection_type),                           \
		      "phy-connection-type required for CPU port");                                \
                                                                                                   \
	/* Each port should name an ethernet or phy-handle property */                             \
	BUILD_ASSERT(DT_NODE_HAS_PROP(pt, ethernet) || DT_NODE_HAS_PROP(pt, phy_handle),           \
		      "CPU port requires an ethernet phandle, user ports a "                       \
		      "phy-handle phandle");                                                       \
                                                                                                   \
	static K_MUTEX_DEFINE(ksz8463_link_mutex_##pt##n);                                         \
                                                                                                   \
	static struct ksz8463_port_config ksz8463_##n##pt = {                                      \
		.disable_eee = DT_PROP(pt, microchip_disable_eee),                                 \
		.eee_enabled = true,                                                               \
		.autoneg_enabled = true,                                                           \
		.autoneg_intvl = DT_PROP(pt, microchip_autoneg_poll_interval),                     \
		.autoneg_timeout = DT_PROP(pt, microchip_autoneg_timeout),                         \
		.link_mutex = &ksz8463_link_mutex_##pt##n,                                         \
		.ptdev = DEVICE_DT_GET(pt),                                                        \
	};                                                                                         \
                                                                                                   \
	static const struct dsa_port_config ksz8463_pcg_##n##pt = {                                \
		.mcfg = NET_ETH_MAC_DT_CONFIG_INIT(pt),                                            \
		.port_idx = DT_REG_ADDR(pt),                                                       \
		.phy_dev = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(pt, phy_handle)),                      \
		.phy_mode = DT_PROP_OR(pt, phy_connection_type, "internal"),                       \
		.ethernet_connection = DEVICE_DT_GET_OR_NULL(DT_PHANDLE(pt, ethernet)),            \
		.prv_config = &ksz8463_##n##pt,                                                    \
	};                                                                                         \
                                                                                                   \
	DSA_PORT_INST_INIT(pt, n, &ksz8463_pcg_##n##pt);

/* clang-format and checkpatch disagree on spacing */
/* clang-format off */
#define KSZ8463_INST_PTDEV_ARRAY(n)								   \
	(const struct device * [DT_INST_CHILD_NUM(n)]) { 0 }
/* clang-format on */

#define KSZ8463_INIT(n)                                                                            \
	KSZ8463_PINCTRL_DT_INST_DEFINE(n);                                                         \
                                                                                                   \
	static struct ksz8463_data ksz8463_data_##n = {                                            \
		.ptdevs = KSZ8463_INST_PTDEV_ARRAY(n),                                             \
	};                                                                                         \
                                                                                                   \
	/* Pedantic overflow check */                                                              \
	BUILD_ASSERT(DT_INST_CHILD_NUM(n) < UINT8_MAX);                                            \
                                                                                                   \
	/* Overflow? */                                                                            \
	BUILD_ASSERT(DT_INST_PROP(n, microchip_spi_operational_timeout) <= UINT16_MAX,             \
		      "microchip,spi-operational-timeout may be at most 65535");                   \
                                                                                                   \
	/* Must be exactly one CPU port */                                                         \
	BUILD_ASSERT(DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(n, KSZ8463_IS_CPU_PORT, (+)) == 1u,     \
		      "The ethernet phandle must be set for exactly one "                          \
		      "port - the CPU port.");                                                     \
                                                                                                   \
	/* Remaining ports - the user ports - all need a phy-handle */                             \
	BUILD_ASSERT(DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(n, KSZ8463_IS_USER_PORT, (+)) ==        \
			      DT_INST_CHILD_NUM_STATUS_OKAY(n) - 1u,                               \
		      "At least one user port is missing the phy-handle "                          \
		      "property");                                                                 \
                                                                                                   \
	BUILD_ASSERT(DT_INST_ENUM_IDX(n, microchip_port_led_mode) <= BIT_MASK(2),                  \
		      "Invalid choice of LED mode");                                               \
                                                                                                   \
	BUILD_ASSERT(DT_INST_ENUM_IDX(n, microchip_fixed_link_speed) <= KSZ8463_SPEED_MAX,         \
		      "Invalid choice for microchip,fixed-link-speed");                            \
                                                                                                   \
	BUILD_ASSERT(DT_INST_PROP(n, microchip_spi_operational_timeout) > 1,                       \
		      "Invalid operational timeout");                                              \
                                                                                                   \
	static const struct ksz8463_config ksz8463_config_##n = {                                  \
		.mld_snoop_en = DT_INST_PROP(n, microchip_mld_snoop_en),                           \
		.igmp_snoop_en = DT_INST_PROP(n, microchip_igmp_snoop_en),                         \
		.num_ptdevs = DT_INST_CHILD_NUM(n),                                                \
		.led_mode = DT_INST_ENUM_IDX(n, microchip_port_led_mode),                          \
		.fixed_speed = DT_INST_ENUM_IDX(n, microchip_fixed_link_speed),                    \
		.spi_oper_timeout = DT_INST_PROP(n, microchip_spi_operational_timeout),            \
		.pkt_sz_chk_en = DT_INST_PROP(n, microchip_legal_packet_size_check_en),            \
		.rst_gpio = KSZ8463_GPIO_DT_SPEC_OR_NULL(n, reset_gpios),                          \
		.irq_gpio = KSZ8463_GPIO_DT_SPEC_OR_NULL(n, int_gpios),                            \
		.dev = DEVICE_DT_INST_GET(n),                                                      \
		.pincfg = KSZ8463_PINCTRL_DT_INST_CFG_OR_NULL(n),                                  \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, ksz8463_init, NULL, &ksz8463_data_##n, &ksz8463_config_##n,       \
			      POST_KERNEL, CONFIG_ETH_INIT_PRIORITY, NULL);                        \
                                                                                                   \
	struct ksz8463_prv_data ksz8463_prv_data_##n = {                                           \
		.dev = DEVICE_DT_INST_GET(n),                                                      \
	};                                                                                         \
                                                                                                   \
	BUILD_ASSERT(DT_INST_CHILD_NUM_STATUS_OKAY(n), "No ports enabled");                        \
                                                                                                   \
	DSA_SWITCH_INST_INIT(n, &ksz8463_dsa_api, &ksz8463_prv_data_##n, KSZ8463_PORT_INIT);

DT_INST_FOREACH_STATUS_OKAY(KSZ8463_INIT)
