// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Driver for Microsemi VSC85xx PHYs
 *
 * Author: Nagaraju Lakkaraju
 * License: Dual MIT/GPL
 * Copyright (c) 2016 Microsemi Corporation
 */

// #include <linux/firmware.h>
// #include <linux/jiffies.h>
// #include <linux/kernel.h>
// #include <linux/module.h>
// #include <linux/mdio.h>
// #include <linux/mii.h>
// #include <linux/phy.h>
// #include <linux/of.h>
// #include <linux/netdevice.h>
#include "mscc-phy-vsc8531.h"
#include "mscc_serdes.h"
#include "mscc.h"
#include "phy.h"
#include "firmware.h"
#include "misc.h"
#include "ethtool.h"

#include <errno.h>
#include <stdio.h>
#include <stdbool.h>

unsigned long volatile jiffies = 0;

static const struct vsc85xx_hw_stat vsc85xx_hw_stats[] = {
	{
		.string	= "phy_receive_errors",
		.reg	= MSCC_PHY_ERR_RX_CNT,
		.page	= MSCC_PHY_PAGE_STANDARD,
		.mask	= ERR_CNT_MASK,
	}, {
		.string	= "phy_false_carrier",
		.reg	= MSCC_PHY_ERR_FALSE_CARRIER_CNT,
		.page	= MSCC_PHY_PAGE_STANDARD,
		.mask	= ERR_CNT_MASK,
	}, {
		.string	= "phy_cu_media_link_disconnect",
		.reg	= MSCC_PHY_ERR_LINK_DISCONNECT_CNT,
		.page	= MSCC_PHY_PAGE_STANDARD,
		.mask	= ERR_CNT_MASK,
	}, {
		.string	= "phy_cu_media_crc_good_count",
		.reg	= MSCC_PHY_CU_MEDIA_CRC_VALID_CNT,
		.page	= MSCC_PHY_PAGE_EXTENDED,
		.mask	= VALID_CRC_CNT_CRC_MASK,
	}, {
		.string	= "phy_cu_media_crc_error_count",
		.reg	= MSCC_PHY_EXT_PHY_CNTL_4,
		.page	= MSCC_PHY_PAGE_EXTENDED,
		.mask	= ERR_CNT_MASK,
	},
};

static const struct vsc85xx_hw_stat vsc8584_hw_stats[] = {
	{
		.string	= "phy_receive_errors",
		.reg	= MSCC_PHY_ERR_RX_CNT,
		.page	= MSCC_PHY_PAGE_STANDARD,
		.mask	= ERR_CNT_MASK,
	}, {
		.string	= "phy_false_carrier",
		.reg	= MSCC_PHY_ERR_FALSE_CARRIER_CNT,
		.page	= MSCC_PHY_PAGE_STANDARD,
		.mask	= ERR_CNT_MASK,
	}, {
		.string	= "phy_cu_media_link_disconnect",
		.reg	= MSCC_PHY_ERR_LINK_DISCONNECT_CNT,
		.page	= MSCC_PHY_PAGE_STANDARD,
		.mask	= ERR_CNT_MASK,
	}, {
		.string	= "phy_cu_media_crc_good_count",
		.reg	= MSCC_PHY_CU_MEDIA_CRC_VALID_CNT,
		.page	= MSCC_PHY_PAGE_EXTENDED,
		.mask	= VALID_CRC_CNT_CRC_MASK,
	}, {
		.string	= "phy_cu_media_crc_error_count",
		.reg	= MSCC_PHY_EXT_PHY_CNTL_4,
		.page	= MSCC_PHY_PAGE_EXTENDED,
		.mask	= ERR_CNT_MASK,
	}, {
		.string	= "phy_serdes_tx_good_pkt_count",
		.reg	= MSCC_PHY_SERDES_TX_VALID_CNT,
		.page	= MSCC_PHY_PAGE_EXTENDED_3,
		.mask	= VALID_CRC_CNT_CRC_MASK,
	}, {
		.string	= "phy_serdes_tx_bad_crc_count",
		.reg	= MSCC_PHY_SERDES_TX_CRC_ERR_CNT,
		.page	= MSCC_PHY_PAGE_EXTENDED_3,
		.mask	= ERR_CNT_MASK,
	}, {
		.string	= "phy_serdes_rx_good_pkt_count",
		.reg	= MSCC_PHY_SERDES_RX_VALID_CNT,
		.page	= MSCC_PHY_PAGE_EXTENDED_3,
		.mask	= VALID_CRC_CNT_CRC_MASK,
	}, {
		.string	= "phy_serdes_rx_bad_crc_count",
		.reg	= MSCC_PHY_SERDES_RX_CRC_ERR_CNT,
		.page	= MSCC_PHY_PAGE_EXTENDED_3,
		.mask	= ERR_CNT_MASK,
	},
};

#if IS_ENABLED(CONFIG_OF_MDIO)
static const struct vsc8531_edge_rate_table edge_table[] = {
	{MSCC_VDDMAC_3300, { 0, 2,  4,  7, 10, 17, 29, 53} },
	{MSCC_VDDMAC_2500, { 0, 3,  6, 10, 14, 23, 37, 63} },
	{MSCC_VDDMAC_1800, { 0, 5,  9, 16, 23, 35, 52, 76} },
	{MSCC_VDDMAC_1500, { 0, 6, 14, 21, 29, 42, 58, 77} },
};
#endif

static const int vsc85xx_internal_delay[] = {200, 800, 1100, 1700, 2000, 2300,
					     2600, 3400};

static int vsc85xx_phy_read_page(struct phy_device *phydev)
{
	return __phy_read(phydev, MSCC_EXT_PAGE_ACCESS);
}

static int vsc85xx_phy_write_page(struct phy_device *phydev, int page)
{
	return __phy_write(phydev, MSCC_EXT_PAGE_ACCESS, page);
}

static int vsc85xx_get_sset_count(struct phy_device *phydev)
{
	struct vsc8531_private *priv = phydev->priv;

	if (!priv)
		return 0;

	return priv->nstats;
}

static void vsc85xx_get_strings(struct phy_device *phydev, u8 *data)
{
	struct vsc8531_private *priv = phydev->priv;
	int i;

	if (!priv)
		return;

	for (i = 0; i < priv->nstats; i++)
		strscpy(data + i * ETH_GSTRING_LEN, priv->hw_stats[i].string,
			ETH_GSTRING_LEN);
}

static u64 vsc85xx_get_stat(struct phy_device *phydev, int i)
{
	struct vsc8531_private *priv = phydev->priv;
	int val;

	val = phy_read_paged(phydev, priv->hw_stats[i].page,
			     priv->hw_stats[i].reg);
	if (val < 0)
		return U64_MAX;

	val = val & priv->hw_stats[i].mask;
	priv->stats[i] += val;

	return priv->stats[i];
}

static void vsc85xx_get_stats(struct phy_device *phydev,
			      struct ethtool_stats *stats, u64 *data)
{
	struct vsc8531_private *priv = phydev->priv;
	int i;

	if (!priv)
		return;

	for (i = 0; i < priv->nstats; i++)
		data[i] = vsc85xx_get_stat(phydev, i);
}

static int vsc85xx_led_cntl_set(struct phy_device *phydev,
				u8 led_num,
				u8 mode)
{
	int rc;
	u16 reg_val;

	mutex_lock(&phydev->lock);
	reg_val = phy_read(phydev, MSCC_PHY_LED_MODE_SEL);
	reg_val &= ~LED_MODE_SEL_MASK(led_num);
	reg_val |= LED_MODE_SEL(led_num, (u16)mode);
	rc = phy_write(phydev, MSCC_PHY_LED_MODE_SEL, reg_val);
	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_mdix_get(struct phy_device *phydev, u8 *mdix)
{
	u16 reg_val;

	reg_val = phy_read(phydev, MSCC_PHY_DEV_AUX_CNTL);
	if (reg_val & HP_AUTO_MDIX_X_OVER_IND_MASK)
		*mdix = ETH_TP_MDI_X;
	else
		*mdix = ETH_TP_MDI;

	return 0;
}

static int vsc85xx_mdix_set(struct phy_device *phydev, u8 mdix)
{
	int rc;
	u16 reg_val;

	reg_val = phy_read(phydev, MSCC_PHY_BYPASS_CONTROL);
	if (mdix == ETH_TP_MDI || mdix == ETH_TP_MDI_X) {
		reg_val |= (DISABLE_PAIR_SWAP_CORR_MASK |
			    DISABLE_POLARITY_CORR_MASK  |
			    DISABLE_HP_AUTO_MDIX_MASK);
	} else {
		reg_val &= ~(DISABLE_PAIR_SWAP_CORR_MASK |
			     DISABLE_POLARITY_CORR_MASK  |
			     DISABLE_HP_AUTO_MDIX_MASK);
	}
	rc = phy_write(phydev, MSCC_PHY_BYPASS_CONTROL, reg_val);
	if (rc)
		return rc;

	reg_val = 0;

	if (mdix == ETH_TP_MDI)
		reg_val = FORCE_MDI_CROSSOVER_MDI;
	else if (mdix == ETH_TP_MDI_X)
		reg_val = FORCE_MDI_CROSSOVER_MDIX;

	rc = phy_modify_paged(phydev, MSCC_PHY_PAGE_EXTENDED,
			      MSCC_PHY_EXT_MODE_CNTL, FORCE_MDI_CROSSOVER_MASK,
			      reg_val);
	if (rc < 0)
		return rc;

	return genphy_restart_aneg(phydev);
}

static int vsc85xx_downshift_get(struct phy_device *phydev, u8 *count)
{
	int reg_val;

	reg_val = phy_read_paged(phydev, MSCC_PHY_PAGE_EXTENDED,
				 MSCC_PHY_ACTIPHY_CNTL);
	if (reg_val < 0)
		return reg_val;

	reg_val &= DOWNSHIFT_CNTL_MASK;
	if (!(reg_val & DOWNSHIFT_EN))
		*count = DOWNSHIFT_DEV_DISABLE;
	else
		*count = ((reg_val & ~DOWNSHIFT_EN) >> DOWNSHIFT_CNTL_POS) + 2;

	return 0;
}

static int vsc85xx_downshift_set(struct phy_device *phydev, u8 count)
{
	if (count == DOWNSHIFT_DEV_DEFAULT_COUNT) {
		/* Default downshift count 3 (i.e. Bit3:2 = 0b01) */
		count = ((1 << DOWNSHIFT_CNTL_POS) | DOWNSHIFT_EN);
	} else if (count > DOWNSHIFT_COUNT_MAX || count == 1) {
		phydev_err(phydev, "Downshift count should be 2,3,4 or 5\n");
		return -ERANGE;
	} else if (count) {
		/* Downshift count is either 2,3,4 or 5 */
		count = (((count - 2) << DOWNSHIFT_CNTL_POS) | DOWNSHIFT_EN);
	}

	return phy_modify_paged(phydev, MSCC_PHY_PAGE_EXTENDED,
				MSCC_PHY_ACTIPHY_CNTL, DOWNSHIFT_CNTL_MASK,
				count);
}

static int vsc85xx_wol_set(struct phy_device *phydev,
			   struct ethtool_wolinfo *wol)
{
	const u8 *mac_addr = phydev->attached_dev->dev_addr;
	int rc;
	u16 reg_val;
	u8  i;
	u16 pwd[3] = {0, 0, 0};
	struct ethtool_wolinfo *wol_conf = wol;

	rc = phy_select_page(phydev, MSCC_PHY_PAGE_EXTENDED_2);
	if (rc < 0)
		return phy_restore_page(phydev, rc, rc);

	if (wol->wolopts & WAKE_MAGIC) {
		/* Store the device address for the magic packet */
		for (i = 0; i < ARRAY_SIZE(pwd); i++)
			pwd[i] = mac_addr[5 - (i * 2 + 1)] << 8 |
				 mac_addr[5 - i * 2];
		__phy_write(phydev, MSCC_PHY_WOL_LOWER_MAC_ADDR, pwd[0]);
		__phy_write(phydev, MSCC_PHY_WOL_MID_MAC_ADDR, pwd[1]);
		__phy_write(phydev, MSCC_PHY_WOL_UPPER_MAC_ADDR, pwd[2]);
	} else {
		__phy_write(phydev, MSCC_PHY_WOL_LOWER_MAC_ADDR, 0);
		__phy_write(phydev, MSCC_PHY_WOL_MID_MAC_ADDR, 0);
		__phy_write(phydev, MSCC_PHY_WOL_UPPER_MAC_ADDR, 0);
	}

	if (wol_conf->wolopts & WAKE_MAGICSECURE) {
		for (i = 0; i < ARRAY_SIZE(pwd); i++)
			pwd[i] = wol_conf->sopass[5 - (i * 2 + 1)] << 8 |
				 wol_conf->sopass[5 - i * 2];
		__phy_write(phydev, MSCC_PHY_WOL_LOWER_PASSWD, pwd[0]);
		__phy_write(phydev, MSCC_PHY_WOL_MID_PASSWD, pwd[1]);
		__phy_write(phydev, MSCC_PHY_WOL_UPPER_PASSWD, pwd[2]);
	} else {
		__phy_write(phydev, MSCC_PHY_WOL_LOWER_PASSWD, 0);
		__phy_write(phydev, MSCC_PHY_WOL_MID_PASSWD, 0);
		__phy_write(phydev, MSCC_PHY_WOL_UPPER_PASSWD, 0);
	}

	reg_val = __phy_read(phydev, MSCC_PHY_WOL_MAC_CONTROL);
	if (wol_conf->wolopts & WAKE_MAGICSECURE)
		reg_val |= SECURE_ON_ENABLE;
	else
		reg_val &= ~SECURE_ON_ENABLE;
	__phy_write(phydev, MSCC_PHY_WOL_MAC_CONTROL, reg_val);

	rc = phy_restore_page(phydev, rc, rc > 0 ? 0 : rc);
	if (rc < 0)
		return rc;

	if (wol->wolopts & WAKE_MAGIC) {
		/* Enable the WOL interrupt */
		reg_val = phy_read(phydev, MII_VSC85XX_INT_MASK);
		reg_val |= MII_VSC85XX_INT_MASK_WOL;
		rc = phy_write(phydev, MII_VSC85XX_INT_MASK, reg_val);
		if (rc)
			return rc;
	} else {
		/* Disable the WOL interrupt */
		reg_val = phy_read(phydev, MII_VSC85XX_INT_MASK);
		reg_val &= (~MII_VSC85XX_INT_MASK_WOL);
		rc = phy_write(phydev, MII_VSC85XX_INT_MASK, reg_val);
		if (rc)
			return rc;
	}
	/* Clear WOL iterrupt status */
	reg_val = phy_read(phydev, MII_VSC85XX_INT_STATUS);

	return 0;
}

static void vsc85xx_wol_get(struct phy_device *phydev,
			    struct ethtool_wolinfo *wol)
{
	int rc;
	u16 reg_val;
	u8  i;
	u16 pwd[3] = {0, 0, 0};
	struct ethtool_wolinfo *wol_conf = wol;

	rc = phy_select_page(phydev, MSCC_PHY_PAGE_EXTENDED_2);
	if (rc < 0)
		goto out_restore_page;

	reg_val = __phy_read(phydev, MSCC_PHY_WOL_MAC_CONTROL);
	if (reg_val & SECURE_ON_ENABLE)
		wol_conf->wolopts |= WAKE_MAGICSECURE;
	if (wol_conf->wolopts & WAKE_MAGICSECURE) {
		pwd[0] = __phy_read(phydev, MSCC_PHY_WOL_LOWER_PASSWD);
		pwd[1] = __phy_read(phydev, MSCC_PHY_WOL_MID_PASSWD);
		pwd[2] = __phy_read(phydev, MSCC_PHY_WOL_UPPER_PASSWD);
		for (i = 0; i < ARRAY_SIZE(pwd); i++) {
			wol_conf->sopass[5 - i * 2] = pwd[i] & 0x00ff;
			wol_conf->sopass[5 - (i * 2 + 1)] = (pwd[i] & 0xff00)
							    >> 8;
		}
	}

out_restore_page:
	phy_restore_page(phydev, rc, rc > 0 ? 0 : rc);
}

#if IS_ENABLED(CONFIG_OF_MDIO)
static int vsc85xx_edge_rate_magic_get(struct phy_device *phydev)
{
	u32 vdd, sd;
	int i, j;
	struct device *dev = &phydev->mdio.dev;
	struct device_node *of_node = dev->of_node;
	u8 sd_array_size = ARRAY_SIZE(edge_table[0].slowdown);

	if (!of_node)
		return -ENODEV;

	if (of_property_read_u32(of_node, "vsc8531,vddmac", &vdd))
		vdd = MSCC_VDDMAC_3300;

	if (of_property_read_u32(of_node, "vsc8531,edge-slowdown", &sd))
		sd = 0;

	for (i = 0; i < ARRAY_SIZE(edge_table); i++)
		if (edge_table[i].vddmac == vdd)
			for (j = 0; j < sd_array_size; j++)
				if (edge_table[i].slowdown[j] == sd)
					return (sd_array_size - j - 1);

	return -EINVAL;
}

static int vsc85xx_dt_led_mode_get(struct phy_device *phydev,
				   char *led,
				   u32 default_mode)
{
	struct vsc8531_private *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	struct device_node *of_node = dev->of_node;
	u32 led_mode;
	int err;

	if (!of_node)
		return -ENODEV;

	led_mode = default_mode;
	err = of_property_read_u32(of_node, led, &led_mode);
	if (!err && !(BIT(led_mode) & priv->supp_led_modes)) {
		phydev_err(phydev, "DT %s invalid\n", led);
		return -EINVAL;
	}

	return led_mode;
}

#else
static int vsc85xx_edge_rate_magic_get(struct phy_device *phydev)
{
	return 0;
}

static int vsc85xx_dt_led_mode_get(struct phy_device *phydev,
				   char *led,
				   u8 default_mode)
{
	return default_mode;
}
#endif /* CONFIG_OF_MDIO */

static int vsc85xx_dt_led_modes_get(struct phy_device *phydev,
				    u32 *default_mode)
{
	struct vsc8531_private *priv = phydev->priv;
	char led_dt_prop[28];
	int i, ret;

	for (i = 0; i < priv->nleds; i++) {
		ret = sprintf(led_dt_prop, "vsc8531,led-%d-mode", i);
		if (ret < 0)
			return ret;

		ret = vsc85xx_dt_led_mode_get(phydev, led_dt_prop,
					      default_mode[i]);
		if (ret < 0)
			return ret;
		priv->leds_mode[i] = ret;
	}

	return 0;
}

static int vsc85xx_edge_rate_cntl_set(struct phy_device *phydev, u8 edge_rate)
{
	int rc;

	mutex_lock(&phydev->lock);
	rc = phy_modify_paged(phydev, MSCC_PHY_PAGE_EXTENDED_2,
			      MSCC_PHY_WOL_MAC_CONTROL, EDGE_RATE_CNTL_MASK,
			      edge_rate << EDGE_RATE_CNTL_POS);
	mutex_unlock(&phydev->lock);

	return rc;
}

static int vsc85xx_mac_if_set(struct phy_device *phydev,
			      phy_interface_t interface)
{
	int rc;
	u16 reg_val;

	mutex_lock(&phydev->lock);
	reg_val = phy_read(phydev, MSCC_PHY_EXT_PHY_CNTL_1);
	reg_val &= ~(MAC_IF_SELECTION_MASK);
	switch (interface) {
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII:
		reg_val |= (MAC_IF_SELECTION_RGMII << MAC_IF_SELECTION_POS);
		break;
	case PHY_INTERFACE_MODE_RMII:
		reg_val |= (MAC_IF_SELECTION_RMII << MAC_IF_SELECTION_POS);
		break;
	case PHY_INTERFACE_MODE_MII:
	case PHY_INTERFACE_MODE_GMII:
		reg_val |= (MAC_IF_SELECTION_GMII << MAC_IF_SELECTION_POS);
		break;
	default:
		rc = -EINVAL;
		goto out_unlock;
	}
	rc = phy_write(phydev, MSCC_PHY_EXT_PHY_CNTL_1, reg_val);
	if (rc)
		goto out_unlock;

	rc = genphy_soft_reset(phydev);

out_unlock:
	mutex_unlock(&phydev->lock);

	return rc;
}

/* Set the RGMII RX and TX clock skews individually, according to the PHY
 * interface type, to:
 *  * 0.2 ns (their default, and lowest, hardware value) if delays should
 *    not be enabled
 *  * 2.0 ns (which causes the data to be sampled at exactly half way between
 *    clock transitions at 1000 Mbps) if delays should be enabled
 */
static int vsc85xx_update_rgmii_cntl(struct phy_device *phydev, u32 rgmii_cntl,
				     u16 rgmii_rx_delay_mask,
				     u16 rgmii_tx_delay_mask)
{
	u16 rgmii_rx_delay_pos = ffs(rgmii_rx_delay_mask) - 1;
	u16 rgmii_tx_delay_pos = ffs(rgmii_tx_delay_mask) - 1;
	int delay_size = ARRAY_SIZE(vsc85xx_internal_delay);
	struct device *dev = &phydev->mdio.dev;
	u16 reg_val = 0;
	u16 mask = 0;
	s32 rx_delay;
	s32 tx_delay;
	int rc = 0;

	/* For traffic to pass, the VSC8502 family needs the RX_CLK disable bit
	 * to be unset for all PHY modes, so do that as part of the paged
	 * register modification.
	 * For some family members (like VSC8530/31/40/41) this bit is reserved
	 * and read-only, and the RX clock is enabled by default.
	 */
	if (rgmii_cntl == VSC8502_RGMII_CNTL)
		mask |= VSC8502_RGMII_RX_CLK_DISABLE;

	if (phy_interface_is_rgmii(phydev))
		mask |= rgmii_rx_delay_mask | rgmii_tx_delay_mask;

	rx_delay = phy_get_internal_delay(phydev, dev, vsc85xx_internal_delay,
					  delay_size, true);
	if (rx_delay < 0) {
		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID ||
		    phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
			rx_delay = RGMII_CLK_DELAY_2_0_NS;
		else
			rx_delay = RGMII_CLK_DELAY_0_2_NS;
	}

	tx_delay = phy_get_internal_delay(phydev, dev, vsc85xx_internal_delay,
					  delay_size, false);
	if (tx_delay < 0) {
		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID ||
		    phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
			tx_delay = RGMII_CLK_DELAY_2_0_NS;
		else
			tx_delay = RGMII_CLK_DELAY_0_2_NS;
	}

	reg_val |= rx_delay << rgmii_rx_delay_pos;
	reg_val |= tx_delay << rgmii_tx_delay_pos;

	if (mask)
		rc = phy_modify_paged(phydev, MSCC_PHY_PAGE_EXTENDED_2,
				      rgmii_cntl, mask, reg_val);

	return rc;
}

static int vsc85xx_default_config(struct phy_device *phydev)
{
	phydev->mdix_ctrl = ETH_TP_MDI_AUTO;

	return vsc85xx_update_rgmii_cntl(phydev, VSC8502_RGMII_CNTL,
					 VSC8502_RGMII_RX_DELAY_MASK,
					 VSC8502_RGMII_TX_DELAY_MASK);
}

static int vsc85xx_get_tunable(struct phy_device *phydev,
			       struct ethtool_tunable *tuna, void *data)
{
	switch (tuna->id) {
	case ETHTOOL_PHY_DOWNSHIFT:
		return vsc85xx_downshift_get(phydev, (u8 *)data);
	default:
		return -EINVAL;
	}
}

static int vsc85xx_set_tunable(struct phy_device *phydev,
			       struct ethtool_tunable *tuna,
			       const void *data)
{
	switch (tuna->id) {
	case ETHTOOL_PHY_DOWNSHIFT:
		return vsc85xx_downshift_set(phydev, *(u8 *)data);
	default:
		return -EINVAL;
	}
}

/* mdiobus lock should be locked when using this function */
static void vsc85xx_tr_write(struct phy_device *phydev, u16 addr, u32 val)
{
	__phy_write(phydev, MSCC_PHY_TR_MSB, val >> 16);
	__phy_write(phydev, MSCC_PHY_TR_LSB, val & GENMASK(15, 0));
	__phy_write(phydev, MSCC_PHY_TR_CNTL, TR_WRITE | TR_ADDR(addr));
}

static int vsc85xx_eee_init_seq_set(struct phy_device *phydev)
{
	static const struct reg_val init_eee[] = {
		{0x0f82, 0x0012b00a},
		{0x1686, 0x00000004},
		{0x168c, 0x00d2c46f},
		{0x17a2, 0x00000620},
		{0x16a0, 0x00eeffdd},
		{0x16a6, 0x00071448},
		{0x16a4, 0x0013132f},
		{0x16a8, 0x00000000},
		{0x0ffc, 0x00c0a028},
		{0x0fe8, 0x0091b06c},
		{0x0fea, 0x00041600},
		{0x0f80, 0x00000af4},
		{0x0fec, 0x00901809},
		{0x0fee, 0x0000a6a1},
		{0x0ffe, 0x00b01007},
		{0x16b0, 0x00eeff00},
		{0x16b2, 0x00007000},
		{0x16b4, 0x00000814},
	};
	unsigned int i;
	int oldpage;

	mutex_lock(&phydev->lock);
	oldpage = phy_select_page(phydev, MSCC_PHY_PAGE_TR);
	if (oldpage < 0)
		goto out_unlock;

	for (i = 0; i < ARRAY_SIZE(init_eee); i++)
		vsc85xx_tr_write(phydev, init_eee[i].reg, init_eee[i].val);

out_unlock:
	oldpage = phy_restore_page(phydev, oldpage, oldpage);
	mutex_unlock(&phydev->lock);

	return oldpage;
}

/* phydev->bus->mdio_lock should be locked when using this function */
int phy_base_write(struct phy_device *phydev, u32 regnum, u16 val)
{
	if (unlikely(!mutex_is_locked(&phydev->mdio.bus->mdio_lock))) {
		dev_err(&phydev->mdio.dev, "MDIO bus lock not held!\n");
		dump_stack();
	}

	return __phy_package_write(phydev, regnum, val);
}

/* phydev->bus->mdio_lock should be locked when using this function */
int phy_base_read(struct phy_device *phydev, u32 regnum)
{
	if (unlikely(!mutex_is_locked(&phydev->mdio.bus->mdio_lock))) {
		dev_err(&phydev->mdio.dev, "MDIO bus lock not held!\n");
		dump_stack();
	}

	return __phy_package_read(phydev, regnum);
}

u32 vsc85xx_csr_read(struct phy_device *phydev,
		     enum csr_target target, u32 reg)
{
	unsigned long deadline;
	u32 val, val_l, val_h;

	phy_base_write(phydev, MSCC_EXT_PAGE_ACCESS, MSCC_PHY_PAGE_CSR_CNTL);

	/* CSR registers are grouped under different Target IDs.
	 * 6-bit Target_ID is split between MSCC_EXT_PAGE_CSR_CNTL_20 and
	 * MSCC_EXT_PAGE_CSR_CNTL_19 registers.
	 * Target_ID[5:2] maps to bits[3:0] of MSCC_EXT_PAGE_CSR_CNTL_20
	 * and Target_ID[1:0] maps to bits[13:12] of MSCC_EXT_PAGE_CSR_CNTL_19.
	 */

	/* Setup the Target ID */
	phy_base_write(phydev, MSCC_EXT_PAGE_CSR_CNTL_20,
		       MSCC_PHY_CSR_CNTL_20_TARGET(target >> 2));

	if ((target >> 2 == 0x1) || (target >> 2 == 0x3))
		/* non-MACsec access */
		target &= 0x3;
	else
		target = 0;

	/* Trigger CSR Action - Read into the CSR's */
	phy_base_write(phydev, MSCC_EXT_PAGE_CSR_CNTL_19,
		       MSCC_PHY_CSR_CNTL_19_CMD | MSCC_PHY_CSR_CNTL_19_READ |
		       MSCC_PHY_CSR_CNTL_19_REG_ADDR(reg) |
		       MSCC_PHY_CSR_CNTL_19_TARGET(target));

	/* Wait for register access*/
	deadline = jiffies + msecs_to_jiffies(PROC_CMD_NCOMPLETED_TIMEOUT_MS);
	do {
		usleep_range(500, 1000);
		val = phy_base_read(phydev, MSCC_EXT_PAGE_CSR_CNTL_19);
	} while (time_before(jiffies, deadline) &&
		!(val & MSCC_PHY_CSR_CNTL_19_CMD));

	if (!(val & MSCC_PHY_CSR_CNTL_19_CMD))
		return 0xffffffff;

	/* Read the Least Significant Word (LSW) (17) */
	val_l = phy_base_read(phydev, MSCC_EXT_PAGE_CSR_CNTL_17);

	/* Read the Most Significant Word (MSW) (18) */
	val_h = phy_base_read(phydev, MSCC_EXT_PAGE_CSR_CNTL_18);

	phy_base_write(phydev, MSCC_EXT_PAGE_ACCESS,
		       MSCC_PHY_PAGE_STANDARD);

	return (val_h << 16) | val_l;
}

int vsc85xx_csr_write(struct phy_device *phydev,
		      enum csr_target target, u32 reg, u32 val)
{
	unsigned long deadline;

	phy_base_write(phydev, MSCC_EXT_PAGE_ACCESS, MSCC_PHY_PAGE_CSR_CNTL);

	/* CSR registers are grouped under different Target IDs.
	 * 6-bit Target_ID is split between MSCC_EXT_PAGE_CSR_CNTL_20 and
	 * MSCC_EXT_PAGE_CSR_CNTL_19 registers.
	 * Target_ID[5:2] maps to bits[3:0] of MSCC_EXT_PAGE_CSR_CNTL_20
	 * and Target_ID[1:0] maps to bits[13:12] of MSCC_EXT_PAGE_CSR_CNTL_19.
	 */

	/* Setup the Target ID */
	phy_base_write(phydev, MSCC_EXT_PAGE_CSR_CNTL_20,
		       MSCC_PHY_CSR_CNTL_20_TARGET(target >> 2));

	/* Write the Least Significant Word (LSW) (17) */
	phy_base_write(phydev, MSCC_EXT_PAGE_CSR_CNTL_17, (u16)val);

	/* Write the Most Significant Word (MSW) (18) */
	phy_base_write(phydev, MSCC_EXT_PAGE_CSR_CNTL_18, (u16)(val >> 16));

	if ((target >> 2 == 0x1) || (target >> 2 == 0x3))
		/* non-MACsec access */
		target &= 0x3;
	else
		target = 0;

	/* Trigger CSR Action - Write into the CSR's */
	phy_base_write(phydev, MSCC_EXT_PAGE_CSR_CNTL_19,
		       MSCC_PHY_CSR_CNTL_19_CMD |
		       MSCC_PHY_CSR_CNTL_19_REG_ADDR(reg) |
		       MSCC_PHY_CSR_CNTL_19_TARGET(target));

	/* Wait for register access */
	deadline = jiffies + msecs_to_jiffies(PROC_CMD_NCOMPLETED_TIMEOUT_MS);
	do {
		usleep_range(500, 1000);
		val = phy_base_read(phydev, MSCC_EXT_PAGE_CSR_CNTL_19);
	} while (time_before(jiffies, deadline) &&
		 !(val & MSCC_PHY_CSR_CNTL_19_CMD));

	if (!(val & MSCC_PHY_CSR_CNTL_19_CMD))
		return -ETIMEDOUT;

	phy_base_write(phydev, MSCC_EXT_PAGE_ACCESS,
		       MSCC_PHY_PAGE_STANDARD);

	return 0;
}

static void vsc85xx_coma_mode_release(struct phy_device *phydev)
{
	/* The coma mode (pin or reg) provides an optional feature that
	 * may be used to control when the PHYs become active.
	 * Alternatively the COMA_MODE pin may be connected low
	 * so that the PHYs are fully active once out of reset.
	 */

	/* Enable output (mode=0) and write zero to it */
	vsc85xx_phy_write_page(phydev, MSCC_PHY_PAGE_EXTENDED_GPIO);
	__phy_modify(phydev, MSCC_PHY_GPIO_CONTROL_2,
		     MSCC_PHY_COMA_MODE | MSCC_PHY_COMA_OUTPUT, 0);
	vsc85xx_phy_write_page(phydev, MSCC_PHY_PAGE_STANDARD);
}

static int vsc85xx_config_init(struct phy_device *phydev)
{
	int rc, i, phy_id;
	struct vsc8531_private *vsc8531 = phydev->priv;

	rc = vsc85xx_default_config(phydev);
	if (rc)
		return rc;

	rc = vsc85xx_mac_if_set(phydev, phydev->interface);
	if (rc)
		return rc;

	rc = vsc85xx_edge_rate_cntl_set(phydev, vsc8531->rate_magic);
	if (rc)
		return rc;

	phy_id = phydev->drv->phy_id & phydev->drv->phy_id_mask;
	if (PHY_ID_VSC8531 == phy_id || PHY_ID_VSC8541 == phy_id ||
	    PHY_ID_VSC8530 == phy_id || PHY_ID_VSC8540 == phy_id) {
		rc = vsc8531_pre_init_seq_set(phydev);
		if (rc)
			return rc;
	}

	rc = vsc85xx_eee_init_seq_set(phydev);
	if (rc)
		return rc;

	for (i = 0; i < vsc8531->nleds; i++) {
		rc = vsc85xx_led_cntl_set(phydev, i, vsc8531->leds_mode[i]);
		if (rc)
			return rc;
	}

	return 0;
}

static int __phy_write_mcb_s6g(struct phy_device *phydev, u32 reg, u8 mcb,
			       u32 op)
{
	unsigned long deadline;
	u32 val;
	int ret;

	ret = vsc85xx_csr_write(phydev, PHY_MCB_TARGET, reg,
				op | (1 << mcb));
	if (ret)
		return -EINVAL;

	deadline = jiffies + msecs_to_jiffies(PROC_CMD_NCOMPLETED_TIMEOUT_MS);
	do {
		usleep_range(500, 1000);
		val = vsc85xx_csr_read(phydev, PHY_MCB_TARGET, reg);

		if (val == 0xffffffff)
			return -EIO;

	} while (time_before(jiffies, deadline) && (val & op));

	if (val & op)
		return -ETIMEDOUT;

	return 0;
}

/* Trigger a read to the specified MCB */
int phy_update_mcb_s6g(struct phy_device *phydev, u32 reg, u8 mcb)
{
	return __phy_write_mcb_s6g(phydev, reg, mcb, PHY_MCB_S6G_READ);
}

/* Trigger a write to the specified MCB */
int phy_commit_mcb_s6g(struct phy_device *phydev, u32 reg, u8 mcb)
{
	return __phy_write_mcb_s6g(phydev, reg, mcb, PHY_MCB_S6G_WRITE);
}

static int vsc85xx_ack_interrupt(struct phy_device *phydev)
{
	int rc = 0;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		rc = phy_read(phydev, MII_VSC85XX_INT_STATUS);

	return (rc < 0) ? rc : 0;
}

static int vsc85xx_config_intr(struct phy_device *phydev)
{
	int rc;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		rc = vsc85xx_ack_interrupt(phydev);
		if (rc)
			return rc;

		vsc8584_config_macsec_intr(phydev);
		vsc8584_config_ts_intr(phydev);

		rc = phy_write(phydev, MII_VSC85XX_INT_MASK,
			       MII_VSC85XX_INT_MASK_MASK);
	} else {
		rc = phy_write(phydev, MII_VSC85XX_INT_MASK, 0);
		if (rc < 0)
			return rc;
		rc = phy_read(phydev, MII_VSC85XX_INT_STATUS);
		if (rc < 0)
			return rc;

		rc = vsc85xx_ack_interrupt(phydev);
	}

	return rc;
}

static irqreturn_t vsc85xx_handle_interrupt(struct phy_device *phydev)
{
	int irq_status;

	irq_status = phy_read(phydev, MII_VSC85XX_INT_STATUS);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}

	if (!(irq_status & MII_VSC85XX_INT_MASK_MASK))
		return IRQ_NONE;

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static int vsc85xx_config_aneg(struct phy_device *phydev)
{
	int rc;

	rc = vsc85xx_mdix_set(phydev, phydev->mdix_ctrl);
	if (rc < 0)
		return rc;

	return genphy_config_aneg(phydev);
}

static int vsc85xx_read_status(struct phy_device *phydev)
{
	int rc;

	rc = vsc85xx_mdix_get(phydev, &phydev->mdix);
	if (rc < 0)
		return rc;

	return genphy_read_status(phydev);
}

static int vsc85xx_probe(struct phy_device *phydev)
{
	struct vsc8531_private *vsc8531;
	int rate_magic;
	u32 default_mode[2] = {VSC8531_LINK_1000_ACTIVITY,
	   VSC8531_LINK_100_ACTIVITY};

	rate_magic = vsc85xx_edge_rate_magic_get(phydev);
	if (rate_magic < 0)
		return rate_magic;

	vsc8531 = devm_kzalloc(&phydev->mdio.dev, sizeof(*vsc8531), GFP_KERNEL);
	if (!vsc8531)
		return -ENOMEM;

	phydev->priv = vsc8531;

	vsc8531->rate_magic = rate_magic;
	vsc8531->nleds = 2;
	vsc8531->supp_led_modes = VSC85XX_SUPP_LED_MODES;
	vsc8531->hw_stats = vsc85xx_hw_stats;
	vsc8531->nstats = ARRAY_SIZE(vsc85xx_hw_stats);
	vsc8531->stats = devm_kcalloc(&phydev->mdio.dev, vsc8531->nstats,
				      sizeof(u64), GFP_KERNEL);
	if (!vsc8531->stats)
		return -ENOMEM;

	return vsc85xx_dt_led_modes_get(phydev, default_mode);
}

/* Microsemi VSC85xx PHYs */
static struct phy_driver vsc85xx_driver[] = {
{
	.phy_id		= PHY_ID_VSC8501,
	.name		= "Microsemi GE VSC8501 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_BASIC_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init	= &stub,
	.config_aneg    = &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr	= &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc85xx_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8502,
	.name		= "Microsemi GE VSC8502 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_BASIC_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init	= &vsc85xx_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr	= &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc85xx_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8504,
	.name		= "Microsemi GE VSC8504 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8584_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.aneg_done	= &genphy_aneg_done,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8574_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8514,
	.name		= "Microsemi GE VSC8514 SyncE",
	.phy_id_mask	= 0xfffffff0,
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8514_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8514_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page      = &vsc85xx_phy_read_page,
	.write_page     = &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8530,
	.name		= "Microsemi FE VSC8530",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_BASIC_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init	= &vsc85xx_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr	= &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc85xx_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8531,
	.name		= "Microsemi VSC8531",
	.phy_id_mask    = 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc85xx_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc85xx_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8540,
	.name		= "Microsemi FE VSC8540 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_BASIC_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init	= &vsc85xx_config_init,
	.config_aneg	= &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr	= &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc85xx_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8541,
	.name		= "Microsemi VSC8541 SyncE",
	.phy_id_mask    = 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc85xx_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc85xx_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8552,
	.name		= "Microsemi GE VSC8552 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8584_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8574_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC856X,
	.name		= "Microsemi GE VSC856X SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8584_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8584_probe,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8572,
	.name		= "Microsemi GE VSC8572 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8584_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.aneg_done	= &genphy_aneg_done,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = &vsc8584_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8574_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8574,
	.name		= "Microsemi GE VSC8574 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8584_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.aneg_done	= &genphy_aneg_done,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = vsc85xx_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8574_probe,
	.set_wol	= &vsc85xx_wol_set,
	.get_wol	= &vsc85xx_wol_get,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8575,
	.name		= "Microsemi GE VSC8575 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8584_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.aneg_done	= &genphy_aneg_done,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = &vsc8584_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8584_probe,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8582,
	.name		= "Microsemi GE VSC8582 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8584_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.aneg_done	= &genphy_aneg_done,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = &vsc8584_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8584_probe,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
},
{
	.phy_id		= PHY_ID_VSC8584,
	.name		= "Microsemi GE VSC8584 SyncE",
	.phy_id_mask	= 0xfffffff0,
	/* PHY_GBIT_FEATURES */
	.soft_reset	= &genphy_soft_reset,
	.config_init    = &vsc8584_config_init,
	.config_aneg    = &vsc85xx_config_aneg,
	.aneg_done	= &genphy_aneg_done,
	.read_status	= &vsc85xx_read_status,
	.handle_interrupt = &vsc8584_handle_interrupt,
	.config_intr    = &vsc85xx_config_intr,
	.suspend	= &genphy_suspend,
	.resume		= &genphy_resume,
	.probe		= &vsc8584_probe,
	.get_tunable	= &vsc85xx_get_tunable,
	.set_tunable	= &vsc85xx_set_tunable,
	.read_page	= &vsc85xx_phy_read_page,
	.write_page	= &vsc85xx_phy_write_page,
	.get_sset_count = &vsc85xx_get_sset_count,
	.get_strings    = &vsc85xx_get_strings,
	.get_stats      = &vsc85xx_get_stats,
	.link_change_notify = &vsc85xx_link_change_notify,
}

};

module_phy_driver(vsc85xx_driver);

static struct mdio_device_id __maybe_unused vsc85xx_tbl[] = {
	{ PHY_ID_MATCH_VENDOR(PHY_VENDOR_MSCC) },
	{ }
};

MODULE_DEVICE_TABLE(mdio, vsc85xx_tbl);

MODULE_DESCRIPTION("Microsemi VSC85xx PHY driver");
MODULE_AUTHOR("Nagaraju Lakkaraju");
MODULE_LICENSE("Dual MIT/GPL");

MODULE_FIRMWARE(MSCC_VSC8584_REVB_INT8051_FW);
MODULE_FIRMWARE(MSCC_VSC8574_REVB_INT8051_FW);
