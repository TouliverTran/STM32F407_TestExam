#include "mscc-phy-vsc8531.h"
#include "mscc_serdes.h"
#include "mscc.h"
#include "phy.h"
#include "misc.h"
#include "ethtool.h"
#include "uapi/mii.h"

#include <errno.h>
#include <stdio.h>
#include <stdbool.h>
#include <strings.h>

static const int vsc85xx_internal_delay[] = {200, 800, 1100, 1700, 2000, 2300,
					     2600, 3400};

/**
 * mii_bmcr_encode_fixed - encode fixed speed/duplex settings to a BMCR value
 * @speed: a SPEED_* value
 * @duplex: a DUPLEX_* value
 *
 * Encode the speed and duplex to a BMCR value. 2500, 1000, 100 and 10 Mbps are
 * supported. 2500Mbps is encoded to 1000Mbps. Other speeds are encoded as 10
 * Mbps. Unknown duplex values are encoded to half-duplex.
 */
static inline u16 mii_bmcr_encode_fixed(int speed, int duplex)
{
	u16 bmcr;

	switch (speed) {
		case SPEED_2500:
		case SPEED_1000:
			bmcr = BMCR_SPEED1000;
			break;

		case SPEED_100:
			bmcr = BMCR_SPEED100;
			break;

		case SPEED_10:
		default:
			bmcr = BMCR_SPEED10;
			break;
	}

	if (duplex == DUPLEX_FULL)
		bmcr |= BMCR_FULLDPLX;

	return bmcr;
}

/**
 * genphy_setup_forced - configures/forces speed/duplex from @phydev
 * @phydev: target phy_device struct
 *
 * Description: Configures MII_BMCR to force speed/duplex
 *   to the values in phydev. Assumes that the values are valid.
 *   Please see phy_sanitize_settings().
 */
int genphy_setup_forced(struct phy_device *phydev)
{
	u16 ctl;

	phydev->pause = 0;
	phydev->asym_pause = 0;

	ctl = mii_bmcr_encode_fixed(phydev->speed, phydev->duplex);

	return phy_modify(phydev, MII_BMCR,
			  ~(BMCR_LOOPBACK | BMCR_ISOLATE | BMCR_PDOWN), ctl);
}

/**
 * phy_poll_reset - Safely wait until a PHY reset has properly completed
 * @phydev: The PHY device to poll
 *
 * Description: According to IEEE 802.3, Section 2, Subsection 22.2.4.1.1, as
 *   published in 2008, a PHY reset may take up to 0.5 seconds.  The MII BMCR
 *   register must be polled until the BMCR_RESET bit clears.
 *
 *   Furthermore, any attempts to write to PHY registers may have no effect
 *   or even generate MDIO bus errors until this is complete.
 *
 *   Some PHYs (such as the Marvell 88E1111) don't entirely conform to the
 *   standard and do not fully reset after the BMCR_RESET bit is set, and may
 *   even *REQUIRE* a soft-reset to properly restart autonegotiation.  In an
 *   effort to support such broken PHYs, this function is separate from the
 *   standard phy_init_hw() which will zero all the other bits in the BMCR
 *   and reapply all driver-specific and board-specific fixups.
 */
static int phy_poll_reset(struct phy_device *phydev)
{
	/* Poll until the reset bit clears (50ms per retry == 0.6 sec) */
	int ret, val;

	ret = phy_read_poll_timeout(phydev, MII_BMCR, val, !(val & BMCR_RESET),
				    50000, 600000, true);
	if (ret)
		return ret;
	/* Some chips (smsc911x) may still need up to another 1ms after the
	 * BMCR_RESET bit is cleared before they are usable.
	 */
	msleep(1);
	return 0;
}


/**
 * genphy_soft_reset - software reset the PHY via BMCR_RESET bit
 * @phydev: target phy_device struct
 *
 * Description: Perform a software PHY reset using the standard
 * BMCR_RESET bit and poll for the reset bit to be cleared.
 *
 * Returns: 0 on success, < 0 on failure
 */
int genphy_soft_reset(struct phy_device *phydev)
{
	u16 res = BMCR_RESET;
	int ret;

	if (phydev->autoneg == AUTONEG_ENABLE)
		res |= BMCR_ANRESTART;

	ret = phy_modify(phydev, MII_BMCR, BMCR_ISOLATE, res);
	if (ret < 0)
		return ret;

	/* Clause 22 states that setting bit BMCR_RESET sets control registers
	 * to their default value. Therefore the POWER DOWN bit is supposed to
	 * be cleared after soft reset.
	 */
	phydev->suspended = 0;

	ret = phy_poll_reset(phydev);
	if (ret)
		return ret;

	/* BMCR may be reset to defaults */
	if (phydev->autoneg == AUTONEG_DISABLE)
		ret = genphy_setup_forced(phydev);

	return ret;
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
	/* commented as the RX clock is enabled by default for VSC8541*/
	// if (rgmii_cntl == VSC8502_RGMII_CNTL)
	// 	mask |= VSC8502_RGMII_RX_CLK_DISABLE;

	if (phy_interface_is_rgmii(phydev))
		mask |= rgmii_rx_delay_mask | rgmii_tx_delay_mask;

	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID ||
		phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
		rx_delay = RGMII_CLK_DELAY_2_0_NS;
	else
		rx_delay = RGMII_CLK_DELAY_0_2_NS;


	if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID ||
		phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
		tx_delay = RGMII_CLK_DELAY_2_0_NS;
	else
		tx_delay = RGMII_CLK_DELAY_0_2_NS;

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

	oldpage = phy_select_page(phydev, MSCC_PHY_PAGE_TR);
	if (oldpage < 0)
		{
			oldpage = phy_restore_page(phydev, oldpage, oldpage);
			return oldpage;
		}

	for (i = 0; i < ARRAY_SIZE(init_eee); i++)
		vsc85xx_tr_write(phydev, init_eee[i].reg, init_eee[i].val);

	oldpage = phy_restore_page(phydev, oldpage, oldpage);
	return oldpage;
}

static int vsc85xx_mac_if_set(struct phy_device *phydev,
			      phy_interface_t interface)
{
	int rc;
	u16 reg_val;

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
			return rc;
	}

	rc = phy_write(phydev, MSCC_PHY_EXT_PHY_CNTL_1, reg_val);
	if (rc)
		return rc;

	rc = genphy_soft_reset(phydev);

	return rc;
}

static int vsc85xx_edge_rate_cntl_set(struct phy_device *phydev, u8 edge_rate)
{
	int rc;

	rc = phy_modify_paged(phydev, MSCC_PHY_PAGE_EXTENDED_2,
			      MSCC_PHY_WOL_MAC_CONTROL, EDGE_RATE_CNTL_MASK,
			      edge_rate << EDGE_RATE_CNTL_POS);

	return rc;
}


static int vsc8531_pre_init_seq_set(struct phy_device *phydev)
{
	int rc;
	static const struct reg_val init_seq[] = {
		{0x0f90, 0x00688980},
		{0x0696, 0x00000003},
		{0x07fa, 0x0050100f},
		{0x1686, 0x00000004},
	};
	unsigned int i;
	int oldpage;

	rc = phy_modify_paged(phydev, MSCC_PHY_PAGE_STANDARD,
			      MSCC_PHY_EXT_CNTL_STATUS, SMI_BROADCAST_WR_EN,
			      SMI_BROADCAST_WR_EN);
	if (rc < 0)
		return rc;
	rc = phy_modify_paged(phydev, MSCC_PHY_PAGE_TEST,
			      MSCC_PHY_TEST_PAGE_24, 0, 0x0400);
	if (rc < 0)
		return rc;
	rc = phy_modify_paged(phydev, MSCC_PHY_PAGE_TEST,
			      MSCC_PHY_TEST_PAGE_5, 0x0a00, 0x0e00);
	if (rc < 0)
		return rc;
	rc = phy_modify_paged(phydev, MSCC_PHY_PAGE_TEST,
			      MSCC_PHY_TEST_PAGE_8, TR_CLK_DISABLE, TR_CLK_DISABLE);
	if (rc < 0)
		return rc;

	oldpage = phy_select_page(phydev, MSCC_PHY_PAGE_TR);
	if (oldpage < 0)
		oldpage = phy_restore_page(phydev, oldpage, oldpage);
		return oldpage;

	for (i = 0; i < ARRAY_SIZE(init_seq); i++)
		vsc85xx_tr_write(phydev, init_seq[i].reg, init_seq[i].val);

	oldpage = phy_restore_page(phydev, oldpage, oldpage);
	return oldpage;
}


int vsc85xx_config_init(struct phy_device *phydev)
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

	// for (i = 0; i < vsc8531->nleds; i++) {
	// 	rc = vsc85xx_led_cntl_set(phydev, i, vsc8531->leds_mode[i]);
	// 	if (rc)
	// 		return rc;
	// }

	return 0;
}