#ifndef __PHY_H
#define __PHY_H

#include "misc.h"
#include "ethtool.h"

/**
 * enum phy_state - PHY state machine states:
 *
 * @PHY_DOWN: PHY device and driver are not ready for anything.  probe
 * should be called if and only if the PHY is in this state,
 * given that the PHY device exists.
 * - PHY driver probe function will set the state to @PHY_READY
 *
 * @PHY_READY: PHY is ready to send and receive packets, but the
 * controller is not.  By default, PHYs which do not implement
 * probe will be set to this state by phy_probe().
 * - start will set the state to UP
 *
 * @PHY_UP: The PHY and attached device are ready to do work.
 * Interrupts should be started here.
 * - timer moves to @PHY_NOLINK or @PHY_RUNNING
 *
 * @PHY_NOLINK: PHY is up, but not currently plugged in.
 * - irq or timer will set @PHY_RUNNING if link comes back
 * - phy_stop moves to @PHY_HALTED
 *
 * @PHY_RUNNING: PHY is currently up, running, and possibly sending
 * and/or receiving packets
 * - irq or timer will set @PHY_NOLINK if link goes down
 * - phy_stop moves to @PHY_HALTED
 *
 * @PHY_CABLETEST: PHY is performing a cable test. Packet reception/sending
 * is not expected to work, carrier will be indicated as down. PHY will be
 * poll once per second, or on interrupt for it current state.
 * Once complete, move to UP to restart the PHY.
 * - phy_stop aborts the running test and moves to @PHY_HALTED
 *
 * @PHY_HALTED: PHY is up, but no polling or interrupts are done.
 * - phy_start moves to @PHY_UP
 *
 * @PHY_ERROR: PHY is up, but is in an error state.
 * - phy_stop moves to @PHY_HALTED
 */
enum phy_state {
	PHY_DOWN = 0,
	PHY_READY,
	PHY_HALTED,
	PHY_ERROR,
	PHY_UP,
	PHY_RUNNING,
	PHY_NOLINK,
	PHY_CABLETEST,
};

/**
 * enum phy_interface_t - Interface Mode definitions
 *
 * @PHY_INTERFACE_MODE_NA: Not Applicable - don't touch
 * @PHY_INTERFACE_MODE_INTERNAL: No interface, MAC and PHY combined
 * @PHY_INTERFACE_MODE_MII: Media-independent interface
 * @PHY_INTERFACE_MODE_GMII: Gigabit media-independent interface
 * @PHY_INTERFACE_MODE_SGMII: Serial gigabit media-independent interface
 * @PHY_INTERFACE_MODE_TBI: Ten Bit Interface
 * @PHY_INTERFACE_MODE_REVMII: Reverse Media Independent Interface
 * @PHY_INTERFACE_MODE_RMII: Reduced Media Independent Interface
 * @PHY_INTERFACE_MODE_REVRMII: Reduced Media Independent Interface in PHY role
 * @PHY_INTERFACE_MODE_RGMII: Reduced gigabit media-independent interface
 * @PHY_INTERFACE_MODE_RGMII_ID: RGMII with Internal RX+TX delay
 * @PHY_INTERFACE_MODE_RGMII_RXID: RGMII with Internal RX delay
 * @PHY_INTERFACE_MODE_RGMII_TXID: RGMII with Internal RX delay
 * @PHY_INTERFACE_MODE_RTBI: Reduced TBI
 * @PHY_INTERFACE_MODE_SMII: Serial MII
 * @PHY_INTERFACE_MODE_XGMII: 10 gigabit media-independent interface
 * @PHY_INTERFACE_MODE_XLGMII:40 gigabit media-independent interface
 * @PHY_INTERFACE_MODE_MOCA: Multimedia over Coax
 * @PHY_INTERFACE_MODE_PSGMII: Penta SGMII
 * @PHY_INTERFACE_MODE_QSGMII: Quad SGMII
 * @PHY_INTERFACE_MODE_TRGMII: Turbo RGMII
 * @PHY_INTERFACE_MODE_100BASEX: 100 BaseX
 * @PHY_INTERFACE_MODE_1000BASEX: 1000 BaseX
 * @PHY_INTERFACE_MODE_2500BASEX: 2500 BaseX
 * @PHY_INTERFACE_MODE_5GBASER: 5G BaseR
 * @PHY_INTERFACE_MODE_RXAUI: Reduced XAUI
 * @PHY_INTERFACE_MODE_XAUI: 10 Gigabit Attachment Unit Interface
 * @PHY_INTERFACE_MODE_10GBASER: 10G BaseR
 * @PHY_INTERFACE_MODE_25GBASER: 25G BaseR
 * @PHY_INTERFACE_MODE_USXGMII:  Universal Serial 10GE MII
 * @PHY_INTERFACE_MODE_10GKR: 10GBASE-KR - with Clause 73 AN
 * @PHY_INTERFACE_MODE_QUSGMII: Quad Universal SGMII
 * @PHY_INTERFACE_MODE_1000BASEKX: 1000Base-KX - with Clause 73 AN
 * @PHY_INTERFACE_MODE_MAX: Book keeping
 *
 * Describes the interface between the MAC and PHY.
 */
typedef enum {
	PHY_INTERFACE_MODE_NA,
	PHY_INTERFACE_MODE_INTERNAL,
	PHY_INTERFACE_MODE_MII,
	PHY_INTERFACE_MODE_GMII,
	PHY_INTERFACE_MODE_SGMII,
	PHY_INTERFACE_MODE_TBI,
	PHY_INTERFACE_MODE_REVMII,
	PHY_INTERFACE_MODE_RMII,
	PHY_INTERFACE_MODE_REVRMII,
	PHY_INTERFACE_MODE_RGMII,
	PHY_INTERFACE_MODE_RGMII_ID,
	PHY_INTERFACE_MODE_RGMII_RXID,
	PHY_INTERFACE_MODE_RGMII_TXID,
	PHY_INTERFACE_MODE_RTBI,
	PHY_INTERFACE_MODE_SMII,
	PHY_INTERFACE_MODE_XGMII,
	PHY_INTERFACE_MODE_XLGMII,
	PHY_INTERFACE_MODE_MOCA,
	PHY_INTERFACE_MODE_PSGMII,
	PHY_INTERFACE_MODE_QSGMII,
	PHY_INTERFACE_MODE_TRGMII,
	PHY_INTERFACE_MODE_100BASEX,
	PHY_INTERFACE_MODE_1000BASEX,
	PHY_INTERFACE_MODE_2500BASEX,
	PHY_INTERFACE_MODE_5GBASER,
	PHY_INTERFACE_MODE_RXAUI,
	PHY_INTERFACE_MODE_XAUI,
	/* 10GBASE-R, XFI, SFI - single lane 10G Serdes */
	PHY_INTERFACE_MODE_10GBASER,
	PHY_INTERFACE_MODE_25GBASER,
	PHY_INTERFACE_MODE_USXGMII,
	/* 10GBASE-KR - with Clause 73 AN */
	PHY_INTERFACE_MODE_10GKR,
	PHY_INTERFACE_MODE_QUSGMII,
	PHY_INTERFACE_MODE_1000BASEKX,
	PHY_INTERFACE_MODE_MAX,
} phy_interface_t;

#define BITS_TO_LONGS(nr) (((nr) + (8 * sizeof(long) - 1)) / (8 * sizeof(long)))
#define DECLARE_BITMAP(name,bits) \
	unsigned long name[BITS_TO_LONGS(bits)]

/* PHY interface mode bitmap handling */
#define DECLARE_PHY_INTERFACE_MASK(name) \
	DECLARE_BITMAP(name, PHY_INTERFACE_MODE_MAX)

struct phy_driver {
	u32 phy_id;
	u32 phy_id_mask;
};


struct phy_device {
	struct mdio_device mdio;

	/* Information about the PHY type */
	/* And management functions */
	struct phy_driver *drv;

	struct device_link *devlink;

	u32 phy_id;

	// struct phy_c45_device_ids c45_ids;
	unsigned is_c45:1;
	unsigned is_internal:1;
	unsigned is_pseudo_fixed_link:1;
	unsigned is_gigabit_capable:1;
	unsigned has_fixups:1;
	unsigned suspended:1;
	unsigned suspended_by_mdio_bus:1;
	unsigned sysfs_links:1;
	unsigned loopback_enabled:1;
	unsigned downshifted_rate:1;
	unsigned is_on_sfp_module:1;
	unsigned mac_managed_pm:1;
	unsigned wol_enabled:1;

	unsigned autoneg:1;
	/* The most recently read link state */
	unsigned link:1;
	unsigned autoneg_complete:1;

	/* Interrupts are enabled */
	unsigned interrupts:1;
	unsigned irq_suspended:1;
	unsigned irq_rerun:1;

	int rate_matching;

	enum phy_state state;

	u32 dev_flags;

	phy_interface_t interface;

	/*
	 * forced speed & duplex (no autoneg)
	 * partner speed & duplex & pause (autoneg)
	 */
	int speed;
	int duplex;
	int port;
	int pause;
	int asym_pause;
	u8 master_slave_get;
	u8 master_slave_set;
	u8 master_slave_state;

	/* Union of PHY and Attached devices' supported link modes */
	/* See ethtool.h for more info */
	__ETHTOOL_DECLARE_LINK_MODE_MASK(supported);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(advertising);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(lp_advertising);
	/* used with phy_speed_down */
	__ETHTOOL_DECLARE_LINK_MODE_MASK(adv_old);
	/* used for eee validation */
	__ETHTOOL_DECLARE_LINK_MODE_MASK(supported_eee);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(advertising_eee);
	bool eee_enabled;

	/* Host supported PHY interface types. Should be ignored if empty. */
	DECLARE_PHY_INTERFACE_MASK(host_interfaces);

	/* Energy efficient ethernet modes which should be prohibited */
	u32 eee_broken_modes;

#ifdef CONFIG_LED_TRIGGER_PHY
	struct phy_led_trigger *phy_led_triggers;
	unsigned int phy_num_led_triggers;
	struct phy_led_trigger *last_triggered;

	struct phy_led_trigger *led_link_trigger;
#endif
	/*
	 * Interrupt number for this PHY
	 * -1 means no interrupt
	 */
	int irq;

	/* private data pointer */
	/* For use by PHYs to maintain extra state */
	void *priv;

	/* shared data pointer */
	/* For use by PHYs inside the same package that need a shared state. */
	struct phy_package_shared *shared;

	/* Reporting cable test results */
	struct sk_buff *skb;
	void *ehdr;
	struct nlattr *nest;

	u8 lock;

	/* This may be modified under the rtnl lock */
	bool sfp_bus_attached;
	struct sfp_bus *sfp_bus;
	struct phylink *phylink;
	struct net_device *attached_dev;
	struct mii_timestamper *mii_ts;
	struct pse_control *psec;

	u8 mdix;
	u8 mdix_ctrl;

	int pma_extable;

	unsigned int link_down_events;

	void (*phy_link_change)(struct phy_device *phydev, bool up);
	void (*adjust_link)(struct net_device *dev);

#if IS_ENABLED(CONFIG_MACSEC)
	/* MACsec management functions */
	const struct macsec_ops *macsec_ops;
#endif
};

/**
 * phy_interface_mode_is_rgmii - Convenience function for testing if a
 * PHY interface mode is RGMII (all variants)
 * @mode: the &phy_interface_t enum
 */
static inline bool phy_interface_mode_is_rgmii(phy_interface_t mode)
{
	return mode >= PHY_INTERFACE_MODE_RGMII &&
		mode <= PHY_INTERFACE_MODE_RGMII_TXID;
};

/**
 * phy_interface_is_rgmii - Convenience function for testing if a PHY interface
 * is RGMII (all variants)
 * @phydev: the phy_device struct
 */
static inline bool phy_interface_is_rgmii(struct phy_device *phydev)
{
	return phy_interface_mode_is_rgmii(phydev->interface);
};













/**
 * 
 * 
 * TO DO
 * 
 *
 * 
 * 
 *
 */
/**
 * phy_write - Convenience function for writing a given PHY register
 * @phydev: the phy_device struct
 * @regnum: register number to write
 * @val: value to write to @regnum
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static inline int phy_write(struct phy_device *phydev, u32 regnum, u16 val)
{
	// return mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, regnum, val);
	return -EINVAL;
}

/**
 * phy_read - Convenience function for reading a given PHY register
 * @phydev: the phy_device struct
 * @regnum: register number to read
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static inline int phy_read(struct phy_device *phydev, u32 regnum)
{
	// return mdiobus_read(phydev->mdio.bus, phydev->mdio.addr, regnum);
	return -EINVAL;
}

/**
 * phy_modify_paged() - Convenience function for modifying a paged register
 * @phydev: a pointer to a &struct phy_device
 * @page: the page for the phy
 * @regnum: register number
 * @mask: bit mask of bits to clear
 * @set: bit mask of bits to set
 *
 * Same rules as for phy_read() and phy_write().
 */
static inline int phy_modify_paged(struct phy_device *phydev, int page, u32 regnum,
		     u16 mask, u16 set)
{
	// int ret = phy_modify_paged_changed(phydev, page, regnum, mask, set);

	// return ret < 0 ? ret : 0;
	return -EINVAL;
}

/**
 * phy_modify - Convenience function for modifying a given PHY register
 * @phydev: the phy_device struct
 * @regnum: register number to write
 * @mask: bit mask of bits to clear
 * @set: new value of bits set in mask to write to @regnum
 *
 * NOTE: MUST NOT be called from interrupt context,
 * because the bus read/write functions may wait for an interrupt
 * to conclude the operation.
 */
static inline int phy_modify(struct phy_device *phydev, u32 regnum, u16 mask, u16 set)
{
	// int ret;

	// phy_lock_mdio_bus(phydev);
	// ret = __phy_modify(phydev, regnum, mask, set);
	// phy_unlock_mdio_bus(phydev);

	// return ret;
	return -EINVAL;
}

/**
 * phy_save_page() - take the bus lock and save the current page
 * @phydev: a pointer to a &struct phy_device
 *
 * Take the MDIO bus lock, and return the current page number. On error,
 * returns a negative errno. phy_restore_page() must always be called
 * after this, irrespective of success or failure of this call.
 */
static inline int phy_save_page(struct phy_device *phydev)
{
	// phy_lock_mdio_bus(phydev);
	// return __phy_read_page(phydev);
	return -EINVAL;
}

/**
 * phy_select_page() - take the bus lock, save the current page, and set a page
 * @phydev: a pointer to a &struct phy_device
 * @page: desired page
 *
 * Take the MDIO bus lock to protect against concurrent access, save the
 * current PHY page, and set the current page.  On error, returns a
 * negative errno, otherwise returns the previous page number.
 * phy_restore_page() must always be called after this, irrespective
 * of success or failure of this call.
 */
static inline int phy_select_page(struct phy_device *phydev, int page)
{
	// int ret, oldpage;

	// oldpage = ret = phy_save_page(phydev);
	// if (ret < 0)
	// 	return ret;

	// if (oldpage != page) {
	// 	ret = __phy_write_page(phydev, page);
	// 	if (ret < 0)
	// 		return ret;
	// }

	// return oldpage;
	return -EINVAL;
}

/**
 * phy_restore_page() - restore the page register and release the bus lock
 * @phydev: a pointer to a &struct phy_device
 * @oldpage: the old page, return value from phy_save_page() or phy_select_page()
 * @ret: operation's return code
 *
 * Release the MDIO bus lock, restoring @oldpage if it is a valid page.
 * This function propagates the earliest error code from the group of
 * operations.
 *
 * Returns:
 *   @oldpage if it was a negative value, otherwise
 *   @ret if it was a negative errno value, otherwise
 *   phy_write_page()'s negative value if it were in error, otherwise
 *   @ret.
 */
int phy_restore_page(struct phy_device *phydev, int oldpage, int ret)
{
	// int r;

	// if (oldpage >= 0) {
	// 	r = __phy_write_page(phydev, oldpage);

	// 	/* Propagate the operation return code if the page write
	// 	 * was successful.
	// 	 */
	// 	if (ret >= 0 && r < 0)
	// 		ret = r;
	// } else {
	// 	/* Propagate the phy page selection error code */
	// 	ret = oldpage;
	// }

	// phy_unlock_mdio_bus(phydev);

	// return ret;
	return -EINVAL;
}
#endif /* __PHY_H */