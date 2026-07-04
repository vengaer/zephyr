/*
 * Copyright 2025 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief Distributed Switch Architecture (DSA)
 */

#ifndef ZEPHYR_INCLUDE_NET_DSA_CORE_H_
#define ZEPHYR_INCLUDE_NET_DSA_CORE_H_

#include <assert.h>
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/ethernet.h>

/**
 * @brief Distributed Switch Architecture (DSA)
 * @defgroup dsa_core Distributed Switch Architecture (DSA)
 * @since 4.2
 * @version 0.8.0
 * @ingroup networking
 * @{
 */

/** @cond INTERNAL_HIDDEN */

#if defined(CONFIG_DSA_PORT_MAX_COUNT)
#define DSA_PORT_MAX_COUNT CONFIG_DSA_PORT_MAX_COUNT
#else
#define DSA_PORT_MAX_COUNT 0
#endif

/** @endcond */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Macro for DSA port instance initialization.
 *
 * @param port	DSA port node identifier.
 * @param n	DSA instance number.
 * @param cfg	Pointer to dsa_port_config.
 */
#define DSA_PORT_INST_INIT(port, n, cfg)                                                           \
	ETH_NET_DEVICE_DT_DEFINE(port, dsa_port_initialize, NULL, &dsa_switch_context_##n, cfg,    \
				 CONFIG_ETH_INIT_PRIORITY, &dsa_eth_api, NET_ETH_MTU)

static_assert(!IS_ENABLED(CONFIG_DSA_CASCADING) || CONFIG_ETH_INIT_PRIORITY < CONFIG_NET_INIT_PRIO,
	      "Network stack must be initialized after Ethernet drivers");

/**
 * @brief Macro for DSA switch instance initialization.
 *
 * @param n	DSA instance number.
 * @param _dapi	Pointer to dsa_api.
 * @param data	Pointer to private data.
 * @param fn	DSA port instance init function.
 */
#define DSA_SWITCH_INST_INIT(n, _dapi, data, fn)                                                   \
	STRUCT_SECTION_ITERABLE(dsa_switch_context, dsa_switch_context_##n) = {                    \
		.dapi = _dapi,                                                                     \
		.prv_data = data,                                                                  \
		.init_ports = 0,                                                                   \
		.num_ports = DT_INST_CHILD_NUM_STATUS_OKAY(n),                                     \
		.dev = DEVICE_DT_INST_GET(n),                                                      \
		.port_cascade = DSA_CASCADING_PORTS(n),                                            \
	};                                                                                         \
	DT_INST_FOREACH_CHILD_STATUS_OKAY_VARGS(n, fn, n);

/** @cond INTERNAL_HIDDEN */
#ifdef CONFIG_DSA_CASCADING

#define DSA_CASCADING_FIELDS(n)                                                                    \
	COND_CODE_1(IS_ENABLED(CONFIG_DSA_CASCADING),                                              \
		({ DT_INST_FOREACH_STATUS_OKAY(n, DSA_CASCADE_PORT_POPULATE) }),                   \
		({ 0 }))

#define DSA_CASCADE_PORT_POPULATE(port)                                                            \
	[DT_REG_ADDR(port)] = COND_CODE_1(							   \
			DT_NODE_HAS_STATUS_OKAY(DT_PHANDLE(port, dsa_cascade_ports)),		   \
			(&(struct const dsa_cascade) {                                             \
				.dev = DEVICE_DT_GET(DT_PHANDLE(port, dsa_cascade_ports)),         \
				.port_idx = DT_PHA(port, dsa_cascade_ports, port_idx),             \
			}),									   \
			(NULL)								           \
		),

/* DSA cascade port abstraction.
 *
 * Used during initialization.
 */
struct dsa_cascade {
	/* Pointer to the device representing the downstream switch */
	const struct device *dev;

	/* Zero-based index of the downstream switch port on @c dev */
	unsigned int port_idx;
};

/** @endcond */

/** DSA switch context data */
struct dsa_switch_context {
	union {
		/** Pointers to all DSA user network interfaces */
		struct net_if *iface_user[DSA_PORT_MAX_COUNT];

		/** Pointers to DSA cascading network interfaces */
		struct net_if *iface_cascade[DSA_PORT_MAX_COUNT];

		/** Pointers to cascade ports parsed from devicetree */
		const struct dsa_cascade *port_cascade[DSA_PORT_MAX_COUNT];
	};

	/** Pointer to DSA conduit network interface */
	struct net_if *iface_conduit;

	/** DSA specific API callbacks */
	struct dsa_api *dapi;

	/** Instance specific data */
	void *prv_data;

	/** Number of ports in the DSA switch */
	uint8_t num_ports;

	/** Number of initialized ports in the DSA switch */
	uint8_t init_ports;

	/** DSA tagger data provided by instance when connecting to tag protocol */
	void *tagger_data;

	/** Pointer to the switch associated with this context */
	const struct device *const dev;

#if defined(CONFIG_DSA_CASCADING) || defined(__DOXYGEN__)

	/** Bitmask used to synchronize initialization */
	uint8_t init_bits[ROUND_UP(DSA_PORT_MAX_COUNT, 8u) >> 3u];

	/** Bitmask used to identify cascading ports during initialization */
	uint8_t cascade_bits[ROUND_UP(DSA_PORT_MAX_COUNT, 8u) >> 3u];

#endif /* CONFIG_DSA_CASCADING || __DOXYGEN__ */
};

/**
 * Structure to provide DSA switch api callbacks - it is an augmented
 * struct ethernet_api.
 */
struct dsa_api {
	/** DSA helper callbacks */

	/** Handle receive packet on conduit port for untagging and redirection */
	struct net_if *(*recv)(struct net_if *iface, struct net_pkt *pkt);

	/** Transmit packet on the user port with tagging */
	struct net_pkt *(*xmit)(struct net_if *iface, struct net_pkt *pkt);

	/** Port init */
	int (*port_init)(const struct device *dev);

	/** Port link change */
	void (*port_phylink_change)(const struct device *phy_dev, struct phy_link_state *state,
				    const struct device *dev);

#if defined(CONFIG_NET_L2_PTP_TIMESTAMPING) || defined(__DOXYGEN__)
	/**
	 * Port TX timestamp handling
	 * @kconfig_dep{CONFIG_NET_L2_PTP_TIMESTAMPING}
	 */
	int (*port_txtstamp)(const struct device *dev, struct net_pkt *pkt);
#endif
	/** Switch setup */
	int (*switch_setup)(const struct dsa_switch_context *dsa_switch_ctx);

	/** Connect the switch to the tag protocol */
	int (*connect_tag_protocol)(struct dsa_switch_context *dsa_switch_ctx, int tag_proto);

	/** Get the device capabilities */
	enum ethernet_hw_caps (*get_capabilities)(const struct device *dev);

	/** Set specific hardware configuration */
	int (*set_config)(const struct device *dev,
			  enum ethernet_config_type type,
			  const struct ethernet_config *config);

	/** Get hardware specific configuration */
	int (*get_config)(const struct device *dev,
			  enum ethernet_config_type type,
			  struct ethernet_config *config);
};

/**
 * Structure of DSA port configuration.
 */
struct dsa_port_config {
	/** Port mac address */
	struct net_eth_mac_config mcfg;
	/** Port index */
	const int port_idx;
	/** PHY device */
	const struct device *phy_dev;
	/** PHY mode */
	const char *phy_mode;
	/** Tag protocol */
	const int tag_proto;
	/** Ethernet device connected to the port */
	const struct device *ethernet_connection;
#if defined(CONFIG_NET_L2_PTP_TIMESTAMPING) || defined(__DOXYGEN__)
	/**
	 * PTP clock used on the port
	 * @kconfig_dep{CONFIG_NET_L2_PTP_TIMESTAMPING}
	 */
	const struct device *ptp_clock;
#endif
	/** Instance specific config */
	void *prv_config;
};

/** @cond INTERNAL_HIDDEN */

/*
 * DSA port init
 *
 * Returns:
 *  - 0 if ok, < 0 if error
 */
int dsa_port_initialize(const struct device *dev);

/*
 * DSA transmit function
 *
 * param dev: Port device to transmit
 * param pkt: Network packet
 *
 * Returns:
 *  - 0 if ok, < 0 if error
 */
int dsa_xmit(const struct device *dev, struct net_pkt *pkt);

/*
 * DSA receive function
 *
 * param iface: Interface of conduit port
 * param pkt: Network packet
 *
 * Returns:
 *  - Interface to redirect
 */
struct net_if *dsa_recv(struct net_if *iface, struct net_pkt *pkt);

/*
 * Look up switch context for the provided device.
 *
 * param dev: Device representing the switch.
 *
 * Returns:
 *  - Address of the DSA switch context associated with dev, or NULL if no
 *    such context is found.
 */
struct dsa_switch_context *dsa_switch_context_lookup_by_dev(const struct device *dev);

/*
 * DSA ethernet init function to handle flags
 *
 * param iface: Interface of port
 *
 * Returns:
 *  - 0 if ok, < 0 if error
 */
int dsa_eth_init(struct net_if *iface);

/* Ethernet APIs definition for switch ports */
extern const struct ethernet_api dsa_eth_api;

/** @endcond */

/**
 * @brief      Get network interface of a user port
 *
 * @param      iface     Conduit port
 * @param[in]  port_idx  Port index
 *
 * @return     network interface of the user if successful
 * @return     NULL if user port does not exist
 */
struct net_if *dsa_user_get_iface(struct net_if *iface, int port_idx);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#endif /* ZEPHYR_INCLUDE_NET_DSA_CORE_H_ */
