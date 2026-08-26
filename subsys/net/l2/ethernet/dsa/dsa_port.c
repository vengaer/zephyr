/*
 * SPDX-FileCopyrightText: Copyright 2025-2026 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_dsa_port, CONFIG_NET_DSA_LOG_LEVEL);

#include <zephyr/devicetree.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/dsa_core.h>
#include <zephyr/net/dsa_tag.h>

#include <stdbool.h>

#if defined(CONFIG_NET_INTERFACE_NAME_LEN)
#define INTERFACE_NAME_LEN CONFIG_NET_INTERFACE_NAME_LEN
#else
#define INTERFACE_NAME_LEN 10
#endif

#define DSA_NUM_PORTS_STATUS_OKAY DT_NUM_INST_STATUS_OKAY(zephyr_dsa_port)

static void dsa_configure_dsa_downstream_of(struct net_if *upstream_iface)
{
	struct dsa_switch_context *upstream_switch_ctx, *downstream_switch_ctx;
	const struct dsa_port_config *upstream_cfg, *downstream_cfg;
	const struct device *upstream_dev, *downstream_dev;
	struct ethernet_context *downstream_eth_ctx;
	struct net_if *downstream_iface;

	upstream_dev = net_if_get_device(upstream_iface);
	upstream_cfg = upstream_dev->config;
	upstream_switch_ctx = upstream_dev->data;

	downstream_dev = upstream_switch_ctx->dev_dsa[upstream_cfg->port_idx];
	downstream_iface = net_if_lookup_by_dev(downstream_dev);

	if (downstream_iface == NULL) {
		LOG_ERR("DSA: Could not locate downstream interface");
		return;
	}

	downstream_eth_ctx = net_if_l2_data(downstream_iface);

	/* Mark the downstream port for cascading */
	downstream_eth_ctx->dsa_port = DSA_PORT;

	downstream_switch_ctx = downstream_dev->data;
	downstream_cfg = downstream_dev->config;

	/* Associate the ports with one another */
	upstream_switch_ctx->iface_dsa[upstream_cfg->port_idx] = downstream_iface;
	downstream_switch_ctx->iface_dsa[downstream_cfg->port_idx] = upstream_iface;

	/* Packets arriving at the upstream switch should be routed through its DSA port
	 * to the downstream counterpart
	 */
	upstream_switch_ctx->iface_conduit = downstream_iface;

	LOG_DBG("DSA: connected iface %d (upstream) to %d (downstream)",
		net_if_get_by_iface(upstream_iface), net_if_get_by_iface(downstream_iface));
}

static void dsa_configure_dsa_downstreams(void)
{
	unsigned int i;
	const struct ethernet_context *eth_ctx;
	uint8_t dsa_upstreams[(DSA_NUM_PORTS_STATUS_OKAY + 7u) / 8u] = {0};

	/* At this point, upstream ports have the type DSA_PORT whereas downstream counterparts
	 * are still DSA_USER_PORT. Since dsa_configure_dsa_downstream_of changes the type of
	 * downstream DSA ports to DSA_PORT, two loops are required here. The first identifies
	 * all upstream ports, the second configures the downstreams.
	 */
	i = 0u;
	STRUCT_SECTION_FOREACH(net_if, iface) {
		eth_ctx = net_if_l2_data(iface);

		switch (eth_ctx->dsa_port) {
		case DSA_PORT:
			/* DSA upstreams have type DSA_PORT here */
			dsa_upstreams[i / 8u] |= BIT(i & 7u);
			break;
		case DSA_USER_PORT:
			/* DSA downstreams have type DSA_USER_PORT at this point */
			break;
		default:
			/* Don't care about remaining types */
			continue;
		}

		++i;
	}

	i = 0u;
	STRUCT_SECTION_FOREACH(net_if, iface) {
		eth_ctx = net_if_l2_data(iface);

		/* Care only about DSA and user ports */
		if (eth_ctx->dsa_port != DSA_USER_PORT && eth_ctx->dsa_port != DSA_PORT) {
			continue;
		}

		if ((dsa_upstreams[i / 8u] & BIT(i & 7u)) != 0) {
			dsa_configure_dsa_downstream_of(iface);
		}

		++i;
	}
}

int dsa_port_initialize(const struct device *dev)
{
	static unsigned int init_ports_total;

	const struct dsa_port_config *cfg = dev->config;
	struct dsa_switch_context *dsa_switch_ctx = dev->data;
	struct net_if *iface = net_if_lookup_by_dev(dev);
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);
	struct ethernet_context *eth_ctx_conduit = NULL;
	int err = 0;

	dsa_switch_ctx->init_ports++;
	init_ports_total++;

	/* If the port has an associated DSA device, it is a DSA port */
	if (dsa_switch_ctx->dev_dsa[cfg->port_idx] != NULL) {
		if (cfg->ethernet_connection != NULL) {
			LOG_ERR("DSA: node designated as both CPU and DSA port");
			err = -EINVAL;
			goto out;
		}

		eth_ctx->dsa_port = DSA_PORT;

		/* Upstream DSA port is the host */
		dsa_switch_ctx->iface_host = iface;
	}

	/* Find the connection of conduit port and cpu port */
	if (dsa_switch_ctx->iface_conduit == NULL && cfg->ethernet_connection != NULL) {
		dsa_switch_ctx->iface_conduit = net_if_lookup_by_dev(cfg->ethernet_connection);
		if (dsa_switch_ctx->iface_conduit == NULL) {
			LOG_ERR("DSA: Conduit iface NOT found!");
		}

		/* Set up tag protocol on the cpu port */
		eth_ctx->dsa_port = DSA_CPU_PORT;
		dsa_tag_setup(dev);

		/* Provide DSA information to the conduit port */
		eth_ctx_conduit = net_if_l2_data(dsa_switch_ctx->iface_conduit);
		eth_ctx_conduit->dsa_switch_ctx = dsa_switch_ctx;
		eth_ctx_conduit->dsa_port = DSA_CONDUIT_PORT;

		/* CPU port is the host */
		dsa_switch_ctx->iface_host = iface;
	}

	if (eth_ctx->dsa_port != DSA_PORT && cfg->ethernet_connection == NULL) {
		/* Note that the port may still be a DSA port rather than a user port here. We won't
		 * know for sure until all ports have been initialized. For now though, assume
		 * user port.
		 */
		eth_ctx->dsa_port = DSA_USER_PORT;
		eth_ctx->dsa_switch_ctx = dsa_switch_ctx;
		dsa_switch_ctx->iface_user[cfg->port_idx] = iface;
	}

	if (dsa_switch_ctx->dapi->port_init != NULL) {
		err = dsa_switch_ctx->dapi->port_init(dev);
		if (err != 0) {
			goto out;
		}
	}

out:
	/* All ports are initialized. May need switch setup. */
	if (dsa_switch_ctx->init_ports == dsa_switch_ctx->num_ports) {
		if (dsa_switch_ctx->dapi->switch_setup != NULL) {
			err = dsa_switch_ctx->dapi->switch_setup(dsa_switch_ctx);
		}
	}

	/* DSA downstreams must be configured once all ports have been initialized */
	if (IS_ENABLED(CONFIG_DSA_CASCADING) && init_ports_total == DSA_NUM_PORTS_STATUS_OKAY) {
		dsa_configure_dsa_downstreams();
	}

	return err;
}

static void dsa_port_phylink_change(const struct device *phydev, struct phy_link_state *state,
				    void *user_data)
{
	struct net_if *iface = (struct net_if *)user_data;
	const struct device *dev = net_if_get_device(iface);
	struct dsa_switch_context *dsa_switch_ctx = dev->data;

	if (dsa_switch_ctx->dapi->port_phylink_change != NULL) {
		dsa_switch_ctx->dapi->port_phylink_change(phydev, state, dev);
	}

	net_eth_carrier_set(iface, state->is_up);
}

static void dsa_port_iface_init(struct net_if *iface)
{
	static unsigned int dsa_iface_idx;

	const struct ethernet_context *eth_ctx = net_if_l2_data(iface);
	const struct device *dev = net_if_get_device(iface);
	const struct dsa_port_config *cfg = dev->config;
	char name[INTERFACE_NAME_LEN];
	uint8_t mac_addr[6] = {0};
	int ret;

	/* Set interface name */
	snprintk(name, sizeof(name), "swp%u", dsa_iface_idx);
	dsa_iface_idx++;
	net_if_set_name(iface, name);

	ret = net_eth_mac_load(&cfg->mcfg, mac_addr);
	if (ret >= 0) {
		/* only set MAC address if successfully loaded, this way we won't overwrite a valid
		 * MAC address, that might be already set by the dsa switch.
		 */
		net_if_set_link_addr(iface, mac_addr, sizeof(mac_addr), NET_LINK_ETHERNET);
	}

	switch (eth_ctx->dsa_port) {
	case DSA_PORT:
	case DSA_CPU_PORT:
		/* DSA CPU and cascade ports used only for DSA management */
		net_if_flag_clear(iface, NET_IF_IPV4);
		net_if_flag_clear(iface, NET_IF_IPV6);

		net_if_carrier_off(iface);
		return;
	default:
		break;
	}

	/*
	 * Initialize ethernet context 'work' for this iface to
	 * be able to monitor the carrier status.
	 */
	ethernet_init(iface);

	/* Do not start the interface until link is up */
	net_if_carrier_off(iface);

	if (!device_is_ready(cfg->phy_dev)) {
		LOG_ERR("PHY device (%p) is not ready, cannot init iface", cfg->phy_dev);
		return;
	}

	phy_link_callback_set(cfg->phy_dev, dsa_port_phylink_change, (void *)iface);
}

static const struct device *dsa_port_get_phy(const struct device *dev,
					     struct net_if *iface __unused)
{
	const struct dsa_port_config *cfg = dev->config;

	return cfg->phy_dev;
}

#ifdef CONFIG_NET_L2_PTP_TIMESTAMPING
const struct device *dsa_port_get_ptp_clock(const struct device *dev,
					    struct net_if *iface __unused)
{
	const struct dsa_port_config *cfg = dev->config;

	return cfg->ptp_clock;
}
#endif

static enum ethernet_hw_caps dsa_port_get_capabilities(const struct device *dev,
						       struct net_if *iface __unused)
{
	struct dsa_switch_context *dsa_switch_ctx = dev->data;

	if (dsa_switch_ctx->dapi->get_capabilities == NULL) {
		return (enum ethernet_hw_caps)0;
	}

	return dsa_switch_ctx->dapi->get_capabilities(dev);
}

static int dsa_set_config(const struct device *dev,
			  struct net_if *iface __unused,
			  enum ethernet_config_type type,
			  const struct ethernet_config *config)
{
	struct dsa_switch_context *dsa_switch_ctx = dev->data;

	if (!dsa_switch_ctx->dapi->set_config) {
		return -ENOTSUP;
	}

	return dsa_switch_ctx->dapi->set_config(dev, type, config);
}

static int dsa_get_config(const struct device *dev,
			  struct net_if *iface __unused,
			  enum ethernet_config_type type,
			  struct ethernet_config *config)
{
	struct dsa_switch_context *dsa_switch_ctx = dev->data;

	if (!dsa_switch_ctx->dapi->get_config) {
		return -ENOTSUP;
	}

	return dsa_switch_ctx->dapi->get_config(dev, type, config);
}

const struct ethernet_api dsa_eth_api = {
	.iface_api.init = dsa_port_iface_init,
	.get_phy = dsa_port_get_phy,
	.send = dsa_xmit,
#ifdef CONFIG_NET_L2_PTP_TIMESTAMPING
	.get_ptp_clock = dsa_port_get_ptp_clock,
#endif
	.get_capabilities = dsa_port_get_capabilities,
	.set_config = dsa_set_config,
	.get_config = dsa_get_config,
};
