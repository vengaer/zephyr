/*
 * SPDX-FileCopyrightText: Copyright 2025-2026 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_dsa_core, CONFIG_NET_DSA_LOG_LEVEL);

#include <zephyr/net/ethernet.h>
#include <zephyr/net/dsa_core.h>
#include <zephyr/net/dsa_tag.h>

struct net_if *dsa_recv(struct net_if *iface, struct net_pkt *pkt)
{
	const struct dsa_switch_context *dsa_switch_ctx;
	const struct ethernet_context *eth_ctx;
	const struct dsa_port_config *cfg;
	const struct device *dev;

	if (iface == NULL || pkt == NULL) {
		return iface;
	}

	if (IS_ENABLED(CONFIG_DSA_CASCADING)) {
		/* Resolve cascading */
		do {
			eth_ctx = net_if_l2_data(iface);
			if (eth_ctx == NULL) {
				return iface;
			}

			/* Keep resolving until the first non-DSA port is reached */
			if (eth_ctx->dsa_port != DSA_PORT) {
				break;
			}

			dev = net_if_get_device(iface);
			if (dev == NULL) {
				return iface;
			}

			cfg = dev->config;
			dsa_switch_ctx = dev->data;

			/* Get the next interface in the chain with the help of the tag protocol. */
			iface = dsa_tag_recv(iface, pkt);
		} while (1);
	}

	/* Tag protocol handles the final untag and redirect */
	return dsa_tag_recv(iface, pkt);
}

int dsa_xmit(const struct device *dev, struct net_pkt *pkt)
{
	struct dsa_switch_context *dsa_switch_ctx = dev->data;
	struct net_if *iface = net_if_lookup_by_dev(dev);
	const struct ethernet_api *eth_api_conduit;
	const struct ethernet_context *eth_ctx;
	const struct device *dev_upstream;
	const struct device *dev_conduit;
	struct net_pkt *dsa_pkt;
	struct net_pkt *clone;
	int ret;

#ifdef CONFIG_NET_L2_PTP_TIMESTAMPING
	/* Handle TX timestamp if defines */
	if (net_ntohs(NET_ETH_HDR(pkt)->type) == NET_ETH_PTYPE_PTP &&
	    dsa_switch_ctx->dapi->port_txtstamp != NULL) {
		ret = dsa_switch_ctx->dapi->port_txtstamp(dev, pkt);
		if (ret != 0) {
			return ret;
		}
	}
#endif
	/*
	 * In case of using TX pkt in other places, pkt should not be changed.
	 * Here just clone pkt to use for tagging and sending.
	 * It could be optimized here for performance in the future if some mechanism
	 * implemented marks whether the pkt data will be accessed or not in other
	 * places after sending.
	 */
	clone = net_pkt_clone(pkt, K_NO_WAIT);
	if (clone == NULL) {
		return -ENOBUFS;
	}

	/* Tag protocol handles pkt first */
	dsa_pkt = dsa_tag_xmit(iface, clone);

	if (IS_ENABLED(CONFIG_DSA_CASCADING)) {
		/* Resolve cascading */
		while (dsa_pkt != NULL) {
			iface = dsa_switch_ctx->iface_conduit;
			eth_ctx = net_if_l2_data(iface);

			/* Keep going until a non-DSA port (i.e. the conduit) is reached */
			if (eth_ctx->dsa_port != DSA_PORT) {
				break;
			}

			/* Insert DSA port tag */
			dsa_pkt = dsa_tag_xmit(iface, dsa_pkt);

			dev_upstream = net_if_get_device(iface);
			dsa_switch_ctx = dev_upstream->data;
		}
	} else {
		iface = dsa_switch_ctx->iface_conduit;
	}

	/* Transmit from conduit port */
	dev_conduit = net_if_get_device(iface);
	eth_api_conduit = dev_conduit->api;
	ret = eth_api_conduit->send(dev_conduit, dsa_pkt);

	/* Release the cloned pkt */
	net_pkt_unref(clone);

	return ret;
}

int dsa_eth_init(struct net_if *iface)
{
	struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	if (eth_ctx->dsa_port == DSA_CONDUIT_PORT) {
		net_if_flag_clear(iface, NET_IF_IPV4);
		net_if_flag_clear(iface, NET_IF_IPV6);
	}

	return 0;
}

struct net_if *dsa_conduit_get_iface(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct dsa_switch_context *dsa_switch_ctx;
	const struct ethernet_context *eth_ctx;

	if (net_if_l2(iface) != &NET_L2_GET_NAME(ETHERNET)) {
		return NULL;
	}

	eth_ctx = net_if_l2_data(iface);
	switch (eth_ctx->dsa_port) {
	case DSA_USER_PORT:
	case DSA_CPU_PORT:
	case DSA_PORT:
		break;
	default:
		return NULL;
	}

	dsa_switch_ctx = dev->data;
	return dsa_switch_ctx->iface_conduit;
}
