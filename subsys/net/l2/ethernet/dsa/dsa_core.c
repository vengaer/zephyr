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
	const struct device *dev __unused;
	const struct ethernet_context *eth_ctx __unused;
	const struct dsa_port_config *cfg __unused;
	const struct dsa_switch_context *dsa_switch_ctx __unused;

	if (iface == NULL || pkt == NULL) {
		return iface;
	}

	/* Tag protocol handles to untag and re-direct interface */
	iface = dsa_tag_recv(iface, pkt);
	if (unlikely(!iface)) {
		return NULL;
	}

#ifdef CONFIG_DSA_CASCADING
	/* Resolve casading internally */
	do {
		eth_ctx = net_if_l2_data(iface);
		if (unlikely(!eth_ctx)) {
			return NULL;
		}

		/* Done when reading the first non-DSA port */
		if (eth_ctx->dsa_port != DSA_PORT) {
			break;
		}

		dev = net_if_get_device(iface);
		if (unlikely(!dev)) {
			return NULL;
		}

		cfg = dev->config;
		dsa_switch_ctx = dev->data;

		/* Get the next interface in the chain, potentially redirecting
		 * via the tag protocol. Since the iface_user and iface_cascade
		 * use the same memory, simply relying on the normal tag
		 * protocol recv is sufficient
		 */
		iface = dsa_tag_recv(iface, pkt);
	} while (1);
#endif

	return iface;
}

int dsa_xmit(const struct device *dev, struct net_pkt *pkt)
{
	const struct device *dev_upstream __unused;
	const struct ds_port_config *cfg __unused = dev->config;
	struct dsa_switch_context *dsa_switch_ctx = dev->data;
	struct net_if *iface = net_if_lookup_by_dev(dev);
	const struct ethernet_context *eth_ctx __unused = net_if_l2_data(iface);
	struct net_if *iface_conduit = NULL;
	const struct device *dev_conduit = NULL;
	const struct ethernet_api *eth_api_conduit = NULL;
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

#ifdef CONFIG_DSA_CASCADING
	/* Handle cascading */
	if (eth_ctx->dsa_port == DSA_PORT) {
		/* Reroute the packet through the upstream interface */
		iface = dsa_switch_ctx->iface_cascace[cfg->port_idx];
		dev_upstream = net_if_get_device(iface);
		dsa_switch_ctx = dev_upstream->data;
	}
#endif

	/* Transmit from conduit port */
	iface_conduit = dsa_switch_ctx->iface_conduit;
	dev_conduit = net_if_get_device(iface_conduit);
	eth_api_conduit = dev_conduit->api;
	ret = eth_api_conduit->send(dev_conduit, dsa_pkt);

	/* Release the cloned pkt */
	net_pkt_unref(clone);

	return ret;
}

int dsa_eth_init(struct net_if *iface)
{
	const struct ethernet_context *eth_ctx = net_if_l2_data(iface);

	switch (eth_ctx->dsa_port) {
	case DSA_PORT:
	case DSA_CONDUIT_PORT:
		net_if_flag_clear(iface, NET_IF_IPV4);
		net_if_flag_clear(iface, NET_IF_IPV6);
		break;
	default:
		break;
	}

	return 0;
}

struct dsa_switch_context *dsa_switch_context_lookup_by_dev(const struct device *dev)
{
	STRUCT_SECTION_FOREACH(dsa_switch_context, dsa_switch_ctx) {
		if (dsa_switch_ctx->dev == dev) {
			return dsa_switch_ctx;
		}
	}

	return NULL;
}
