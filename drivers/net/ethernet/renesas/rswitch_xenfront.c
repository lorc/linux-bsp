// SPDX-License-Identifier: GPL-2.0
/* Renesas Ethernet Switch Para-Virtualized driver
 *
 * Copyright (C) 2022 EPAM Systems
 */

#include "linux/dynamic_debug.h"
#include "xen/events.h"
#include "xen/interface/event_channel.h"
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <xen/interface/grant_table.h>
#include <xen/grant_table.h>
#include <xen/xenbus.h>
#include <xen/page.h>
#include "rswitch.h"

static DECLARE_WAIT_QUEUE_HEAD(module_wq);

struct rswitch_vmq_front_info {
	evtchn_port_t tx_evtchn;
	evtchn_port_t rx_evtchn;
	struct net_device *ndev;
	struct xenbus_device *xbdev;
	/* TODO: Make those global for driver itself */
	struct rswitch_desc *desc_bat;
	dma_addr_t desc_bat_dma;
	size_t desc_bat_size;
};

static struct net_device*
		rswitch_vmq_front_ndev_allocate(struct xenbus_device *xbd)
{
	struct net_device *ndev;
	struct rswitch_device *rdev;

	ndev = alloc_etherdev_mqs(sizeof(struct rswitch_device), 1, 1);

	if (!ndev)
		return ERR_PTR(-ENOMEM);

	SET_NETDEV_DEV(ndev, &xbd->dev);
	pr_info("SET_NETDEV_DEV(ndev, %px)\n", &xbd->dev);
	ether_setup(ndev);

	rdev = netdev_priv(ndev);
	rdev->front_info = devm_kzalloc(&xbd->dev,
					sizeof(struct rswitch_vmq_front_info),
					GFP_KERNEL);
	if (!rdev->front_info)
		goto out_free_netdev;

	rdev->front_info->ndev = ndev;
	rdev->front_info->xbdev = xbd;
	rdev->ndev = ndev;
	rdev->pv_device = true;
	rdev->priv = NULL;
	rdev->port = 4; 	/* TODO: This is supposed to be GWCA0 port */
	rdev->etha = NULL;
	rdev->remote_chain = -1;
	rdev->addr = NULL;


	spin_lock_init(&rdev->lock);

	ndev->features = NETIF_F_RXCSUM;
	ndev->hw_features = NETIF_F_RXCSUM;
	ndev->base_addr = (unsigned long)rdev->addr;
	ndev->netdev_ops = &rswitch_netdev_ops;

	return ndev;

out_free_netdev:
	free_netdev(ndev);

	return ERR_PTR(-ENOMEM);
}

static int rswitch_vmq_front_ndev_register(struct rswitch_device *rdev,
					   int index,
					   int tx_chain_num,
					   int rx_chain_num)
{
	struct net_device *ndev = rdev->ndev;
	int err;

	snprintf(ndev->name, IFNAMSIZ, "vmq%d", index);
	netif_napi_add(ndev, &rdev->napi, rswitch_poll, 64);

	eth_hw_addr_random(ndev);

	/* Network device register */
	err = register_netdev(ndev);
	if (err)
		goto out_reg_netdev;

	err = rswitch_rxdmac_init(ndev, NULL, rx_chain_num);
	if (err < 0)
		goto out_rxdmac;

	err = rswitch_txdmac_init(ndev, NULL, tx_chain_num);
	if (err < 0)
		goto out_txdmac;

	/* Print device information */
	netdev_info(ndev, "MAC address %pMn", ndev->dev_addr);

	return 0;

out_txdmac:
	rswitch_rxdmac_free(ndev, NULL);

out_rxdmac:
	unregister_netdev(ndev);

out_reg_netdev:
	netif_napi_del(&rdev->napi);

	return err;
}

static irqreturn_t rswitch_vmq_front_rx_interrupt(int irq, void *dev_id)
{
	struct rswitch_device *rdev = dev_id;

	napi_schedule(&rdev->napi);

	return IRQ_HANDLED;
}

static irqreturn_t rswitch_vmq_front_tx_interrupt(int irq, void *dev_id)
{
	struct rswitch_device *rdev = dev_id;

	napi_schedule(&rdev->napi);

	/* This is better, but there is a possibility for locking issues */
	//	rswitch_tx_free(rdev->ndev, true);

	return IRQ_HANDLED;
}

void rswitch_vmq_front_trigger_tx(struct rswitch_device* rdev)
{
	struct rswitch_vmq_front_info *np = rdev->front_info;

	notify_remote_via_evtchn(np->tx_evtchn);
}

void rswitch_vmq_front_rx_done(struct rswitch_device* rdev)
{
	struct rswitch_vmq_front_info *np = rdev->front_info;

	notify_remote_via_evtchn(np->rx_evtchn);
}

static int rswitch_vmq_front_connect(struct net_device *dev)
{
	struct rswitch_device *rdev = netdev_priv(dev);
	struct rswitch_vmq_front_info *np = rdev->front_info;
	unsigned int tx_chain_id, rx_chain_id, index;
	unsigned int remote_chain_id;
	int err;

	pr_info("otherend: %s\n", np->xbdev->otherend);

	tx_chain_id = xenbus_read_unsigned(np->xbdev->otherend,
					   "tx-chain-id", 0);
	rx_chain_id = xenbus_read_unsigned(np->xbdev->otherend,
					   "rx-chain-id", 0);
	remote_chain_id = xenbus_read_unsigned(np->xbdev->otherend,
					   "remote-chain-id", 0);
	index = xenbus_read_unsigned(np->xbdev->nodename, "if-num", ~0U);

	if (!tx_chain_id || !rx_chain_id) {
		dev_info(&np->xbdev->dev, "backend did not supplied chain id\n");
		return -ENODEV;
	}

	pr_info("tx_chain_id: %d\n", tx_chain_id);
	pr_info("rx_chain_id: %d\n", rx_chain_id);
	pr_info("remote_chain_id: %d\n", remote_chain_id);
	pr_info("index: %d\n", index);

	err = rswitch_vmq_front_ndev_register(rdev, index, tx_chain_id, rx_chain_id);
	if (err)
		return err;

	/* TODO: Add error handling */
	err = xenbus_alloc_evtchn(np->xbdev, &np->rx_evtchn);
	if (err)
		return err;

	err = xenbus_alloc_evtchn(np->xbdev, &np->tx_evtchn);
	if (err)
		return err;

	err = bind_evtchn_to_irqhandler(np->rx_evtchn,
					rswitch_vmq_front_rx_interrupt,
					0, rdev->ndev->name, rdev);
	if (err < 0)
		return err;

	err = bind_evtchn_to_irqhandler(np->tx_evtchn,
					rswitch_vmq_front_tx_interrupt,
					0, rdev->ndev->name, rdev);
	if (err < 0)
		return err;

	rdev->remote_chain = remote_chain_id;

	/* TODO: Add error handling */
	xenbus_printf(XBT_NIL, np->xbdev->nodename, "tx-chain-dma-addr", "%llx",
		      rdev->tx_chain->ring_dma);
	xenbus_printf(XBT_NIL, np->xbdev->nodename, "rx-chain-dma-addr", "%llx",
		      rdev->rx_chain->ring_dma);

	xenbus_printf(XBT_NIL, np->xbdev->nodename, "tx-evtch", "%u", np->tx_evtchn);
	xenbus_printf(XBT_NIL, np->xbdev->nodename, "rx-evtch", "%u", np->rx_evtchn);

	/* TODO: Add error handling */
	return 0;
}

static int rswitch_vmq_front_probe(struct xenbus_device *dev,
				   const struct xenbus_device_id *id)
{
	int err;
	struct net_device *netdev;

	netdev = rswitch_vmq_front_ndev_allocate(dev);
	if (IS_ERR(netdev)) {
		err = PTR_ERR(netdev);
		xenbus_dev_fatal(dev, err, "creating netdev");
		return err;
	}

	dev_set_drvdata(&dev->dev, netdev_priv(netdev));
	err = dma_coerce_mask_and_coherent(&dev->dev, DMA_BIT_MASK(40));

	if (err)
		pr_err("failed to set dma mask: %d\n", err);
	if (dev->dev.dma_mask)
		pr_info("dma mask: %llx\n", *dev->dev.dma_mask);
	else
		pr_info("dma mask is NULL\n");
	pr_info("dev = %px coherent dma mask: %llx\n", &dev->dev, dev->dev.coherent_dma_mask);
	do {
		xenbus_switch_state(dev, XenbusStateInitialising);
		err = wait_event_timeout(module_wq,
				 xenbus_read_driver_state(dev->otherend) !=
				 XenbusStateClosed &&
				 xenbus_read_driver_state(dev->otherend) !=
				 XenbusStateUnknown, 5 * HZ);
	} while (!err);

	return 0;
}

static void xenbus_close(struct xenbus_device *dev)
{
	int ret;

	if (xenbus_read_driver_state(dev->otherend) == XenbusStateClosed)
		return;
	do {
		xenbus_switch_state(dev, XenbusStateClosing);
		ret = wait_event_timeout(module_wq,
				   xenbus_read_driver_state(dev->otherend) ==
				   XenbusStateClosing ||
				   xenbus_read_driver_state(dev->otherend) ==
				   XenbusStateClosed ||
				   xenbus_read_driver_state(dev->otherend) ==
				   XenbusStateUnknown,
				   5 * HZ);
	} while (!ret);

	if (xenbus_read_driver_state(dev->otherend) == XenbusStateClosed)
		return;

	do {
		xenbus_switch_state(dev, XenbusStateClosed);
		ret = wait_event_timeout(module_wq,
				   xenbus_read_driver_state(dev->otherend) ==
				   XenbusStateClosed ||
				   xenbus_read_driver_state(dev->otherend) ==
				   XenbusStateUnknown,
				   5 * HZ);
	} while (!ret);
}

static void rswitch_vmq_front_disconnect_backend(struct rswitch_vmq_front_info *info)
{

}

static int rswitch_vmq_front_remove(struct xenbus_device *dev)
{
	/* struct rswitch_vmq_front_info *info = dev_get_drvdata(&dev->dev); */

	xenbus_close(dev);
	rswitch_vmq_front_disconnect_backend(NULL/* info */);

	/* if (info->netdev->reg_state == NETREG_REGISTERED) */
	/* 	unregister_netdev(info->netdev); */

	/* if (info->queues) { */
	/* 	rtnl_lock(); */
	/* 	xennet_destroy_queues(info); */
	/* 	rtnl_unlock(); */
	/* } */
	/* xennet_free_netdev(info->netdev); */

	return 0;
}

static void rswitch_vmq_front_changed(struct xenbus_device *dev,
			    enum xenbus_state backend_state)
{
	struct rswitch_device *priv = dev_get_drvdata(&dev->dev);
	/* struct rswitch_vmq_front_info *np = priv->front_info; */
	struct net_device *netdev = priv->ndev;

	dev_info(&dev->dev, "%s\n", xenbus_strstate(backend_state));

	wake_up_all(&module_wq);

	switch (backend_state) {
	case XenbusStateInitialising:
	case XenbusStateInitialised:
	case XenbusStateReconfiguring:
	case XenbusStateReconfigured:
	case XenbusStateUnknown:
		break;

	case XenbusStateInitWait:
		if (dev->state != XenbusStateInitialising)
			break;
		if (rswitch_vmq_front_connect(netdev) != 0)
			break;
		xenbus_switch_state(dev, XenbusStateConnected);
		break;

	case XenbusStateConnected:
		/* netdev_notify_peers(netdev); */
		break;

	case XenbusStateClosed:
		if (dev->state == XenbusStateClosed)
			break;
		fallthrough;	/* Missed the backend's CLOSING state */
	case XenbusStateClosing:
		xenbus_frontend_closed(dev);
		break;
	}
}

static const struct xenbus_device_id rswitch_vmq_front_ids[] = {
	{ "renesas_vmq" },
	{ "" }
};

static struct xenbus_driver rswitch_vmq_front_driver = {
	.ids = rswitch_vmq_front_ids,
	.probe = rswitch_vmq_front_probe,
	.remove = rswitch_vmq_front_remove,
	.otherend_changed = rswitch_vmq_front_changed,
};

static int __init rswitch_vmq_front_init(void)
{
	if (!xen_domain())
		return -ENODEV;

	return xenbus_register_frontend(&rswitch_vmq_front_driver);
}
module_init(rswitch_vmq_front_init);


static void __exit rswitch_vmq_front_exit(void)
{
	xenbus_unregister_driver(&rswitch_vmq_front_driver);
}
module_exit(rswitch_vmq_front_exit);

MODULE_DESCRIPTION("Renesas R-Switch PV driver front-end");
MODULE_LICENSE("GPL");
