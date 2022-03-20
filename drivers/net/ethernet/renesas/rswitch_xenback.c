// SPDX-License-Identifier: GPL-2.0
/* Renesas Ethernet Switch Para-Virtualized driver
 *
 * Copyright (C) 2022 EPAM Systems
 */

#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <xen/interface/grant_table.h>
#include <xen/grant_table.h>
#include <xen/xenbus.h>
#include <xen/page.h>
#include "rswitch.h"


struct backend_info {
	struct xenbus_device *dev;
	struct rswitch_device *rdev;

	/* This is the state that will be reflected in xenstore when any
	 * active hotplug script completes.
	 */
	enum xenbus_state state;
	enum xenbus_state frontend_state;
};

int rswitch_xen_ndev_register(struct rswitch_private *priv, int index)
{
	struct platform_device *pdev = priv->pdev;
	struct net_device *ndev;
	struct rswitch_device *rdev;
	int err;

	ndev = alloc_etherdev_mqs(sizeof(struct rswitch_device), 1, 1);
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, &pdev->dev);
	ether_setup(ndev);

	rdev = netdev_priv(ndev);
	rdev->ndev = ndev;
	rdev->priv = priv;
	/* TODO: Fix index calculation */
	priv->rdev[index + 3] = rdev;
	rdev->port = 4; 	/* TODO: This is supposed to be GWCA0 port */
	rdev->etha = NULL;
	rdev->remote_chain = -1;

	rdev->addr = priv->addr;

	spin_lock_init(&rdev->lock);

	ndev->features = NETIF_F_RXCSUM;
	ndev->hw_features = NETIF_F_RXCSUM;
	ndev->base_addr = (unsigned long)rdev->addr;
	snprintf(ndev->name, IFNAMSIZ, "vmq%d", index);
	ndev->netdev_ops = &rswitch_netdev_ops;

	netif_napi_add(ndev, &rdev->napi, rswitch_poll, 64);

	eth_hw_addr_random(ndev);

	/* Network device register */
	err = register_netdev(ndev);
	if (err)
		goto out_reg_netdev;

	err = rswitch_rxdmac_init(ndev, priv);
	if (err < 0)
		goto out_rxdmac;

	err = rswitch_txdmac_init(ndev, priv);
	if (err < 0)
		goto out_txdmac;

	/* Print device information */
	netdev_info(ndev, "MAC address %pMn", ndev->dev_addr);

	return 0;

out_txdmac:
	rswitch_rxdmac_free(ndev, priv);

out_rxdmac:
	unregister_netdev(ndev);

out_reg_netdev:
	netif_napi_del(&rdev->napi);
	free_netdev(ndev);

	return err;
}

int rswitch_xen_connect_devs(struct rswitch_device *rdev1,
			     struct rswitch_device *rdev2)
{
	rdev1->remote_chain = rdev2->rx_chain->index;
	rdev2->remote_chain = rdev1->rx_chain->index;

	return 0;
}


static int rswitch_vmq_back_remove(struct xenbus_device *dev)
{
	struct backend_info *be = dev_get_drvdata(&dev->dev);

	if (be->rdev) {
//		backend_disconnect(be);
//		xenvif_free(be->vif);
		be->rdev = NULL;
	}

	kfree(be);
	dev_set_drvdata(&dev->dev, NULL);
	return 0;
}

/**
 * Entry point to this code when a new device is created.  Allocate the basic
 * structures and switch to InitWait.
 */
static int rswitch_vmq_back_probe(struct xenbus_device *dev,
				  const struct xenbus_device_id *id)
{
	int err = 0;

	struct backend_info *be = kzalloc(sizeof(*be), GFP_KERNEL);

	if (!be) {
		xenbus_dev_fatal(dev, -ENOMEM,
				 "allocating backend structure");
		return -ENOMEM;
	}
	pr_info("%s probed\n", dev->otherend);
	be->dev = dev;
	dev_set_drvdata(&dev->dev, be);

	xenbus_switch_state(dev, XenbusStateInitWait);

	return err;
}

static void set_backend_state(struct xenbus_device *dev,
			      enum xenbus_state state)
{
	while (dev->state != state) {
		pr_info("state: %d %d\n", dev->state, state);
		switch (dev->state) {
		case XenbusStateClosed:
			switch (state) {
			case XenbusStateInitWait:
			case XenbusStateConnected:
				xenbus_switch_state(dev, XenbusStateInitWait);
				break;
			case XenbusStateClosing:
				xenbus_switch_state(dev, XenbusStateClosing);
				break;
			default:
				WARN_ON(1);
			}
			break;
		case XenbusStateInitWait:
		case XenbusStateInitialised:
			switch (state) {
			case XenbusStateConnected:
				/* if (backend_connect(dev)) */
				/* 	return; */
				xenbus_switch_state(dev, XenbusStateConnected);
				break;
			case XenbusStateClosing:
			case XenbusStateClosed:
				xenbus_switch_state(dev, XenbusStateClosing);
				break;
			default:
				WARN_ON(1);
			}
			break;
		case XenbusStateConnected:
			switch (state) {
			case XenbusStateInitWait:
			case XenbusStateClosing:
			case XenbusStateClosed:
				/* down(&pvcalls_back_global.frontends_lock); */
				/* backend_disconnect(dev); */
				/* up(&pvcalls_back_global.frontends_lock); */
				xenbus_switch_state(dev, XenbusStateClosing);
				break;
			default:
				WARN_ON(1);
			}
			break;
		case XenbusStateClosing:
			switch (state) {
			case XenbusStateInitWait:
			case XenbusStateConnected:
			case XenbusStateClosed:
				xenbus_switch_state(dev, XenbusStateClosed);
				break;
			default:
				WARN_ON(1);
			}
			break;
		default:
			WARN_ON(1);
		}
	}
}

/**
 * Callback received when the frontend's state changes.
 */
static void rswitch_vmq_frontend_changed(struct xenbus_device *dev,
					 enum xenbus_state frontend_state)
{
	struct backend_info *be = dev_get_drvdata(&dev->dev);

	pr_info("%s -> %s\n", dev->otherend, xenbus_strstate(frontend_state));
	pr_info("be = %px\n", be);
	be->frontend_state = frontend_state;

	switch (frontend_state) {
	case XenbusStateInitialising:
		set_backend_state(dev, XenbusStateInitWait);
		break;

	case XenbusStateInitialised:
	case XenbusStateConnected:
		set_backend_state(dev, XenbusStateConnected);
		break;

	case XenbusStateReconfiguring:
//		read_xenbus_frontend_xdp(be, dev);
		xenbus_switch_state(dev, XenbusStateReconfigured);
		break;

	case XenbusStateClosing:
		set_backend_state(dev, XenbusStateClosing);
		break;

	case XenbusStateClosed:
		set_backend_state(dev, XenbusStateClosed);
		if (xenbus_dev_is_online(dev))
			break;
		fallthrough;	/* if not online */
	case XenbusStateUnknown:
		set_backend_state(dev, XenbusStateClosed);
		device_unregister(&dev->dev);
		break;

	default:
		xenbus_dev_fatal(dev, -EINVAL, "saw state %d at frontend",
				 frontend_state);
		break;
	}
}

static const struct xenbus_device_id rswitch_vmq_ids[] = {
	{ "renesas_vmq" },
	{ "" }
};

static struct xenbus_driver rswitch_vmq_driver = {
	.ids = rswitch_vmq_ids,
	.probe = rswitch_vmq_back_probe,
	.remove = rswitch_vmq_back_remove,
	.otherend_changed = rswitch_vmq_frontend_changed,
	.allow_rebind = false,
};

static int __init rswitch_vmq_back_init(void)
{
	if (!xen_domain())
		return -ENODEV;

	return xenbus_register_backend(&rswitch_vmq_driver);
}

module_init(rswitch_vmq_back_init);

static void rswitch_vmq_back_exit(void)
{
	return xenbus_unregister_driver(&rswitch_vmq_driver);
}

module_exit(rswitch_vmq_back_exit);
