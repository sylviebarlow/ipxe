/*
 * Copyright (C) 2018 Sylvie Barlow <sylvie.c.barlow@gmail.com>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 * You can also choose to distribute this program under the terms of
 * the Unmodified Binary Distribution Licence (as given in the file
 * COPYING.UBDL), provided that you have satisfied its requirements.
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <byteswap.h>
#include <ipxe/netdevice.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/iobuf.h>
#include <ipxe/malloc.h>
#include <ipxe/pci.h>
#include "icplus.h"

/** @file
 *
 * IC+ network driver
 *
 */

/******************************************************************************
 *
 * Device reset
 *
 ******************************************************************************
 */

/**
 * Reset hardware
 *
 * @v icp		IC+ device
 * @ret rc		Return status code
 */
static int icplus_reset ( struct icplus_nic *icp ) {
	uint32_t asicctrl;
	unsigned int i;

	/* Trigger reset */
	writel ( ICP_ASICCTRL_GLOBALRESET, ( icp->regs + ICP_ASICCTRL ) );

	/* Wait for reset to complete */
	for ( i = 0 ; i < ICP_RESET_MAX_WAIT_MS ; i++ ) {

		/* Check if device is ready */
		asicctrl = readl ( icp->regs + ICP_ASICCTRL );
		if ( ! ( asicctrl & ICP_ASICCTRL_RESETBUSY ) )
			return 0;

		/* Delay */
		mdelay ( 1 );
	}

	DBGC ( icp, "ICPLUS %p timed out waiting for reset (asicctrl %#08x)\n",
	       icp, asicctrl );
	return -ETIMEDOUT;
}

/******************************************************************************
 *
 * Link state
 *
 ******************************************************************************
 */

/**
 * Check link state
 *
 * @v netdev		Network device
 */
static void icplus_check_link ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;

	DBGC ( icp, "ICPLUS %p does not yet support link state\n", icp );
	netdev_link_err ( netdev, -ENOTSUP );
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int icplus_open ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;

	DBGC ( icp, "ICPLUS %p does support open\n", icp );
	return 0;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void icplus_close ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;

	DBGC ( icp, "ICPLUS %p does not yet support close\n", icp );
}

/**
 * Transmit packet
 *
 * @v netdev		Network device
 * @v iobuf		I/O buffer
 * @ret rc		Return status code
 */
static int icplus_transmit ( struct net_device *netdev,
			       struct io_buffer *iobuf ) {
	struct icplus_nic *icp = netdev->priv;

	DBGC ( icp, "ICPLUS %p does not yet support transmit\n", icp );
	( void ) iobuf;
	return -ENOTSUP;
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void icplus_poll ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;

	/* Not yet implemented */
	( void ) icp;
}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void icplus_irq ( struct net_device *netdev, int enable ) {
	struct icplus_nic *icp = netdev->priv;

	DBGC ( icp, "ICPLUS %p does not yet support interrupts\n", icp );
	( void ) enable;
}

/** IC+ network device operations */
static struct net_device_operations icplus_operations = {
	.open		= icplus_open,
	.close		= icplus_close,
	.transmit	= icplus_transmit,
	.poll		= icplus_poll,
	.irq		= icplus_irq,
};

/******************************************************************************
 *
 * PCI interface
 *
 ******************************************************************************
 */

/**
 * Probe PCI device
 *
 * @v pci		PCI device
 * @ret rc		Return status code
 */
static int icplus_probe ( struct pci_device *pci ) {
	struct net_device *netdev;
	struct icplus_nic *icp;
	int rc;

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *icp ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	netdev_init ( netdev, &icplus_operations );
	icp = netdev->priv;
	pci_set_drvdata ( pci, netdev );
	netdev->dev = &pci->dev;
	memset ( icp, 0, sizeof ( *icp ) );

	/* Fix up PCI device */
	adjust_pci_device ( pci );

	/* Map registers */
	icp->regs = ioremap ( pci->membase, ICP_BAR_SIZE );
	if ( ! icp->regs ) {
		rc = -ENODEV;
		goto err_ioremap;
	}

	/* Reset the NIC */
	if ( ( rc = icplus_reset ( icp ) ) != 0 )
		goto err_reset;

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	icplus_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
	icplus_reset ( icp );
 err_reset:
	iounmap ( icp->regs );
 err_ioremap:
	netdev_nullify ( netdev );
	netdev_put ( netdev );
 err_alloc:
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void icplus_remove ( struct pci_device *pci ) {
	struct net_device *netdev = pci_get_drvdata ( pci );
	struct icplus_nic *icp = netdev->priv;

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Reset card */
	icplus_reset ( icp );

	/* Free network device */
	iounmap ( icp->regs );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** IC+ PCI device IDs */
static struct pci_device_id icplus_nics[] = {
	PCI_ROM ( 0x13f0, 0x1023, "ip1000a",	"IP1000A", 0 ),
};

/** IC+ PCI driver */
struct pci_driver icplus_driver __pci_driver = {
	.ids = icplus_nics,
	.id_count = ( sizeof ( icplus_nics ) / sizeof ( icplus_nics[0] ) ),
	.probe = icplus_probe,
	.remove = icplus_remove,
};
