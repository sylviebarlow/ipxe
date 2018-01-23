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
 * EEPROM interface
 *
 ******************************************************************************
 */

/**
 * Read data from EEPROM
 *
 * @v nvs		NVS device
 * @v address		Address from which to read
 * @v data		Data buffer
 * @v len		Length of data buffer
 * @ret rc		Return status code
 */
static int icplus_read_eeprom ( struct nvs_device *nvs, unsigned int address,
				void *data, size_t len ) {
	struct icplus_nic *icp =
		container_of ( nvs, struct icplus_nic, eeprom );
	unsigned int i;
	uint16_t eepromctrl;
	uint16_t *data_word = data;

	/* Sanity check.  We advertise a blocksize of one word, so
	 * should only ever receive single-word requests.
	 */
	assert ( len == sizeof ( *data_word ) );

	/* Initiate read */
	writew ( ( ICP_EEPROMCTRL_OPCODE_READ |
		   ICP_EEPROMCTRL_ADDRESS ( address ) ),
		 ( icp->regs + ICP_EEPROMCTRL ) );

	/* Wait for read to complete */
	for ( i = 0 ; i < ICP_EEPROM_MAX_WAIT_MS ; i++ ) {

		/* If read is not complete, delay 1ms and retry */
		eepromctrl = readw ( icp->regs + ICP_EEPROMCTRL );
		if ( eepromctrl & ICP_EEPROMCTRL_BUSY ) {
			mdelay ( 1 );
			continue;
		}

		/* Extract data */
		*data_word = cpu_to_le16 ( readw ( icp->regs + ICP_EEPROMDATA ));
		return 0;
	}

	DBGC ( icp, "ICPLUS %p timed out waiting for EEPROM read\n", icp );
	return -ETIMEDOUT;
}

/**
 * Write data to EEPROM
 *
 * @v nvs		NVS device
 * @v address		Address to which to write
 * @v data		Data buffer
 * @v len		Length of data buffer
 * @ret rc		Return status code
 */
static int icplus_write_eeprom ( struct nvs_device *nvs,
				 unsigned int address __unused,
				 const void *data __unused,
				 size_t len __unused ) {
	struct icplus_nic *icp =
		container_of ( nvs, struct icplus_nic, eeprom );

	DBGC ( icp, "ICPLUS %p EEPROM write not supported\n", icp );
	return -ENOTSUP;
}

/**
 * Initialise EEPROM
 *
 * @v icp		IC+ device
 */
static void icplus_init_eeprom ( struct icplus_nic *icp ) {

	/* The hardware supports only single-word reads */
	icp->eeprom.word_len_log2 = ICP_EEPROM_WORD_LEN_LOG2;
	icp->eeprom.size = ICP_EEPROM_MIN_SIZE_WORDS;
	icp->eeprom.block_size = 1;
	icp->eeprom.read = icplus_read_eeprom;
	icp->eeprom.write = icplus_write_eeprom;
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
static int icplus_create_tx ( struct icplus_nic *icp ) {
	size_t len = ( sizeof ( icp->tx.entry[0] ) * ICP_TX_NUM );
	int rc;
	unsigned int i;
	struct icplus_tx_descriptor *txd;
	struct icplus_tx_descriptor *next;

	/* Allocate descriptor ring */
	icp->tx.entry = malloc_dma ( len, ICP_ALIGN );
	if ( ! icp->tx.entry ) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	/* Initialise descriptor ring */
	memset ( icp->tx.entry, 0, len );
	for ( i = 0 ; i < ICP_TX_NUM ; i++ ) {
		txd = &icp->tx.entry[i];
		next = &icp->tx.entry[ ( i + 1 ) % ICP_TX_NUM ];
		txd->next = cpu_to_le64 ( virt_to_bus ( next ) );
		txd->flags = ( ICP_TX_UNALIGN | ICP_TX_INDICATE );
		txd->control = ( ICP_TX_SOLE_FRAG | ICP_TX_DONE );
	}

	return 0;

	free_dma ( icp->tx.entry, len );
	icp->tx.entry = NULL;
 err_alloc:
	return rc;
}

static void icplus_destroy_tx ( struct icplus_nic *icp ) {
	size_t len = ( sizeof ( icp->tx.entry[0] ) * ICP_TX_NUM );

	/* Free descriptor ring */
	free_dma ( icp->tx.entry, len );
	icp->tx.entry = NULL;
}

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

	/* Initialise EEPROM */
	icplus_init_eeprom ( icp );

	/* Read EEPROM MAC address */
	if ( ( rc = nvs_read ( &icp->eeprom, ICP_EEPROM_MAC,
			       netdev->hw_addr, ETH_ALEN ) ) != 0 ) {
		DBGC ( icp, "ICPLUS %p could not read EEPROM MAC address: %s\n",
		       icp, strerror ( rc ) );
		goto err_eeprom;
	}

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	icplus_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
 err_eeprom:
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
