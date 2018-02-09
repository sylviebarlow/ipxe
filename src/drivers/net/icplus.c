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
	uint8_t phyctrl;

	/* Read link status */
	phyctrl = readb ( icp->regs + ICP_PHYCTRL );
	DBGC ( icp, "ICP %p PHY control is %02x\n", icp, phyctrl );

	/* Update network device */
	if ( phyctrl & ICP_PHYCTRL_LINKSPEED ) {
		netdev_link_up ( netdev );
	} else {
		netdev_link_down ( netdev );
	}
}

/******************************************************************************
 *
 * Admin queue
 *
 ******************************************************************************
 */

/**
 * Set descriptor ring base address
 *
 * @v icp		IC+ device
 * @v offset		Register offset
 * @v address		Base address
 */
static inline void icplus_set_base ( struct icplus_nic *icp, unsigned int offset,
				     void *base ) {
	physaddr_t phys = virt_to_bus ( base );

	/* Program base address registers */
	writel ( ( phys & 0xffffffffUL ),
		 ( icp->regs + offset + ICP_BASE_LO ) );
	if ( sizeof ( phys ) > sizeof ( uint32_t ) ) {
		writel ( ( ( ( uint64_t ) phys ) >> 32 ),
			 ( icp->regs + offset + ICP_BASE_HI ) );
	} else {
		writel ( 0, ( icp->regs + offset + ICP_BASE_HI ) );
	}
}

/**
 * Reset descriptor ring base address
 *
 * @v icp		IC+ device
 * @v offset		Register offset
 */
static inline void icplus_clear_base ( struct icplus_nic *icp,
				       unsigned int offset ) {

	/* Clear base address registers */
	writel ( 0, ( icp->regs + offset + ICP_BASE_HI ) );
	writel ( 0, ( icp->regs + offset + ICP_BASE_LO ) );
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Create transmit descriptor ring
 *
 * @v icp		IC+ device
 * @ret rc		Return status code
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

	/* Program descriptor base address */
	icplus_set_base ( icp, ICP_TFDLISTPTR, icp->tx.entry );

	/* Reset transmit producer & consumer counters */
	icp->tx.prod = 0;
	icp->tx.cons = 0;

	return 0;

	icplus_clear_base ( icp, ICP_TFDLISTPTR );
	free_dma ( icp->tx.entry, len );
	icp->tx.entry = NULL;
 err_alloc:
	return rc;
}

/**
 * Destroy transmit descriptor ring
 *
 * @v icp		IC+ device
 */
static void icplus_destroy_tx ( struct icplus_nic *icp ) {
	size_t len = ( sizeof ( icp->tx.entry[0] ) * ICP_TX_NUM );

	/* Free descriptor ring */
	icplus_clear_base ( icp, ICP_TFDLISTPTR );
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
	int rc;

	/* Create transmit descriptor ring */
	if ( ( rc = icplus_create_tx ( icp ) ) != 0 )
		goto err_create_tx;

	/* Enable transmitter and receiver */
	writel ( ( ICP_MACCTRL_TXENABLE | ICP_MACCTRL_RXENABLE |
		   ICP_MACCTRL_DUPLEX ), icp->regs + ICP_MACCTRL );

	/* Check link state */
	icplus_check_link ( netdev );
	
	return 0;

	writel ( ( ICP_MACCTRL_TXDISABLE | ICP_MACCTRL_RXDISABLE ),
		 icp->regs + ICP_MACCTRL );
	icplus_destroy_tx ( icp );
 err_create_tx:
	return rc;
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void icplus_close ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;

	/* Disable transmitter and receiver */
	writel ( ( ICP_MACCTRL_TXDISABLE | ICP_MACCTRL_RXDISABLE ),
		 icp->regs + ICP_MACCTRL );

	/* Destroy transmit descriptor ring */
	icplus_destroy_tx ( icp );
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
	struct icplus_tx_descriptor *txd;
	unsigned int tx_idx;
	physaddr_t address;

	/* Check if ring is full */
	if ( ( icp->tx.prod - icp->tx.cons ) >= ICP_TX_NUM ) {
		DBGC ( icp, "ICP %p out of transmit descriptors\n", icp );
		return -ENOBUFS;
	}

	/* Find TX descriptor entry to use */
	tx_idx = ( icp->tx.prod++ % ICP_TX_NUM );
	txd = &icp->tx.entry[tx_idx];

	/* Fill in TX descriptor */
	address = virt_to_bus ( iobuf->data );
	txd->data.address = cpu_to_le64 ( address );
	txd->data.len = cpu_to_le16 ( iob_len ( iobuf ) );
	wmb();
	txd->control = ICP_TX_SOLE_FRAG;
	wmb();

	/* Ring doorbell */
	writew ( ICP_DMACTRL_TXPOLLNOW, icp->regs + ICP_DMACTRL );

	DBGC2 ( icp, "ICP %p TX %d is [%llx,%llx)\n", icp, tx_idx,
		( ( unsigned long long ) address ),
		( ( unsigned long long ) address + iob_len ( iobuf ) ) );
	DBGC2_HDA ( icp, virt_to_phys ( txd ), txd, sizeof ( *txd ) );
	return 0;
}

/**
 * Poll for completed packets
 *
 * @v netdev		Network device
 */
static void icplus_poll_tx ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;
	struct icplus_tx_descriptor *tx;
	unsigned int tx_idx;

	/* Check for completed packets */
	while ( icp->tx.cons != icp->tx.prod ) {

		/* Get next transmit descriptor */
		tx_idx = ( icp->tx.cons % ICP_TX_NUM );
		tx = &icp->tx.entry[tx_idx];

		/* Stop if descriptor is still in use */
		if ( ! ( tx->control & ICP_TX_DONE ) )
			return;

		/* Complete TX descriptor */
		DBGC2 ( icp, "ICP %p TX %d complete\n", icp, tx_idx );
		netdev_tx_complete_next ( netdev );
		icp->tx.cons++;
	}
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void icplus_poll ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;
	uint16_t intstatus;
	uint32_t txstatus;

	/* Check for interrupts */
	intstatus = readw ( icp->regs + ICP_INTSTATUS );

	/* Check if transmit complete bit set */
	if ( intstatus & ICP_INTSTATUS_TXCOMPLETE ) {
		txstatus = readl ( icp->regs + ICP_TXSTATUS );
		if ( txstatus & ICP_TXSTATUS_ERROR )
			DBGC ( icp, "ICP %p TX error: %08x\n", icp, txstatus );
		icplus_poll_tx ( netdev );
	}

	/* Check link state, if applicable */
	if ( intstatus & ICP_INTSTATUS_LINKEVENT ) {
		writew ( ICP_INTSTATUS_LINKEVENT, icp->regs + ICP_INTSTATUS );
		icplus_check_link ( netdev );
	}
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
