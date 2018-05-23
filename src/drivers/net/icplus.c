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
	writel ( ( ICP_ASICCTRL_GLOBALRESET | ICP_ASICCTRL_DMA |
		   ICP_ASICCTRL_FIFO | ICP_ASICCTRL_NETWORK |
		   ICP_ASICCTRL_HOST | ICP_ASICCTRL_AUTOINIT ),
		 ( icp->regs + ICP_ASICCTRL ) );

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
		 ( icp->regs + icp->eepromctrl ) );

	/* Wait for read to complete */
	for ( i = 0 ; i < ICP_EEPROM_MAX_WAIT_MS ; i++ ) {

		/* If read is not complete, delay 1ms and retry */
		eepromctrl = readw ( icp->regs + icp->eepromctrl );
		if ( eepromctrl & ICP_EEPROMCTRL_BUSY ) {
			mdelay ( 1 );
			continue;
		}

		/* Extract data */
		*data_word = cpu_to_le16 ( readw ( icp->regs +
						   icp->eepromdata ) );
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
 * MII interface
 *
 ******************************************************************************
 */

/** Pin mapping for MII bit-bashing interface */
static const uint8_t icplus_mii_bits[] = {
	[MII_BIT_MDC]	= ICP_PHYCTRL_MGMTCLK,
	[MII_BIT_MDIO]	= ICP_PHYCTRL_MGMTDATA,
	[MII_BIT_DRIVE]	= ICP_PHYCTRL_MGMTDIR,
};

/**
 * Read input bit
 *
 * @v basher		Bit-bashing interface
 * @v bit_id		Bit number
 * @ret zero		Input is a logic 0
 * @ret non-zero	Input is a logic 1
 */
static int icplus_mii_read_bit ( struct bit_basher *basher,
				 unsigned int bit_id ) {
	struct icplus_nic *icp = container_of ( basher, struct icplus_nic,
						miibit.basher );
	uint8_t mask = icplus_mii_bits[bit_id];
	uint8_t reg;

	DBG_DISABLE ( DBGLVL_IO );
	reg = readb ( icp->regs + icp->phyctrl );
	DBG_ENABLE ( DBGLVL_IO );
	return ( reg & mask );
}

/**
 * Set/clear output bit
 *
 * @v basher		Bit-bashing interface
 * @v bit_id		Bit number
 * @v data		Value to write
 */
static void icplus_mii_write_bit ( struct bit_basher *basher,
				   unsigned int bit_id, unsigned long data ) {
	struct icplus_nic *icp = container_of ( basher, struct icplus_nic,
						miibit.basher );
	uint8_t mask = icplus_mii_bits[bit_id];
	uint8_t reg;

	DBG_DISABLE ( DBGLVL_IO );
	reg = readb ( icp->regs + icp->phyctrl );
	reg &= ~mask;
	reg |= ( data & mask );
	writeb ( reg, icp->regs + icp->phyctrl );
	readb ( icp->regs + icp->phyctrl ); /* Ensure write reaches chip */
	DBG_ENABLE ( DBGLVL_IO );
}

/** MII bit-bashing interface */
static struct bit_basher_operations icplus_basher_ops = {
	.read = icplus_mii_read_bit,
	.write = icplus_mii_write_bit,
};

/******************************************************************************
 *
 * Link state
 *
 ******************************************************************************
 */

/**
 * Configure PHY
 *
 * @v icp		IC+ device
 * @ret rc		Return status code
 */
static int icplus_init_phy ( struct icplus_nic *icp ) {
	uint32_t asicctrl;
	int rc;

	/* Find PHY address */
	if ( ( rc = mii_find ( &icp->mii ) ) != 0 ) {
		DBGC ( icp, "ICPLUS %p could not find PHY address: %s\n",
		       icp, strerror ( rc ) );
		return rc;
	}

	/* Configure PHY to advertise 1000Mbps if applicable, this will have no
	 effect if using cards with a maximum of 100Mbps */
	asicctrl = readl ( icp->regs + ICP_ASICCTRL );
	if ( asicctrl & ICP_ASICCTRL_PHYSPEED1000 ) {
		if ( ( rc = mii_write ( &icp->mii, MII_CTRL1000,
					ADVERTISE_1000FULL ) ) != 0 ) {
			DBGC ( icp, "ICPLUS %p could not advertise 1000Mbps: "
			       "%s\n", icp, strerror ( rc ) );
			return rc;
		}
	}

	/* Reset PHY */
	if ( ( rc = mii_reset ( &icp->mii ) ) != 0 ) {
		DBGC ( icp, "ICPLUS %p could not reset PHY: %s\n",
		       icp, strerror ( rc ) );
		return rc;
	}

	return 0;
}

/**
 * Check link state
 *
 * @v netdev		Network device
 */
static void icplus_check_link ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;
	uint8_t phyctrl;

	/* Read link status */
	phyctrl = readb ( icp->regs + icp->phyctrl );
	DBGC ( icp, "ICPLUS %p PHY control is %02x\n", icp, phyctrl );

	/* Update network device */
	if ( phyctrl & ICP_PHYCTRL_LINKSPEED ) {
		netdev_link_up ( netdev );
	} else {
		netdev_link_down ( netdev );
	}
}

/******************************************************************************
 *
 * Card management (64 bit)
 *
 ******************************************************************************
 */

/**
 * Initialise descriptor (64 bit)
 *
 * @v desc		Descriptor
 */
static void icplus_setup64 ( union icplus_descriptor *desc ) {
	desc->d64.flags = ( ICP_TX_UNALIGN | ICP64_TX_INDICATE );
	desc->d64.control = ( ICP64_TX_SOLE_FRAG | ICP64_DONE );
}

/**
 * Describe receive data buffer (64 bit)
 *
 * @v desc		Descriptor
 * @v address		Address
 * @v len		Length
 */
static void icplus_describe_rx64 ( union icplus_descriptor *desc,
				   physaddr_t address, size_t len __unused ) {
	desc->d64.data.address = cpu_to_le64 ( address );
	desc->d64.data.len = cpu_to_le16 ( ICP_RX_MAX_LEN );
	wmb();
	desc->d64.control = 0;
}

/**
 * Describe transmit data buffer (64 bit)
 *
 * @v desc		Descriptor
 * @v address		Address
 * @v len		Length
 */
static void icplus_describe_tx64 ( union icplus_descriptor *desc,
				   physaddr_t address, size_t len ) {
	desc->d64.data.address = cpu_to_le64 ( address );
	desc->d64.data.len = cpu_to_le16 ( len );
	wmb();
	desc->d64.control = ICP64_TX_SOLE_FRAG;
}

/**
 * Check descriptor buffer completion (64 bit)
 *
 * @v desc		Descriptor
 * @ret is_completed	Descriptor is complete
 */
static int icplus_completed64 ( union icplus_descriptor *desc ) {
	return ( desc->d64.control & ICP64_DONE );
}

/**
 * Check descriptor buffer completion
 *
 * @v desc		Descriptor
 * @ret len		Length of packet
 */
static int icplus_len64 ( union icplus_descriptor *desc ) {
	if ( desc->d64.flags & ( ICP_RX_ERR_OVERRUN | ICP_RX_ERR_RUNT |
				 ICP_RX_ERR_ALIGN | ICP_RX_ERR_FCS |
				 ICP_RX_ERR_OVERSIZED | ICP_RX_ERR_LEN ) ) {
		return -desc->d64.flags;
	}
	return le16_to_cpu ( desc->d64.len );
}

/**
 * Assign function pointers (64 bit)
 *
 * @v icp		IC+ device
 */
static void icplus_init64 ( struct icplus_nic *icp ) {
	icp->rx.describe = icplus_describe_rx64;
	icp->tx.describe = icplus_describe_tx64;
	icp->rx.setup = icplus_setup64;
	icp->tx.setup = icplus_setup64;
	icp->rx.completed = icplus_completed64;
	icp->tx.completed = icplus_completed64;
	icp->rx.len = icplus_len64;
	icp->eepromctrl = ICP64_EEPROMCTRL;
	icp->eepromdata = ICP64_EEPROMDATA;
	icp->phyctrl = ICP64_PHYCTRL;
	icp->macctrl0 = ICP64_MACCTRL0;
	icp->macctrl1 = ICP64_MACCTRL1;
	icp->txstatus = ICP64_TXSTATUS;
	/* Temporary and needs changing */
	icp->tx.listptr = ICP_TFDLISTPTR;
	icp->rx.listptr = ICP_RFDLISTPTR;
}

/******************************************************************************
 *
 * Card management (32 bit)
 *
 ******************************************************************************
 */

/**
 * Initialise descriptor (32 bit)
 *
 * @v desc		Descriptor
 */
static void icplus_setup32 ( union icplus_descriptor *desc ) {
	desc->d32.frames = cpu_to_le16 ( ICP_TX_UNALIGN | ICP32_TX_INDICATE );
	desc->d32.flags = ICP32_TX_DONE;
	desc->d32.last = cpu_to_le16 ( ICP32_SOLE_FRAG );
}

/**
 * Describe receive data buffer (32 bit)
 *
 * @v desc		Descriptor
 * @v address		Address
 * @v len		Length
 */
static void icplus_describe_rx32 ( union icplus_descriptor *desc,
				   physaddr_t address, size_t len __unused ) {
	desc->d32.address = cpu_to_le32 ( address );
	desc->d32.len = cpu_to_le16 ( ICP_RX_MAX_LEN );
	wmb();
	desc->d32.frames = 0;
}

/**
 * Describe transmit data buffer (32 bit)
 *
 * @v desc		Descriptor
 * @v address		Address
 * @v len		Length
 */
static void icplus_describe_tx32 ( union icplus_descriptor *desc,
				   physaddr_t address, size_t len ) {
	desc->d32.address = cpu_to_le32 ( address );
	desc->d32.len = cpu_to_le16 ( len );
	wmb();
	desc->d32.flags = 0;
}

/**
 * Check receive descriptor buffer completion (32 bit)
 *
 * @v desc		Descriptor
 * @ret is_completed	Descriptor is complete
 */
static int icplus_completed32_rx ( union icplus_descriptor *desc ) {
	return ( desc->d32.frames & cpu_to_le16 ( ICP32_RX_DONE ) );
}

/**
 * Check transmit descriptor buffer completion (32 bit)
 *
 * @v desc		Descriptor
 * @ret is_completed	Descriptor is complete
 */
static int icplus_completed32_tx (union icplus_descriptor *desc ) {
	return ( desc->d32.flags & ICP32_TX_DONE );
}

/**
 * Check descriptor buffer completion
 *
 * @v desc		Descriptor
 * @ret len		Length of packet
 */
static int icplus_len32 ( union icplus_descriptor *desc ) {
	if ( desc->d32.frames & cpu_to_le16 ( ICP_RXFRAMEERROR ) ) {
		return -desc->d32.frames;
	}
	return le16_to_cpu ( desc->d32.frames ) & ICP_RXFRAMEERROR;
}

/**
 * Assign function pointers (32 bit)
 *
 * @v icp		IC+ device
 */
static void icplus_init32 ( struct icplus_nic *icp ) {
	icp->rx.describe = icplus_describe_rx32;
	icp->tx.describe = icplus_describe_tx32;
	icp->rx.setup = icplus_setup32;
	icp->tx.setup = icplus_setup32;
	icp->rx.completed = icplus_completed32_rx;
	icp->tx.completed = icplus_completed32_tx;
	icp->rx.len = icplus_len32;
	icp->eepromctrl = ICP32_EEPROMCTRL;
	icp->eepromdata = ICP32_EEPROMDATA;
	icp->phyctrl = ICP32_PHYCTRL;
	icp->macctrl0 = ICP32_MACCTRL0;
	icp->macctrl1 = ICP32_MACCTRL1;
	icp->txstatus = ICP32_TXSTATUS;
	/* Temporary and needs changing */
	icp->tx.listptr = ICP_TFDLISTPTR;
	icp->rx.listptr = ICP_RFDLISTPTR;
}

/******************************************************************************
 *
 * Network device interface
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
static inline void icplus_set_base ( struct icplus_nic *icp,
				     unsigned int offset, void *base ) {
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
 * Create descriptor ring
 *
 * @v icp		IC+ device
 * @v ring		Descriptor ring
 * @ret rc		Return status code
 */
static int icplus_create_ring ( struct icplus_nic *icp,
				struct icplus_ring *ring ) {
	size_t len = ( sizeof ( ring->entry[0] ) * ICP_NUM_DESC );
	int rc;
	unsigned int i;
	union icplus_descriptor *desc;
	union icplus_descriptor *next;

	/* Allocate descriptor ring */
	ring->entry = malloc_dma ( len, ICP_ALIGN );
	if ( ! ring->entry ) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	/* Initialise descriptor ring */
	memset ( ring->entry, 0, len );
	for ( i = 0 ; i < ICP_NUM_DESC ; i++ ) {
		desc = &ring->entry[i];
		next = &ring->entry[ ( i + 1 ) % ICP_NUM_DESC ];
		desc->next = cpu_to_le64 ( virt_to_bus ( next ) );
		ring->setup ( desc );
	}

	/* Reset transmit producer & consumer counters */
	ring->prod = 0;
	ring->cons = 0;

	DBGC ( icp, "ICP %p %s ring at [%#08lx,%#08lx)\n",
	       icp, ( ( ring == &icp->tx ) ? "TX" : "RX" ),
	       virt_to_bus ( ring->entry ),
	       ( virt_to_bus ( ring->entry ) + len ) );
	return 0;

	free_dma ( ring->entry, len );
	ring->entry = NULL;
 err_alloc:
	return rc;
}

/**
 * Destroy descriptor ring
 *
 * @v icp		IC+ device
 * @v ring		Descriptor ring
 */
static void icplus_destroy_ring ( struct icplus_nic *icp __unused,
				  struct icplus_ring *ring ) {
	size_t len = ( sizeof ( ring->entry[0] ) * ICP_NUM_DESC );

	/* Free descriptor ring */
	free_dma ( ring->entry, len );
	ring->entry = NULL;
}

/**
 * Refill receive descriptor ring
 *
 * @v icp		IC+ device
 */
void icplus_refill_rx ( struct icplus_nic *icp ) {
	union icplus_descriptor *desc;
	struct io_buffer *iobuf;
	unsigned int rx_idx;
	physaddr_t address;
	unsigned int refilled = 0;

	/* Refill ring */
	while ( ( icp->rx.prod - icp->rx.cons ) < ICP_NUM_DESC ) {

		/* Allocate I/O buffer */
		iobuf = alloc_iob ( ICP_RX_MAX_LEN );
		if ( ! iobuf ) {
			/* Wait for next refill */
			break;
		}

		/* Get next receive descriptor */
		rx_idx = ( icp->rx.prod++ % ICP_NUM_DESC );
		desc = &icp->rx.entry[rx_idx];

		/* Populate receive descriptor */
		address = virt_to_bus ( iobuf->data );
		icp->rx.describe ( desc, address, 0 );
		wmb();

		/* Record I/O buffer */
		assert ( icp->rx_iobuf[rx_idx] == NULL );
		icp->rx_iobuf[rx_idx] = iobuf;

		DBGC2 ( icp, "ICP %p RX %d is [%llx,%llx)\n", icp, rx_idx,
			( ( unsigned long long ) address ),
			( ( unsigned long long ) address + ICP_RX_MAX_LEN ) );
		refilled++;
	}

	/* Push descriptors to card, if applicable */
	if ( refilled ) {
		wmb();
		writew ( ICP_DMACTRL_RXPOLLNOW, icp->regs + ICP_DMACTRL );
	}
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
	if ( ( rc = icplus_create_ring ( icp, &icp->tx ) ) != 0 )
		goto err_create_tx;

	/* Create receive descriptor ring */
	if ( ( rc = icplus_create_ring ( icp, &icp->rx ) ) != 0 )
		goto err_create_rx;

	/* Program descriptor base address */
	icplus_set_base ( icp, icp->tx.listptr, icp->tx.entry );
	icplus_set_base ( icp, icp->rx.listptr, icp->rx.entry );

	/* Enable receive mode */
	writeb ( ( ICP_RXMODE_UNICAST | ICP_RXMODE_MULTICAST |
		   ICP_RXMODE_BROADCAST | ICP_RXMODE_ALLFRAMES ),
		 icp->regs + ICP_RXMODE );

	/* Enable transmitter and receiver */
	writew ( ICP_MACCTRL0_DUPLEX, icp->regs + icp->macctrl0 );
	writew ( ( ICP_MACCTRL1_TXENABLE | ICP_MACCTRL1_RXENABLE ),
		 icp->regs + icp->macctrl1 );

	/* Fill receive ring */
	icplus_refill_rx ( icp );

	/* Check link state */
	icplus_check_link ( netdev );

	return 0;

	icplus_reset ( icp );
	icplus_destroy_ring ( icp, &icp->rx );
 err_create_rx:
	icplus_destroy_ring ( icp, &icp->tx );
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
	unsigned int i;

	/* Perform global reset */
	icplus_reset ( icp );

	/* Destroy receive descriptor ring */
	icplus_destroy_ring ( icp, &icp->rx );

	/* Destroy transmit descriptor ring */
	icplus_destroy_ring ( icp, &icp->tx );

	/* Discard any unused receive buffers */
	for ( i = 0 ; i < ICP_NUM_DESC ; i++ ) {
		if ( icp->rx_iobuf[i] )
			free_iob ( icp->rx_iobuf[i] );
		icp->rx_iobuf[i] = NULL;
	}
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
	union icplus_descriptor *desc;
	unsigned int tx_idx;
	physaddr_t address;
	size_t len;

	/* Check if ring is full */
	if ( ( icp->tx.prod - icp->tx.cons ) >= ICP_NUM_DESC ) {
		DBGC ( icp, "ICP %p out of transmit descriptors\n", icp );
		return -ENOBUFS;
	}

	/* Find TX descriptor entry to use */
	tx_idx = ( icp->tx.prod++ % ICP_NUM_DESC );
	desc = &icp->tx.entry[tx_idx];

	/* Fill in TX descriptor */
	address = virt_to_bus ( iobuf->data );
	len = iob_len ( iobuf );
	icp->tx.describe ( desc, address, len );
	wmb();

	/* Ring doorbell */
	writew ( ICP_DMACTRL_TXPOLLNOW, icp->regs + ICP_DMACTRL );

	DBGC2 ( icp, "ICP %p TX %d is [%llx,%llx)\n", icp, tx_idx,
		( ( unsigned long long ) address ),
		( ( unsigned long long ) address + iob_len ( iobuf ) ) );
	DBGC2_HDA ( icp, virt_to_phys ( desc ), desc, sizeof ( *desc ) );
	return 0;
}

/**
 * Poll for completed packets
 *
 * @v netdev		Network device
 */
static void icplus_poll_tx ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;
	union icplus_descriptor *desc;
	unsigned int tx_idx;

	/* Check for completed packets */
	while ( icp->tx.cons != icp->tx.prod ) {

		/* Get next transmit descriptor */
		tx_idx = ( icp->tx.cons % ICP_NUM_DESC );
		desc = &icp->tx.entry[tx_idx];

		/* Stop if descriptor is still in use */
		if ( ! icp->tx.completed ( desc ) )
			return;

		/* Complete TX descriptor */
		DBGC2 ( icp, "ICP %p TX %d complete\n", icp, tx_idx );
		writew ( 0, icp->regs + icp->txstatus );
		netdev_tx_complete_next ( netdev );
		icp->tx.cons++;
	}
}

/**
 * Poll for received packets
 *
 * @v netdev		Network device
 */
static void icplus_poll_rx ( struct net_device *netdev ) {
	struct icplus_nic *icp = netdev->priv;
	union icplus_descriptor *desc;
	struct io_buffer *iobuf;
	unsigned int rx_idx;
	int len;

	/* Check for received packets */
	while ( icp->rx.cons != icp->rx.prod ) {

		/* Get next transmit descriptor */
		rx_idx = ( icp->rx.cons % ICP_NUM_DESC );
		desc = &icp->rx.entry[rx_idx];

		/* Stop if descriptor is still in use */
		if ( ! icp->rx.completed ( desc ) )
			return;

		/* Populate I/O buffer */
		iobuf = icp->rx_iobuf[rx_idx];
		icp->rx_iobuf[rx_idx] = NULL;
		len = icp->rx.len ( desc );

		/* Hand off to network stack */
		if ( len < 0 ) {
			DBGC ( icp, "ICP %p RX %d error (flags %02x)\n",
			       icp, rx_idx, -len );
			netdev_rx_err ( netdev, iobuf, -EIO );
		} else {
			DBGC2 ( icp, "ICP %p RX %d complete (length "
				"%zd)\n", icp, rx_idx, len );
			iob_put ( iobuf, len );
			netdev_rx ( netdev, iobuf );
		}
		icp->rx.cons++;
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
	uint16_t txstatus;

	/* Check for interrupts */
	intstatus = readw ( icp->regs + ICP_INTSTATUS );

	/* Poll for TX completions, if applicable */
	if ( intstatus & ICP_INTSTATUS_TXCOMPLETE ) {
		txstatus = readw ( icp->regs + icp->txstatus );
		if ( txstatus & ( ICP_TXSTATUS_LATECOLLISION |
				  ICP_TXSTATUS_MAXCOLLISIONS |
				  ICP_TXSTATUS_UNDERRUN ) ) {
			DBGC ( icp, "ICP %p TX error: %04x\n", icp, txstatus );
			if ( txstatus & ICP_TXSTATUS_UNDERRUN ) {
				writel (ICP_ASICCTRL_TXRESET,
					icp->regs + ICP_ASICCTRL);
			}
			writew ( ICP_MACCTRL1_TXENABLE,
				 icp->regs + icp->macctrl1 );
		}
		icplus_poll_tx ( netdev );
	}

	/* Poll for RX completions, if applicable */
	if ( intstatus & ICP_INTSTATUS_RXDMACOMPLETE ) {
		writew ( ICP_INTSTATUS_RXDMACOMPLETE,
			 icp->regs + ICP_INTSTATUS );
		icplus_poll_rx ( netdev );
	}

	/* Check link state, if applicable */
	if ( intstatus & ICP_INTSTATUS_LINKEVENT ) {
		writew ( ICP_INTSTATUS_LINKEVENT, icp->regs + ICP_INTSTATUS );
		icplus_check_link ( netdev );
	}

	/* Refill receive ring */
	icplus_refill_rx ( icp );
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

	/* Identify card */
	if ( pci->id->driver_data == ICP_64BIT ) {
		icplus_init64 ( icp );
	} else {
		icplus_init32 ( icp );
	}

	/* Allocate and initialise mii interface */
	icp->miibit.basher.op = &icplus_basher_ops;
	init_mii_bit_basher ( &icp->miibit );
	mii_init ( &icp->mii, &icp->miibit.mdio, 0 );

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
		DBGC ( icp, "ICPLUS %p could not read EEPROM MAC address: "
		       "%s\n", icp, strerror ( rc ) );
		goto err_eeprom;
	}

	DBGC ( icp, "ICPLUS %p MAC address is %s\n",
	       icp, eth_ntoa ( netdev->hw_addr ) ); 

	/* Configure PHY */
	if ( ( rc = icplus_init_phy ( icp ) ) != 0 )
		goto err_phy;

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	icplus_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
 err_register_netdev:
 err_phy:
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
	PCI_ROM ( 0x13f0, 0x1023, "ip1000a",	"IP1000A",	ICP_64BIT ),
	PCI_ROM ( 0x1186, 0x1002, "dfe550tx",	"DFE550TX",	ICP_32BIT ),
};

/** IC+ PCI driver */
struct pci_driver icplus_driver __pci_driver = {
	.ids = icplus_nics,
	.id_count = ( sizeof ( icplus_nics ) / sizeof ( icplus_nics[0] ) ),
	.probe = icplus_probe,
	.remove = icplus_remove,
};
