#ifndef _ICPLUS_H
#define _ICPLUS_H

/** @file
 *
 * IC+ network driver
 *
 */

#include <ipxe/nvs.h>

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

/** BAR size */
#define ICP_BAR_SIZE 0x200

/** Alignment requirement */
#define ICP_ALIGN 0x8

/** Base address low register offset */
#define ICP_BASE_LO 0x0

/** Base address high register offset */
#define ICP_BASE_HI 0x4

/** ASIC control register (double word) */
#define ICP_ASICCTRL 0x30
#define ICP_ASICCTRL_GLOBALRESET	0x00010000UL	/**< Global Reset */
#define ICP_ASICCTRL_RESETBUSY		0x04000000UL	/**< Reset Busy */

/** Maximum time to wait for reset */
#define ICP_RESET_MAX_WAIT_MS 1000

/** DMA control register (word/double word) */
#define ICP_DMACTRL 0x00
#define ICP_DMACTRL_TXPOLLNOW 0x1000

/** EEPROM control register (word) */
#define ICP_EEPROMCTRL 0x4a
#define ICP_EEPROMCTRL_ADDRESS( x )	( (x) << 0 )	/**< Address */
#define ICP_EEPROMCTRL_OPCODE( x )	( (x) << 8 )	/**< Opcode */
#define ICP_EEPROMCTRL_OPCODE_READ \
	ICP_EEPROMCTRL_OPCODE ( 2 )			/**< Read register */
#define ICP_EEPROMCTRL_BUSY		0x8000		/**< EEPROM Busy */

/** Maximum time to wait for reading EEPROM */
#define ICP_EEPROM_MAX_WAIT_MS 1000

/** EEPROM word length */
#define ICP_EEPROM_WORD_LEN_LOG2 1

/** Minimum EEPROM size, in words */
#define ICP_EEPROM_MIN_SIZE_WORDS 0x20

/** Address of MAC address within EEPROM */
#define ICP_EEPROM_MAC 0x10

/** EEPROM data register (word) */
#define ICP_EEPROMDATA 0x48

/** Interupt status register (word) */
#define ICP_INTSTATUS 0x5e
#define ICP_INTSTATUS_TXCOMPLETE	0x0004		/**< TX complete*/
#define ICP_INTSTATUS_LINKEVENT		0x0100		/**< Link event */

/** PHY control register (byte) */
#define ICP_PHYCTRL 0x76
#define ICP_PHYCTRL_LINKSPEED		0xc0		/**< Link speed */

/** List pointer transmit register */
#define ICP_TFDLISTPTR 0x10

/** Transmit status register */
#define ICP_TXSTATUS 0x60
#define ICP_TXSTATUS_ERROR		0x00000001     	/**< TX error */

/** Data fragment */
union icplus_fragment {
	/** Address of data */
	uint64_t address;
	/** Length */
	struct {
		/** Reserved */
		uint8_t reserved[6];
		/** Length of data */
		uint16_t len;
	};
};

/** Transmit descriptor */
struct icplus_tx_descriptor {
	/** Address of next descriptor */
	uint64_t next;
	/** Frame identifier */
	uint16_t id;
	/** Flags */
	uint8_t flags;
	/** Control */
	uint8_t control;
	/** VLAN */
	uint16_t vlan;
	/** Reserved */
	uint16_t reserved_a;
	/** Data buffer */
	union icplus_fragment data;
	/** Reserved */
	uint8_t reserved_b[8];
};

/** Transmit alignment disabled */
#define ICP_TX_UNALIGN 0x01

/** Request transmit completion */
#define ICP_TX_INDICATE 0x40

/** Sole transmit fragment */
#define ICP_TX_SOLE_FRAG 0x01

/** Transmit data complete */
#define ICP_TX_DONE 0x80

/** Receive descriptor */
struct icplus_rx_descriptor {
	/** Address of next descriptor */
	uint64_t next;
	/** Recieved length */
	uint16_t len;
	/** Flags */
	uint8_t flags;
	/** Control */
	uint8_t control;
	/** VLAN */
	uint16_t vlan;
	/** Reserved */
	uint16_t reserved_a;
	/** Data buffer */
	union icplus_fragment data;
	/** Reserved */
	uint8_t reserved_b[8];
};

/** Recieve frame overrun error */
#define ICP_RX_ERR_OVERRUN 0x01

/** Receive runt frame error */
#define ICP_RX_ERR_RUNT 0x02

/** Receive alignment error */
#define ICP_RX_ERR_ALIGN 0x04

/** Receive FCS error */
#define ICP_RX_ERR_FCS 0x08

/** Receive oversized frame error */
#define ICP_RX_ERR_OVERSIZED 0x10

/** Recieve length error */
#define ICP_RX_ERR_LEN 0x20

/** Receive frame descriptor done */
#define ICP_RX_DONE 0x80

/** Transmit descriptor ring */
struct icplus_tx_ring {
	/** Producer counter */
	unsigned int prod;
	/** Consumer counter */
	unsigned int cons;
	/** Ring entries */
	struct icplus_tx_descriptor *entry;
};

/** Number of transmit descriptors */
#define ICP_TX_NUM 4

/** An IC+ network card */
struct icplus_nic {
	/** Registers */
	void *regs;
	/** EEPROM */
	struct nvs_device eeprom;
	/** Transmit descriptor ring */
	struct icplus_tx_ring tx;
};

#endif /* _ICPLUS_H */
