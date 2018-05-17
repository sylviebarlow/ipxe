#ifndef _ICPLUS_H
#define _ICPLUS_H

/** @file
 *
 * IC+ network driver
 *
 */

#include <ipxe/nvs.h>
#include <ipxe/mii_bit.h>

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

/** Card variant (64 bit) */
#define ICP_64BIT 0

/** Card variant (32 bit) */
#define ICP_32BIT 1

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
#define ICP_ASICCTRL_PHYSPEED1000	0x00000040UL	/**< PHY speed 1000 */
#define ICP_ASICCTRL_GLOBALRESET	0x00010000UL	/**< Global reset */
#define ICP_ASICCTRL_DMA		0x00080000UL	/**< DMA */
#define ICP_ASICCTRL_FIFO		0x00100000UL	/**< FIFO */
#define ICP_ASICCTRL_NETWORK		0x00200000UL	/**< Network */
#define ICP_ASICCTRL_HOST		0x00400000UL	/**< Host */
#define ICP_ASICCTRL_AUTOINIT		0x00800000UL	/**< Auto init */
#define ICP_ASICCTRL_RESETBUSY		0x04000000UL	/**< Reset busy */

/** Maximum time to wait for reset */
#define ICP_RESET_MAX_WAIT_MS 1000

/** DMA control register (word/double word) */
#define ICP_DMACTRL 0x00
#define ICP_DMACTRL_RXPOLLNOW		0x0010		/**< Receive poll now */
#define ICP_DMACTRL_TXPOLLNOW 		0x1000		/**< Transmit poll now */

/** EEPROM control register (word) */
#define ICP64_EEPROMCTRL 0x4a
#define ICP32_EEPROMCTRL 0x36
#define ICP_EEPROMCTRL_ADDRESS( x )	( (x) << 0 )	/**< Address */
#define ICP_EEPROMCTRL_OPCODE( x )	( (x) << 8 )	/**< Opcode */
#define ICP_EEPROMCTRL_OPCODE_READ \
	ICP_EEPROMCTRL_OPCODE ( 2 )			/**< Read register */
#define ICP_EEPROMCTRL_BUSY		0x8000		/**< EEPROM busy */

/** Maximum time to wait for reading EEPROM */
#define ICP_EEPROM_MAX_WAIT_MS 1000

/** EEPROM word length */
#define ICP_EEPROM_WORD_LEN_LOG2 1

/** Minimum EEPROM size, in words */
#define ICP_EEPROM_MIN_SIZE_WORDS 0x20

/** Address of MAC address within EEPROM */
#define ICP_EEPROM_MAC 0x10

/** EEPROM data register (word) */
#define ICP64_EEPROMDATA 0x48
#define ICP32_EEPROMDATA 0x34

/** Interupt status register (word) */
#define ICP_INTSTATUS 0x5e
#define ICP_INTSTATUS_TXCOMPLETE	0x0004		/**< TX complete */
#define ICP_INTSTATUS_LINKEVENT		0x0100		/**< Link event */
#define ICP_INTSTATUS_RXDMACOMPLETE	0x0400		/**< RX DMA complete */

/** MAC control register 0 (word) */
#define ICP_MACCTRL0 0x6c
#define ICP_MACCTRL0_DUPLEX		0x0020UL	/**< Duplex select */

/** MAC control register 1 (word) */
#define ICP_MACCTRL1 0x6e
#define ICP_MACCTRL1_TXENABLE		0x0100UL	/**< TX enable */
#define ICP_MACCTRL1_TXDISABLE		0x0200UL	/**< TX disable */
#define ICP_MACCTRL1_RXENABLE		0x0800UL	/**< RX enable */
#define ICP_MACCTRL1_RXDISABLE		0x1000UL	/**< RX disable */

/** PHY control register (byte) */
#define ICP64_PHYCTRL 0x76
#define ICP32_PHYCTRL 0x5e
#define ICP_PHYCTRL_MGMTCLK		0x01		/**< Management clock */
#define ICP_PHYCTRL_MGMTDATA		0x02		/**< Management data */
#define ICP_PHYCTRL_MGMTDIR		0x04		/**< Management direction */
#define ICP_PHYCTRL_LINKSPEED		0xc0		/**< Link speed */

/** Receive mode register (byte) */
#define ICP_RXMODE 0x88
#define ICP_RXMODE_UNICAST		0x01		/**< Receive unicast */
#define ICP_RXMODE_MULTICAST		0x02		/**< Receice multicast */
#define ICP_RXMODE_BROADCAST		0x04		/**< Receive broadcast */
#define ICP_RXMODE_ALLFRAMES		0x08		/**< Receive all frames */

/** List pointer receive register */
#define ICP_RFDLISTPTR 0x1c

/** List pointer transmit register */
#define ICP_TFDLISTPTR 0x10

/** Transmit status register */
#define ICP_TXSTATUS 0x60
#define ICP_TXSTATUS_ERROR		0x00000001UL	/**< TX error */

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

/** 64-bit transmit or receive descriptor */
struct icplus_descriptor64 {
	/** Address of next descriptor */
	uint64_t next;
	/** Actual length */
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

/** 32-bit transmit or receive descriptor */
struct icplus_descriptor32 {
	/** Address of next descriptor */
	uint32_t next;
	/** Frames */
	uint16_t frames;
	/** Flags */
	uint8_t flags;
	/** Reserved */
	uint8_t reserved;
	/** Address */
	uint32_t address;
	/** Length */
	uint16_t len;
	/** Last fragment indicator */
	uint16_t last;
};

/** IC+ descriptor */
union icplus_descriptor {
	/** 64-bit descriptor */ 
	struct icplus_descriptor64 d64;
	/** 32-bit descriptor */
	struct icplus_descriptor32 d32;
	/** Address of next descriptor */
	uint64_t next;
};

/** Descriptor complete 64 bits */
#define ICP64_DONE 0x80

/** Receive descriptor complete 32 bits */
#define ICP32_RX_DONE 0x8000

/** Transmit descriptor complete 32 bits */
#define ICP32_TX_DONE 0x01

/** Transmit alignment disabled */
#define ICP_TX_UNALIGN 0x01

/** Request transmit completion 64 bits */
#define ICP64_TX_INDICATE 0x40

/** Request transmit completion 32 bits */
#define ICP32_TX_INDICATE 0x8000

/** Sole transmit fragment 64 bits */
#define ICP64_TX_SOLE_FRAG 0x01

/** Sole transmit or receive fragment 32 bits */
#define ICP32_SOLE_FRAG 0x8000

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

/** Descriptor ring */
struct icplus_ring {
	/** Producer counter */
	unsigned int prod;
	/** Consumer counter */
	unsigned int cons;
	/** Ring entries */
	union icplus_descriptor *entry;
	/* List pointer register */
	unsigned int listptr;
	/** Initialise descriptor
	 *
	 * @v desc		Descriptor
	 */
	void ( *setup ) ( union icplus_descriptor *desc );
	/** Describe data buffer
	 *
	 * @v desc		Descriptor
	 * @v address		Address
	 * @v len		Length
	 */
	void ( *describe ) ( union icplus_descriptor *desc, physaddr_t address,
			     size_t len );
	/** Check descriptor buffer completion
	 *
	 * @v desc		Descriptor
	 * @ret is_completed	Descriptor is complete
	 */
	int ( *completed ) ( union icplus_descriptor *desc );
};

/** Number of descriptors */
#define ICP_NUM_DESC 4

/** Maximum receive packet length */
#define ICP_RX_MAX_LEN ETH_FRAME_LEN

/** An IC+ network card */
struct icplus_nic {
	/** Registers */
	void *regs;
	/** EEPROM */
	struct nvs_device eeprom;
	/** MII bit bashing interface */
	struct mii_bit_basher miibit;
	/** MII device */
	struct mii_device mii;
	/** Transmit descriptor ring */
	struct icplus_ring tx;
	/** Receive descriptor ring */
	struct icplus_ring rx;
	/** Receive I/O buffers */
	struct io_buffer *rx_iobuf[ICP_NUM_DESC];
	/** EEPROM control register offset */
	unsigned int eepromctrl;
	/** EEPROM data register offset */
	unsigned int eepromdata;
	/** PHY control register offset */
	unsigned int phyctrl;
};

#endif /* _ICPLUS_H */
