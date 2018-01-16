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

/** ASIC control register (double word) */
#define ICP_ASICCTRL 0x30
#define ICP_ASICCTRL_GLOBALRESET	0x00010000UL	/**< Global Reset */
#define ICP_ASICCTRL_RESETBUSY		0x04000000UL	/**< Reset Busy */

/** Maximum time to wait for reset */
#define ICP_RESET_MAX_WAIT_MS 1000

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

/** An IC+ network card */
struct icplus_nic {
	/** Registers */
	void *regs;
	/** EEPROM */
	struct nvs_device eeprom;
};

#endif /* _ICPLUS_H */
