#ifndef _ICPLUS_H
#define _ICPLUS_H

/** @file
 *
 * IC+ network driver
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

/** BAR size */
#define ICP_BAR_SIZE 0x200

/** ASIC control register */
#define ICP_ASICCTRL 0x30
#define ICP_ASICCTRL_GLOBALRESET	0x00010000UL	/**< Global Reset */
#define ICP_ASICCTRL_RESETBUSY		0x04000000UL	/**< Reset Busy */

/** Maximum time to wait for reset */
#define ICP_RESET_MAX_WAIT_MS 1000

/** An IC+ network card */
struct icplus_nic {
	/** Registers */
	void *regs;
};

#endif /* _ICPLUS_H */
