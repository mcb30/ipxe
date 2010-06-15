#ifndef _IPXE_FC_H
#define _IPXE_FC_H

/**
 * @file
 *
 * Fibre Channel
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <stdint.h>

/** A Fibre Channel Frame Header */
struct fc_frame_header {
	/** Routing control
	 *
	 * This is the bitwise OR of one @c fc_r_ctl_routing value and
	 * one @c fc_r_ctl_info value.
	 */
	uint8_t r_ctl;
	/** Destination ID */
	uint8_t d_id[3];
	/** Class-specific control / Priority */
	uint8_t cs_ctl_prio;
	/** Source ID */
	uint8_t s_id[3];
	/** Data structure type */
	uint8_t type;
	/** Frame control - exchange and sequence */
	uint8_t f_ctl_es;
	/** Frame control - acknowledgements  */
	uint8_t f_ctl_ack;
	/** Frame control - miscellaneous */
	uint8_t f_ctl_misc;
	/** Sequence ID */
	uint8_t seq_id;
	/** Data field control */
	uint8_t df_ctl;
	/** Sequence count */
	uint16_t seq_cnt;
	/** Originator exchange ID */
	uint16_t ox_id;
	/** Responder exchange ID */
	uint16_t rx_id;
	/** Parameter
	 *
	 * Contains the relative offset when @c FC_F_CTL_MISC_REL_OFF
	 * is set.
	 */
	uint32_t parameter;
} __attribute__ (( packed ));

/** Fibre Channel Routing Control Routing */
enum fc_r_ctl_routing {
	FC_R_CTL_DATA = 0x00,		/**< Device Data */
	FC_R_CTL_ELS = 0x20,		/**< Extended Link Services */
	FC_R_CTL_FC4_LINK = 0x30,	/**< FC-4 Link Data */
	FC_R_CTL_VIDEO = 0x40,		/**< Video Data */
	FC_R_CTL_EH = 0x50,		/**< Extended Headers */
	FC_R_CTL_BLS = 0x80,		/**< Basic Link Services */
	FC_R_CTL_LINK_CTRL = 0xc0,	/**< Link Control */
	FC_R_CTL_EXT_ROUTE = 0xf0,	/**< Extended Routing */
};

/** Fibre Channel Routing Control Information */
enum fc_r_ctl_info {
	FC_R_CTL_UNCAT = 0x00,		/**< Uncategorized */
	FC_R_CTL_SOL_DATA = 0x01,	/**< Solicited Data */
	FC_R_CTL_UNSOL_CTRL = 0x02,	/**< Unsolicited Control */
	FC_R_CTL_SOL_CTRL = 0x03,	/**< Solicited Control */
	FC_R_CTL_UNSOL_DATA = 0x04,	/**< Unsolicited Data */
	FC_R_CTL_DATA_DESC = 0x05,	/**< Data Descriptor */
	FC_R_CTL_UNSOL_CMD = 0x06,	/**< Unsolicited Command */
	FC_R_CTL_CMD_STAT = 0x07,	/**< Command Status */
};

/** Fibre Channel Data Structure Type */
enum fc_type {
	FC_TYPE_BLS = 0x00,		/**< Basic Link Service */
	FC_TYPE_ELS = 0x01,		/**< Extended Link Service */
	FC_TYPE_FCP = 0x08,		/**< Fibre Channel Protocol */
};

/** Fibre Channel Frame Control - Exchange and Sequence */
enum fc_f_ctl_es {
	FC_F_CTL_ES_RESPONDER = 0x80,	/**< Responder of Exchange */
	FC_F_CTL_ES_RECIPIENT = 0x40,	/**< Sequence Recipient */
	FC_F_CTL_ES_FIRST = 0x20,	/**< First Sequence of Exchange */
	FC_F_CTL_ES_LAST = 0x10,	/**< Last Sequence of Exchange */
	FC_F_CTL_ES_END = 0x08,		/**< Last Data Frame of Sequence */
	FC_F_CTL_ES_TRANSFER = 0x01,	/**< Transfer Sequence Initiative */
};

/** Fibre Channel Frame Control - Miscellaneous */
enum fc_f_ctl_misc {
	FC_F_CTL_MISC_REL_OFF = 0x08,	/**< Relative Offset Present */
};

#endif /* _IPXE_FC_H */
