#ifndef _IPXE_FCP_H
#define _IPXE_FCP_H

/**
 * @file
 *
 * Fibre Channel Protocol
 *
 */

FILE_LICENCE ( GPL2_OR_LATER );

#include <stdint.h>

struct fcp_header {
};



d_id, s_id, local exchange id maps to a socket-type connection
	no, more like a command
	general idea of a mux/demux?
	sequence set maps to a packet (fragmentation)

#endif /* _IPXE_FCP_H */
