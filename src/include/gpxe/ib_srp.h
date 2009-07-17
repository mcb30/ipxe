#ifndef _GPXE_IB_SRP_H
#define _GPXE_IB_SRP_H

/** @file
 *
 * SCSI RDMA Protocol over Infiniband
 *
 */

FILE_LICENCE ( BSD2 );

#include <stdint.h>
#include <gpxe/infiniband.h>
#include <gpxe/srp.h>

struct ib_srp_target {
	/** ID extension */
	struct ib_gid_half id_ext;
	/** I/O controller GUID */
	struct ib_gid_half ioc_guid;
	/** Destination GID */
	struct ib_gid dgid;
	/** Partition key */
	uint16_t pkey;
	/** Service ID */
	struct ib_gid_half service_id;
};

/** SRP initiator port identifier for Infiniband */
struct ib_srp_initiator_port_id {
	/** Identifier extension */
	uint64_t id_ext;
	/** IB channel adapter GUID */
	struct ib_gid_half hca_guid;
} __attribute__ (( packed ));

/** SRP target port identifier for Infiniband */
struct ib_srp_target_port_id {
	/** Identifier extension */
	struct ib_gid_half id_ext;
	/** I/O controller GUID */
	struct ib_gid_half ioc_guid;
} __attribute__ (( packed ));

struct ib_srp_device {
	/** Reference count */
	struct refcnt refcnt;

	/** Infiniband device */
	struct ib_device *ibdev;
	/** Completion queue */
	struct ib_completion_queue *cq;
	/** Queue pair */
	struct ib_queue_pair *qp;
	/** QP is connected */
	int qp_connected;

	/** SRP device */
	struct srp_device *srp;
	/** SRP target */
	struct ib_srp_target target;

	/** Initiator port identifier extension */
	unsigned int id_ext;
};

extern int ib_srp_attach ( struct scsi_device *scsi, struct ib_device *ibdev,
			   const char *root_path );
extern void ib_srp_detach ( struct scsi_device *scsi );

#endif /* _GPXE_IB_SRP_H */
