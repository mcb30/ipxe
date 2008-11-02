#ifndef _GPXE_IB_PACKET_H
#define _GPXE_IB_PACKET_H

/** @file
 *
 * Infiniband packet format
 *
 */

struct ib_device;
struct ib_queue_pair;
struct ib_address_vector;
struct io_buffer;

/** Half of an Infiniband Global Identifier */
struct ib_gid_half {
	uint8_t bytes[8];
};

/** An Infiniband Global Identifier */
struct ib_gid {
	union {
		uint8_t bytes[16];
		uint16_t words[8];
		uint32_t dwords[4];
		struct ib_gid_half half[2];
	} u;
};

/** An Infiniband Local Route Header */
struct ib_local_route_header {
	/** Virtual lane and link version */
	uint8_t vl__lver;
	/** Service level and next link header */
	uint8_t sl__lnh;
	/** Destination LID */
	uint16_t dlid;
	/** Packet length */
	uint16_t length;
	/** Source LID */
	uint16_t slid;
} __attribute__ (( packed ));

/** An Infiniband Link Next Header value */
enum ib_lnh {
	IB_LNH_RAW = 0,
	IB_LNH_IPv6 = 1,
	IB_LNH_BTH = 2,
	IB_LNH_GRH = 3
};

/** An Infiniband Global Route Header */
struct ib_global_route_header {
	/** IP version, traffic class, and flow label
	 *
	 *  4 bits : Version of the GRH
	 *  8 bits : Traffic class
	 * 20 bits : Flow label
	 */
	uint32_t ipver__tclass__flowlabel;
	/** Payload length */
	uint16_t paylen;
	/** Next header */
	uint8_t nxthdr;
	/** Hop limit */
	uint8_t hoplmt;
	/** Source GID */
	struct ib_gid sgid;
	/** Destiniation GID */
	struct ib_gid dgid;
} __attribute__ (( packed ));

#define IB_GRH_IPVER_IPv6 0x06
#define IB_GRH_NXTHDR_IBA 0x1b
#define IB_GRH_HOPLMT_MAX 0xff

/** An Infiniband Base Transport Header */
struct ib_base_transport_header {
	/** Opcode */
	uint8_t opcode;
	/** Transport header version, pad count, migration and solicitation */
	uint8_t tver__padcnt__m__se;
	/** Partition key */
	uint16_t pkey;
	/** Destination queue pair */
	uint32_t dest_qp;
	/** Packet sequence number and acknowledge request */
	uint32_t psn__ack;
} __attribute__ (( packed ));

/** An Infiniband BTH opcode */
enum ib_bth_opcode {
	BTH_OPCODE_UD_SEND = 0x64,
};

/** An Infiniband Datagram Extended Transport Header */
struct ib_datagram_extended_transport_header {
	/** Queue key */
	uint32_t qkey;
	/** Source queue pair */
	uint32_t src_qp;
} __attribute__ (( packed ));

/** All known IB header formats */
union ib_headers {
	struct ib_local_route_header lrh;
	struct {
		struct ib_local_route_header lrh;
		struct ib_global_route_header grh;
		struct ib_base_transport_header bth;
		struct ib_datagram_extended_transport_header deth;
	} __attribute__ (( packed )) lrh__grh__bth__deth;
	struct {
		struct ib_local_route_header lrh;
		struct ib_base_transport_header bth;
		struct ib_datagram_extended_transport_header deth;
	} __attribute__ (( packed )) lrh__bth__deth;
} __attribute__ (( packed ));

/** Maximum size required for IB headers */
#define IB_MAX_HEADER_SIZE sizeof ( union ib_headers )

extern int ib_pull ( struct ib_device *ibdev, struct io_buffer *iobuf,
		     struct ib_address_vector *av );
extern int ib_push ( struct ib_device *ibdev, struct ib_queue_pair *qp,
		     struct io_buffer *iobuf, struct io_buffer *payload,
		     const struct ib_address_vector *av );

#endif /* _GPXE_IB_PACKET_H */
