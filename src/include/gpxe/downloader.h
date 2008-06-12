#ifndef _GPXE_DOWNLOADER_H
#define _GPXE_DOWNLOADER_H

/** @file
 *
 * Image downloader
 *
 */

struct interface;
struct image;

extern int create_downloader ( struct interface *job, struct image *image,
			       int ( * register_image ) ( struct image *image ),
			       int type, ... );

#endif /* _GPXE_DOWNLOADER_H */
