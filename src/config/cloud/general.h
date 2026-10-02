/* Enable IPv6 and HTTPS */
#define NET_PROTO_IPV6
#define DOWNLOAD_PROTO_HTTPS

/* Enable commands useful for headless operation */
#define CERT_CMD
#define DIGEST_CMD
#define NTP_CMD

/* Allow scripts to create custom headers for retrieving metadata */
#define PARAM_CMD

/* Allow scripts to handle errors by powering down the VM to avoid
 * incurring unnecessary costs.
 */
#define POWEROFF_CMD

/* Enable MIME image support */
#define IMAGE_MIME
