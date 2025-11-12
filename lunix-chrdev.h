/*
 * lunix-chrdev.h
 *
 * Definition file for the
 * Lunix:TNG character device
 *
 * Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
 */

#ifndef _LUNIX_CHRDEV_H
#define _LUNIX_CHRDEV_H

/*
 * Lunix:TNG character device
 */
#define LUNIX_CHRDEV_MAJOR 60   /* Reserved for local / experimental use */
#define LUNIX_CHRDEV_BUFSZ 20   /* Buffer size used to hold textual info */

/* Compile-time parameters */

#ifdef __KERNEL__ 

#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "lunix.h"

/*
 *	Mode enums
 */

/* blocking/non-blocking i/o */
enum lunix_blocking_mode_enum {
	MODE_NONBLOCKING = 0,
	MODE_BLOCKING = 1
};

/* rewind/no rewind on EOF */
enum lunix_rewind_mode_enum {
	MODE_NOREWIND = 0,
	MODE_REWIND = 1
};

/* raw/cooked mode */
enum lunix_format_mode_enum {
	MODE_RAW = 0,
	MODE_COOKED = 1
};

/*
 * Private state for an open character device node
 */
struct lunix_chrdev_state_struct {
	enum lunix_msr_enum type;
	struct lunix_sensor_struct *sensor;

	/* A buffer used to hold cached textual info */
	int buf_lim;
	unsigned char buf_data[LUNIX_CHRDEV_BUFSZ];
	uint32_t buf_timestamp;

	struct semaphore lock;

	/*
	 * Mode settings: 
	 * - blocking/non-blocking
	 * - rewind/no rewind on EOF
	 * - raw/cooked (unformatted/formatted) data
	 * 	TODO: ioctl for switching between modes
	 */
	enum lunix_blocking_mode_enum blocking_mode;
	enum lunix_rewind_mode_enum rewind_mode;
	enum lunix_format_mode_enum	format_mode;
};

/*
 * Function prototypes
 */
int lunix_chrdev_init(void);
void lunix_chrdev_destroy(void);

#endif /* __KERNEL__ */

#include <linux/ioctl.h>

/*
 * Definition of ioctl commands
 */
#define LUNIX_IOC_MAGIC     LUNIX_CHRDEV_MAJOR
//#define LUNIX_IOC_EXAMPLE _IOR(LUNIX_IOC_MAGIC, 0, void *)

#define LUNIX_IOC_MAXNR 0

#endif /* _LUNIX_H */
