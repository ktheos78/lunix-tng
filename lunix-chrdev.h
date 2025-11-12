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

/* enum for blocking_state */
enum lunix_blocking_mode_enum {
	MODE_NONBLOCKING = 0,
	MODE_BLOCKING = 1
};

/* enum for rewind on EOF */
enum lunix_rewind_mode_enum {
	MODE_NOREWIND = 0,
	MODE_REWIND = 1
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
	 * Mode settings: blocking(1)/non-blocking(0)
	 				  rewind on EOF(1)/no rewind(0)
	 * 		TODO: ioctl for raw/cooked mode, EOF mode
	 */
	enum lunix_blocking_mode_enum blocking_mode;
	enum lunix_rewind_mode_enum rewind_mode;
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
