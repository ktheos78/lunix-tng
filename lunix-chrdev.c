/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Konstantinos Theos
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

// TODO (?): look into making it atomic maybe
/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	debug("entering\n");

	struct lunix_sensor_struct *sensor;
	WARN_ON (!(sensor = state->sensor));

	/* check if sensor's last update is newer than state buffer's timestamp*/
	if (sensor->msr_data[state->type]->last_update > state->buf_timestamp)
		return 1;

	debug("leaving\n");
	return 0;
}


/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	/* declarations */
	struct lunix_sensor_struct *sensor;
	uint32_t last_update;
	uint16_t data;
	int ret;
	
	debug("entering\n");

	WARN_ON (!(sensor = state->sensor));
	ret = 0;

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	spin_lock(&sensor->lock);
	data = sensor->msr_data[state->type]->values[0];
	last_update = sensor->msr_data[state->type]->last_update;
	spin_unlock(&sensor->lock);

	/*
	 * Any new data available?
	 */
	if (last_update <= state->buf_timestamp) {
		ret = -EAGAIN;
		goto out;
	}

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */

	/* update buffer and buffer size using look-up table */
	// TODO: format data using LUT, copy to buffer, set buf_lim
	switch (state->type) {

		case BATT:
			break;

		case TEMP:
			break;

		case LIGHT:
			break;

		default:
			ret = -EFAULT;
			goto out;
	}

	/* update buffer timestamp */
	state->buf_timestamp = last_update;

	ret = 0;
out:
	debug("Leaving with ret = %d\n", ret);
	return ret;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	int ret;
	unsigned int minor, sensor_type, sensor_number;
	struct lunix_chrdev_state_struct *s;	/* device state information */

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	minor = iminor(inode);

	/* minor / 8 -> sensor number */
	sensor_number = minor / 8;		

	/* minor % 8 -> sensor type */
	sensor_type = minor % 8;	

	/* Allocate a new Lunix character device private state structure */
	s = kmalloc(sizeof(struct lunix_chrdev_state_struct *), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		pr_err("Failed to allocate %ld bytes for private state structure. ret = %d\n",
				sizeof(struct lunix_chrdev_state_struct *), 
				ret);
		goto out;
	}
		
	s->type = sensor_type;							/* (0 -> batt, 1 -> temp, 2 -> light) */
	s->sensor = &lunix_sensors[sensor_number];
	s->buf_lim = LUNIX_CHRDEV_BUFSZ;
	s->buf_timestamp = ktime_get_real_seconds();

	/* set blocking_mode according to O_NONBLOCK */
	if (filp->f_flags & O_NONBLOCK)
		s->blocking_mode = MODE_NONBLOCKING;
	else
		s->blocking_mode = MODE_BLOCKING;

	/* auto-rewind on EOF by default */
	s->rewind_mode = MODE_REWIND;

	/* zero-out buffer */
	memset(s->buf_data, 0, LUNIX_CHRDEV_BUFSZ);

	/* init semaphore */
	sema_init(&s->lock, 1);

	/* store state struct in file's private data */
	filp->private_data = (void *)s;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* deallocate character device state struct allocated by open() */
	kfree(filp->private_data);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;
	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	debug("Entering\n");

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* hold state lock for state update call */
	if (down_interruptible(&state->lock)) {
		ret = -ERESTARTSYS;
		goto out_no_lock;
	}

	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {

			/* return if mode is set to non-blocking */
			if (state->blocking_mode == MODE_NONBLOCKING) {
				ret = -EAGAIN;
				goto out;
			}

			/* release state lock */
			up(&state->lock);

			/* wait with wake-up event being fresh data */
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state))) {
				ret = -ERESTARTSYS;
				goto out_no_lock;
			}

			/* reacquire state lock and loop */
			if (down_interruptible(&state->lock)) {
				ret = -ERESTARTSYS;
				goto out_no_lock;
			}
		}
	}

	/* End of file */
	if (*f_pos >= state->buf_lim) {
		ret = 0;

		/* auto-rewind on EOF */
		if (state->rewind_mode == MODE_REWIND)
			*f_pos = 0;

		goto out;
	}
	
	/* Determine the number of cached bytes to copy to userspace */
	cnt = min_t(size_t, cnt, state->buf_lim - *f_pos);	/* typesafe macro */

	/* copy to user buffer */
	if (copy_to_user(usrbuf, &state->buf_data[*f_pos], cnt)) {
		ret = -EFAULT;
		goto out;
	}

	/* adjust offset, return number of read bytes */
	*f_pos += cnt;
	ret = cnt;

out:
	up(&state->lock);	/* release state lock */
out_no_lock:
	debug("Leaving with ret = %ld\n", (long)ret);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
