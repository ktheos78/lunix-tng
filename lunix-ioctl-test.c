#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

/* ioctl definitions */
#define LUNIX_IOC_MAGIC 60
#define LUNIX_IOC_MAXNR 5

/* setters */
#define LUNIX_IOC_SET_FORMAT	_IOW(LUNIX_IOC_MAGIC, 0, int)	/* raw/cooked */
#define LUNIX_IOC_SET_BLOCKING	_IOW(LUNIX_IOC_MAGIC, 1, int)	/* blocking/non-blocking */
#define LUNIX_IOC_SET_REWIND	_IOW(LUNIX_IOC_MAGIC, 2, int)	/* rewind/no rewind on EOF */

/* getters */
#define LUNIX_IOC_GET_FORMAT	_IOR(LUNIX_IOC_MAGIC, 3, int)	/* raw/cooked */
#define LUNIX_IOC_GET_BLOCKING	_IOR(LUNIX_IOC_MAGIC, 4, int)	/* blocking/non-blocking */
#define LUNIX_IOC_GET_REWIND	_IOR(LUNIX_IOC_MAGIC, 5, int)	/* rewind/no rewind on EOF */

/* macros for tests */
enum {
	MODE_NONBLOCKING = 0,
	MODE_BLOCKING = 1
};

enum {
	MODE_NOREWIND = 0,
	MODE_REWIND = 1
};

enum {
	MODE_RAW = 0,
	MODE_COOKED = 1
};

int main(void)
{
    int fd, mode, i = 0;
    char buf[16];

    /* open /dev/lunix0-batt */
    if ((fd = open("/dev/lunix0-temp", O_RDWR)) == -1) {
        perror("open");
        exit(1);
    }


    /**************
     *   FORMAT   *
     **************/

    printf("Formatting mode is currently cooked. (default)\n");
    printf("Reading from sensor...\n");
    if (read(fd, buf, sizeof(buf)) == -1) {
        perror("read");
        exit(1);
    }
    printf("Read: %s\n\n", buf);

    printf("Changing formatting mode to raw...\n");
    mode = MODE_RAW;
    if (ioctl(fd, LUNIX_IOC_SET_FORMAT, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Getting formatting mode...\n");
    if (ioctl(fd, LUNIX_IOC_GET_FORMAT, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Current mode is: %s.\n\n", mode ? "cooked" : "raw");
    printf("Reading from sensor...\n");
    if (read(fd, buf, sizeof(buf)) == -1) {
        perror("read");
        exit(1);
    }
    printf("Read: %s\n\n", buf);

    printf("Reverting formatting mode to cooked...\n");
    mode = MODE_COOKED;
    if (ioctl(fd, LUNIX_IOC_SET_FORMAT, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Reading from sensor...\n");
    if (read(fd, buf, sizeof(buf)) == -1) {
        perror("read");
        exit(1);
    }
    printf("Read: %s\n\n", buf);

    printf("Getting formatting mode...\n");
    if (ioctl(fd, LUNIX_IOC_GET_FORMAT, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Current mode is: %s.\n\n", mode ? "cooked" : "raw");


    /****************
     *   BLOCKING   *
     ****************/

    printf("Changing mode to non-blocking...\n");
    mode = MODE_NONBLOCKING;
    if (ioctl(fd, LUNIX_IOC_SET_BLOCKING, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Getting blocking mode...\n");
    if (ioctl(fd, LUNIX_IOC_GET_BLOCKING, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Current mode is: %s.\n\n", mode ? "blocking" : "nonblocking");
    printf("Reading from sensor...\n");
    if (read(fd, buf, sizeof(buf)) == -1 && errno == EAGAIN)
        printf("Caught EAGAIN, non-blocking mode works!\n\n");
    else
        printf("Didn't catch EAGAIN\n\n");
    
    printf("Reverting blocking mode to blocking...\n");
    mode = MODE_BLOCKING;
    if (ioctl(fd, LUNIX_IOC_SET_BLOCKING, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Getting blocking mode...\n");
    if (ioctl(fd, LUNIX_IOC_GET_BLOCKING, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Current mode is: %s.\n\n", mode ? "blocking" : "nonblocking");
    printf("Reading from sensor... (blocking again)\n");
    if (read(fd, buf, sizeof(buf)) == -1) {
        perror("read");
        exit(1);
    }
    printf("Read: %s\n\n", buf);


    /**************
     *   REWIND   *
     **************/    

    printf("Getting rewind mode...\n");
    if (ioctl(fd, LUNIX_IOC_GET_REWIND, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Current mode is: %s.\n\n", mode ? "rewind" : "no rewind");
    printf("Reading from sensor...\n");

    while (i < 5) 
    {
        if (read(fd, buf, sizeof(buf)) == -1) {
            perror("read");
            exit(1);
        }
        printf("Read (%d): %s\n", i, buf);
        i++;
    }

    printf("Changing mode to no rewind...\n");
    mode = MODE_NOREWIND;
    if (ioctl(fd, LUNIX_IOC_SET_REWIND, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Getting rewind mode...\n");
    if (ioctl(fd, LUNIX_IOC_GET_REWIND, &mode) == -1) {
        perror("ioctl");
        exit(1);
    }
    printf("Current mode is: %s.\n\n", mode ? "rewind" : "no rewind");
    printf("Reading from sensor...\n");
    i = 0;
    while (i < 5) 
    {
        if (read(fd, buf, sizeof(buf)) == -1) {
            perror("read");
            exit(1);
        }
        printf("Read (%d): %s\n", i, buf);
        i++;
    }

    printf("\nioctl tests complete.\n");
    return 0;
}