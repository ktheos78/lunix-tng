#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>

int main(void)
{
    int fd, i = 0;
    pid_t p;

    if ((fd = open("/dev/lunix0-temp", O_RDONLY)) == -1) {
        perror("open");
        exit(1);
    }

    if ((p = fork()) == -1) {
        perror("fork");
        exit(1);
    }

    /* child */
    if (p == 0) {
        char buf[32];
        while (i < 3)
        {
            read(fd, buf, sizeof(buf));
            printf("[Child] Read: %s\n", buf);
            i++;
        }

        printf("[Child] Done, exiting...\n");
        exit(0);
    }

    /* parent*/
    if (p > 0) {
        char buf[32];
        while (i < 10)
        {
            read(fd, buf, sizeof(buf));
            printf("[Parent] Read: %s\n", buf);
            i++;
        }

        printf("[Parent] Done.\n");
    }

    printf("About to write to invalid buffer...\n");
    ssize_t r = read(fd, (void *)0xDEADBEEF, 1);
    if (r < 0 && errno == EFAULT) {
        printf("Caught EFAULT!\n");
    }
}