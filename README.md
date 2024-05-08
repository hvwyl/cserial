# cserial
A simple C library for serial communication in linux.
```
serial.c    source file of library
serial.h    header file of library
test.c      a serial monitor using the library
```

Usage:
```
#include <stdio.h>

#include "serial.h"

int main()
{
    int fd;

    char str[] = "hello world";

    int i;
    char recv[5];

    // 115200, 8, n, 1, n
    if ((fd = serial_open("/dev/pts/2", 115200, 8, PARITY_NONE, 1, FLOW_NONE, 0, 0)) < 0)
        return fd;

    // send data
    serial_write(fd, str, sizeof(str));

    // recv data
    for (i = 0; i < sizeof(recv); i++)
        while (!serial_read(fd, &recv[i], 1));
    for (i = 0; i < sizeof(recv); i++)
        putchar(recv[i]);
    putchar('\n');

    serial_close(fd);
    return 0;
}
```
