#ifndef _SERIAL_H
#define _SERIAL_H

enum PARITY {
    PARITY_NONE,
    PARITY_EVEN,
    PARITY_ODD,
    PARITY_SPACE,
    PARITY_MARK
};

enum FLOWCTRL {
    FLOW_NONE,
    FLOW_HARDWARE,
    FLOW_SOFTWARE
};

int serial_open(char *port, int baud, int databits, enum PARITY parity, int stopbits, enum FLOWCTRL flow, int vtime, int vmin);
int serial_close(int fd);
int serial_dtr(int fd, int mode);
int serial_rts(int fd, int mode);
ssize_t serial_read(int fd, void *buf, size_t count);
ssize_t serial_write(int fd, const void *buf, size_t count);

#endif