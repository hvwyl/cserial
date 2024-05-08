#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#ifndef __USE_MISC
#define __USE_MISC
#endif

#include <termios.h>

#include "serial.h"

#define len(arr) (sizeof(arr)/sizeof(arr[0]))

struct speed_map {
    int     value;
    speed_t speed;
};

const struct speed_map speeds[] = {
#ifdef B50
    {.value = 50, .speed = B50},
#endif

#ifdef B75
    {.value = 75, .speed = B75},
#endif

#ifdef B110
    {.value = 110, .speed = B110},
#endif

#ifdef B134
    {.value = 134, .speed = B134},
#endif

#ifdef B150
    {.value = 150, .speed = B150},
#endif

#ifdef B200
    {.value = 200, .speed = B200},
#endif

#ifdef B300
    {.value = 300, .speed = B300},
#endif

#ifdef B600
    {.value = 600, .speed = B600},
#endif

#ifdef B1200
    {.value = 1200, .speed = B1200},
#endif

#ifdef B1800
    {.value = 1800, .speed = B1800},
#endif

#ifdef B2400
    {.value = 2400, .speed = B2400},
#endif

#ifdef B4800
    {.value = 4800, .speed = B4800},
#endif

#ifdef B7200
    {.value = 7200, .speed = B7200},
#endif

#ifdef B9600
    {.value = 9600, .speed = B9600},
#endif

#ifdef B14400
    {.value = 14400, .speed = B14400},
#endif

#ifdef B19200
    {.value = 19200, .speed = B19200},
#endif

#ifdef B28800
    {.value = 28800, .speed = B28800},
#endif

#ifdef B38400
    {.value = 38400, .speed = B38400},
#endif

#ifdef B57600
    {.value = 57600, .speed = B57600},
#endif

#ifdef B76800
    {.value = 76800, .speed = B76800},
#endif

#ifdef B115200
    {.value = 115200, .speed = B115200},
#endif

#ifdef B230400
    {.value = 230400, .speed = B230400},
#endif

#ifdef B460800
    {.value = 460800, .speed = B460800},
#endif

#ifdef B500000
    {.value = 500000, .speed = B500000},
#endif

#ifdef B576000
    {.value = 576000, .speed = B576000},
#endif

#ifdef B921600
    {.value = 921600, .speed = B921600},
#endif

#ifdef B1000000
    {.value = 1000000, .speed = B1000000},
#endif

#ifdef B1152000
    {.value = 1152000, .speed = B1152000},
#endif

#ifdef B1500000
    {.value = 1500000, .speed = B1500000},
#endif

#ifdef B2000000
    {.value = 2000000, .speed = B2000000},
#endif

#ifdef B2500000
    {.value = 2500000, .speed = B2500000},
#endif

#ifdef B3000000
    {.value = 3000000, .speed = B3000000},
#endif

#ifdef B3500000
    {.value = 3500000, .speed = B3500000},
#endif

#ifdef B4000000
    {.value = 4000000, .speed = B4000000},
#endif
};

static speed_t value_to_speed(int baud)
{
    int l = 0;
    int h = len(speeds)-1;
    int p;
    while (l <= h) {
        p = (h+l)/2;
        if (speeds[p].value > baud) {
            h = p - 1;
        } else if (speeds[p].value < baud) {
            l = p + 1;
        } else {
            return speeds[p].speed;
        }
    }
    return -1;
}

int serial_open(char *port, int baud, int databits, enum PARITY parity, int stopbits, enum FLOWCTRL flow, int vtime, int vmin)
{
    #define ERROR(...) {fprintf(stderr, ##__VA_ARGS__);close(fd);return -1;}
    int fd;
    struct termios opt;
    speed_t speed;
    if ((fd = open(port, O_RDWR|O_NOCTTY)) < 0) {
        fprintf(stderr, "Unable to open serial port: %s\n", strerror(errno));
        return fd;
    }
    if ((speed = value_to_speed(baud)) < 0)
        ERROR("Unspported baud rate %d\n", baud);
    
    if(tcgetattr(fd, &opt) < 0)
        ERROR("Unable to getattr: %s\n", strerror(errno));

    /* set baud */
    if(cfsetispeed(&opt, speed) < 0)
        ERROR("Unable to setispeed: %s\n", strerror(errno));

    if(cfsetospeed(&opt, speed) < 0)
        ERROR("Unable to setospeed: %s\n", strerror(errno));

    /* using raw data mode */
    cfmakeraw(&opt);

    /* ignore modem control lines and enable receiver */
    opt.c_cflag |= (CLOCAL | CREAD);

    /* set timeout in deciseconds and minimum number of characters*/
    opt.c_cc[VTIME] = (cc_t)vtime;
    opt.c_cc[VMIN] = (cc_t)vmin;

    /* set databits */
    opt.c_cflag &= ~CSIZE;
    switch (databits) {
    case 5:
        opt.c_cflag |= CS5;
        break;
    case 6:
        opt.c_cflag |= CS6;
        break;
    case 7:
        opt.c_cflag |= CS7;
        break;
    case 8:
        opt.c_cflag |= CS8;
        break;
    default:
        ERROR("Unsupported databits %d\n", databits);
    }
    
    /* set parity */
    opt.c_iflag &= ~(PARMRK | INPCK);
    opt.c_iflag |= IGNPAR;
    switch (parity) {
    case PARITY_NONE:
        opt.c_cflag &= ~PARENB;
        break;
    case PARITY_EVEN:
        opt.c_cflag &= ~PARODD;
        opt.c_cflag |= PARENB;
        break;
    case PARITY_ODD:
        opt.c_cflag |= (PARENB | PARODD);
        break;
    #ifdef CMSPAR
    case PARITY_SPACE:
        opt.c_cflag &= ~PARODD;
        opt.c_cflag |= (PARENB | CMSPAR);
        break;
    case PARITY_MARK:
        opt.c_cflag |= (PARENB | CMSPAR | PARODD);
        break;
    #else
    case PARITY_SPACE:
    case PARITY_MARK:
        fprintf(stderr, "Unsupported PARITY_SPACE and PARITY_MARK, ignore parity parameter\n");
        break;
    #endif
    }

    /* set stopbits */
    switch (stopbits) {
    case 1:
        opt.c_cflag &= ~CSTOPB;
        break;
    case 2:
        opt.c_cflag |= CSTOPB;
        break;
    default:
        ERROR("Unsupported stopbits %d\n", stopbits);
    }

    /* set flowcontrol */
    switch (flow) {
    case FLOW_NONE:
        opt.c_cflag &= ~CRTSCTS;
        opt.c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
    case FLOW_HARDWARE:
        opt.c_cflag |= CRTSCTS;
        opt.c_iflag &= ~(IXON | IXOFF | IXANY);
        break;
    case FLOW_SOFTWARE:
        opt.c_cflag &= ~CRTSCTS;
        opt.c_iflag |= (IXON | IXOFF | IXANY);
        break;
    }

    if(tcsetattr(fd, TCSANOW, &opt) < 0)
        ERROR("Unable to setattr: %s\n", strerror(errno));

    /* flush buffer */
    tcflush(fd, TCIOFLUSH);

    return fd;

    #undef ERROR
}

int serial_close(int fd)
{
    return close(fd);
}

int serial_dtr(int fd, int mode)
{
    int status;
    if (ioctl(fd, TIOCMSET, &status) < 0)
        return -1;
    
    if (mode == 0)
        status &= ~TIOCM_DTR;
    else
        status |= TIOCM_DTR;
    
    return ioctl(fd, TIOCMSET, &status);
}

int serial_rts(int fd, int mode)
{
    int status;
    if (ioctl(fd, TIOCMSET, &status) < 0)
        return -1;
    
    if (mode == 0)
        status &= ~TIOCM_RTS;
    else
        status |= TIOCM_RTS;
    
    return ioctl(fd, TIOCMSET, &status);
}

ssize_t serial_read(int fd, void *buf, size_t count)
{
    return read(fd, buf, count);
}

ssize_t serial_write(int fd, const void *buf, size_t count)
{
    return write(fd, buf, count);
}