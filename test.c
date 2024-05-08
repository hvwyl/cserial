#include <stdio.h>

#include <unistd.h>
#include <sys/types.h>

#include <termios.h>

#include "serial.h"

char getch_noecho()
{
    struct termios oldopt, newopt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldopt);
    newopt = oldopt;
    newopt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newopt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldopt);
    return ch;
}

int main()
{
    pid_t fpid;
    int fd;
    char send, recv;

    // 115200, 8, n, 1, n
    if ((fd = serial_open("/dev/pts/2", 115200, 8, PARITY_NONE, 1, FLOW_NONE, 0, 0)) < 0)
        return fd;
    
    fpid = fork();
    if (fpid == 0) {
        while (1) {
            if(serial_read(fd,&recv,1)) {
                if (recv == '\r')
                    putchar('\n');
                else
                    putchar(recv);
                fflush(stdout);
            }
        }
    } else {
        while (1) {
            send = getch_noecho();
            if (send == '\n')
                serial_write(fd, "\r\n", 2);
            else
                serial_write(fd, &send, 1);
        }
    }
    
    serial_close(fd);
    return 0;
}
