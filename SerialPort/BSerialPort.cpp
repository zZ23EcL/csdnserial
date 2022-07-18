/* 
 * File:   BSerialPort.cpp
 */
#include <fcntl.h>
#include <locale>
#include <termios.h>
#include <pty.h>
#include <unistd.h>
#include <string.h>

#include "BSerialPort.h"


static int speed_arr[] = {
    B921600, B460800, B230400, B115200, B57600, B38400, B19200,
    B9600, B4800, B2400, B1200, B300,
};

static int name_arr[] = {
    921600, 460800, 230400, 115200, 57600, 38400, 19200,
    9600, 4800, 2400, 1200, 300,
};

BSerialPort::BSerialPort() {
    this->fd = -1;
    this->sfd = -1;
    this->function = NULL;
}

BSerialPort::BSerialPort(const BSerialPort& orig) {
    this->fd = orig.fd;
    this->sfd = orig.sfd;
    this->function = orig.function;
}

/*
 * Function: BSerialPort
 * 
 * 构造函数，建立并将串口口初始化为 9600,N,1
 * 或初始化一个虚拟串口，如果要定义一个虚拟串口
 * 请将type参数传递为Virtual
 * 
 * Prarm:
 *      filename: 串口设备文件名
 *      type：串口类型，可以是Actual或Virtual;
 */
BSerialPort::BSerialPort(char* filename, SerialType type) {
    switch (type) {
        case Actual:
            this->sfd = -1;
            this->fd = ::open(filename, O_RDWR);
            if (this->fd > 0) {
                if (!set_Speed(9600))
                    ::close(this->fd);
                if (!set_Parity(8, 1, 'N'))
                    ::close(this->fd);
            } else {
                printf("serial port open failed %s\n", filename);
            }
            break;
        case Virtual:
            this->fd = -1;
            this->sfd = -1;
            int ret = -1;
            char spty_name[100];

            memset(spty_name, '\0', 100);
            ret = ::openpty(&fd, &sfd, spty_name, NULL, NULL);
            if (ret == -1) {
                printf("SerialPort::SerialPort:openpty returned error");
            } else {
                /*如果链接文件存在则将其删除*/
                if ((access(filename, F_OK)) != -1) {
                    remove(filename);
                }
                symlinkat(spty_name, sfd, filename);
            }
            break;
    }
}

BSerialPort::BSerialPort(const char* filename, SerialType type) {
    switch (type) {
        case Actual:
            this->sfd = -1;
            this->fd = ::open(filename, O_RDWR);
            if (this->fd > 0) {
                if (!set_Speed(9600))
                    ::close(this->fd);
                if (!set_Parity(8, 1, 'N'))
                    ::close(this->fd);
            } else {
                printf("serial port open failed %s\n", filename);
            }
            break;
        case Virtual:
            this->fd = -1;
            this->sfd = -1;
            int ret = -1;
            char spty_name[100];

            memset(spty_name, '\0', 100);
            ret = ::openpty(&fd, &sfd, spty_name, NULL, NULL);
            if (ret == -1) {
                printf("SerialPort::SerialPort:openpty returned error");
            } else {
                /*如果链接文件存在则将其删除*/
                if ((access(filename, F_OK)) != -1) {
                    remove(filename);
                }
                symlinkat(spty_name, sfd, filename);
            }
            break;
    }
}

BSerialPort::~BSerialPort() {
}

/*
 * Function: 
 */
bool BSerialPort::open(char* filename) {
    if (this->fd > 0) {
        ::close(this->fd);
        printf("serial port open failed %s\n", filename);
    }
    this->fd = ::open(filename, O_RDWR);
    if (this->fd > 0) {
        if (!set_Speed(9600))
            ::close(this->fd);
        if (!set_Parity(8, 1, 'N'))
            ::close(this->fd);
    }
}

/*
 * Function: 
 */
bool BSerialPort::set_Speed(int speed) {
    struct termios Opt;

    if (this->fd < 0)
        return false;
    tcgetattr(this->fd, &Opt);

    for (int i = 0; i < sizeof (speed_arr) / sizeof (int); i++) {
        if (speed == name_arr[i]) {
            tcflush(this->fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            int status = tcsetattr(this->fd, TCSANOW, &Opt);
            if (status == 0)
                return true;
            else if (status == -1)
                return false;
        }
        tcflush(this->fd, TCIOFLUSH);
    }
    printf("set speed failed");
    return false;
}

/*
 * Function: 
 */
bool BSerialPort::set_Parity(int databits, int stopbits, char parity) {
    struct termios options;
    if (tcgetattr(this->fd, &options) != 0) {
        return false;
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            printf("databits set failed");
            return false;
    }

    switch (parity) {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;
            options.c_iflag &= ~INPCK;
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB);
            options.c_iflag |= INPCK;
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;
            options.c_cflag &= ~PARODD;
            options.c_iflag |= INPCK;
            break;
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;
            break;
        default:
            printf("parity set failed");
            return false;
    }

    switch (stopbits) {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            printf("stopbits set failed");
            return false;
    }

    if (parity != 'n')
        options.c_iflag |= INPCK;
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;

    options.c_lflag &= ~(ECHO | ICANON);

    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    options.c_oflag &= ~(ONLCR | OCRNL | ONOCR | ONLRET);

    tcflush(this->fd, TCIFLUSH);
    if (tcsetattr(this->fd, TCSANOW, &options) != 0)
        return false;
    return true;
}

/*
 * Function: 
 */
int BSerialPort::read(char* dest, int bytes) {
    int byte = ::read(this->fd, dest, bytes);
    return byte;
}

/*
 * Function: 
 */
int BSerialPort::write(char* buff, int bytes) {
    int byte = ::write(this->fd, buff, bytes);
    return byte;
}

/*
 * Function: 
 */
void BSerialPort::close() {
    ::close(this->fd);
}

/*
 * Function: 
 */
bool BSerialPort::operator==(BSerialPort& serial) {
    if (this->fd == serial.fd && this->fd != 0 && serial.fd != 0)
        return true;
    else
        return false;
}
