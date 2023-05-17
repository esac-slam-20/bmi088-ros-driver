#include "file.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/gpio.h>

File::File(const char* name, int flag)
{
    _fd = open(name, flag);
    if (_fd < 0) {
        std::cerr << "Error when open " << name << ": " << std::strerror(errno) << std::endl;
    }
}

File::~File()
{
    if (_fd >= 0) {
        close(_fd);
    }
}

int File::fd()
{
    return _fd;
}

bool File::valid()
{
    return _fd >= 0;
}

int File::Read(void* __buf, size_t __nbytes)
{
    int ret = read(_fd, __buf, __nbytes);
    if (ret < 0) {
        perror("err on read");
    }
    return ret;
}

int File::Write(const void* __buf, size_t __n)
{
    int ret = write(_fd, __buf, __n);
        if (ret < 0) {
        perror("err on write");
    }
    return ret;
}

int File::LSeek(long offset, int whence)
{
    int ret = lseek(_fd, offset, whence);
    if (ret < 0) {
        perror("err on lseek");
    }
    return ret;
}

int File::IOCtl(unsigned long int __request, ...)
{
    int ret = ioctl(_fd, __request);
    if (ret < 0) {
        perror("err on ioctl");
    }
    return ret;
}
