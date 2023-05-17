#pragma once

#include <stddef.h>

class File
{
private:
    int _fd;
public:
    File(const char* name, int flag);
    ~File();

    int fd();
    bool valid();

    int Read(void *__buf, size_t __nbytes);
    int Write(const void *__buf, size_t __n);
    int LSeek(long offset, int whence);
    int IOCtl(unsigned long int __request, ...);
};
