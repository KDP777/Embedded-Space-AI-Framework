#ifndef V4L2_INIT_H
#define V4L2_INIT_H

struct buffer
{
    void *start;
    int len;
};

int v4l2_init(int fd);
struct buffer v4l2_getImage(int fd);

#endif // V4L2_INIT_H