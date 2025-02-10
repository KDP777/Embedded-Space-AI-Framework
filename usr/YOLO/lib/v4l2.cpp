#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>    //包含了close函数
#include <sys/ioctl.h>
#include <fcntl.h>     //包含了open函数
#include <linux/videodev2.h>
#include <malloc.h>
#include <sys/mman.h>
#include <v4l2.h>

struct buffer *buffers = (struct buffer*)calloc(4, sizeof(*buffers));

/*
 * @brief v4l2设备初始化
 * @param fd 摄像头设备的文件描述符
 * @return 成功 1
 * 		   失败 0
 */

int v4l2_init(int fd)
{
    //设置视频格式
    struct v4l2_format captureFormat;
    captureFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    captureFormat.fmt.pix.width       = 640;
    captureFormat.fmt.pix.height      = 480;
    captureFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    captureFormat.fmt.pix.field       =V4L2_FIELD_ANY;

    int ff = ioctl (fd, VIDIOC_S_FMT, &captureFormat);

    if (ff < 0)
    {
        printf("failture VIDIOC_S_FMT\n");
        return 0;
    }

    unsigned long file_length;
    //计算图片大小
    file_length = captureFormat.fmt.pix.bytesperline * captureFormat.fmt.pix.height;

    struct v4l2_requestbuffers req;

    memset(&req, 0, sizeof(req));//初始化结构体
    req.count    = 1;// 缓冲区内缓冲帧的数目
    req.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE;// 缓冲帧数据格式
    req.memory   = V4L2_MEMORY_MMAP;// 区别是内存映射还是用户指针方式

    //向设备申请缓冲
    ff = ioctl(fd, VIDIOC_REQBUFS, &req);

    if (ff < 0)
    {
       printf("failture VIDIOC_REQBUFS\n");
       return 0;
    }

    if (req.count < 1)
    {
        printf("failture insufficient buffer memory\n");
        return 0;
    }

    //内存中建立对应空间
//    struct buffer *buffers = (struct buffer*)calloc(req.count, sizeof(*buffers));

    for(int n_buf = 0; n_buf < req.count; ++n_buf)
    {
        struct v4l2_buffer buf;

        memset(&buf, 0, sizeof(buf));//初始化结构体
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buf;

        //获取缓冲帧的地址、长度
        if (-1 == ioctl (fd, VIDIOC_QUERYBUF, &buf))
        {
            printf ("VIDIOC_QUERYBUF error\n");
            return 0;
        }

        buffers[n_buf].len = buf.length;

        //通过mmap建立映射关系
        buffers[n_buf].start=mmap(NULL,buf.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,buf.m.offset);

        if (MAP_FAILED == buffers[n_buf].start)
        {
            printf ("mmap failed\n");
            return 0;
        }
    }

    //把缓冲帧放入缓冲队列
    //VIDIOC_QBUF：  把帧放入队列
    //VIDIOC_DQBUF： 从队列中取出帧
    for (int i = 0; i < req.count; ++i)
    {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;

        //申请到的缓冲进入列队
        if (-1 == ioctl (fd, VIDIOC_QBUF, &buf))
        {
            printf ("VIDIOC_QBUF failed\n");
            return 0;
        }
    }

    //启动数据流
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl (fd, VIDIOC_STREAMON, &type))
    {
        perror("start stream faild\n");
        return 0;
    }
    return 1;
}

/*
 * @brief 读取一帧
 * @param fd 摄像头设备的文件描述符
 * @return 成功 一帧图像数据
 * 		   失败 直接退出程序
 */

struct buffer v4l2_getImage(int fd)//struct buffer* buffers
{
    int ff1;
    struct v4l2_buffer buf;

    memset(&buf,0,sizeof(buf));
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;

    //出列采集的帧缓冲
    int n = ioctl (fd, VIDIOC_DQBUF, &buf);
    if (n < 0)
    {
        printf("failture VIDIOC_DQBUF\n");
        exit(0);
    }

//    QImage image;
//    image.loadFromData((uchar*)buffers[buf.index].start, buffers[buf.index].len);

//    image = image.scaled(ui->label->size());
//    ui->label->setPixmap(QPixmap::fromImage(image));

    struct buffer ret_buffer;

    ret_buffer.len = buffers[buf.index].len;
    ret_buffer.start = buffers[buf.index].start;

    //重新入列
    n = ioctl(fd, VIDIOC_QBUF, &buf);
    if (n < 0)
    {
        printf("failture VIDIOC_QBUF\n");
        exit(0);
    }
    return ret_buffer;
}