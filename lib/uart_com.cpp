/**************串口通讯用户使用示例****************/
//char recv_buff[20];
///串口测试
//uart_com ttyS1("/dev/ttyS1");
//
//ttyS1.uart_send("hello, Linux!");
//
////	while(1){
////		cout<<"Hello Pthread!"<<endl;
////		sleep(5);
////	}
//
//sleep(10);
//
//ttyS1.uart_recv(recv_buff, sizeof(recv_buff));
//
//printf("GoodBye Linux!\r\n");

/***********************************************************/


#include "uart_com.hpp"

//串口相关标志位定义
const tcflag_t uart_data_bits_5 = CS5; //5位数据位
const tcflag_t uart_data_bits_6 = CS6; //6位数据位
const tcflag_t uart_data_bits_7 = CS7; //7位数据位
const tcflag_t uart_data_bits_8 = CS8; //8位数据位
const tcflag_t uart_parity_none = ~PARENB; //无校验位
const tcflag_t uart_parity_en = PARENB; //有校验位
const tcflag_t uart_parity_odd = PARODD; //奇校验
const tcflag_t uart_parity_even = ~PARODD; //偶校验
const tcflag_t uart_stop_bits_2 = CSTOPB; //两位停止位
const tcflag_t uart_stop_bits_1 = ~CSTOPB; //一位停止位

/*
 * 描述：以默认参数配置串口
 * 参数：串口设备文件路径
 * 返回：无
 */
uart_com::uart_com(const char* _dev_name){
	struct termios options;

	dev_name = _dev_name;
	// 打开串口设备
//	uart_fd = open(dev_name, O_RDWR | O_NOCTTY); //阻塞模式
	uart_fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY); //非阻塞模式
    if (uart_fd == -1) {
        perror("open_port: Unable to open serial port - ");
        return;
    }

    /*
     * 默认串口配置：
     * 波特率 115200,
     * 8位数据位，
     * 无校验位，
     * 1位停止位
     */
    baudrate = B115200;
    data_bits = uart_data_bits_8;
    parity =  uart_parity_none;
    stop_bits = uart_stop_bits_1;

    // 获取并配置串口选项
	tcgetattr(uart_fd, &options);
	cfsetispeed(&options, baudrate); // 输入波特率
	cfsetospeed(&options, baudrate); // 输出波特率

	options.c_cflag |= (CLOCAL | CREAD); // 开启接收
	options.c_cflag &= ~CSIZE; // 清除当前数据位设置

	options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
//	tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control

	options.c_lflag &= ~ECHO; // Disable echo
	options.c_lflag &= ~ECHOE; // Disable erasure
	options.c_lflag &= ~ECHONL; // Disable new-line echo

//	options.c_lflag &= ~ICANON; // Disable Canonical mode

	options.c_cflag |= data_bits;    // 8位数据位

	switch(parity)
	{
		case uart_parity_none:
			options.c_cflag &= parity; // 关闭校验位
			break;
		case uart_parity_even:
			options.c_cflag |= uart_parity_en; // 开启校验位
			options.c_cflag &= parity; // 设置校验位
			options.c_iflag |= (INPCK|ISTRIP);
			break;
		case uart_parity_odd:
			options.c_cflag |= uart_parity_en; // 开启校验位
			options.c_cflag |= parity; // 设置校验位
			options.c_iflag |= (INPCK|ISTRIP);
			break;
		default:
			options.c_cflag &= parity; // 关闭校验位
			break;
	}

	options.c_cflag &= stop_bits; // 1位停止位

	//阻塞模式-超时时间和最小读取字节设置
	options.c_cc[VTIME] = 0; // 超时时间,单位0.1s
	options.c_cc[VMIN] = 0; // 读取字符的最小数目

	// 清空串口
	tcflush(uart_fd,TCIFLUSH);

	tcsetattr(uart_fd, TCSANOW, &options);

}

/*
 * 描述：手动参数配置串口
 * 参数：串口设备文件路径，以及串口配置参数
 * 返回：无
 */
uart_com::uart_com(const char* _dev_name, int _baudrate, tcflag_t _data_bits, tcflag_t _parity, tcflag_t _stop_bits){
	struct termios options;

	dev_name = _dev_name;
	// 打开串口设备
//	uart_fd = open(dev_name, O_RDWR | O_NOCTTY); //阻塞模式
	uart_fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY); //非阻塞模式
    if (uart_fd == -1) {
        perror("open_port: Unable to open serial port - ");
        return;
    }

    /*
     * 默认串口配置：
     * 波特率 115200,
     * 8位数据位，
     * 无校验位，
     * 1位停止位
     */
    baudrate = _baudrate;
    data_bits = _data_bits;
    parity =  _parity;
    stop_bits = _stop_bits;

    // 获取并配置串口选项
	tcgetattr(uart_fd, &options);
	cfsetispeed(&options, baudrate); // 输入波特率
	cfsetospeed(&options, baudrate); // 输出波特率

	options.c_cflag |= (CLOCAL | CREAD); // 开启接收
	options.c_cflag &= ~CSIZE; // 清除当前数据位设置

	options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
//	tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control

	options.c_lflag &= ~ECHO; // Disable echo
	options.c_lflag &= ~ECHOE; // Disable erasure
	options.c_lflag &= ~ECHONL; // Disable new-line echo

//	options.c_lflag &= ~ICANON; // Disable Canonical mode

	options.c_cflag |= data_bits;    // 8位数据位

	switch(parity)
	{
		case uart_parity_none:
			options.c_cflag &= parity; // 关闭校验位
			break;
		case uart_parity_even:
			options.c_cflag |= uart_parity_en; // 开启校验位
			options.c_cflag &= parity; // 设置校验位
			options.c_iflag |= (INPCK|ISTRIP);
			break;
		case uart_parity_odd:
			options.c_cflag |= uart_parity_en; // 开启校验位
			options.c_cflag |= parity; // 设置校验位
			options.c_iflag |= (INPCK|ISTRIP);
			break;
		default:
			options.c_cflag &= parity; // 关闭校验位
			break;
	}

	options.c_cflag &= stop_bits; // 1位停止位

	//阻塞模式-超时时间和最小读取字节设置
	options.c_cc[VTIME] = 0; // 超时时间,单位0.1s
	options.c_cc[VMIN] = 0; // 读取字符的最小数目

	// 清空串口
	tcflush(uart_fd,TCIFLUSH);

	tcsetattr(uart_fd, TCSANOW, &options);

}

uart_com::~uart_com(){
	close(this->uart_fd);
}

int uart_com::uart_send(const char* send_message){
	int send_len;

	send_len = write(this->uart_fd, send_message, strlen(send_message));
    if (send_len == -1) {
        perror("uart_port_send: Unable to write to serial port - ");
    }

    return send_len;
}

int uart_com::uart_recv(char* buffer, int buffer_len){
	int recv_len;

	recv_len = read(this->uart_fd, buffer, buffer_len);
    if (recv_len == -1) {
        perror("uart_port_receive: Unable to read from serial port - ");
    } else {
        buffer[recv_len] = '\0'; // 确保字符串以null结尾
        printf("Received: %s", buffer);
    }

    return recv_len;
}


