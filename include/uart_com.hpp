#ifndef __UART_COM__
#define __UART_COM__

#include "common.hpp"
#include <termios.h>  //termios结构体，以及相关函数

//配置串口的相关变量
extern const tcflag_t uart_data_bits_5;
extern const tcflag_t uart_data_bits_6;
extern const tcflag_t uart_data_bits_7;
extern const tcflag_t uart_data_bits_8;
extern const tcflag_t uart_parity_none;
extern const tcflag_t uart_parity_en;
extern const tcflag_t uart_parity_odd;
extern const tcflag_t uart_parity_even;
extern const tcflag_t uart_stop_bits_2;
extern const tcflag_t uart_stop_bits_1;

class uart_com {
public:
	/*
	 * 描述：建立串口设备，并采用默认配置
	 * 输入：串口设备名
	 */
    /*
     * 默认串口配置：
     * 波特率 115200,
     * 8位数据位，
     * 无校验位，
     * 1位停止位
     */
	uart_com(const char* _dev_name);
	/*
	 * 描述：建立串口设备，并采用输入的串口配置项
	 * 输入：串口设备名，波特率，数据位，奇/偶/无校验，停止位
	 */
	uart_com(const char* _dev_name, int _baudrate, tcflag_t _data_bits, tcflag_t _parity, tcflag_t _stop_bits);
	~uart_com();

	/*
	 * 描述：串口发送
	 * 输入： 所需发送的信息
	 * 输出：发送是否成功标志
	 */
	int uart_send(const char* send_message);
	/*
	 * 描述：串口接收
	 * 输入： 接收串口的char数组，buffer数组的长度
	 * 输出：收到的数组长度
	 */
	int uart_recv(char* buffer, int buffer_len);

	const char* dev_name; //串口设备名
	int uart_fd; //设备id
	int baudrate; //波特率
	tcflag_t data_bits; //数据位
	tcflag_t parity; //校验位
	tcflag_t stop_bits; //停止位

};

#endif // __UART_COM__
