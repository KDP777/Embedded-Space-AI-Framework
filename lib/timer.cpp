#include "common.hpp"

int timer_fd;

using namespace std;

int timer_init(void){
	timer_fd = open("/dev/timer_space",O_RDWR);

	if(timer_fd<0){
		cout<<"Open Timer for Space Failed!"<<endl;
	}

	return timer_fd;
}

int set_time_ms(unsigned long long time){
	int ret;

	ret = write(timer_fd, &time, 8);

	if(ret<0){
		cout<<"Timer Set Failed!"<<endl;
	}

	return ret;
}

unsigned long long get_time_ms(void){
	int ret;
	unsigned long long time;

	ret = read(timer_fd, &time, 8);

	if(ret<0){
		cout<<"Timer Set Failed!"<<endl;
		return -1;
	}

	return time;
}

void delay_ms(unsigned long ms){
	int ret;
	ret = ioctl(timer_fd, 0, ms*1000);

	if(ret < 0){
		cout<<"Delay Error:Please check the input!"<<endl;
	}

	return;
}

void Wait2Time_ms(unsigned long ms){
	int ret;
	//加保护
	if(ms<get_time_ms()){
		cout<<"Wait2Time_ms:input time is too small!"<<endl;
		return;
	}

	ret = ioctl(timer_fd, 1, ms);

	if(ret<0){
		cout<<"Wait2Time Error: Please check the input time!"<<endl;
	}

	return;
}
