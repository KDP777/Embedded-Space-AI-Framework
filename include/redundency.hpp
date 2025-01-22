#ifndef __REDUN__
#define __REDUN__

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

//描述：三取二写数据
//输入：arg1-共享内存指针
//	    arg2-写入数据指针
//      arg3-数据长度(当量：字节)
//		arg4-三取二地址偏移
//输出：无
template <typename T>
void threeChooseTwo_write(void* shmPtr, void* src_data, int add_bias, int len=sizeof(T)){

	char* shmPtr1 = (char*)shmPtr;
	char* shmPtr2 = shmPtr1+add_bias;
	char* shmPtr3 = shmPtr2+add_bias;

	//保存三份数据
	memcpy(shmPtr1, src_data, len);
	memcpy(shmPtr2, src_data, len);
	memcpy(shmPtr3, src_data, len);

	return;
}

template <typename T>
void threeChooseTwo_write(void* shmPtr, T src_data, int add_bias, int len=sizeof(T)){

	char* shmPtr1 = (char*)shmPtr;
	char* shmPtr2 = shmPtr1+add_bias;
	char* shmPtr3 = shmPtr2+add_bias;

	//保存三份数据
	memcpy(shmPtr1, (void *)(&src_data), len);
	memcpy(shmPtr2, (void *)(&src_data), len);
	memcpy(shmPtr3, (void *)(&src_data), len);

	return;
}

//描述：三取二读数据
template <typename T>
T* threeChooseTwo_read(void* shmPtr, int add_bias, int len=sizeof(T)){
	char* shmPtr1 = (char*)shmPtr;
	char* shmPtr2 = shmPtr1+add_bias;
	char* shmPtr3 = shmPtr2+add_bias;

	if( memcmp(shmPtr1, shmPtr2, len) == 0 ){
		return (T*)shmPtr; //1,2相等
	}
	else if(memcmp(shmPtr1, shmPtr3, len) == 0){
		return (T*)shmPtr; //1,3相等
	}
	else{
		// 2,3相等返回2,否则返回第一个指针的数
		return (memcmp(shmPtr2, shmPtr3, len) == 0?  (T*)shmPtr2: (T*)shmPtr); 
	}

	return 0;

}

#endif  //__REDUN__
