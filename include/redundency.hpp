#ifndef __REDUNDENCY__
#define __REDUNDENCY__

#include "common.hpp"

using namespace std;

//描述：三取二写数据
//输入：arg1-起始指针
//	    arg2-写入数值
//		arg3-地址偏移
//输出：成功输出1，失败输出-1
template <typename T>
int threeChooseTwo_write(T* stratPtr, T val, int add_bias){
	int bias_T;
	switch(sizeof(T)){
	case 1:
		bias_T = add_bias;
		break;
	case 2:
		bias_T = add_bias>>1;
		break;
	case 4:
		bias_T = add_bias>>2;
		break;
	case 8:
		bias_T = add_bias>>3;
		break;
	case 16:
		bias_T = add_bias>>4;
		break;
	case 32:
		bias_T = add_bias>>5;
		break;
	default:
		bias_T = add_bias/sizeof(T);
	}

	T* stratPtr_bias1 = (T*)(stratPtr+bias_T);
	T* stratPtr_bias2 = (T*)(stratPtr+bias_T+bias_T);

	*stratPtr = val;
	*stratPtr_bias1 = val;
	*stratPtr_bias2 = val;

	return 1;
}

//描述：三取二读数据-所有整型
//输入：arg1-起始指针
//	    arg2-写入数值
//		arg3-地址偏移
//输出：读取的数值
template <typename T>
T threeChooseTwo_read(T* stratPtr, int add_bias){
	int bias_T;
	switch(sizeof(T)){
	case 1:
		bias_T = add_bias;
		break;
	case 2:
		bias_T = add_bias>>1;
		break;
	case 4:
		bias_T = add_bias>>2;
		break;
	case 8:
		bias_T = add_bias>>3;
		break;
	case 16:
		bias_T = add_bias>>4;
		break;
	case 32:
		bias_T = add_bias>>5;
		break;
	default:
		bias_T = add_bias/sizeof(T);
	}

	T SEU_result;

	T* stratPtr_bias1 = (T*)(stratPtr+bias_T);
	T* stratPtr_bias2 = (T*)(stratPtr+bias_T+bias_T);

	T D1 = *stratPtr;
	T D2 = *(stratPtr_bias1);
	T D3 = *(stratPtr_bias2);

	// 三个数都相同
	if( (D1 == D2 ) || (D1 == D3) ){
		return D1;
	}
	else{
		cout<<"ThreeChooseTwo read warning: Single Event Upsets!"<<endl;
		SEU_result = ((D1&D2)|(D1&D3)|(D2&D3));
		return SEU_result;
	}

}

template<>
float threeChooseTwo_read<float>(float* stratPtr, int add_bias); //threeChooseTwo_read的float特例化函数

template<>
double threeChooseTwo_read<double>(double* stratPtr, int add_bias); //threeChooseTwo_read的double特例化函数

#endif  //__REDUNDENCY__
