#include "redundency.hpp"

//描述：三取二读数据-float
//输入：arg1-起始指针
//	    arg2-写入数值
//		arg3-地址偏移
//输出：读取的数值
template<>
float threeChooseTwo_read<float>(float* stratPtr, int add_bias){
	int bias_T = add_bias>>2 ;

	float SEU_result;

	float D1 = *stratPtr;
	float D2 = *(stratPtr+bias_T);
	float D3 = *(stratPtr+bias_T+bias_T);

	if( (D1 == D2 ) || (D1 == D3) ){
		return D1;
	}
	else if(D2 == D3){
		cout<<"ThreeChooseTwo read warning: Single Event Upsets!"<<endl;
		return D2;
	}
	else{
		cout<<"ThreeChooseTwo<float> read Error: three numbers all different!"<<endl;
		return D3;
	}
}

//描述：三取二读数据-double
//输入：arg1-起始指针
//	    arg2-写入数值
//		arg3-地址偏移
//输出：读取的数值
template<>
double threeChooseTwo_read<double>(double* stratPtr, int add_bias){
	int bias_T = add_bias>>3;

	double SEU_result;

	double D1 = *stratPtr;
	double D2 = *(stratPtr+bias_T);
	double D3 = *(stratPtr+bias_T+bias_T);

	if( (D1 == D2 ) || (D1 == D3) ){
		return D1;
	}
	else if(D2 == D3){
		cout<<"ThreeChooseTwo read warning: Single Event Upsets!"<<endl;
		return D2;
	}
	else{
		cout<<"ThreeChooseTwo<double> read Error: three numbers all different!"<<endl;
		return D3;
	}
}