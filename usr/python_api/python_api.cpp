#include "common.hpp"
#include "proc.hpp"
#include "rapidcsv.h"
#include "redundency.hpppp"

using namespace std;
using namespace Eigen;

/**********************参数设置************************/
//记录算法模块总数量
#define MOD_NUM 1

//节拍相关参数
unsigned long time_sched; //绝对等待时间
pulse Pulse = 
{
	1000,		//Pulse.time 单节拍对应的时间/ms
 	0,			//Pulse.cnt 初始节拍数
 	1000-10,	//Pulse.MaxWaitT 单节拍拍最大等待时间/ms
};


/**********算法相关全局变量*******/
PyObject* pModule;
PyObject* pFunc;

/*******************前置任务配置**********************/
void algorithm_module::set_preMod(void){
	switch(this->mod_flg){
	case 0:
		return; //无前置任务
	case 1:
		this->pre_mod[0] = 1; //模块0为前置任务
	default:
		return; //无前置任务
	}
	return;
}

/*******************算法函数-请根据算法设计调整**********************/
int task_python(unsigned long tickNum,unsigned long tickGap){
	double a = 16.33;

	/******调用Python函数*****/
	//传递给python的输入参数
	PyObject* func_args = Py_BuildValue("(i)", 5);

	PyObject* Py_ret = PyObject_CallObject(pFunc, func_args);

	/******解析返回结果*******/
	// //解析单个数值
	// double ret_val;
	// PyArg_Parse(Py_ret,"d",&ret_val);
	// cout<<ret_val<<endl;

	//解析多个数值
	// double ret_val1,ret_val2;
	// PyArg_ParseTuple(Py_ret,"d|d",&ret_val1,&ret_val2);
	// cout<<ret_val1<<" "<<ret_val2<<endl;

	//解析一个二维数组
	Py_ssize_t size_1 = PyObject_Size(Py_ret); //得到行数或列数

	PyObject* iter = PyObject_GetIter(Py_ret);
	PyObject* row1 = PyIter_Next(iter);

	PyObject* iter2 = PyObject_GetIter(row1);
	PyObject* data = PyIter_Next(iter2);
	cout <<PyFloat_Check(data)<<endl;

	double ret_val;
	PyArg_Parse(data,"d",&ret_val);
	cout<<ret_val<<endl;

	return 1;
}

/**********************实际运行算法初始化-请根据算法设计调整*************************/
int Algo_init(){
	PyRun_SimpleString("import sys"); //加载sys库
	PyRun_SimpleString("sys.path.append('/home/kdp-pi/Space_AI/usr/python_demo/python_script')"); //添加需要python脚本文件所在目录

	/***********载入python模块文件以及函数************/
	pModule = PyImport_ImportModule("test_kdp"); //加载模板，即python脚本文件名
	if (pModule == NULL) {
		cout << "python module not found!" << endl;
		return 0;
	}

	pFunc = PyObject_GetAttrString(pModule, "test_func"); //加载模块中的具体函数
	if(!pFunc || !PyCallable_Check(pFunc)){
		cout << "python module function not found!" << endl;
		return 0;
	}

	return 1;

}

/*********************CPU故障调度函数************************/
void CPU_check(unsigned long vld_cnt, algorithm_module* algo_mod){ 
	unsigned long cnt[MOD_NUM];
	int i,cpuIter_i;

	for(i = 0;i<MOD_NUM;i++){
		cnt[i] = *(threeChooseTwo_read<unsigned long>(&shmPtr.ModCnt[i],REDUN_BIAS));
	}
	
	// 分支1 - 各模块工作正常分支
	bool all_correct = true;
	for(cpuIter_i=0;cpuIter_i<MOD_NUM;cpuIter_i++){
		if(cnt[cpuIter_i] != vld_cnt){
			all_correct = false;
			break;
		}
	}

	if(all_correct==true) return;

	//分支2 - 模块0的cpu失能
	if(cpuIter_i == 0){
		//空

		return;
	}

	//分支3 - 模块1的cpu失能
	if(cpuIter_i == 1){
		//空
		
		return;
	}

	//分支4 - 模块1和2的cpu失能
	if(cnt[0] != vld_cnt && cnt[1] != vld_cnt){
		cout<<"Code Bug! It is Impossible branch"<<endl;
	}

	return;

}


/***************************时序调度函数-用户程序的主函数*****************************/
//important: 该函数名需与proc.hpp中__proc_entry__定义的变量相同
int Process_sched(void){
	int i,j; //遍历循环计数
	/********************** IPC共享内存初始化 ****************************/
	shmPtr.DataHub = (double*)(shmPtr.ModCPU+20);  //共享数据地址初始化, 20可以改成cpu数量

	//设置算法模块起始有效标识符为99999
	for(i=0;i<MOD_NUM;i++){
		threeChooseTwo_write<unsigned long>(&shmPtr.ModCnt[i],99999,REDUN_BIAS);
	}

	/********************** 运行算法模块相关初始化 ***************************/
	//创建MOD_NUM个算法模块
	algorithm_module algo_mod[MOD_NUM];
	for(i=0;i<MOD_NUM;i++){ //算法模块参数初始化
		//设置算法模块标识
		algo_mod[i].mod_flg = i;
		algo_mod[i].total_modNum = MOD_NUM;

		//前置任务初始化
		for(int j=0;j<MOD_NUM;j++){
			algo_mod[i].pre_mod[j] = 0;
		}
	}

	//激活算法模块
	switch(proc_modFlg){
	case 0:
		algo_mod[0].activate(0,Pulse.MaxWaitT, task_python);
		break;
	case 1:
		algo_mod[1].activate(0,Pulse.MaxWaitT, task_python);
		break;
	default:
		algo_mod[0].activate(0,Pulse.MaxWaitT, task_python);
	}

	/*********************** 主循环程序参数以及各模块初始化 *************************/
	Algo_init(); //算法初始化

	/*********************** 主循环整体时序循环 *************************/
	//1. 进程间-时间对齐
	unsigned long ready_flg = 0;
	Pulse.cnt = 1; //初始化完成，节拍数置0
	while(1){ //等待其他合作进程初始化完成
		//遍历查询cpu_status标志位
		for(i=0;i<MOD_NUM;i++){
			ready_flg += threeChooseTwo_read<unsigned long>(&shmPtr.ModCnt[i],REDUN_BIAS);
		}

		if(ready_flg == MOD_NUM){
			//流程对齐，开始启动运算
			set_time_ms(0); //初始化校时
			time_sched = get_time_ms();
			break;
		}

		ready_flg = 0;
		algo_mod[proc_modFlg].iterNum_align(Pulse.cnt);//每一次查询结束后都置0
	}

	//2. 主循环
	while(1){
		
		//执行与cpu绑定的任务
		for(i=0;i<MOD_NUM;i++){
			if(algo_mod[i].runing_cpu>=0){
				algo_mod[i].run(Pulse.cnt, time_sched);
			}
		}

		//每一拍加时间
		Pulse.cnt++; //节拍数＋1
		time_sched += Pulse.time; 
		Wait2Time_ms(time_sched);
		cout<<get_time_ms()<<endl;

		//一个循环节拍结束后，检查运行情况
		CPU_check(Pulse.cnt, algo_mod);
		break;//只运行一次

	}

	return 1;
}