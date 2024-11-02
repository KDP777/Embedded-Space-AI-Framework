/*用户的使用模板-套用PID_OrbitCtl的Demo；
  实际编写用户程序请参考以下格式；
*/

#include "common.hpp"
#include "proc.hpp"
#include "PID_ctl.hpp"
#include "rapidcsv.h"
#include "redundency.hpp"

using namespace std;
using namespace Eigen;

/**************算法模块参数设置-配置项********************/
//算法模块总数量
#define MOD_NUM 2

//算法循环节拍相关参数
unsigned long time_sched; //绝对时间-调度任务使用
pulse Pulse = 
{
	100,		//Pulse.time 节拍间隔时间/ms
 	0,			//Pulse.cnt 初始节拍数
 	90,	//Pulse.MaxWaitT 单节拍最大等待时间/ms
};

/**************具体算法函数需要使用的全局变量-配置项******************/
vector<double> Tx_follow_set(80000); //跟随的标称位置
vector<double> Ty_follow_set(80000);
vector<double> Tz_follow_set(80000);
vector<double> vx_IMU(80000); //模仿惯导输出的速度
vector<double> vy_IMU(80000);
vector<double> vz_IMU(80000);

double v_init[3]; //起始速度
double T_init[3]; //起始位置
VectorXd v_cur(3); //算法中间计算量-现在的速度
VectorXd T_cur(3); //算法中间计算量-现在的位置
VectorXd T_set(3); //算法中间计算量-设定的位置

PID_OrbitCtl pid_controller(0.6, 0.3, 0, 0.0098, 0.1); //初始化PID控制器

/******************************************************************/
/*
 * 描述：算法具体函数构造，根据不同项目编写
 * 输入：无
 * 输出：无
 */
void move_simulate(VectorXd T_0, VectorXd v_0, VectorXd a_0, double t, double* T_out, double* v_out){

	VectorXd gm_uint = T_0/(T_0.norm());
	VectorXd a = a_0 + gm_uint*0.0098;

	VectorXd T = T_0 + v_0*t + 0.5*t*t*a;
	VectorXd v = v_0 + a*t;

	T_out[0] = T(0);
	T_out[1] = T(1);
	T_out[2] = T(2);

	v_out[0] = v[0];
	v_out[1] = v[1];
	v_out[2] = v[2];

}

int Mod0_PIDCtlEstPos(unsigned long tickNum, unsigned long tickGap){
	VectorXd a_read(3);
	T_set<<Tx_follow_set[tickNum],Ty_follow_set[tickNum],Tz_follow_set[tickNum]; //设置跟随轨道位置

	//设置时间-ms转化为s为单位
	double t_s = ((double)tickGap*Pulse.time)/1000;

	//获取PID计算的加速度值
	a_read<<threeChooseTwo_read<double>(&shmPtr.DataHub[9],1024),
			threeChooseTwo_read<double>(&shmPtr.DataHub[10],1024),
			threeChooseTwo_read<double>(&shmPtr.DataHub[11],1024);
	//运动估计
	move_simulate(T_cur, v_cur, a_read, t_s, T_cur.data(), v_cur.data());

	cout<<tickNum<<endl;
	cout<<T_set-T_cur<<endl;
	cout<<endl;

	//将结果写入共享内存
	threeChooseTwo_write<double>(&shmPtr.DataHub[0],T_set(0),1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[1],T_set(1),1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[2],T_set(2),1024);

	threeChooseTwo_write<double>(&shmPtr.DataHub[3],T_cur(0),1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[4],T_cur(1),1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[5],T_cur(2),1024);

	threeChooseTwo_write<double>(&shmPtr.DataHub[6],v_cur(0),1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[7],v_cur(1),1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[8],v_cur(2),1024);

	return 1;

}

int Mod1_PIDCtlGetAcc(unsigned long tickNum, unsigned long tickGap){
	//从共享内存获取模块需要导入的数据
	VectorXd Tset_in(3); Tset_in<<threeChooseTwo_read<double>(&shmPtr.DataHub[0],1024),
								  threeChooseTwo_read<double>(&shmPtr.DataHub[1],1024),
								  threeChooseTwo_read<double>(&shmPtr.DataHub[2],1024);

	VectorXd Tcur_in(3); Tcur_in<<threeChooseTwo_read<double>(&shmPtr.DataHub[3],1024),
								  threeChooseTwo_read<double>(&shmPtr.DataHub[4],1024),
								  threeChooseTwo_read<double>(&shmPtr.DataHub[5],1024);

	VectorXd vcur_in(3); vcur_in<<threeChooseTwo_read<double>(&shmPtr.DataHub[6],1024),
								  threeChooseTwo_read<double>(&shmPtr.DataHub[7],1024),
								  threeChooseTwo_read<double>(&shmPtr.DataHub[8],1024);

	pid_controller.t = (double)(1*Pulse.time)/1000; //由ms转化为s为单位
	pid_controller.compute_va(Tset_in,Tcur_in,vcur_in);

	//将解算的加速度a放入共享内存中
	threeChooseTwo_write<double>(&shmPtr.DataHub[9],pid_controller.a_est(0),1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[10],pid_controller.a_est(1),1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[11],pid_controller.a_est(2),1024);

	return 1;
}

/*
 * 描述：算法部分初始化，主要针对算法函数需要使用的全局变量进行初始化
 * 输入：无
 * 输出：int，代表初始化是否成功
 */
int Algo_init(){
	//读取导航输入数据
	rapidcsv::Document csv_doc("/home/kdp-pi/Space_AI/data/StarLink-44235_44235_Fixed_Position_Velocity_part1.csv");
	vector<double> Tx_follow = csv_doc.GetColumn<double>("x (km)");
	vector<double> Ty_follow = csv_doc.GetColumn<double>("y (km)");
	vector<double> Tz_follow = csv_doc.GetColumn<double>("z (km)");
	vector<double> vx = csv_doc.GetColumn<double>("vx (km/sec)");
	vector<double> vy = csv_doc.GetColumn<double>("vy (km/sec)");
	vector<double> vz = csv_doc.GetColumn<double>("vz (km/sec)");

	copy(Tx_follow.begin(), Tx_follow.begin()+Tx_follow_set.size(), Tx_follow_set.begin());
	copy(Ty_follow.begin(), Ty_follow.begin()+Ty_follow_set.size(), Ty_follow_set.begin());
	copy(Tz_follow.begin(), Tz_follow.begin()+Tz_follow_set.size(), Tz_follow_set.begin());

	copy(vx.begin(), vx.begin()+vx_IMU.size(), vx_IMU.begin());
	copy(vy.begin(), vy.begin()+vy_IMU.size(), vy_IMU.begin());
	copy(vz.begin(), vz.begin()+vz_IMU.size(), vz_IMU.begin());

	v_cur<<vx_IMU[0],vy_IMU[0],vz_IMU[0]; //起始速度
	T_cur<<-2700,4500,4200; //起始位置

	//首帧加速度-0
	threeChooseTwo_write<double>(&shmPtr.DataHub[9],0,1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[10],0,1024);
	threeChooseTwo_write<double>(&shmPtr.DataHub[11],0,1024);

	return 1;

}

/*
 * 描述：各个算法模块相关前置任务的配置（在algorithm_moudle类下，pre_mod存储其他任务是否为当前任务前置任务的标识符，
 *			1代表前置任务，0代表非前置任务）
 * 输入：无
 * 输出：无
 */
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

/*
 * 描述：确认各个cpu的运行状态，确定故障的cpu调度逻辑，在每一次时间循环后调用；
 * 输入：算法循环次数，算法模块类的数组指针（algo_mod[0]代表模块0，algo_mod[1]代表模块1，...)
 * 输出：无
 */
void CPU_check(unsigned long vld_cnt, algorithm_module* algo_mod){ 
	unsigned long cnt[MOD_NUM];
	int i,cpuIter_i;

	for(i = 0;i<MOD_NUM;i++){
		cnt[i] = threeChooseTwo_read<unsigned long>(&shmPtr.ModCnt[i],1024);
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
		//设置cpu_status参数
		cpu_status[threeChooseTwo_read<int>(&shmPtr.ModCPU[0],1024)] = -1;
		//激活当前cpu的模块0
		algo_mod[0].activate(cnt[cpuIter_i], Pulse.MaxWaitT, Mod0_PIDCtlEstPos);

		cout<<"模块0 cpu失能"<<endl;
		return;
	}

	//分支3 - 模块1的cpu失能
	if(cpuIter_i == 1){
		//设置cpu_status参数
		cpu_status[threeChooseTwo_read<int>(&shmPtr.ModCPU[1],1024)] = -1;
		//激活当前cpu的模块1
		algo_mod[1].activate(cnt[cpuIter_i], Pulse.MaxWaitT, Mod1_PIDCtlGetAcc);

		cout<<"模块1 cpu失能"<<endl;
		return;
	}

	//分支4 - cpu全部失能！
	if(cnt[0] != vld_cnt && cnt[1] != vld_cnt){
		cout<<"Code Bug! It is Impossible branch"<<endl;
	}

	return;

}


/*
 * 描述：算法模块的调度主程序，包括了进程初始化，共享内存初始化，算法模块初始化，以及程序主体的循环
 * 输入：无
 * 输出：int 1代表正常，其他值代表故障
 */
int Process_sched(void){
	int i,j,ret; //遍历循环计数
	/********************** IPC共享内存初始化 ****************************/
	shmPtr.DataHub = (double*)(shmPtr.ModCPU+cpuNum);  //共享数据地址初始化, cpuNum为cpu核心数

	//设置算法模块起始计数为99999
	for(i=0;i<MOD_NUM;i++){
		threeChooseTwo_write<unsigned long>(&shmPtr.ModCnt[i],99999,1024);
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

	//激活算法模块-根据当前进程cpu赋值，
	/*各个算法模块的实际运行函数需要根据具体项目更改
	 *本demo使用Mod0_PIDCtlEstPos和Mod1_PIDCtlGetAcc
	 */
	switch(proc_modFlg){
	case 0:
		algo_mod[0].activate(99999,Pulse.MaxWaitT, Mod0_PIDCtlEstPos);
		break;
	case 1:
		algo_mod[1].activate(99999,Pulse.MaxWaitT, Mod1_PIDCtlGetAcc);
		break;
	default:
		algo_mod[0].activate(99999,Pulse.MaxWaitT, Mod0_PIDCtlEstPos);
	}

	/*********************** 主循环程序参数以及各模块初始化 *************************/
	ret = Algo_init(); //算法初始化

	if(!ret){
		cout<<"Algorithm Module Init Failed!"<<endl;
		exit(0);
	}

	/*********************** 主循环整体时序循环 *************************/
	/*
	 *进程间-时间对齐
	 */
	unsigned long ready_flg = 0;
	Pulse.cnt = 0; //初始化完成，节拍数置0
	while(1){ //等待其他合作进程初始化完成
		//遍历查询cpu_status标志位
		for(i=0;i<MOD_NUM;i++){
			ready_flg += threeChooseTwo_read<unsigned long>(&shmPtr.ModCnt[i],1024);
		}

		if(ready_flg == 0){
			//各cpu进程对齐，开始启动循环
			set_time_ms(0); //初始化校时
			time_sched = get_time_ms();
			break;
		}

		ready_flg = 0;
		algo_mod[proc_modFlg].iterNum_align(Pulse.cnt);//每一次查询结束后都置0
	}

	/*
	 *主循环
	 */ 
	while(1){
		//执行与cpu绑定的任务
		for(i=0;i<MOD_NUM;i++){
			if(algo_mod[i].runing_cpu==proc_cpuFlg){
				algo_mod[i].run(Pulse.cnt, time_sched);
			}
		}

		//每一拍加时间
		Pulse.cnt++; //节拍数＋1
		time_sched += Pulse.time; 
		Wait2Time_ms(time_sched);

		//一个循环节拍结束后，检查cpu运行情况
		CPU_check(Pulse.cnt, algo_mod);

		if(Pulse.cnt>79998) break;

	}

	return 1;
}