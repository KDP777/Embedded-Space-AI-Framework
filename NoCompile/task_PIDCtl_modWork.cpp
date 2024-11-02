#include "proc.hpp"
#include "common.hpp"
#include "PID_ctl.hpp"
#include "redundency.hpp"

using namespace std;
using namespace Eigen;

VectorXd v_cur(3);
VectorXd T_cur(3);
VectorXd T_set(3); 

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


int PIDCtl_getData(unsigned long t, double Tx_set, double Ty_set, double Tz_set, double* T_init, double* v_init, int init_flg){
	VectorXd a_read(3);
	T_set<<Tx_set,Ty_set,Tz_set; //设置跟随轨道位置

	//设置时间-ms转化为s为单位
	double t_s = (double)t/1000;
	
	if (init_flg){ //初始化飞行器运动状态
		v_cur<<v_init[0],v_init[1],v_init[2];
		T_cur<<T_init[0],T_init[1],T_init[2];
	}
	else{
		//获取PID计算的加速度值
		a_read<<threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[9],1024),
				threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[10],1024),
				threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[11],1024);
		//运动估计
		move_simulate(T_cur, v_cur, a_read, t_s, T_cur.data(), v_cur.data());
	}

	cout<<T_set-T_cur<<endl;
	cout<<endl;

	//将结果写入共享内存
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[0],T_set(0),1024);
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[1],T_set(1),1024);
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[2],T_set(2),1024);

	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[3],T_cur(0),1024);
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[4],T_cur(1),1024);
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[5],T_cur(2),1024);

	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[6],v_cur(0),1024);
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[7],v_cur(1),1024);
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[8],v_cur(2),1024);

	return 1;

}

int PIDCtl_estimate(unsigned long t, PID_OrbitCtl* pid_controller){
	//从共享内存获取模块需要导入的数据
	VectorXd Tset_in(3); Tset_in<<threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[0],1024),
								  threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[1],1024),
								  threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[2],1024);

	VectorXd Tcur_in(3); Tcur_in<<threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[3],1024),
								  threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[4],1024),
								  threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[5],1024);

	VectorXd vcur_in(3); vcur_in<<threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[6],1024),
								  threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[7],1024),
								  threeChooseTwo_read<double>(&shmPtr.ptr_dataHub[8],1024);

	pid_controller->t = (double)t/1000; //由ms转化为s为单位
	pid_controller->compute_va(Tset_in,Tcur_in,vcur_in);

	//将解算的加速度a放入共享内存中
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[9],pid_controller->a_est(0),1024);
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[10],pid_controller->a_est(1),1024);
	threeChooseTwo_write<double>(&shmPtr.ptr_dataHub[11],pid_controller->a_est(2),1024);

	return 1;
}