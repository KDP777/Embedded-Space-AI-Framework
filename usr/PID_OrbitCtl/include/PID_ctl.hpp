#ifndef __PID_ctl__
#define __PID_ctl__

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Core> //添加Eigen库
#include <Eigen/Dense> //稠密矩阵的计算

using namespace Eigen;

//位置式PID实现
class PID
{
	public:
		int integrate_num;
		double kp;
		double ki;
		double kd;
		double ctl_T;
		double iter_cnt;

		PID(double kp_i, double ki_i, double kd_i, double ctl_T_=1);
		PID(void){}; //设置一个空的初始化函数

		~PID();

		double compute(double setpoint, double measure);

	private:
		double err_pre;
		double err_pre2;
		double integral;
};

class PID_OrbitCtl
{
	public:
		PID_OrbitCtl( double kp_Tx, double ki_Tx, double kd_Tx, 
					  double kp_Ty, double ki_Ty, double kd_Ty, 
					  double kp_Tz, double ki_Tz, double kd_Tz, 
					  double gm_i, double delta_t, double _lmt=100000
					);

		~PID_OrbitCtl();

		int compute_va(VectorXd setpoint, VectorXd cur_T, VectorXd cur_v);

		PID Tx_PID;
		PID Ty_PID;
		PID Tz_PID;

		PID Vx_PID;
		PID Vy_PID;
		PID Vz_PID;

		double wT;
		double wV;
		double t;
		double gm;
		VectorXd v_est;
		VectorXd a_est;
		double lmt;

};

//运行模块函数声明
int PIDCtl_getData(unsigned long t, double Tx_set, double Ty_set, double Tz_set, double* T_init, double* v_init, int init_flg);
int PIDCtl_estimate(unsigned long t, PID_OrbitCtl* pid_controller);

#endif //__PID_ctl__