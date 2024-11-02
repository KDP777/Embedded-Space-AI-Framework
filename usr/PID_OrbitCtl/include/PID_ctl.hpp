#ifndef __PID_ctl__
#define __PID_ctl__

#include <iostream>
#include "common.hpp"

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

		~PID();

		double compute(double setpoint, double measure);

	private:
		double err_pre;
		double err_pre2;
		double integral;
};

class PID_OrbitCtl : public PID
{
	public:
		PID_OrbitCtl(double kp_i, double ki_i, double kd_i, double gm_i, double ctl_T_=1);
		~PID_OrbitCtl();

		int compute_va(VectorXd setpoint, VectorXd cur_T, VectorXd cur_v);

		double t;
		double gm;
		VectorXd v_est;
		VectorXd a_est;

};

//运行模块函数声明
int PIDCtl_getData(unsigned long t, double Tx_set, double Ty_set, double Tz_set, double* T_init, double* v_init, int init_flg);
int PIDCtl_estimate(unsigned long t, PID_OrbitCtl* pid_controller);

#endif //__PID_ctl__