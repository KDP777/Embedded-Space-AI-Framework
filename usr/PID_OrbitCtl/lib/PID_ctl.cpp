#include "PID_ctl.hpp"

using namespace std;
using namespace Eigen;

PID::PID(double kp_i, double ki_i, double kd_i, double ctl_T_){
	//对比例，积分，微分系数赋值
	kp = kp_i;
	ki = ki_i;
	kd = kd_i;
	ctl_T = ctl_T_;

	err_pre = 0;
	err_pre2 = 0;
	integral = 0;
	iter_cnt = 0;
}

PID::~PID(){}

// 位置式PID
double PID::compute(double setpoint, double measure){
	double err = setpoint - measure;
	double item_p,item_i,item_d,u_output;
	double int_err,diff_err;
	int i,pre_ix;

	// P分量
	item_p = this->kp*(err);

	//I分量
	integral += err;
	item_i = this->ki*integral*ctl_T;

	//D分量
	item_d = iter_cnt==0? 0:(this->kd*(err-err_pre))/ctl_T;

	u_output = item_p+item_i+item_d;

	err_pre2 = err_pre;
	err_pre = err;
	iter_cnt++;

	return u_output;

}


PID_OrbitCtl::PID_OrbitCtl(double kp_i, double ki_i, double kd_i, double gm_i, double delta_t):PID(kp_i,ki_i,kd_i){
	t = delta_t;
	gm = gm_i;	

	v_est = VectorXd(3);
	a_est = VectorXd(3);

	a_est.setZero();
	v_est.setZero();

}

PID_OrbitCtl::~PID_OrbitCtl(){
	this->~PID();
}

int PID_OrbitCtl::compute_va(VectorXd setpoint, VectorXd cur_T, VectorXd cur_v){
	double deltaX,deltaY,deltaZ;

	VectorXd gm_uint = cur_T/(cur_T.norm());

	deltaX = this->compute(setpoint(0), cur_T(0));
	deltaY = this->compute(setpoint(1), cur_T(1));
	deltaZ = this->compute(setpoint(2), cur_T(2));

	this->a_est(0) =  2*(deltaX-cur_v(0)*this->t)/(this->t*this->t);
	this->a_est(1) =  2*(deltaY-cur_v(1)*this->t)/(this->t*this->t);
	this->a_est(2) =  2*(deltaZ-cur_v(2)*this->t)/(this->t*this->t);

	this->v_est = cur_v + this->a_est*this->t;

	this->a_est = this->a_est - gm_uint*gm;

	return 1;
}