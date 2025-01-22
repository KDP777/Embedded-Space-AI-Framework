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
	this->integral += err;
	item_i = this->ki*this->integral*ctl_T;

	//D分量
	item_d = iter_cnt==0? 0:(this->kd*(err-err_pre))/ctl_T;

	u_output = item_p+item_i+item_d;

	err_pre2 = err_pre;
	err_pre = err;
	iter_cnt++;

	return u_output;

}


PID_OrbitCtl::PID_OrbitCtl( double kp_Tx, double ki_Tx, double kd_Tx, 
							double kp_Ty, double ki_Ty, double kd_Ty, 
							double kp_Tz, double ki_Tz, double kd_Tz, 
							double gm_i, double delta_t, double _lmt
							){
	t = delta_t;
	gm = gm_i;
	lmt = _lmt;

	v_est = VectorXd(3);
	a_est = VectorXd(3);

	a_est.setZero();
	v_est.setZero();

	Tx_PID = PID(kp_Tx,ki_Tx,kd_Tx,delta_t);
	Ty_PID = PID(kp_Ty,ki_Ty,kd_Ty,delta_t);
	Tz_PID = PID(kp_Tz,ki_Tz,kd_Tz,delta_t);
}

PID_OrbitCtl::~PID_OrbitCtl(){
}

int PID_OrbitCtl::compute_va(VectorXd setpoint, VectorXd cur_T, VectorXd cur_v){
	double deltaX,deltaY,deltaZ;
	double deltaVx,deltaVy,deltaVz;
	VectorXd a_T(3),a_V(3);
	VectorXd T_err(3);

	VectorXd gm_uint = cur_T/(cur_T.norm());

	//通过T计算加速度a_T
	deltaX = this->Tx_PID.compute(setpoint(0), cur_T(0));
	deltaY = this->Ty_PID.compute(setpoint(1), cur_T(1));
	deltaZ = this->Tz_PID.compute(setpoint(2), cur_T(2));

	a_T(0) =  2*(deltaX-cur_v(0)*this->t)/(this->t*this->t);
	a_T(1) =  2*(deltaY-cur_v(1)*this->t)/(this->t*this->t);
	a_T(2) =  2*(deltaZ-cur_v(2)*this->t)/(this->t*this->t);

	T_err = cur_T - setpoint;

	this->a_est<< a_T(0),a_T(1),a_T(2); 

	this->v_est = cur_v + this->a_est*this->t;

	this->a_est = this->a_est - gm_uint*this->gm;

	for(int i=0;i<3;i++){
		if (this->a_est(i)>lmt){
			this->a_est(i) = lmt;
			continue;
		}

		if (this->a_est(i)<-lmt){
			this->a_est(i) = -lmt;
		}

	}

	return 1;
}