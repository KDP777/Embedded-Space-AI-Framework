#include "common.hpp"
#include "CraterNav.hpp"

using namespace std;
using namespace Eigen;

MatrixXd project2D(MatrixXd K_cam, MatrixXd R_cam, MatrixXd	T_cam, MatrixXd	points3D){
	MatrixXd KR = K_cam*R_cam;
	MatrixXd points2D(points3D.rows(),2);
	MatrixXd points3D_item(3,1);
	MatrixXd T_31;
	MatrixXd xyz_cam(3,1);
	int i;

	if (T_cam.rows()==3)
		T_31 = T_cam;
	else
		T_31 = T_cam.transpose();


	for (i=0;i<points3D.rows();i++){
		points3D_item(0,0) = points3D(i,0);
		points3D_item(1,0) = points3D(i,1);
		points3D_item(2,0) = points3D(i,2);

		xyz_cam = KR*(points3D_item-T_31);

		points2D(i,0) = xyz_cam(0,0)/xyz_cam(2,0);
		points2D(i,1) = xyz_cam(1,0)/xyz_cam(2,0);
	}

	return points2D;
}


MatrixXd PNP_solveT(MatrixXd K, MatrixXd R, MatrixXd points2D, MatrixXd points3D){
	MatrixXd A_mat(points2D.rows()*2, 3);
	MatrixXd AT_mat(3, points2D.rows()*2);
	MatrixXd b_mat(points2D.rows()*2, 1);
	MatrixXd ATA(3,3);
	MatrixXd ATb(3,1);
	MatrixXd T_solve = MatrixXd::Zero(3,1);
	int i;

	if(points2D.rows()!=points3D.rows()){
		cout<< "PNP_SolveT Error: points2D and points3D must have the same rows!"<<endl;
		return T_solve;
	}

	if(points2D.rows()<2){
		cout<< "PNP_SolveT Error: points2D rows must larger than or equal to 2!"<<endl;
		return T_solve;
	}

	//计算K*R矩阵
	MatrixXd KR = K*R;

	//构建A b矩阵
	for(i=0; i<points2D.rows(); i++){
		A_mat(i*2, 0) = points2D(i,0)*KR(2,0)-KR(0,0);
		A_mat(i*2, 1) = points2D(i,0)*KR(2,1)-KR(0,1);
		A_mat(i*2, 2) = points2D(i,0)*KR(2,2)-KR(0,2);

		b_mat(i*2,0) = (points2D(i,0)*KR(2,0)-KR(0,0))*points3D(i,0) +
					   (points2D(i,0)*KR(2,1)-KR(0,1))*points3D(i,1) +
					   (points2D(i,0)*KR(2,2)-KR(0,2))*points3D(i,2);

		A_mat(i*2+1, 0) = points2D(i,1)*KR(2,0) - KR(1,0);
		A_mat(i*2+1, 1) = points2D(i,1)*KR(2,1) - KR(1,1);
		A_mat(i*2+1, 2) = points2D(i,1)*KR(2,2) - KR(1,2);

		b_mat(i*2+1,0) = (points2D(i,1)*KR(2,0)-KR(1,0))*points3D(i,0) +
						 (points2D(i,1)*KR(2,1)-KR(1,1))*points3D(i,1) +
						 (points2D(i,1)*KR(2,2)-KR(1,2))*points3D(i,2);

	}

	//通过求逆求解Ax=b的线性问题，使用最小二乘方法
	AT_mat = A_mat.transpose();
	ATA = AT_mat*A_mat;
	ATb = AT_mat*b_mat;
	T_solve = ATA.inverse()*ATb;

	return T_solve;

}