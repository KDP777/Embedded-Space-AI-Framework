#ifndef __CRATERNAV__
#define __CRATERNAV__

#include "common.hpp"

using namespace Eigen;

// 构建三角形的基本元素，角度cos值，角度sin值，三角形边长，以及三角形中心
extern void triBuild_base(MatrixXd points2D, double* cos_feature, double* sin_feature, double* Edge_len, double* tri_center);
// 以各个点对应的角度进行排序，【角度最大，角度次大，角度最小】
extern void triBuild_degSort(MatrixXd points2D, double* cos_feature, int* sort_ix);
// 获得三角形三个边的对应边长比
extern void triBuild_edgeProp(double* edge_len, double* edge_prop);
// 判断三角形内是否有其他点, 
// 返回-1，无其他点，返回序号，则说明checkPoints该序号对应的点在三角形内
extern int triBulid_pointInside(MatrixXd TriPoints, MatrixXd checkPoints);

extern int ExternCrater_vld(VectorXd Tri_img, VectorXd Tri_lib, 
						MatrixXd ExternCrater_img, MatrixXd ExternCrater_lib, 
						double* th, double* ExCrater, int ExCrater_numTh);

//库三角形构建
MatrixXd libTri_rebuild(MatrixXd K_cam, MatrixXd R_cam, MatrixXd T_cam, MatrixXd crater3D, MatrixXd lib_project, MatrixXd libCraterTri, MatrixXd max_dist);
//像面三角形构建
MatrixXd ImgTri_build(MatrixXd crater_PCA, MatrixXd PCA_peakVal, MatrixXd _craterSet_num, MatrixXd deg_limit, MatrixXd edgeLen_limit, double* group_center);
//三角形匹配
MatrixXd Tri_match(MatrixXd ImgTri_out,MatrixXd libTri,MatrixXd group_center,MatrixXd max_dist_group,MatrixXd cos_th,MatrixXd k_diffTh);
//重投影误差
MatrixXd Reproject_vld(MatrixXd match_tri, MatrixXd crater_PCA,MatrixXd crater3D,MatrixXd K_cam,MatrixXd R_cam,MatrixXd th_reproj_, double* vldcnt_sort);
//重投影误差验证2
MatrixXd Reproject_vld_2(MatrixXd match_tri, MatrixXd crater_PCA,MatrixXd crater3D,MatrixXd K_cam,MatrixXd R_cam,MatrixXd th_reproj_, double* vldcnt_sort);


//IC验证坑检验
MatrixXd IC_vld(MatrixXd matchTri_cand,MatrixXd ImgTri, MatrixXd LibTri, 
			MatrixXd crater_PCA, MatrixXd lib_project, MatrixXd vldCrater_th, int Tri_num);


/*************utils(VisionMeasurement)*****************/
extern MatrixXd project2D(MatrixXd K_cam, MatrixXd R_cam, MatrixXd	T_cam, MatrixXd	points3D);
extern MatrixXd PNP_solveT(MatrixXd K, MatrixXd R, MatrixXd points2D, MatrixXd points3D);

#endif //__CRATERNAV__
