#include "proc.hpp"
#include "common.hpp"
#include "CraterNav.hpp"

using namespace std;
using namespace Eigen;

double cos_delta_th = cos(10*M_PI/180); //设置相似三角形成立角度cos阈值-10°
double repro_th= 3;
unsigned long long t_1;
unsigned long long t_2;
unsigned long long t_3;
unsigned long long t_4;

int readCSV_mat(string file_name, int* mat_shape, double* data){
	ifstream csv_file(file_name); //打开文件

	bool first_line = true;
	int var_cnt = 0;
	int word_cnt = 0;
	int data_pos = 0;
	string line; //缓存行
	string word; //缓存词
	while(getline(csv_file,line)){
		if (first_line){
			first_line = false;
			continue; //跳过第一行
		}

		word_cnt = 0; 
		istringstream in(line);
		while(getline(in, word, ',')){
			switch(word_cnt){
			case 0:
				word_cnt++;
				continue; //跳过第一列
			case 1:
				mat_shape[var_cnt*2] = stoi(word); //写入变量行数
				break;
			case 2:
				mat_shape[var_cnt*2+1] = stoi(word); //写入变量列数
				break;
			default:
				data[data_pos+word_cnt-3] = stod(word); //写入矩阵数据
			} 				
			word_cnt++;

		}
		var_cnt++;
		data_pos += word_cnt-3;

	}

	return var_cnt;
}

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

int sector(double delta_u, double delta_v){
	if( (delta_u<0) && (delta_v>=0) )
		return 1;
	
	if( (delta_u>=0) && (delta_v>=0) )
		return 2;

	if( (delta_u>=0) && (delta_v<0) )
		return 3;

	return 4;

}

void SecPos_feature(MatrixXd points2D, int* secPos){
	double delta_u[2], delta_v[2];
	int sec_1,sec_2;

	if(points2D.rows() != 3 || points2D.cols() != 2){
		cout<<"Error: SecPos Feature must take 3x2 matrix as input!"<<endl;
		return;
	}

	//遍历三个三角形顶点，获得三个相对位置的特征量
	delta_u[0] = points2D(0,0) - points2D(1,0);
	delta_u[1] = points2D(0,0) - points2D(2,0);

	delta_v[0] = points2D(0,1) - points2D(1,1);
	delta_v[1] = points2D(0,1) - points2D(2,1);

	sec_1 = sector(delta_u[0],delta_v[0]);
	sec_2 = sector(delta_u[1],delta_v[1]);

	//确保点顺序为逆时针
	if ((sec_1 == 1 && sec_2 == 4)||(sec_1 == 4 && sec_2 == 1)){
		secPos[0] = 4;
		secPos[1] = 1;
	}
	else if (sec_1 <= sec_2){
		secPos[0] = sec_1;
		secPos[1] = sec_2;
	}
	else{
		secPos[0] = sec_2;
		secPos[1] = sec_1;
	}

	delta_u[0] = points2D(1,0) - points2D(0,0);
	delta_u[1] = points2D(1,0) - points2D(2,0);

	delta_v[0] = points2D(1,1) - points2D(0,1);
	delta_v[1] = points2D(1,1) - points2D(2,1);

	sec_1 = sector(delta_u[0],delta_v[0]);
	sec_2 = sector(delta_u[1],delta_v[1]);

	//确保点顺序为逆时针
	if ((sec_1 == 1 && sec_2 == 4)||(sec_1 == 4 && sec_2 == 1)){
		secPos[2] = 4;
		secPos[3] = 1;
	}
	else if(sec_1 <= sec_2){
		secPos[2] = sec_1;
		secPos[3] = sec_2;
	}
	else{
		secPos[2] = sec_2;
		secPos[3] = sec_1;
	}

	delta_u[0] = points2D(2,0) - points2D(0,0);
	delta_u[1] = points2D(2,0) - points2D(1,0);

	delta_v[0] = points2D(2,1) - points2D(0,1);
	delta_v[1] = points2D(2,1) - points2D(1,1);

	sec_1 = sector(delta_u[0],delta_v[0]);
	sec_2 = sector(delta_u[1],delta_v[1]);

	//确保点顺序为逆时针
	if ((sec_1 == 1 && sec_2 == 4)||(sec_1 == 4 && sec_2 == 1)){
		secPos[4] = 4;
		secPos[5] = 1;
	}
	else if (sec_1 <= sec_2){
		secPos[4] = sec_1;
		secPos[5] = sec_2;
	}
	else{
		secPos[4] = sec_2;
		secPos[5] = sec_1;
	}

	return;

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

int craterNav(void){
	cout<<"###craterNav Begin###"<<endl;
	MatrixXd T_cal(3,1);
	MatrixXd Point_pairs(100,5);
	int pair_num = 0;
	int i,j,k;
	MatrixXd Points2D_tmp(3,2);

	/***********************读取csv数据，赋值给对应的Matrix变量**************************/
	int mat_shape[20];
	double data[6000];
	//读取文件1
	int val_cnt = readCSV_mat("../data/crater30_match_20240823.csv", mat_shape, data);

	//将读取的csv数据赋值给Matrix
	int ptr_bias = 0;
	// K矩阵
	MatrixXd K_cam(mat_shape[0],mat_shape[1]);
	memcpy(K_cam.data(), data+ptr_bias, sizeof(double)*K_cam.size());
	ptr_bias += K_cam.size();

	// R矩阵
	MatrixXd R_cam(mat_shape[2],mat_shape[3]);
	memcpy(R_cam.data(), data+ptr_bias, sizeof(double)*R_cam.size());
	ptr_bias += R_cam.size();

	// T矩阵
	MatrixXd T_cam(mat_shape[4],mat_shape[5]);
	memcpy(T_cam.data(), data+ptr_bias, sizeof(double)*T_cam.size());
	ptr_bias += T_cam.size();

	// triangle_lib矩阵
	MatrixXd triangle_lib(mat_shape[6],mat_shape[7]);
	memcpy(triangle_lib.data(), data+ptr_bias, sizeof(double)*triangle_lib.size());
	ptr_bias += triangle_lib.size();

	// valid_lib矩阵
	MatrixXd valid_lib(mat_shape[8],mat_shape[9]);
	memcpy(valid_lib.data(), data+ptr_bias, sizeof(double)*valid_lib.size());
	ptr_bias += valid_lib.size();

	// crater_extract矩阵
	MatrixXd crater_extract(mat_shape[10],mat_shape[11]);
	memcpy(crater_extract.data(), data+ptr_bias, sizeof(double)*crater_extract.size());
	ptr_bias += crater_extract.size();


	//读取文件2
	val_cnt = readCSV_mat("../data/libCrater_20240829.csv", mat_shape, data);
	ptr_bias = 0;
	// crater3D矩阵-陨坑库三维信息
	MatrixXd crater3D(mat_shape[0],mat_shape[1]);
	memcpy(crater3D.data(), data+ptr_bias, sizeof(double)*crater3D.size());
	ptr_bias += crater3D.size();
	// 陨坑库三角形排列
	MatrixXd libCraterTri(mat_shape[2],mat_shape[3]);
	memcpy(libCraterTri.data(), data+ptr_bias, sizeof(double)*libCraterTri.size());
	ptr_bias += libCraterTri.size();

	//读取文件3
	val_cnt = readCSV_mat("../data/crater_PCA_20240829.csv", mat_shape, data);
	ptr_bias = 0;
	// 提取的陨坑信息
	MatrixXd crater_PCA(mat_shape[0],mat_shape[1]);
	memcpy(crater_PCA.data(), data+ptr_bias, sizeof(double)*crater_PCA.size());
	ptr_bias += crater_PCA.size();
	// 提取陨坑的峰值信息
	MatrixXd PCA_peakVal(mat_shape[2],mat_shape[3]);
	memcpy(PCA_peakVal.data(), data+ptr_bias, sizeof(double)*PCA_peakVal.size());
	ptr_bias += PCA_peakVal.size();

	t_1 = get_time_ms();
	/***************陨坑匹配算法-陨坑库三角形构建*****************/
	// 陨坑库中三角形投影至像面
	MatrixXd lib_project = project2D(K_cam,R_cam,T_cam, crater3D);
	MatrixXd libTri(libCraterTri.rows(), 14);
	double cos_feature[3], sin_feature[3], edge_len[3], edge_prop[3], tri_center[2];

	for (i=0;i<libCraterTri.rows();i++){
		Points2D_tmp.row(0) = lib_project.row(libCraterTri(i,0));
		Points2D_tmp.row(1) = lib_project.row(libCraterTri(i,1));
		Points2D_tmp.row(2) = lib_project.row(libCraterTri(i,2));

		//构建三角形基本属性
		triBuild_base(Points2D_tmp,cos_feature,sin_feature,edge_len,tri_center);
		//获得边长比
		triBuild_edgeProp(edge_len, edge_prop);

		libTri(i,0) = libCraterTri(i,0); //三角序号a-最大角
		libTri(i,1) = libCraterTri(i,1); //三角序号b-次大角
		libTri(i,2) = libCraterTri(i,2); //三角序号c-最小角
		libTri(i,3) = cos_feature[0]; //cos(a)
		libTri(i,4) = cos_feature[1]; //cos(b)
		libTri(i,5) = cos_feature[2]; //cos(c)
		libTri(i,6) = edge_len[0]; //三角形边长bc
		libTri(i,7) = edge_len[1]; //三角形边长ac
		libTri(i,8) = edge_len[2]; //三角形边长ab
		libTri(i,9) = edge_prop[0]; //三角形边长比   ac/bc
		libTri(i,10) = edge_prop[1]; //三角形边长比2  ab/bc
		libTri(i,11) = edge_prop[2]; //三角形边长比3  ab/ac
		libTri(i,12) = tri_center[0]; //三角形中心点u
		libTri(i,13) = tri_center[1]; //三角形中心点v
	}
	t_2 = get_time_ms();

	/*************************从PCA提取的陨坑中找到5个陨坑以构成三角形********************************/
	int PCA_ix[PCA_peakVal.rows()], dist_ix[PCA_peakVal.rows()];
	double dist[PCA_peakVal.rows()];
	int center_PCAcrater;

	//生成索引顺序数
	for(i=0;i<PCA_peakVal.rows();i++) {
		PCA_ix[i] = i;
		dist_ix[i] = i;
	}
	//对峰值进行排序
	sort(PCA_ix,PCA_ix+PCA_peakVal.rows(), [&](const int& cmp_a, const int& cmp_b){ return PCA_peakVal(cmp_a,0)>PCA_peakVal(cmp_b,0);});

	center_PCAcrater = PCA_ix[0];

	for (i=0;i<crater_PCA.rows();i++){
		dist[i] = fabs(crater_PCA(i,0) - crater_PCA(center_PCAcrater,0))+fabs(crater_PCA(i,1) - crater_PCA(center_PCAcrater,1));
	}

	// 对与主要峰值点的距离进行排序
	sort(dist_ix,dist_ix+crater_PCA.rows(), [&](const int& cmp_a, const int& cmp_b){ return dist[cmp_a]<dist[cmp_b];});


	/********************************匹配陨坑*************************************/
	MatrixXd PCA_cand(5,2);
	MatrixXi travel_ix(10,3);
	int sort_ix[3];
	int travel_cnt = 0;
	MatrixXd match_tri(500,7);
	double delta_cos, edge_len_tmp[3];
	int pointInside,match_cnt = 0;

	PCA_cand.row(0) = crater_PCA.row(dist_ix[0]);
	PCA_cand.row(1) = crater_PCA.row(dist_ix[1]);
	PCA_cand.row(2) = crater_PCA.row(dist_ix[2]);
	PCA_cand.row(3) = crater_PCA.row(dist_ix[3]);
	PCA_cand.row(4) = crater_PCA.row(dist_ix[4]);

	//获得所有可能的 三角形序号组合
	for (i=0;i<3;i++){
		for(j=i+1;j<4;j++){
			for(k=j+1;k<5;k++){
				travel_ix(travel_cnt,0) = i;
				travel_ix(travel_cnt,1) = j;
				travel_ix(travel_cnt,2) = k;
				travel_cnt++;
			}
		}
	}

	for(i=0;i<travel_cnt;i++){
		//*****************构建三角形**************************
		Points2D_tmp.row(0) = PCA_cand.row(travel_ix(i,0));
		Points2D_tmp.row(1) = PCA_cand.row(travel_ix(i,1));
		Points2D_tmp.row(2) = PCA_cand.row(travel_ix(i,2));

		//判断内部有无陨坑
		pointInside = triBulid_pointInside(Points2D_tmp,PCA_cand);
		if (pointInside != -1)
			continue;


		triBuild_base(Points2D_tmp,cos_feature,sin_feature,edge_len,tri_center);
		triBuild_degSort(Points2D_tmp,cos_feature,sort_ix);	

		if (cos_feature[0] < cos(100 *M_PI/180))
			continue;

		//根据sort_ix调整edge排序
		edge_len_tmp[0] = edge_len[0];
		edge_len_tmp[1] = edge_len[1];
		edge_len_tmp[2] = edge_len[2];

		edge_len[0] = edge_len_tmp[sort_ix[0]];
		edge_len[1] = edge_len_tmp[sort_ix[1]];
		edge_len[2] = edge_len_tmp[sort_ix[2]];

		if (edge_len[2]<10)
			continue;

		//获得边长比
		triBuild_edgeProp(edge_len, edge_prop);

		//***************与陨坑库内三角形进行相似判断********************
		for (j=0;j<libTri.rows();j++){
			delta_cos = fabs(libTri(j,3)-cos_feature[0]) + fabs(libTri(j,4)-cos_feature[1]) + fabs(libTri(j,5)-cos_feature[2]);

			if(delta_cos<0.03){
				//满足条件，加入待选的三角形序列
				match_tri(match_cnt,0) = (double)dist_ix[travel_ix(i,0)];
				match_tri(match_cnt,1) = (double)dist_ix[travel_ix(i,1)];
				match_tri(match_cnt,2) = (double)dist_ix[travel_ix(i,2)];
				match_tri(match_cnt,3) = (double)libTri(j,0);
				match_tri(match_cnt,4) = (double)libTri(j,1);
				match_tri(match_cnt,5) = (double)libTri(j,2);

				match_tri(match_cnt,6) = delta_cos;

				match_cnt++;
			}

		}

	}

	t_3 = get_time_ms();

	MatrixXd match_tri_vld = match_tri.block(0,0,match_cnt,7);

	cout<<match_tri_vld<<endl;

	copy(dist_ix,dist_ix+5,ostream_iterator<int>(cout,","));

	cout<<center_PCAcrater<<endl;
	cout<<"匹配三角形个数: "<<match_cnt<<endl;
	cout<<"lib triangle build time:"<<t_2-t_1<<"ms"<<endl;
	cout<<"t3-t2:"<<t_3-t_2<<"ms"<<endl;
	// cout<<"reproject validation time:"<<t_4-t_3<<"ms"<<endl;
	cout<<"total time:"<<t_3-t_1<<"ms"<<endl;

	return 1;
}

