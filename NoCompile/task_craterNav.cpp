#include "proc.hpp"
#include "common.hpp"

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

void triCosSin_feature(MatrixXd points2D, double* cos_feature, double* sin_feature){
	if(points2D.rows() != 3 || points2D.cols() != 2){
		cout<<"Error: Cos Feature must take 3x2 matrix as input!"<<endl;
		return;
	}

	VectorXd vec_ab = points2D.row(1) - points2D.row(0);
	VectorXd vec_ac = points2D.row(2) - points2D.row(0);
	VectorXd vec_bc = points2D.row(2) - points2D.row(1);

	double ab_norm = vec_ab.norm();
	double ac_norm = vec_ac.norm();
	double bc_norm = vec_bc.norm();

	cos_feature[0] = vec_ab.dot(vec_ac)/(ab_norm*ac_norm);
	cos_feature[1] = -vec_ab.dot(vec_bc)/(ab_norm*bc_norm);
	cos_feature[2] = vec_ac.dot(vec_bc)/(ac_norm*bc_norm);

	sin_feature[0] = sqrt(1-cos_feature[0]*cos_feature[0]);
	sin_feature[1] = sqrt(1-cos_feature[1]*cos_feature[1]);
	sin_feature[2] = sqrt(1-cos_feature[2]*cos_feature[2]);

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

	/***********************读取csv数据，赋值给对应的Matrix变量**************************/
	int mat_shape[20];
	double data[500];
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

	t_1 = get_time_ms();
	/***************陨坑匹配算法-陨坑库三角形构建*****************/
	// 陨坑库中三角形投影至像面
	MatrixXd lib_project = project2D(K_cam,R_cam,T_cam, triangle_lib);

	// 三角形顶点象限位置特征生成
	int lib_vertexPos[6];
	SecPos_feature(lib_project, lib_vertexPos);

	// 构建cos角度特征
	double lib_cos[3],lib_sin[3],lib_degree[3];
	triCosSin_feature(lib_project, lib_cos, lib_sin);

	//计算角度分支
	// lib_degree[0] = acos(lib_cos[0]);
	// lib_degree[1] = acos(lib_cos[1]);
	// lib_degree[2] = acos(lib_cos[2]);

	t_2 = get_time_ms();
	/*************陨坑匹配算法-遍历提取陨坑构筑的三角形************/
	int i,j,k;
	int sec_tmp[6], points_match[3];
	double delta_u[2],delta_v[2], img_cos[3], img_sin[3], img_degree[3], cos_delta[3];
	MatrixXd img_tri(3,2);
	int candidate_num = 0;
	MatrixXd candidate_tri(500,4);
	bool relPos_flag = false;

	// 遍历序号获取
	int travel_num = crater_extract.rows()*(crater_extract.rows()-1)*(crater_extract.rows()-2)/6;
	MatrixXi travel_ix(travel_num,3);

	int travel_cnt = 0;
	for (i=0;i<crater_extract.rows()-2;i++){
		for(j=i+1;j<crater_extract.rows()-1;j++){
			for(k=j+1;k<crater_extract.rows();k++){
				travel_ix(travel_cnt,0) = i;
				travel_ix(travel_cnt,1) = j;
				travel_ix(travel_cnt,2) = k;
				travel_cnt += 1;
			}
		}
	}

	// 遍历构筑三角形
	for (i=0;i<travel_num;i++){

		relPos_flag = true;
		img_tri(0,0) = crater_extract(travel_ix(i,0),0);
		img_tri(0,1) = crater_extract(travel_ix(i,0),1);
		img_tri(1,0) = crater_extract(travel_ix(i,1),0);
		img_tri(1,1) = crater_extract(travel_ix(i,1),1);
		img_tri(2,0) = crater_extract(travel_ix(i,2),0);
		img_tri(2,1) = crater_extract(travel_ix(i,2),1);

		//获得该三角形三点的相对位置
		SecPos_feature(img_tri,sec_tmp);

		//先对三个定点的相对位置进行判断，与库中三角形相对位置一致才继续运算，否则不构筑三角形
		if ((lib_vertexPos[0] == sec_tmp[0]) && (lib_vertexPos[1] == sec_tmp[1])){ //lib1 -> tmp1

			if((lib_vertexPos[2] == sec_tmp[2]) && (lib_vertexPos[3] == sec_tmp[3])){ //lib2-> tmp2
				if((lib_vertexPos[4] == sec_tmp[4]) && (lib_vertexPos[5] == sec_tmp[5])){ //lib3 -> tmp3
					relPos_flag = false;
					points_match[0] = 0;
					points_match[1] = 1;
					points_match[2] = 2;
				}
			}
			else if((lib_vertexPos[2] == sec_tmp[4]) && (lib_vertexPos[3] == sec_tmp[5])){ //lib2 -> tmp3
				if((lib_vertexPos[4] == sec_tmp[2]) && (lib_vertexPos[5] == sec_tmp[3])){ //lib3 -> tmp2
					relPos_flag = false;
					points_match[0] = 0;
					points_match[1] = 2;
					points_match[2] = 1;
				}
			}
		}
		else if ((lib_vertexPos[0] == sec_tmp[2]) && (lib_vertexPos[1] == sec_tmp[3])){ //lib1 -> tmp2
			if((lib_vertexPos[2] == sec_tmp[0]) && (lib_vertexPos[3] == sec_tmp[1])){ //lib2 -> tmp 1
				if((lib_vertexPos[4] == sec_tmp[4]) && (lib_vertexPos[5] == sec_tmp[5])){ //lib3 -> tmp3
					relPos_flag = false;
					points_match[0] = 1;
					points_match[1] = 0;
					points_match[2] = 2;
				}
			}
			else if((lib_vertexPos[2] == sec_tmp[4]) && (lib_vertexPos[3] == sec_tmp[5])){ //lib2 ->tmp3
				if((lib_vertexPos[4] == sec_tmp[0]) && (lib_vertexPos[5] == sec_tmp[1])){ //lib3 -> tmp1
					relPos_flag = false;
					points_match[0] = 1;
					points_match[1] = 2;
					points_match[2] = 0;
				}

			}

		}
		else if ((lib_vertexPos[0] == sec_tmp[4]) && (lib_vertexPos[1] == sec_tmp[5])){ //lib1 -> tmp3
			if((lib_vertexPos[2] == sec_tmp[0]) && (lib_vertexPos[3] == sec_tmp[1])){ //lib2 -> tmp 1
				if((lib_vertexPos[4] == sec_tmp[2]) && (lib_vertexPos[5] == sec_tmp[3])){ //lib3 -> tmp2
					relPos_flag = false;
					points_match[0] = 2;
					points_match[1] = 0;
					points_match[2] = 1;
				}
			}
			else if((lib_vertexPos[2] == sec_tmp[2]) && (lib_vertexPos[3] == sec_tmp[3])){ //lib2 ->tmp2
				if((lib_vertexPos[4] == sec_tmp[0]) && (lib_vertexPos[5] == sec_tmp[1])){ //lib3 -> tmp1
					relPos_flag = false;
					points_match[0] = 2;
					points_match[1] = 1;
					points_match[2] = 0;
				}

			}

		}
		else{
			relPos_flag = true;
		}

		if (relPos_flag)
			continue;

		//构筑三角形内角特征
		triCosSin_feature(img_tri, img_cos, img_sin);

		//计算角度分支
		// img_degree[0]= acos(img_cos[0]);
		// img_degree[1]= acos(img_cos[1]);
		// img_degree[2]= acos(img_cos[2]);

		// cos_delta[0] = fabs(img_degree[0] - lib_degree[0])*180/M_PI;
		// cos_delta[1] = fabs(img_degree[1] - lib_degree[1])*180/M_PI;
		// cos_delta[2] = fabs(img_degree[2] - lib_degree[2])*180/M_PI;


		// //判据-三角形内角相似
		// if ((cos_delta[0] < 10) && (cos_delta[1] <10) && (cos_delta[2] < 10)){
		// 	candidate_tri(candidate_num,0) = (double)i;
		// 	candidate_tri(candidate_num,1) = (double)j;
		// 	candidate_tri(candidate_num,2) = (double)k;
		// 	candidate_tri(candidate_num,3) = cos_delta[0]+cos_delta[1]+cos_delta[2];

		// 	candidate_num++;
		// }

		// 直接使用cos分支
		cos_delta[0] = img_cos[points_match[0]]*lib_cos[0]+img_sin[points_match[0]]*lib_sin[0];
		cos_delta[1] = img_cos[points_match[1]]*lib_cos[1]+img_sin[points_match[1]]*lib_sin[1];
		cos_delta[2] = img_cos[points_match[2]]*lib_cos[2]+img_sin[points_match[2]]*lib_sin[2];


		//判据-三角形内角相似
		if ((cos_delta[0] > cos_delta_th) && (cos_delta[1] > cos_delta_th) && (cos_delta[2] > cos_delta_th)){
			candidate_tri(candidate_num,0) = (double)travel_ix(i,points_match[0]);
			candidate_tri(candidate_num,1) = (double)travel_ix(i,points_match[1]);
			candidate_tri(candidate_num,2) = (double)travel_ix(i,points_match[2]);
			candidate_tri(candidate_num,3) = cos_delta[0]+cos_delta[1]+cos_delta[2];

			candidate_num++;
		}


	}

	t_3 = get_time_ms();

	/*************陨坑匹配算法-重投影误差验证三角形匹配对************/
	int cos_ix[candidate_num]; 
	MatrixXd points2D(3,2);
	MatrixXd repro_uv(3,2);
	MatrixXd repro_err(3,2);
	int repro_i;
	double repro_err_sum, min_reproErr = 1000000;
	int match_ix = 0;

	//生成索引顺序数
	for(i=0;i<candidate_num;i++) cos_ix[i] = i;

	//对delta_cos这个特征的值进行排序,输出排序的索引
	sort(cos_ix,cos_ix+candidate_num, [&](const int& cmp_a, const int& cmp_b){ return candidate_tri(cmp_a,3)>candidate_tri(cmp_b,3);});

	for(repro_i=0;repro_i<candidate_num;repro_i++){
		repro_err_sum = 0;

		points2D.row(0) = crater_extract.row( candidate_tri(cos_ix[repro_i],0) );
		points2D.row(1) = crater_extract.row( candidate_tri(cos_ix[repro_i],1) );
		points2D.row(2) = crater_extract.row( candidate_tri(cos_ix[repro_i],2) );
		//解算位置
		MatrixXd T_tmp = PNP_solveT(K_cam,R_cam,points2D,triangle_lib);

		//重投影
		repro_uv = project2D(K_cam,R_cam,T_tmp,triangle_lib);
		repro_err = repro_uv-points2D;
		repro_err_sum += repro_err.row(0).norm();
		repro_err_sum += repro_err.row(1).norm();
		repro_err_sum += repro_err.row(2).norm();

		if( repro_err_sum < min_reproErr){
			min_reproErr = repro_err_sum;
			T_cal = T_tmp;
			match_ix = cos_ix[repro_i];
		}

	}

	/*******************输出匹配点对*******************/
	Point_pairs.row(0) << crater_extract.row(candidate_tri(match_ix,0)), triangle_lib.row(0);
	Point_pairs.row(1) << crater_extract.row(candidate_tri(match_ix,1)), triangle_lib.row(1);
	Point_pairs.row(2) << crater_extract.row(candidate_tri(match_ix,2)), triangle_lib.row(2);

	t_4 = get_time_ms();

	cout<<"lib triangle build time:"<<t_2-t_1<<"ms"<<endl;
	cout<<"allround triangle build time:"<<t_3-t_2<<"ms"<<endl;
	cout<<"reproject validation time:"<<t_4-t_3<<"ms"<<endl;
	cout<<"total time:"<<t_4-t_1<<"ms"<<endl;
	cout<<T_cal<<endl;

	return 1;
}

