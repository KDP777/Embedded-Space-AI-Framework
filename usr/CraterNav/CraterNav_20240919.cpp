#include "common.hpp"
#include "proc.hpp"
#include "CraterNav.hpp"
#include "rapidcsv.h"
#include "redundency.hpp"

using namespace std;
using namespace Eigen;

/**********************参数设置************************/
//记录算法模块总数量
#define MOD_NUM 1

//节拍相关参数
unsigned long time_sched; //绝对等待时间
pulse Pulse = 
{
	1000,		//Pulse.time 单节拍对应的时间/ms
 	0,			//Pulse.cnt 初始节拍数
 	1000-10,	//Pulse.MaxWaitT 单节拍拍最大等待时间/ms
};


/***********算法相关全局变量-请根据算法设计进行调整**********/
unsigned long long t1;
unsigned long long t2;
unsigned long long t3;
unsigned long long t4;
unsigned long long t5;
unsigned long long t6;
unsigned long long t7;
unsigned long long t8;
unsigned long long t9;
unsigned long long t10;


/*******************前置任务配置**********************/
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

/*******************算法函数-请根据算法设计调整**********************/
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

int craterNav(unsigned long tickNum, unsigned long tickGap){
	MatrixXd T_cal(3,1);
	MatrixXd Point_pairs(100,5);
	int pair_num = 0;
	MatrixXd Points2D_tmp(3,2);

	/***********************读取csv数据，赋值给对应的Matrix变量**************************/
	int mat_shape[20];
	double data[200000];
	//读取文件1
	int val_cnt = readCSV_mat("./data/libTri_20240924_approach1_25.csv", mat_shape, data);

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

	// 陨坑库三维位置矩阵
	MatrixXd crater3D(mat_shape[6],mat_shape[7]);
	memcpy(crater3D.data(), data+ptr_bias, sizeof(double)*crater3D.size());
	ptr_bias += crater3D.size();

	// 预存库三角形矩阵
	MatrixXd libCraterTri(mat_shape[8],mat_shape[9]);
	memcpy(libCraterTri.data(), data+ptr_bias, sizeof(double)*libCraterTri.size());
	ptr_bias += libCraterTri.size();

	// 建库阈值
	MatrixXd max_dist(mat_shape[10],mat_shape[11]);
	memcpy(max_dist.data(), data+ptr_bias, sizeof(double)*max_dist.size());
	ptr_bias += max_dist.size();


	//读取文件2
	val_cnt = readCSV_mat("./data/ImgTri_20240924_approach1_25.csv", mat_shape, data);
	ptr_bias = 0;
	// PCA模板提取的陨坑位置
	MatrixXd crater_PCA(mat_shape[0],mat_shape[1]);
	memcpy(crater_PCA.data(), data+ptr_bias, sizeof(double)*crater_PCA.size());
	ptr_bias += crater_PCA.size();
	// PCA模板提取陨坑峰值
	MatrixXd PCA_peakVal(mat_shape[2],mat_shape[3]);
	memcpy(PCA_peakVal.data(), data+ptr_bias, sizeof(double)*PCA_peakVal.size());
	ptr_bias += PCA_peakVal.size();
	//邻近坑数量
	MatrixXd _craterSet_num(mat_shape[4],mat_shape[5]);
	memcpy(_craterSet_num.data(), data+ptr_bias, sizeof(double)*_craterSet_num.size());
	ptr_bias += _craterSet_num.size();
	//角度阈值
	MatrixXd deg_limit(mat_shape[6],mat_shape[7]);
	memcpy(deg_limit.data(), data+ptr_bias, sizeof(double)*deg_limit.size());
	ptr_bias += deg_limit.size();
	//边长阈值
	MatrixXd edgeLen_limit(mat_shape[8],mat_shape[9]);
	memcpy(edgeLen_limit.data(), data+ptr_bias, sizeof(double)*edgeLen_limit.size());
	ptr_bias += edgeLen_limit.size();
	//距离中心陨坑点距离阈值
	MatrixXd max_dist_group(mat_shape[10],mat_shape[11]);
	memcpy(max_dist_group.data(), data+ptr_bias, sizeof(double)*max_dist_group.size());
	ptr_bias += max_dist_group.size();

	/***************陨坑匹配算法*****************/

	//Matlab和C++序号补偿
	libCraterTri.col(0) = libCraterTri.col(0)-MatrixXd::Ones(libCraterTri.rows(),1);
	libCraterTri.col(1) = libCraterTri.col(1)-MatrixXd::Ones(libCraterTri.rows(),1);
	libCraterTri.col(2) = libCraterTri.col(2)-MatrixXd::Ones(libCraterTri.rows(),1);

	t1 = get_time_ms();

	// 陨坑库中三角形投影至像面
	MatrixXd lib_project = project2D(K_cam,R_cam,T_cam, crater3D);
	// 构建库三角形
	MatrixXd libTri =  libTri_rebuild(K_cam,R_cam,T_cam,crater3D,lib_project,libCraterTri, max_dist);
	
	t2 = get_time_ms();

	//像面三角形构建
	double craterGroup_center[2];
	MatrixXd ImgTri = ImgTri_build(crater_PCA,PCA_peakVal,_craterSet_num,deg_limit,edgeLen_limit,craterGroup_center);

	VectorXd group_center(2); group_center<<craterGroup_center[0],craterGroup_center[1];

	t3 = get_time_ms();

	//三角形匹配
	//阈值设置
	MatrixXd cos_th(1,1);
	cos_th(0,0) = 0.15;
	MatrixXd k_diffTh(1,1);
	k_diffTh(0,0) = 0.105;

	MatrixXd matchTri_1 = Tri_match(ImgTri, libTri, group_center, max_dist_group, cos_th, k_diffTh);

	t4 = get_time_ms();

	//重投影验证
	//阈值设置
	MatrixXd th_reproj_(1,2);
	th_reproj_<<5,10;
	
	int match_cnt = matchTri_1.rows();
	double vldcnt_sort[match_cnt];
	MatrixXd repro_results = Reproject_vld_2(matchTri_1, crater_PCA, crater3D, K_cam, R_cam, th_reproj_, vldcnt_sort);

	t5 = get_time_ms();

	//获取需要IC验证的匹配三角形对
	int i,j,k,CanTri_cnt,maxnum_CandTri = 10;
	MatrixXd matchTri_2(maxnum_CandTri,matchTri_1.cols());

	CanTri_cnt = 0;
	for(i=0;i<maxnum_CandTri;i++){
		if( repro_results((int)vldcnt_sort[i],3) > 0){
			CanTri_cnt++;
			matchTri_2.row(i) = matchTri_1.row(vldcnt_sort[i]);
		}
	}


	//阈值设置
	MatrixXd vldCrater_th(1,4);
	vldCrater_th<<25,25,50,9;
	// //IC验证坑检验
	MatrixXd ICvld_results = IC_vld(matchTri_2, ImgTri, libTri, crater_PCA, lib_project, vldCrater_th, CanTri_cnt);

	t6 = get_time_ms();

	cout<<"初步匹配三角形个数："<<matchTri_1.rows()<<endl;
	cout<<"Lib Tri Build: "<<t2-t1<<endl;
	cout<<"Img Tri Build: "<<t3-t2<<endl;
	cout<<"Tri Match Total:"<<t6-t3<<endl;
	cout<<"Tri Match Step1: "<<t4-t3<<endl;
	cout<<"Reproj Filter: "<<t5-t4<<endl;
	cout<<"Tri Match Step2(IC vld): "<<t6-t5<<endl;
	cout<<"Total time: "<<t6-t1<<endl;

	return 1;
}

/**********************实际运行算法初始化-请根据算法设计调整*************************/
int Algo_init(){
	cout<<"###craterNav Begin###"<<endl;

	return 1;

}

/*********************CPU故障调度函数-请根据实际设计故障调度策略************************/
void CPU_check(unsigned long vld_cnt, algorithm_module* algo_mod){ 
	unsigned long cnt[MOD_NUM];
	int i,cpuIter_i;

	for(i = 0;i<MOD_NUM;i++){
		cnt[i] = *(threeChooseTwo_read<unsigned long>(&shmPtr.ModCnt[i],REDUN_BIAS));
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
		//无操作

		return;
	}

	//分支3 - 模块1的cpu失能
	if(cpuIter_i == 1){
		//无操作

		return;
	}

	//分支4 - 模块1和2的cpu失能
	if(cnt[0] != vld_cnt && cnt[1] != vld_cnt){
		cout<<"Code Bug! It is Impossible branch"<<endl;
	}

	return;

}

/***************************时序调度函数-用户程序的主函数*****************************/
//important: 该函数名需与proc.hpp中__proc_entry__定义的变量相同
int Process_sched(void){
	int i,j; //遍历循环计数
	/********************** IPC共享内存初始化 ****************************/
	shmPtr.DataHub = (double*)(shmPtr.ModCPU+20);  //共享数据地址初始化, 20可以改成cpu数量

	//设置算法模块起始有效标识符为99999
	for(i=0;i<MOD_NUM;i++){
		threeChooseTwo_write<unsigned long>(&shmPtr.ModCnt[i],99999,REDUN_BIAS);
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

	//激活算法模块
	switch(proc_modFlg){
	case 0:
		algo_mod[0].activate(0,Pulse.MaxWaitT, craterNav);
		break;
	case 1:
		algo_mod[1].activate(0,Pulse.MaxWaitT, craterNav);
		break;
	default:
		algo_mod[0].activate(0,Pulse.MaxWaitT, craterNav);
	}

	/*********************** 主循环程序参数以及各模块初始化 *************************/
	Algo_init(); //算法初始化

	/*********************** 主循环整体时序循环 *************************/
	//1. 进程间-时间对齐
	unsigned long ready_flg = 0;
	Pulse.cnt = 1; //初始化完成，节拍数置0
	while(1){ //等待其他合作进程初始化完成
		//遍历查询cpu_status标志位
		for(i=0;i<MOD_NUM;i++){
			ready_flg += *(threeChooseTwo_read<unsigned long>(&shmPtr.ModCnt[i],REDUN_BIAS));
		}

		if(ready_flg == MOD_NUM){
			//流程对齐，开始启动运算
			set_time_ms(0); //初始化校时
			time_sched = get_time_ms();
			break;
		}

		ready_flg = 0;
		algo_mod[proc_modFlg].iterNum_align(Pulse.cnt);//每一次查询结束后都置0
	}

	//2. 主循环
	while(1){
		
		//执行与cpu绑定的任务
		for(i=0;i<MOD_NUM;i++){
			if(algo_mod[i].runing_cpu>=0){
				algo_mod[i].run(Pulse.cnt, time_sched);
			}
		}

		//每一拍加时间
		Pulse.cnt++; //节拍数＋1
		time_sched += Pulse.time; 
		Wait2Time_ms(time_sched);
		cout<<get_time_ms()<<endl;

		//一个循环节拍结束后，检查运行情况
		CPU_check(Pulse.cnt, algo_mod);

		break; //只运行一次

	}

	return 1;
}