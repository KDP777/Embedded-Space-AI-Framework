#include "common.hpp"
#include "CraterNav.hpp"

using namespace std;
using namespace Eigen;

unsigned long long t1_1;
unsigned long long t2_1;

MatrixXi PosMap_generate(MatrixXd points_uv, double bias ,int map_size){
	// 初始化一个全0的地图
	MatrixXi PosMap = MatrixXi::Zero(map_size, map_size);

	int i,j,k;
	double tmpU_max,tmpU_min,tmpV_max,tmpV_min;

	for(i=0;i<points_uv.rows();i++){
		//超出画幅的点跳过
		if (points_uv(i,0)< bias || points_uv(i,1)< bias || points_uv(i,0)>512-bias || points_uv(i,1)>512-bias ){
			continue;
		}

		//计算其容差范围
		tmpU_min = floor(points_uv(i,0) - bias);
		tmpV_min = floor(points_uv(i,1) - bias);

		tmpU_max = floor(points_uv(i,0) + bias);
		tmpV_max = floor(points_uv(i,1) + bias);

		//将map容差范围内的数置1
		for(j=tmpU_min;j<tmpU_max;j++){
			for(k=tmpV_min;k<tmpV_max;k++){
				PosMap(k,j) = 1;
			}
		}
	}

	return PosMap;
}

int PosMap_fastConv(MatrixXi PosMap, MatrixXd UV){
	int i, cnt_result;
	int tmp_u,tmp_v;

	cnt_result = 0;
	for(i=0;i<UV.rows();i++){
		//超出画幅的点跳过
		if (UV(i,0)< 0 || UV(i,1)< 0 || UV(i,0)>511 || UV(i,1)>511 ){
			continue;
		}

		tmp_u = (int)round(UV(i,0));
		tmp_v = (int)round(UV(i,1));

		if ( PosMap(tmp_v, tmp_u) ){
			cnt_result++;
		}
	}

	return cnt_result;
}


// 构建三角形的基本元素，角度cos值，角度sin值，三角形边长，以及三角形中心
void triBuild_base(MatrixXd points2D, double* cos_feature, double* sin_feature, double* Edge_len, double* tri_center){
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

	Edge_len[0] = bc_norm;
	Edge_len[1] = ac_norm;
	Edge_len[2] = ab_norm;

	tri_center[0] = (points2D(0,0)+points2D(1,0)+points2D(2,0))/3;
	tri_center[1] = (points2D(0,1)+points2D(1,1)+points2D(2,1))/3;

	return;
}

// 以各个点对应的角度进行排序，【角度最大，角度次大，角度最小】
void triBuild_degSort(MatrixXd points2D, double* cos_feature, int* sort_ix){
	sort_ix[0] = 0; sort_ix[1] = 1; sort_ix[2] = 2;
	sort(sort_ix,sort_ix+3, [&](const int& cmp_a, const int& cmp_b){ return cos_feature[cmp_a]<cos_feature[cmp_b];});

	MatrixXd tmp_a = points2D;
	double cos_feature_tmp[3] = {cos_feature[0],cos_feature[1],cos_feature[2]};

	points2D.row(0) = tmp_a.row(sort_ix[0]);
	points2D.row(1) = tmp_a.row(sort_ix[1]);
	points2D.row(2) = tmp_a.row(sort_ix[2]);

	cos_feature[0] = cos_feature_tmp[sort_ix[0]];
	cos_feature[1] = cos_feature_tmp[sort_ix[1]];
	cos_feature[2] = cos_feature_tmp[sort_ix[2]];
}

// 获得三角形三个边的对应边长比
void triBuild_edgeProp(double* edge_len, double* edge_prop){
	edge_prop[0] = edge_len[1]/edge_len[0]; // ac/bc
	edge_prop[1] = edge_len[2]/edge_len[0]; // ab/bc
	edge_prop[2] = edge_len[2]/edge_len[1]; // ab/ac
}

// 获得二维向量的叉积-标量
double cross_2d(VectorXd a, VectorXd b){
	return a(0)*b(1)-b(0)*a(1);
}

// 判断三角形内是否有其他点, 
// 返回-1，无其他点，返回序号，则说明checkPoints该序号对应的点在三角形内
int triBulid_pointInside(MatrixXd TriPoints, MatrixXd checkPoints){
	int ret = -1;
	int i;

	VectorXd ab = TriPoints.row(1) - TriPoints.row(0);
	VectorXd bc = TriPoints.row(2) - TriPoints.row(1);
	VectorXd ca = TriPoints.row(0) - TriPoints.row(2);
	VectorXd ac = -ca;
	VectorXd ba = -ab;
	VectorXd cb = -bc;

	VectorXd ap(2);
	VectorXd bp(2);
	VectorXd cp(2);

	double ap_mul = cross_2d(ac,ab);
	double bp_mul = cross_2d(ba,bc);
	double cp_mul = cross_2d(cb,ca);

	for(i=0;i<checkPoints.rows();i++){
		ap = checkPoints.row(i) - TriPoints.row(0);
		bp = checkPoints.row(i) - TriPoints.row(1);
		cp = checkPoints.row(i) - TriPoints.row(2);

		if ( ((cross_2d(ap,ab)*ap_mul) > 0) &&
			 ((cross_2d(bp,bc)*bp_mul) > 0) &&
			 ((cross_2d(cp,ca)*cp_mul) > 0)
			){
			ret = i;
			break;
		}
	}

	return ret;

}

int ExternCrater_vld(VectorXd Tri_img, VectorXd Tri_lib, 
						MatrixXd ExternCrater_img, MatrixXd ExternCrater_lib, 
						double* th, double* ExCrater, int ExCrater_numTh){

	int vldCrater_cnt = 0;
	VectorXd dl_img = Tri_img.segment(9,2);
	VectorXd dl_lib = Tri_lib.segment(9,2);
	VectorXd dc_img = Tri_img.segment(12,2);
	VectorXd dc_lib = Tri_lib.segment(12,2);
	double TH = th[0]*Tri_img(6);
	double TH1 = th[1]*Tri_img(6);
	double TH2 = th[2]*Tri_img(6);

	double lamuda = Tri_lib(6)/Tri_img(6);
	double dldc_dotImg, dldc_crossImg, dldc_dotLib, dldc_crossLib, I, C;
	double I_min,C_min;

	VectorXd ExVec_img(2);
	VectorXd ExVec_lib(2);

	int i,j;

	for(i=0;i<ExternCrater_img.rows();i++){
		if ((i == Tri_img(0)) || (i == Tri_img(1)) || (i == Tri_img(2)))
			continue; 

		ExVec_img(0) = ExternCrater_img(i,0)-dc_img(0);
		ExVec_img(1) = ExternCrater_img(i,1)-dc_img(1);

		dldc_dotImg = dl_img.dot(ExVec_img);
		dldc_crossImg = cross_2d(dl_img, ExVec_img);

		for(j=0;j<ExternCrater_lib.rows();j++){
			if ((j == Tri_lib(0)) || (j == Tri_lib(1)) || (j == Tri_lib(2))){
				continue; 
			}

			ExVec_lib(0) = ExternCrater_lib(j,0) - dc_lib(0);
			ExVec_lib(1) = ExternCrater_lib(j,1) - dc_lib(1);
			dldc_dotLib = dl_lib.dot(ExVec_lib);
			dldc_crossLib = cross_2d(dl_lib, ExVec_lib);

			I = fabs(dldc_dotImg - dldc_dotLib/(lamuda*lamuda));
			C = fabs(dldc_crossImg - dldc_crossLib/(lamuda*lamuda));

			//符合条件判断
			if((I<TH1) && (C<TH1) && (I+C<TH2) && sqrt(I*I+C*C)<TH ){

				if(vldCrater_cnt<ExCrater_numTh){
					ExCrater[vldCrater_cnt*2]   = i;
					ExCrater[vldCrater_cnt*2+1] = j;
				}

				vldCrater_cnt++;
				break;
			}

		}
	}


	return vldCrater_cnt;
}


//库三角形重构建
MatrixXd libTri_rebuild(MatrixXd K_cam, MatrixXd R_cam, MatrixXd T_cam, MatrixXd crater3D, MatrixXd lib_project, MatrixXd libCraterTri, MatrixXd max_dist){
	MatrixXd T_cal(3,1);
	MatrixXd Point_pairs(100,5);
	int pair_num = 0;
	int i,j,k,build_cnt = 0;
	MatrixXd Points2D_tmp(3,2);

	MatrixXd libTri(libCraterTri.rows(), 15);
	double cos_feature[3], sin_feature[3], edge_len[3], edge_prop[3], tri_center[2];
	double dist_th = max_dist(0,0);

	for (i=0;i<libCraterTri.rows();i++){
		//通过离中心的距离筛选掉超出视场的三角形
		if (libCraterTri(i,11)>dist_th)
			continue;

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
		libTri(i,9) = Points2D_tmp(2,0) - Points2D_tmp(1,0); //三角形最长边向量
		libTri(i,10) = Points2D_tmp(2,1) - Points2D_tmp(1,1); //三角形最长边向量
		libTri(i,11) = libTri(i,10)/libTri(i,9) ; //最长边BC的斜率
		libTri(i,12) = tri_center[0]; //三角形中心点u
		libTri(i,13) = tri_center[1]; //三角形中心点v
		
		//A点相对于BC边位置
		if ( (libTri(i,11)*(Points2D_tmp(0,0)-Points2D_tmp(1,0))+Points2D_tmp(1,1)) > Points2D_tmp(0,1)){
			libTri(i,14) = (libTri(i,11)>0? 0:1);
		}
		else{
			libTri(i,14) = (libTri(i,11)>0? 1:0);
		}

		build_cnt++;
	}

	MatrixXd libTri_output = libTri.block(0,0,build_cnt,15);

	return libTri_output;

}

//像面三角形的构建
MatrixXd ImgTri_build(MatrixXd crater_PCA, MatrixXd PCA_peakVal, MatrixXd _craterSet_num, MatrixXd deg_limit, MatrixXd edgeLen_limit, double* group_center){
	MatrixXd T_cal(3,1);
	MatrixXd Point_pairs(100,5);
	int pair_num = 0;
	int i,j,k;
	MatrixXd Points2D_tmp(3,2);

	double cos_feature[3], sin_feature[3], edge_len[3], edge_prop[3], tri_center[2];

	/*************************从PCA提取的陨坑中找到craterSet_num个陨坑以构成三角形********************************/
	int PCA_ix[PCA_peakVal.rows()], dist_ix[PCA_peakVal.rows()];
	double dist[PCA_peakVal.rows()];
	int center_PCAcrater;
	int craterSet_num = _craterSet_num(0,0);
	MatrixXi center_set(craterSet_num,craterSet_num);
	int center_ix = 0;
	double peakSum, peakSum_max = 0;

	//生成索引顺序数
	for(i=0;i<PCA_peakVal.rows();i++) {
		PCA_ix[i] = i;
		dist_ix[i] = i;
	}
	//对峰值进行排序
	sort(PCA_ix,PCA_ix+PCA_peakVal.rows(), [&](const int& cmp_a, const int& cmp_b){ return PCA_peakVal(cmp_a,0)>PCA_peakVal(cmp_b,0);});

	center_ix = 0; //初始化中心陨坑点
	for( j = 0; j<craterSet_num; j++){
		peakSum = 0;
		center_PCAcrater = PCA_ix[j];

		for (i=0;i<crater_PCA.rows();i++){
			dist[i] = (crater_PCA.row(i) - crater_PCA.row(center_PCAcrater)).norm();
		}

		// 对与主要峰值点的距离进行排序
		sort(dist_ix,dist_ix+crater_PCA.rows(), [&](const int& cmp_a, const int& cmp_b){ return dist[cmp_a]<dist[cmp_b];});

		for (k=0; k<craterSet_num; k++){
			center_set(j,k) = dist_ix[k];
			peakSum += PCA_peakVal(dist_ix[k],0);
			// cout<<dist_ix[k]<<endl;
		}

		if(peakSum>peakSum_max){
			center_ix = j;
			peakSum_max = peakSum;
		}
		// cout<<peakSum<<endl;
		// cout<<endl;

	}

	// cout<<center_ix<<endl;

	// /********************************构建图像上三角形*************************************/
	int tri_num = craterSet_num*(craterSet_num-1)*(craterSet_num-2)/6;
	MatrixXd PCA_cand(craterSet_num,2);
	MatrixXi travel_ix(tri_num,3);
	MatrixXd ImgTri(tri_num,15);
	int vld_Trinum=0;
	int sort_ix[3];
	int travel_cnt = 0;
	double edge_len_tmp[3];
	int pointInside;

	group_center[0] = crater_PCA(center_set(center_ix,0),0);
	group_center[1] = crater_PCA(center_set(center_ix,0),1);

	for (i=0;i<craterSet_num;i++){
		PCA_cand.row(i) = crater_PCA.row( center_set(center_ix,i) );
	}

	//获得所有可能的 三角形序号组合
	for (i=0;i<craterSet_num-2;i++){
		for(j=i+1;j<craterSet_num-1;j++){
			for(k=j+1;k<craterSet_num;k++){
				travel_ix(travel_cnt,0) = i;
				travel_ix(travel_cnt,1) = j;
				travel_ix(travel_cnt,2) = k;
				travel_cnt++;
			}
		}
	}

	for(i=0;i<travel_cnt;i++){
		//*****************构建PCA提取陨坑的三角形**************************
		Points2D_tmp.row(0) = PCA_cand.row(travel_ix(i,0));
		Points2D_tmp.row(1) = PCA_cand.row(travel_ix(i,1));
		Points2D_tmp.row(2) = PCA_cand.row(travel_ix(i,2));

		//判断内部有无陨坑
		pointInside = triBulid_pointInside(Points2D_tmp,PCA_cand);
		if (pointInside != -1)
			continue;

		triBuild_base(Points2D_tmp,cos_feature,sin_feature,edge_len,tri_center);
		triBuild_degSort(Points2D_tmp,cos_feature,sort_ix);	

		// 三角形有效性判断
		if ( (cos_feature[0] < cos( deg_limit(0,1) *M_PI/180)) || (cos_feature[2] > cos( deg_limit(0,0) *M_PI/180)) )
			continue;

		//根据sort_ix调整edge排序
		edge_len_tmp[0] = edge_len[0];
		edge_len_tmp[1] = edge_len[1];
		edge_len_tmp[2] = edge_len[2];

		edge_len[0] = edge_len_tmp[sort_ix[0]];
		edge_len[1] = edge_len_tmp[sort_ix[1]];
		edge_len[2] = edge_len_tmp[sort_ix[2]];

		if ((edge_len[0]>edgeLen_limit(0,1)) || (edge_len[2]<edgeLen_limit(0,0)) )
			continue;

		//获得边长比
		triBuild_edgeProp(edge_len, edge_prop);

		ImgTri(vld_Trinum,0) = center_set(center_ix,travel_ix(i,sort_ix[0])); //三角序号a-最大角
		ImgTri(vld_Trinum,1) = center_set(center_ix,travel_ix(i,sort_ix[1])); //三角序号b-次大角
		ImgTri(vld_Trinum,2) = center_set(center_ix,travel_ix(i,sort_ix[2])); //三角序号c-最小角
		ImgTri(vld_Trinum,3) = cos_feature[0]; //cos(a)
		ImgTri(vld_Trinum,4) = cos_feature[1]; //cos(b)
		ImgTri(vld_Trinum,5) = cos_feature[2]; //cos(c)
		ImgTri(vld_Trinum,6) = edge_len[0]; //三角形边长bc
		ImgTri(vld_Trinum,7) = edge_len[1]; //三角形边长ac
		ImgTri(vld_Trinum,8) = edge_len[2]; //三角形边长ab
		ImgTri(vld_Trinum,9) = Points2D_tmp(sort_ix[2],0)-Points2D_tmp(sort_ix[1],0); //三角形最长边向量
		ImgTri(vld_Trinum,10) = Points2D_tmp(sort_ix[2],1)-Points2D_tmp(sort_ix[1],1); //三角形最长边向量
		ImgTri(vld_Trinum,11) = ImgTri(vld_Trinum,10)/ImgTri(vld_Trinum,9) ; //最长边BC的斜率
		ImgTri(vld_Trinum,12) = tri_center[0]; //三角形中心点u
		ImgTri(vld_Trinum,13) = tri_center[1]; //三角形中心点v

		//A点相对于BC边位置
		if ( (ImgTri(vld_Trinum,11)*(Points2D_tmp(sort_ix[0],0)-Points2D_tmp(sort_ix[1],0))+Points2D_tmp(sort_ix[1],1)) > Points2D_tmp(sort_ix[0],1)){
			ImgTri(vld_Trinum,14) = (ImgTri(vld_Trinum,11)>0? 0:1);
		}
		else{
			ImgTri(vld_Trinum,14) = (ImgTri(vld_Trinum,11)>0? 1:0);
		}
		vld_Trinum++;

	}

	MatrixXd ImgTri_out = ImgTri.block(0,0,vld_Trinum,15);

	return ImgTri_out;
}

//三角形匹配
MatrixXd Tri_match(MatrixXd ImgTri_out,MatrixXd libTri,MatrixXd group_center,MatrixXd max_dist_group,MatrixXd cos_th,MatrixXd k_diffTh){
	MatrixXd T_cal(3,1);
	MatrixXd Point_pairs(100,5);
	int pair_num = 0;
	int i,j,k;
	MatrixXd Points2D_tmp(3,2);

	//*******************************三角匹配*******************************
	MatrixXd match_tri(2000,10);
	int match_cnt = 0;
	double delta_cos,k_diff,LongEdge_prop;

	for (i=0;i<ImgTri_out.rows();i++){

		for(j=0;j<libTri.rows();j++){

			// 判断顶点相对位置
			if (ImgTri_out(i,14) != libTri(j,14))
				continue;

			LongEdge_prop = libTri(j,6)/ImgTri_out(i,6);
			// 边长比判断
			if(LongEdge_prop < 0.5 || LongEdge_prop > 2)
				continue;

			// 确定库三角形离像面陨坑集群中心的距离有没有超出范围-超出范围直接跳过
			if(fabs(libTri(j,12)-group_center(0))>max_dist_group(0,0) || fabs(libTri(j,13)-group_center(1))>max_dist_group(0,0) )
				continue;

			// 计算两个三角形角度的差异
			delta_cos =  fabs(libTri(j,4)-ImgTri_out(i,4)) + fabs(libTri(j,5)-ImgTri_out(i,5));

			if(delta_cos<cos_th(0,0)){
					// 计算斜率差异
					k_diff = fabs( (ImgTri_out(i,11)-libTri(j,11))/(1+ImgTri_out(i,11)*libTri(j,11)) );

					if( k_diff < k_diffTh(0,0) ){

							//满足全部预设条件，加入待选的三角形序列
							match_tri(match_cnt,0) = (double)ImgTri_out(i,0);
							match_tri(match_cnt,1) = (double)ImgTri_out(i,1);
							match_tri(match_cnt,2) = (double)ImgTri_out(i,2);
							match_tri(match_cnt,3) = (double)libTri(j,0);
							match_tri(match_cnt,4) = (double)libTri(j,1);
							match_tri(match_cnt,5) = (double)libTri(j,2);

							match_tri(match_cnt,6) = delta_cos;
							match_tri(match_cnt,7) = k_diff;
							match_tri(match_cnt,8) = i;
							match_tri(match_cnt,9) = j;

							match_cnt++;
					}

			}
		}
	}

	MatrixXd matchTri = match_tri.block(0,0,match_cnt,match_tri.cols());

	return matchTri;
}

//重投影验证 - 1
MatrixXd Reproject_vld(MatrixXd match_tri, MatrixXd crater_PCA,MatrixXd crater3D,MatrixXd K_cam,MatrixXd R_cam,MatrixXd th_reproj_, double* vldcnt_sort){
	int i,j,k;
	double repro_err_sum,dist_vld;
	double th_reproj3 = th_reproj_(0,0);
	double th_reprojVld = th_reproj_(0,1);
	int match_cnt = match_tri.rows();

	MatrixXd points2D(3,2);
	MatrixXd points3D(3,3);
	MatrixXd repro_uv(3,2);
	MatrixXd repro_err(3,2);
	MatrixXd repro_uv_vld(crater3D.rows(),2);
	VectorXd repro_vldcnt(match_cnt);

	int tri_vldcnt,tri_vldix[match_cnt];
	double point_vld[match_cnt*3];
	double deltaT;
	bool triVld_flag;
	int vldCrater_num;

	MatrixXd Point2D_solve(match_cnt*3,2);
	MatrixXd Point3D_solve(match_cnt*3,3);
	MatrixXd T_tmp(3,1);
	MatrixXd reproject_err(match_cnt,3);

	for(i=0; i<match_cnt; i++){
		vldcnt_sort[i] = i;
		tri_vldcnt = 0;

		points2D.row(0) = crater_PCA.row( match_tri(i,0) );
		points2D.row(1) = crater_PCA.row( match_tri(i,1) );
		points2D.row(2) = crater_PCA.row( match_tri(i,2) );

		points3D.row(0) = crater3D.row( match_tri(i,3) );
		points3D.row(1) = crater3D.row( match_tri(i,4) );
		points3D.row(2) = crater3D.row( match_tri(i,5) );

		//解算位置
		T_tmp = PNP_solveT(K_cam,R_cam,points2D,points3D);

		//重投影验证-三角形三个点的验证
		repro_uv = project2D(K_cam,R_cam,T_tmp,points3D);
		repro_err = repro_uv-points2D;
		reproject_err(i,0) = repro_err.row(0).norm();
		reproject_err(i,1) = repro_err.row(1).norm();
		reproject_err(i,2) = repro_err.row(2).norm();

		if ((reproject_err(i,0)+reproject_err(i,1)+reproject_err(i,2)) > th_reproj3){
			repro_vldcnt(i) = 0;
			continue;
		}

		//重投影验证-所有提取坑的邻近验证
		repro_uv_vld = project2D(K_cam,R_cam,T_tmp,crater3D);

		for(j=0;j<crater_PCA.rows();j++){
			for (k=0;k<repro_uv_vld.rows();k++){
				dist_vld = (crater_PCA.row(j) - repro_uv_vld.row(k)).norm();
				if (dist_vld < th_reprojVld)
					tri_vldcnt++;
			}
		}
		
		repro_vldcnt(i) = tri_vldcnt;
	}

	// 对repro_vldcnt进行排序
	sort(vldcnt_sort,vldcnt_sort+match_cnt, [&](const int& cmp_a, const int& cmp_b){ return repro_vldcnt(cmp_a)>repro_vldcnt(cmp_b);});

	
	MatrixXd output(reproject_err.rows(),4);

	output<<reproject_err,repro_vldcnt;

	return output;
}

//重投影验证 - 2
MatrixXd Reproject_vld_2(MatrixXd match_tri, MatrixXd crater_PCA,MatrixXd crater3D,MatrixXd K_cam,MatrixXd R_cam,MatrixXd th_reproj_, double* vldcnt_sort){
	int i,j,k;
	double repro_err_sum,dist_vld;
	double th_reproj3 = th_reproj_(0,0);
	double th_reprojVld = th_reproj_(0,1);
	int match_cnt = match_tri.rows();

	MatrixXd points2D(3,2);
	MatrixXd points3D(3,3);
	MatrixXd repro_uv(3,2);
	MatrixXd repro_err(3,2);
	MatrixXd repro_uv_vld(crater3D.rows(),2);
	VectorXd repro_vldcnt(match_cnt);

	int tri_vldcnt,tri_vldix[match_cnt];
	double point_vld[match_cnt*3];
	double deltaT;
	bool triVld_flag;
	int vldCrater_num;

	MatrixXd Point2D_solve(match_cnt*3,2);
	MatrixXd Point3D_solve(match_cnt*3,3);
	MatrixXd T_tmp(3,1);
	MatrixXd reproject_err(match_cnt,3);
	MatrixXi base_map(512,512);

	//提取陨坑的位置全部转为整数
	// MatrixXi crater_PCA_int(crater_PCA.rows(),crater_PCA.cols());
	// for (k=0; k<crater_PCA.rows() ; k++){
	// 	crater_PCA_int(k, 0) = round(crater_PCA(k,0));
	// 	crater_PCA_int(k, 1) = round(crater_PCA(k,1));
	// }

	base_map = PosMap_generate(crater_PCA, th_reprojVld, 512);

	for(i=0; i<match_cnt; i++){
		vldcnt_sort[i] = i;
		tri_vldcnt = 0;

		points2D.row(0) = crater_PCA.row( match_tri(i,0) );
		points2D.row(1) = crater_PCA.row( match_tri(i,1) );
		points2D.row(2) = crater_PCA.row( match_tri(i,2) );

		points3D.row(0) = crater3D.row( match_tri(i,3) );
		points3D.row(1) = crater3D.row( match_tri(i,4) );
		points3D.row(2) = crater3D.row( match_tri(i,5) );

		//解算位置
		T_tmp = PNP_solveT(K_cam,R_cam,points2D,points3D);

		//重投影验证-三角形三个点的验证
		repro_uv = project2D(K_cam,R_cam,T_tmp,points3D);
		repro_err = repro_uv-points2D;
		reproject_err(i,0) = repro_err.row(0).norm();
		reproject_err(i,1) = repro_err.row(1).norm();
		reproject_err(i,2) = repro_err.row(2).norm();

		if ((reproject_err(i,0)+reproject_err(i,1)+reproject_err(i,2)) > th_reproj3){
			repro_vldcnt(i) = 0;
			continue;
		}

		//重投影验证-所有提取坑的重投影邻近验证
		repro_uv_vld = project2D(K_cam,R_cam,T_tmp,crater3D);

		tri_vldcnt = PosMap_fastConv(base_map, repro_uv_vld);

		repro_vldcnt(i) = tri_vldcnt;
	}

	// 对repro_vldcnt进行排序
	sort(vldcnt_sort,vldcnt_sort+match_cnt, [&](const int& cmp_a, const int& cmp_b){ return repro_vldcnt(cmp_a)>repro_vldcnt(cmp_b);});

	
	MatrixXd output(reproject_err.rows(),4);

	output<<reproject_err,repro_vldcnt;

	return output;
}


MatrixXd IC_vld(MatrixXd matchTri_cand,MatrixXd ImgTri, MatrixXd LibTri, 
			MatrixXd crater_PCA, MatrixXd lib_project, MatrixXd vldCrater_th, int Tri_num){

	double ExCrater_th[3] = {vldCrater_th(0,0),vldCrater_th(0,1),vldCrater_th(0,2)};
	int ExCrater_numTh = (int)vldCrater_th(0,3);
	int i,j,k,vld_num;
	int imgTri_ix, libTri_ix;
	double ExCrater[ExCrater_numTh*2];
	MatrixXd IC_vld_result(Tri_num,ExCrater_numTh*2+1);

	for(i=0;i<Tri_num;i++){

		imgTri_ix = matchTri_cand(i,8);
		libTri_ix = matchTri_cand(i,9);
		vld_num = ExternCrater_vld(ImgTri.row(imgTri_ix), LibTri.row(libTri_ix), 
						crater_PCA, lib_project, ExCrater_th, ExCrater, ExCrater_numTh);

		if (vld_num<ExCrater_numTh)
			vld_num = 0;

		IC_vld_result(i,0) = vld_num;
		for (j=0;j<ExCrater_numTh*2;j++){
			IC_vld_result(i,j+1) = ExCrater[j];
		}
	}

	return IC_vld_result;

}