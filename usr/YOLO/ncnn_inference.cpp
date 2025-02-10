/*用户的使用模板-套用PID_OrbitCtl的Demo；
  实际编写用户程序请参考以下格式；
*/

#include "common.hpp"
#include "proc.hpp"
#include "redundency.hpp"
#include "ncnn_model.h"
#include "v4l2.h" //v4l2获取相机图像
#include <sys/socket.h>  //socket 通信
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <fcntl.h>  //包含了open函数

using namespace std;

/**************算法模块参数设置-配置项********************/
//算法模块总数
#define MOD_NUM 2

//算法循环节拍相关参数
unsigned long time_sched; //绝对时间-调度任务使用
pulse Pulse = 
{
	1200,		//Pulse.time 节拍间隔时间/ms
 	0,			//Pulse.cnt 初始节拍数
 	1199,	//Pulse.MaxWaitT 单节拍最大等待时间/ms
};

void* shmNoRedun;

/**************具体算法函数需要使用的全局变量-配置项******************/
//ncnn网络
char Param_path[] = "/home/kdp-pi/Space_AI/usr/YOLO/yolo10n_ncnn_model/model.ncnn.param";
char Model_path[] = "/home/kdp-pi/Space_AI/usr/YOLO/yolo10n_ncnn_model/model.ncnn.bin";
char ex_out0[] = "out0";
char ex_out1[] = "out1";
char ex_out2[] = "out2";

//socket通信
#define DEST_PORT 7777   
#define DSET_IP_ADDRESS "10.7.7.77"
#define Send_BufLen 61440 //60kB
int sock_fd, send_num, recv_num, len; //Socket发送相关变量
struct sockaddr_in addr_serv;
char stop_img[3] = {'i','m','g'}; //终止符
int i, ptr_bias; //拼接提取框Buffer相关变量
char rect_sendBuf[1000]; //YOLO提取框发送Buffer
int x,y,height,width;

//统计时间
unsigned long t1,t2,t3,t4,t5;

//图像相关变量
std::vector<uchar> jpg_buffer; //存放压缩jpg的buffer
std::vector<int> params; //编码为jepg的参数设置
int encode_imglen;
int UDP_imgSendCnt;
int img_height = 480;
int img_width = 640;

//opencv图像，摄像头相关变量
cv::Mat img_ori(480,640,CV_8UC3);
struct buffer buffers_img;
char cam_name[] = "/dev/video0";
int cam_fd;

//共享内存数据交互设置
int* shmData_flags;
unsigned long* Mod0_doneFlg; //Mod0的完成标志-模块完成的计数
unsigned long* Mod1_doneFlg; //Mod1的完成标志-模块完成的计数

int* shmObjNum_Mod0; //数据交换区域-Mod0输出的Object数量
int* shmObjNum_Mod1; //数据交换区域-Mod1输出的Object数量
Object* shmObj_Mod0; //数据交换区域-Mod0输出的Object数据
Object* shmObj_Mod1; //数据交换区域-Mod1输出的Object数据

/******************************************************************/
/*
 * 描述：算法具体函数构造，根据不同项目编写
 * 输入：无
 * 输出：无
 */
int YOLOv10_out1(unsigned long tickNum, unsigned long tickGap){
    int i;
    vector<Object> objects; // 网络输出的数据格式

    // 缓存图像
    buffers_img = v4l2_getImage(cam_fd);
    buffers_img = v4l2_getImage(cam_fd);

    cv::Mat img_YUYV(img_height,img_width,CV_8UC2,(char*)buffers_img.start);
    cv::cvtColor(img_YUYV,img_ori,cv::COLOR_YUV2BGR_YUYV);

    t1 = get_time_ms();

    //图像存入共享缓存中
    memcpy(shmNoRedun,img_ori.data,img_ori.total()*img_ori.elemSize());

    threeChooseTwo_write<int>(&shmData_flags[0],1,REDUN_BIAS);

    //启动YOLO
    NCNN_Pre(img_ori); //预处理

    NCNN_NetRun(2); //网络推理

    NCNN_Post(objects, 2); //后处理

    t2 = get_time_ms();

    cout<<"YOLO_Stride_16 运行时间: "<<t2-t1<<"ms"<<endl<<endl;

    // //输出打印
    // for(i=0;i<objects.size();i++){
    //     cout<<objects[i].rect<<endl;
    //     cout<<objects[i].label<<endl;
    //     cout<<objects[i].prob<<endl;
    // }

    //写入共享内存
    int obj_size = objects.size();
    //写入object个数   
    threeChooseTwo_write<int>(shmObjNum_Mod0, &obj_size, REDUN_BIAS); 
    //写入object内容
    threeChooseTwo_write<Object>(shmObj_Mod0, &objects[0], REDUN_BIAS, sizeof(Object)*obj_size);

    // 数据交换有效性验证
    // delay_ms(500);
    // int n = *(shmObjNum_Mod1);
    // Object* obj = (Object*)(shmObj_Mod1);
    // for(i=0;i<n;i++){
    // 	cout<<obj[i].rect<<endl;
    //     cout<<obj[i].label<<endl;
    //     cout<<obj[i].prob<<endl;
    // }

    return 1;
}

int YOLOv10_out2(unsigned long tickNum, unsigned long tickGap){
    int i;
    vector<Object> objects; // 网络输出的数据格式
    cv::Mat img_exchange(img_height,img_width,CV_8UC3);

    t1 = get_time_ms();

    //获取图像缓存，最多等待100ms
    for(i=0;i<100;i++){
    	if(*(threeChooseTwo_read<int>(&shmData_flags[0],REDUN_BIAS)) == 1){
    		memcpy(img_exchange.data,shmNoRedun,img_height*img_width*3);
    		break;
    	}
    	delay_ms(1);
    }

    threeChooseTwo_write<int>(&shmData_flags[0],0,REDUN_BIAS);

    //共享内存数据交换地址-MOD2
    int* shmPtr_size = (int*)shmPtr.DataHub+50;
    Object* shmPtr_data = (Object*)(shmPtr_size+2);

    //启动YOLO
    NCNN_Pre(img_exchange); //预处理

    NCNN_NetRun(1); //网络推理

    NCNN_Post(objects, 1); //后处理

    t2 = get_time_ms();

	// //输出打印
    // for(i=0;i<objects.size();i++){
    //     cout<<objects[i].rect<<endl;
    //     cout<<objects[i].label<<endl;
    //     cout<<objects[i].prob<<endl;
    // }

    //写入共享内存
    int obj_size = objects.size();
    //写入object个数  
    threeChooseTwo_write<int>(shmObjNum_Mod1, &obj_size, REDUN_BIAS);
	//写入object内容
    threeChooseTwo_write<Object>(shmObj_Mod1, &objects[0], REDUN_BIAS, sizeof(Object)*obj_size);

    /********************** UDP 数据传送给上位机显示 *************************/

    //jpg编码图像数据
    cv::imencode(".jpg", img_exchange, jpg_buffer, params); //图像jepg编码-放到frame中

    encode_imglen =  jpg_buffer.size();//发送图像大小
    UDP_imgSendCnt = encode_imglen/Send_BufLen + 1;

    //UDP发送图像数据
    for(int i=0; i<UDP_imgSendCnt;i++){
        if (i != UDP_imgSendCnt-1){
             send_num = sendto(sock_fd, jpg_buffer.data()+Send_BufLen*i, 
                                Send_BufLen, 0, 
                                (struct sockaddr *)&addr_serv, len);
        }
        else{
             send_num = sendto(sock_fd, jpg_buffer.data()+Send_BufLen*i, 
                                encode_imglen-Send_BufLen*i, 0, 
                                (struct sockaddr *)&addr_serv, len);
        }

        delay_ms(1); //延时1ms，保护
       
    }

    //UDP发送结束符-图像数据
    send_num = sendto(sock_fd, &stop_img[0], 3, 0, (struct sockaddr *)&addr_serv, len);

    //循环查询Mod0的完成状态-最多100ms
    for(i=0;i<100;i++){
    	if (*(threeChooseTwo_read<unsigned long>(Mod0_doneFlg,REDUN_BIAS)) == (tickNum+1))
    	{
    		break;
    	}
    	delay_ms(1);
    }

    if(i == 499) return 0; //查询结果失败 返回失败

    // 装载Mod1的Object发送数据
    memset(&rect_sendBuf[0], 0, 1000); //区域清零
    ptr_bias = 0;

    for(i=0;i<objects.size();i++){
        x = round(objects[i].rect.x);
        y = round(objects[i].rect.y);
        height = round(objects[i].rect.height);
        width = round(objects[i].rect.width);

        memcpy(rect_sendBuf+ptr_bias, &x, sizeof(x));
        ptr_bias += sizeof(x);

        memcpy(rect_sendBuf+ptr_bias,&y,sizeof(y));
        ptr_bias += sizeof(y);

        memcpy(rect_sendBuf+ptr_bias,&height,sizeof(height));
        ptr_bias += sizeof(height);

        memcpy(rect_sendBuf+ptr_bias,&width,sizeof(width));
        ptr_bias += sizeof(width);

        memcpy(rect_sendBuf+ptr_bias,&objects[i].label,sizeof(objects[i].label));
        ptr_bias += sizeof(objects[i].label);

        memcpy(rect_sendBuf+ptr_bias,&objects[i].prob,sizeof(objects[i].prob));
        ptr_bias += sizeof(objects[i].prob);
    }

    // 装载Mod0的Object发送数据
	int Mod0_n = *(shmObjNum_Mod0);
    Object* objects_Mod0 = (Object*)(shmObj_Mod0);

    for(i=0;i<Mod0_n;i++){
        x = round(objects_Mod0[i].rect.x);
        y = round(objects_Mod0[i].rect.y);
        height = round(objects_Mod0[i].rect.height);
        width = round(objects_Mod0[i].rect.width);

        memcpy(rect_sendBuf+ptr_bias, &x, sizeof(x));
        ptr_bias += sizeof(x);

        memcpy(rect_sendBuf+ptr_bias,&y,sizeof(y));
        ptr_bias += sizeof(y);

        memcpy(rect_sendBuf+ptr_bias,&height,sizeof(height));
        ptr_bias += sizeof(height);

        memcpy(rect_sendBuf+ptr_bias,&width,sizeof(width));
        ptr_bias += sizeof(width);

        memcpy(rect_sendBuf+ptr_bias,&objects_Mod0[i].label,sizeof(objects[i].label));
        ptr_bias += sizeof(objects_Mod0[i].label);

        memcpy(rect_sendBuf+ptr_bias,&objects_Mod0[i].prob,sizeof(objects[i].prob));
        ptr_bias += sizeof(objects_Mod0[i].prob);
    }

    //UDP发送-rect数据
    send_num = sendto(sock_fd, rect_sendBuf, ptr_bias, 0, (struct sockaddr *)&addr_serv, len);

    // // 数据交换有效性验证
    // for(i=0;i<Mod0_n;i++){
    // 	cout<<objects_Mod0[i].rect<<endl;
    //     cout<<objects_Mod0[i].label<<endl;
    //     cout<<objects_Mod0[i].prob<<endl;
    // }

    t3 = get_time_ms();

    cout<<"YOLO_Stride_32 运行时间:"<<t2-t1<<"ms"<<endl<<endl;
    // cout<<"数据传输时间："<<t3-t2<<"ms"<<endl;

    return 1;
}

int YOLOv10_out(unsigned long tickNum, unsigned long tickGap){
    int i;
    vector<Object> objects; // 网络输出的数据格式

    // 缓存图像
    buffers_img = v4l2_getImage(cam_fd);
    buffers_img = v4l2_getImage(cam_fd);

    cv::Mat img_YUYV(img_height,img_width,CV_8UC2,(char*)buffers_img.start);
    cv::cvtColor(img_YUYV,img_ori,cv::COLOR_YUV2BGR_YUYV);

    t1 = get_time_ms();

    NCNN_Pre(img_ori); //预处理

    NCNN_NetRun(1); //网络推理

    NCNN_Post(objects, 1); //后处理

    NCNN_NetRun(2); //网络推理

    NCNN_Post(objects, 2); //后处理

    // for(i=0;i<objects.size();i++){
    //     cout<<objects[i].rect<<endl;
    //     cout<<objects[i].label<<endl;
    //     cout<<objects[i].prob<<endl;
    // }

    //写入共享内存
    int obj_size = objects.size();
    //写入object个数   
    threeChooseTwo_write<int>(shmObjNum_Mod0, &obj_size, REDUN_BIAS); 
    //写入object内容
    threeChooseTwo_write<Object>(shmObj_Mod0, &objects[0], REDUN_BIAS, sizeof(Object)*obj_size);

    t2 = get_time_ms();

      /********************** UDP 数据传送给上位机显示 *************************/

    //jpg编码图像数据
    cv::imencode(".jpg", img_ori, jpg_buffer, params); //图像jepg编码-放到frame中

    encode_imglen =  jpg_buffer.size();//发送图像大小
    UDP_imgSendCnt = encode_imglen/Send_BufLen + 1;

    //UDP发送图像数据
    for(int i=0; i<UDP_imgSendCnt;i++){
        if (i != UDP_imgSendCnt-1){
             send_num = sendto(sock_fd, jpg_buffer.data()+Send_BufLen*i, 
                                Send_BufLen, 0, 
                                (struct sockaddr *)&addr_serv, len);
        }
        else{
             send_num = sendto(sock_fd, jpg_buffer.data()+Send_BufLen*i, 
                                encode_imglen-Send_BufLen*i, 0, 
                                (struct sockaddr *)&addr_serv, len);
        }

        delay_ms(1); //延时1ms，保护
       
    }

    //UDP发送结束符-图像数据
    send_num = sendto(sock_fd, &stop_img[0], 3, 0, (struct sockaddr *)&addr_serv, len);

    // 装载Mod1的Object发送数据
    memset(&rect_sendBuf[0], 0, 1000); //区域清零
    ptr_bias = 0;

    for(i=0;i<objects.size();i++){
        x = round(objects[i].rect.x);
        y = round(objects[i].rect.y);
        height = round(objects[i].rect.height);
        width = round(objects[i].rect.width);

        memcpy(rect_sendBuf+ptr_bias, &x, sizeof(x));
        ptr_bias += sizeof(x);

        memcpy(rect_sendBuf+ptr_bias,&y,sizeof(y));
        ptr_bias += sizeof(y);

        memcpy(rect_sendBuf+ptr_bias,&height,sizeof(height));
        ptr_bias += sizeof(height);

        memcpy(rect_sendBuf+ptr_bias,&width,sizeof(width));
        ptr_bias += sizeof(width);

        memcpy(rect_sendBuf+ptr_bias,&objects[i].label,sizeof(objects[i].label));
        ptr_bias += sizeof(objects[i].label);

        memcpy(rect_sendBuf+ptr_bias,&objects[i].prob,sizeof(objects[i].prob));
        ptr_bias += sizeof(objects[i].prob);
    }

    //UDP发送-rect数据
    send_num = sendto(sock_fd, rect_sendBuf, ptr_bias, 0, (struct sockaddr *)&addr_serv, len);

    t3 = get_time_ms();

    cout<<"YOLO运行时间: "<<t2-t1<<"ms"<<endl<<endl;
    // cout<<"数据传输时间: "<<t3-t2<<"ms"<<endl;

    return 1;
}

int DoNothing(unsigned long tickNum, unsigned long tickGap){
	return 1;
}

/*
 * 描述：算法部分初始化，主要针对算法函数需要使用的全局变量进行初始化
 * 输入：无
 * 输出：int，代表初始化是否成功
 */

int Algo_init(){
	//测试代码-读取特定图像
    // char imagepath[] = "../usr/YOLO/ImgData/bus.jpg";
    // img_ori = cv::imread(imagepath, 1);

    //图像发送相关设置
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
	params.push_back(90); // 图像质量从0到100

	//Socket UDP设置
	sock_fd = socket(AF_INET,SOCK_DGRAM,0);
    if(sock_fd == 0){
        cout<<"socket failed!"<<endl;
        exit(-1);
    }

    memset(&addr_serv, 0, sizeof(addr_serv));  
    addr_serv.sin_family = AF_INET; 
    addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);  
    addr_serv.sin_port = htons(DEST_PORT);
    len = sizeof(addr_serv);

    //网络读取
    NCNN_NetLoad(Param_path,Model_path, 2);

    //分模块初始化
    switch (proc_modFlg){
    case 0:
        //摄像头初始化
        cam_fd = open(cam_name, O_RDWR, 0);
        v4l2_init(cam_fd);
        break;
    case 1:
        break;
    default:
        break;
    }

    //共享内存数据段设置
    shmData_flags = (int*)shmPtr.DataHub; //算法模块计算中间的标志位

    Mod0_doneFlg = &shmPtr.ModCnt[0];
    Mod1_doneFlg = &shmPtr.ModCnt[1];

	shmObjNum_Mod0 = shmData_flags+10; //算法模块0输出数量以及Obj数据
	shmObj_Mod0 = (Object*)(shmObjNum_Mod0+1);

	shmObjNum_Mod1 = shmObjNum_Mod0+258; //算法模块1输出数量以及Obj数据，偏移258*4个字节
	shmObj_Mod1 = (Object*)(shmObjNum_Mod1+1);

    return 1;

}


/*
 * 描述：各个算法模块相关前置任务的配置（在algorithm_moudle类下，pre_mod存储其他任务是否为当前任务前置任务的标识符，
 *			1代表前置任务，0代表非前置任务）
 * 输入：无
 * 输出：无
 */
void algorithm_module::set_preMod(void){
	switch(this->mod_flg){
	case 0:
        return; //无前置任务
	case 1:
		return; //无前置任务
	default:
		return; //无前置任务
	}
	return;
}

/*
 * 描述：确认各个cpu的运行状态，确定故障的cpu调度逻辑，在每一次时间循环后调用；
 * 输入：算法循环次数，算法模块类的数组指针（algo_mod[0]代表模块0，algo_mod[1]代表模块1，...)
 * 输出：无
 */
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
        //摄像头初始化
        cam_fd = open(cam_name, O_RDWR, 0);
        v4l2_init(cam_fd);

		//设置cpu_status参数
		cpu_status[*(threeChooseTwo_read<int>(&shmPtr.ModCPU[0],REDUN_BIAS))] = -1;
		//设置周期时间
		Pulse.time = 1700; //2s一个周期
		Pulse.MaxWaitT = 1699; //最大等待时间
		//激活当前cpu的模块0
		algo_mod[0].activate(cnt[cpuIter_i], Pulse.MaxWaitT, DoNothing);
		//更换当前cpu模块1的运行函数
		algo_mod[1].activate(cnt[cpuIter_i], Pulse.MaxWaitT, YOLOv10_out);

		cout<<"模块0 cpu失能"<<endl;
		return;
	}

	//分支3 - 模块1的cpu失能
	if(cpuIter_i == 1){
		//设置cpu_status参数
		cpu_status[*(threeChooseTwo_read<int>(&shmPtr.ModCPU[1],REDUN_BIAS))] = -1;
		//设置周期时间
		Pulse.time = 1700; //2s一个周期
		Pulse.MaxWaitT = 1699; //最大等待时间
		//激活当前cpu的模块0
		algo_mod[0].activate(cnt[cpuIter_i], Pulse.MaxWaitT, YOLOv10_out);
		//更换当前cpu模块1的运行函数
		algo_mod[1].activate(cnt[cpuIter_i], Pulse.MaxWaitT, DoNothing);

		cout<<"模块1 cpu失能"<<endl;
		return;
	}

	//分支4 - cpu全部失能！
	if(cnt[0] != vld_cnt && cnt[1] != vld_cnt){
		cout<<"Code Bug! It is Impossible branch"<<endl;
	}

	return;

}


/*
 * 描述：算法模块的调度主程序，包括了进程初始化，共享内存初始化，算法模块初始化，以及程序主体的循环
 * 输入：无
 * 输出：int 1代表正常，其他值代表故障
 */
int Process_sched(void){
	int i,j,ret; //遍历循环计数
	/********************** IPC共享内存初始化 ****************************/
	shmPtr.DataHub = (void*)(shmPtr.ModCPU+cpuNum);  //共享数据地址初始化, cpuNum为cpu核心数
	shmNoRedun = (void*)( (char*)shmPtr.origin + REDUN_BIAS*3 ); //设置不使用3取2的内存地址起始指针

	//设置算法模块起始计数为99999
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

	/*激活算法模块-根据当前进程cpu赋值，
	 *各个算法模块的实际运行函数需要根据具体项目更改
	 *本demo使用Mod0_PIDCtlEstPos和Mod1_PIDCtlGetAcc
	 */
	switch(proc_modFlg){
	case 0:
		algo_mod[0].activate(99999,Pulse.MaxWaitT, YOLOv10_out1);
		break;
	case 1:
		algo_mod[1].activate(99999,Pulse.MaxWaitT, YOLOv10_out2);
		break;
	default:
		algo_mod[0].activate(99999,Pulse.MaxWaitT, YOLOv10_out1);
	}

	/*********************** 程序参数及各模块初始化 *************************/
	ret = Algo_init(); //算法初始化

	if(!ret){
		cout<<"Algorithm Module Init Failed!"<<endl;
		exit(0);
	}

	/*********************** 主循环整体时序循环 *************************/
	/*
	 *进程间-时间对齐
	 */
	unsigned long ready_flg = 0;
	Pulse.cnt = 0; //初始化完成，节拍数置0
	while(1){ //等待其他合作进程初始化完成
		//遍历查询cpu_status标志位
		for(i=0;i<MOD_NUM;i++){
			ready_flg += *(threeChooseTwo_read<unsigned long>(&shmPtr.ModCnt[i],REDUN_BIAS));
		}

		if(ready_flg == 0){
			//各cpu进程对齐，开始启动循环
			set_time_ms(0); //初始化校时
			time_sched = get_time_ms();
			break;
		}

		ready_flg = 0;
		algo_mod[proc_modFlg].iterNum_align(Pulse.cnt);//每一次查询结束后都置0
	}

	/*
	 *主循环
	 */ 
	while(1){
		//执行与cpu绑定的任务
		for(i=0;i<MOD_NUM;i++){
			if(algo_mod[i].runing_cpu==proc_cpuFlg[0]){
				algo_mod[i].run(Pulse.cnt, time_sched);
			}
		}

		//每一拍加时间
		Pulse.cnt++; //节拍数＋1
		time_sched += Pulse.time; 
		Wait2Time_ms(time_sched);
        t3 = get_time_ms();
        // cout<<"Current Time:"<<t3<<endl;

		//一个循环节拍结束后，检查cpu运行情况
		CPU_check(Pulse.cnt, algo_mod);

	}

	return 1;
}