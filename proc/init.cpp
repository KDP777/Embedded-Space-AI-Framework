#include "common.hpp"
#include "proc.hpp"
#include "signal.h"

using namespace std;

//执行任务入口
int (*task)(void) = __proc_entry__;

int proc_cpuNum;
int proc_cpuFlg[1000]; //进程绑定cpu标识
int proc_modFlg; //进程任务模块标识
struct _shmPtr shmPtr; //共享内存地址-结构体
char cpu_status[4]; //标识cpu的可用情况-cpu在硬件层面是否可用
int cpuNum;

void Ctrl_C_func(int sigFlg){
	exit(1);
	return;
}

/*
 * 主程序入口
 * 描述：对程序的进程进行初始化
 * 参数：进程运行cpu核心等
 * 返回：无
 */

int main(int argc, const char** argv){
	pid_t pid = getpid(); //获得进程pid

	int ret, task_ret, python_ret, shm_id;

	cout<<"###Space Linux Init Begin###"<<endl;
	cout<<"PID: "<<pid<<endl;

	//键盘按键中断检测
	signal(SIGINT, Ctrl_C_func);  //Ctrl+C

	/*********************** 输入参数检测 ***************************/

	if (argc < 5){
		cerr<<"Input Error: the Process input needs (-mod n, -cpu m)!"<<endl;
		return 0;
	}

	string modflg_str = "-mod";
	string cpuflg_str = "-cpu";
	string cpuall_str = "all";

	string modflg_in = argv[1];
	string cpuflg_in = argv[3];
	string cpuall_in = argv[4];

	//获得执行任务模块的序号
	if (modflg_str == modflg_in )
		proc_modFlg = atoi(argv[2]);
	else{
		cerr<<"Input Error: Please give the Algo module index (e.g. -mod 0)!"<<endl;
		return 0;
	}

	proc_cpuNum = argc-4;

	if ((proc_cpuNum<0) || (cpuflg_str != cpuflg_in)){
		cerr<<"Input Error: Please give the Algo module index (e.g. -cpu 2)!"<<endl;
		return 0;
	}

	/*********************** 设置进程的CPU核心 **********************/
	cpu_set_t cpuset;
	int i;
	//获取cpu的核心数
	cpuNum = sysconf(_SC_NPROCESSORS_CONF);

	CPU_ZERO(&cpuset); //cpu组清空

	if(cpuall_in == cpuall_str){
		for(i=0;i<cpuNum;i++){
			CPU_SET(i, &cpuset);
			proc_cpuFlg[i] = i;
		}
		proc_cpuNum = i;
	}
	else{
		for(i=0;i<proc_cpuNum;i++){
			if(atoi(argv[4+i])>=cpuNum){
				cout<<"Input CPU core Error!"<<endl;
				exit(0);
			}

			CPU_SET(atoi(argv[4+i]), &cpuset);
			proc_cpuFlg[i] = atoi(argv[4+i]);
		}
	}



	//将该进程与cpuset绑定
	sched_setaffinity(pid, sizeof(cpu_set_t), &cpuset);

	/*********************** 设置进程的属性 ************************/
	struct sched_param sched; //调度策略结构

	//获得SCHED_FIFO调度策略的最大优先级
	sched.sched_priority = sched_get_priority_max(SCHED_FIFO);
	//设置进程的调度方式-实时FIFO策略
	sched_setscheduler(pid,SCHED_FIFO,&sched);
	
	/************************ 创建子线程（暂不使用） *****************************/
	// pthread_t thread = pthread_self(); //获得当前的父线程
	// pthread_t thread_fork1;
	// pthread_attr_t p_attr;

	// // 设置线程的调度方式及其优先级
	// pthread_attr_setschedpolicy(&p_attr, SCHED_FIFO);
	// pthread_attr_setschedparam(&p_attr,&sched);

	// // 设置该线程的cpu亲和性
	// pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);

	// ret = pthread_create(&thread_fork1 , &p_attr, pthread_work, NULL); //创建线程

	/************************ 进程间通信（IPC）-共享内存（shared memory） ***************************/
	// 获取共享内存的指针
	shmPtr.origin = getShm_with_key(SHM_SIZE,"/home/kdp-pi/Space_AI/",777);
	shmPtr.ModCnt = (unsigned long *)shmPtr.origin; 
	shmPtr.ModCPU = (int*)(shmPtr.ModCnt+cpuNum); //cpuNum为可用的cpu数量

	// cpu 状态置1-代表全部可用
	cpu_status[0] = 1; //一共4个cpu，4个标志位
	cpu_status[1] = 1; 
	cpu_status[2] = 1; 
	cpu_status[3] = 1;

	/*********************** 相关底层驱动初始化 ********************/
	timer_init();

	/*********************** python 调用初始化 ********************/
	Py_Initialize();

	python_ret = Py_IsInitialized();

	if(python_ret == 0){
		cout<< "Python Initialize Fail!"<<endl;
	}

	/********************** 运行指定任务************************/

	task_ret = task();

	if (task_ret == 1)
		cout<< "###Process Running Over###"<<endl;
	else{
		cout<< "Error:Process Running with Exception!"<<endl;
		return 0;
	}

	Py_Finalize(); //释放Python解释器相关资源
	
	return 1;
}
