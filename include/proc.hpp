#ifndef __PROC__
#define __PROC__
#include "common.hpp"
#include "redundency.hpp"

/********************进程配置************************/
// 声明入口函数- int(*)(void)
int Process_sched(void);

/*
 * 描述：进程入口函数，init.cpp主函数调用task函数
 * 		在项目中一定要对task函数指针赋值！
 * 参数：无
 * 返回：int型，成功返回1,失败返回非1
 *
 * int (*task)(void);
 */
#define __proc_entry__ Process_sched //指定入口函数

#define SHM_SIZE 0xC00000 //共享内存大小-12M
#define REDUN_BIAS 0x100000 //三取二区域偏移地址

/**************进程相关的全局变量申明*****************/
extern int proc_cpuNum; //进程绑定cpu数量
extern int proc_cpuFlg[1000]; //进程绑定cpu标识
extern int proc_modFlg; //进程任务模块标识
extern struct _shmPtr{
	void* origin;
	unsigned long* ModCnt;
	int* ModCPU;
	void* DataHub;
} shmPtr; //共享内存地址-结构体
extern char cpu_status[4]; //标识cpu的可用情况-cpu在硬件层面是否可用
extern int cpuNum;

typedef struct _pulse{
	unsigned long time;
	unsigned long cnt;
	unsigned long MaxWaitT;
}pulse;
extern unsigned long time_sched; //绝对等待时间

/****** IPC 进程间通信模块 ***********/
/*
 * 描述：获得共享内存的地址指针
 * 输入：所要申请共享内存的大小，key值生成的文件路径和标识数字
 * 输出：申请共享内存的虚拟地址指针
 */
extern void* getShm_with_key(unsigned int length, const char* key_file_path, int key_number);

/****** 进程绑定的算法模块类-负责各个模块的调度 ********/
class algorithm_module{
public:
	unsigned long iterNum;
	int runing_cpu;
	int mod_flg;
	int total_modNum;
	int pre_mod[100]; //预留100个模块的空间
	unsigned long MaxWaitT_pre; //前置任务最大等待时间
	int (*run_task)(unsigned long, unsigned long); //模块调用函数的指针

	unsigned long* shm_ModCntPtr; //共享内存模块运行计数指针
	int* shm_ModCPUPtr; //共享内存模块运行CPU指针

	algorithm_module(void){
		mod_flg = -1;
		total_modNum = -1;
		runing_cpu = -1;
		iterNum = 0;
		MaxWaitT_pre = 9999;

		shm_ModCntPtr = &shmPtr.ModCnt[0];
		shm_ModCPUPtr = &shmPtr.ModCPU[0];
	};


	algorithm_module(int mod_i, int total_modNum_i){
		mod_flg = mod_i;
		total_modNum = total_modNum_i;
		runing_cpu = -1;
		iterNum = 0;
		MaxWaitT_pre = 9999;
		//初始化-无前置任务
		for(int i=0;i<total_modNum_i;i++){
			pre_mod[i] = 0;
		}

		shm_ModCntPtr = &shmPtr.ModCnt[0];
		shm_ModCPUPtr = &shmPtr.ModCPU[0];
	};

	int cpu_set(void){
		if(this->runing_cpu>=0) //该进程使用的cpu有效
		{
			threeChooseTwo_write<int>(&this->shm_ModCPUPtr[this->mod_flg], &this->runing_cpu, REDUN_BIAS); //写入共享内存区域
		}
		return this->runing_cpu;
	}

	int run(unsigned long tickNum, unsigned long time_sched){
		unsigned long complete_tickNum = tickNum+1;
		unsigned long waitTime_max = time_sched + this->MaxWaitT_pre; //最大等MaxWaitT_mod时间
		bool mod_en;//模块使能标志
		int run_flg = 1;

		//确认前置任务是否完成
		while(1){
			mod_en = true;
			for(int i=0;i<this->total_modNum;i++){
				if (this->pre_mod[i] == 1){
					if (*(threeChooseTwo_read<unsigned long>(&this->shm_ModCntPtr[i],REDUN_BIAS)) != complete_tickNum)
						mod_en = false;
				}
			}


			if(mod_en == true)
				break; //跳出循环

			if (get_time_ms()>waitTime_max){
				mod_en = false;
				break;
			}

			delay_ms(1);
		}

		unsigned long tickGap = complete_tickNum - *(threeChooseTwo_read<unsigned long>(&this->shm_ModCntPtr[this->mod_flg],REDUN_BIAS));
		if(mod_en){
			//运行算法模块
			run_flg = this->run_task(tickNum, tickGap);
		}


		if(run_flg == 1){
			//模块运行节拍数+1，并写入共享内存
			this->iterNum = complete_tickNum;
			threeChooseTwo_write<unsigned long>(&this->shm_ModCntPtr[this->mod_flg],&this->iterNum,REDUN_BIAS);
		}

		return 1;
	}

	void iterNum_align(unsigned long iterNum_i){
		this->iterNum = iterNum_i;
		threeChooseTwo_write<unsigned long>(&this->shm_ModCntPtr[this->mod_flg],&this->iterNum,REDUN_BIAS);
	}

	void activate(unsigned long iterNum_i,unsigned long maxWaitTime, int (*run_task_i)(unsigned long, unsigned long)){
		//激活cpu为当前进程cpu
		this->runing_cpu = proc_cpuFlg[0];
		threeChooseTwo_write<int>(&this->shm_ModCPUPtr[this->mod_flg],&this->runing_cpu, REDUN_BIAS); //写入共享内存区域

		//设置最大的前置任务等待时间
		this->MaxWaitT_pre = maxWaitTime;

		//设置好前置任务标志
		this->set_preMod();

		//设置好实际运行的函数
		this->run_task = run_task_i;

		//设置已经运行的循环次数
		this->iterNum_align(iterNum_i);

	}

	//设置前置任务标志
	void set_preMod(void);
};

#endif //__PROC__
