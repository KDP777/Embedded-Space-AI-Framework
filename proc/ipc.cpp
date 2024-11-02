//进程间通信相关封装函数
#include "common.hpp"

using namespace std;

void* getShm_with_key(unsigned int length, const char* key_file_path, int key_number)
{
	int shmid;
	int ret;
	key_t key;

	//获得共享内存的键值，一个真实的路径加数字可唯一确定一个key值
	key = ftok(key_file_path,key_number);
	if(key<0){
		cout<<"share memory create Failed: ftok key get failed!"<<endl;
		return NULL;
	}

	//arg1：共享内存键值key
	//arg2：共享内存段的大小，以字节为单位
	//arg3：共享内存的标志位，用于指定创建共享内存的权限和行为
	//输出：成功输出共享内存的标识符，失败返回-1
	shmid = shmget(key, length, IPC_CREAT|0666);
	if(shmid<0){
		cout<<"share memory create Failed: shmget failed!"<<endl;
		return NULL;
	}

	//映射地址空间
	//arg1：共享内存标识符 shmget输出
	//arg2：指定共享内存段附加到当前进程地址空间，NULL代表系统自动分配
	//arg3：共享内存的标志位，用于指定附加共享内存的权限和行为
	//输出：成功返回当前进程的虚拟内存指针（void*），失败返回-1
	void* shmPtr = shmat(shmid,NULL,0);
	if(shmPtr == (void*)-1){
		cout<<"share memory create Failed: shmat failed"<<endl;
	}
	return shmPtr;
}