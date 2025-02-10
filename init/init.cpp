#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>

using namespace std;

int main(){

	cout<<"Hello Git&Cmake"<<endl;

	char id;

	cpu_set_t cpuset;
	pthread_t thread;
	pid_t pid = getpid();

	CPU_ZERO(&cpuset);
	CPU_SET(3, &cpuset);

	thread = pthread_self();
	pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);

	while(1){
		cout<<"Hello Pthread!"<<endl;
		sleep(5);
	}

	return 1;
}
