/*
* 辅助参数类
*用于重写 输入输出重定向 以及两个缓冲区参数
*/
#ifndef SupportClass_class
#define SupportClass_class

#include "../TestCase/TestCase.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <time.h>
#include <fcntl.h>
#include <iostream>
#include <list>
#include <signal.h>
#include <sstream>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <map>

typedef list<string> StringList;
typedef list<TestCase *> TestCaseList;

#define TEXT_SZ 10*1024*1024

#define INIT 0
#define START 1
#define END 2

struct shared_use_st {
  int currentIndex;   //当前测试用例ID
  int count;
  char text[TEXT_SZ]; //记录写入和读取的文本
  char result[TEXT_SZ];
};

class processTime{
	public :
		struct timeval startTime;
		struct timeval endTime;
		double interval;
		double pinterval;
		//时间差 s
		double setInterval(){
			return 1000.0 * (endTime.tv_sec - startTime.tv_sec)
				+ (endTime.tv_usec - startTime.tv_usec)/1000.0;
		}
		double setPinterval(processTime pre){
			return 1000.0 * (startTime.tv_sec - pre.startTime.tv_sec)
				+ (startTime.tv_usec - pre.startTime.tv_usec) / 1000.0;
		}
		void show(){
			printf("pinterval = %lf ms ,interval = %lf ms\n", pinterval, interval);
		}
		void showPinterval(string p){
			cout << p << ": pinterval = " << pinterval << " ms" << endl;
		}
		/* 
			初始化各个时间 
				0:最开始初始化 
				1:起始时间 
				2:终止时间*/
		void init(int i){
			switch (i){
				case 0:
					gettimeofday(&startTime, NULL);
					gettimeofday(&endTime, NULL);
					break;
				case 1:
					gettimeofday(&startTime, NULL);
					break;
				case 2:
					gettimeofday(&endTime, NULL);
					interval = setInterval();
					break;
			}
		}
};

class SupportClass {

private:
  StringList testExecPath; //插桩路径保存
  int currentIndex;        //当前测试用例序号
public:
  /* 上一个用例时间，当前用例时间*/
  map<string,processTime> processTimes;
 
  SupportClass();

  /* 获取当前测试用例  */
  TestCase *getCurrentTestCase();
  /* 设置当前 测试用例实体类 从内存中获取*/
  void setCurrentTestCase();
  /*  设置当前 测试用例序号 同步到内存区域中 */
  void setCurrentIndex();
  /* 获取当前测试用例ID */
  int getCurrentIndex();
  /* 获取所有的测试用例数目 */
  int getTestCaseCount();
  /* 用例执行完成后 设置测试结果 */
  void setCurTestCaseResult(string exeSituation);
  /* 在当前 测试用例 中根据激励名称以及参数名 获取参数值 */
  int getParamValueWithNameAndKey(string processName, string key);
  /* 记录当前激励 执行情况 */
  void setCurProcessResult(string processName, double mtime);

  /* 路径 */
  void showTestExecPath();
  void cleanTestExecPath();

  /* 类型转换 */
  string d2s(double l);
  /* 字符串 按某 字符 分割成list数组 */
  list<string> stringSplit(string s, const char *str);
  /* 信号处理以及设置定时器 */
  void setHandler();
  void setAlarm(int seconds);

  /* 共享内存 创建并准备 */
  void createMem();
  /* 将 测试用例实体集 放入 共享内存中 */
  void putTestCasesInMem();
  /* 从 共享内存中 读写测试用例实体集 */
  void getTestCasesInMem();
  /* 分离 共享内存 */
  void pullMem();

  /* 记录处理时间
  0:最开始初始化
  1:起始时间
  2:终止时间*/
  void timeHandle(string, int,string preProcessName = "");
};
#endif
