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
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <list>
#include <signal.h>
#include <sstream>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>

typedef list<string> StringList;
typedef list<TestCase *> TestCaseList;

#define TEXT_SZ 10*1024*1024
struct shared_use_st {
    int currentIndex;               //当前测试用例ID
    int count;
    char text[TEXT_SZ] ;            //记录写入和读取的文本
    char result[TEXT_SZ];
    int pTlen;  //real len of char[]
    int pRlen;
    bool errorFlag;
};

class SupportClass {

private:
  StringList testExecPath; //插桩路径保存
  int currentIndex;        //当前测试用例序号
public:
	struct myData{
		double takeoff_alt;
		double time;
		double battery_remaining;
		double wind_speed;
	};
	myData mt;
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

  int getParamValueFormNamesWithKey(string processNames[], string key);

  /* 记录当前激励 执行情况 */
  void setCurProcessResult(string processName, long mtime, int flag);

  /* 路径 */
  void showTestExecPath();
  void cleanTestExecPath();

  /* 类型转换 */
  string ltos(double l);
  /* 字符串 按某 字符 分割成list数组 */
  list<string> stringSplit(string s, const char *str);
  /* 信号处理以及设置定时器 */
  void setHandlerAndAlarm(int seconds);
  //void setAlarm(int seconds);

  /* 共享内存 创建并准备 */
  void createMem();
  /* 将 测试用例实体集 放入 共享内存中 */
  void putTestCasesInMem();
  /* 从 共享内存中 读写测试用例实体集 */
  void getTestCasesInMem();
  /* 分离 共享内存 */
  void pullMem();

  void setMyData(double wind_speed,double takeoff_alt, double time, double battery_remaining);
};


#endif
