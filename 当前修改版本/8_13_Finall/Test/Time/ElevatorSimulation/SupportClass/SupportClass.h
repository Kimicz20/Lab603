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

typedef list<TestCase *> TestCaseList;

#define TEXT_SZ 10*1024*1024

#define INIT 0
#define START 1
#define END 2
#define NOT_FIND_T 300

struct shared_use_st {
    int currentIndex;               //当前测试用例ID
    int count;
    char text[TEXT_SZ] ;            //记录写入和读取的文本
    char result[TEXT_SZ];
    int pTlen;  //real len of char[]
    int pRlen;
    bool errorFlag;
};

//激励时间类
class processTime{
	public :
		struct timeval startTime;
		struct timeval endTime;
		double interval = 0;
		double pinterval = 0;
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
		void showInterval(string p){
			cout << p << ": interval = " << interval << " ms" << endl;
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

//不等式实体类
class Inequation{
public:
	//不等式的字符串表示
	string strInequation;
	//不等式所有参数
	list<pair<string, double>> process;
	//不等号 
	string symbol;
	//常量值
	int value;
	//不等式是否满足
	bool isOK;

	void setStrInequation(string str){
		strInequation = str;
	}

	//判断是否满足
	bool isEqual(){
		double result = 0;
		for (list<pair<string, double>>::iterator iter = process.begin(); iter != process.end(); ++iter)
		{
			if ((*iter).second == NOT_FIND_T)
				return false;
			result += (*iter).second;
		}
		if (symbol == "=")
		{
			return (result == value);
		}
		else if (symbol == "<"){
			return (result < value);
		}
		else if (symbol == ">"){
			return (result > value);
		}
		else if (symbol == "<="){
			return (result <= value);
		}
		else if (symbol == ">="){
			return (result >= value);
		}
	}
	
	/* 字符串 按某 字符 分割成list数组 */
	list<string> stringSplit(string s,string tmp) {
		int l = 0;
		int r = 0;
		list<string> arr;
		if (s.find(tmp) != std::string::npos){
			while (r != std::string::npos) {
				r = s.find_first_of(tmp, l);
				if (r != std::string::npos)
					arr.push_back(s.substr(l, r - l));
				else
					arr.push_back(s.substr(l, s.length() - l));
				l = r + tmp.length();
			}
		}
		else{
			arr.push_back(s);
		}
		return arr;
		
	}

	//不等式初始化
	void init(){
		// 1.不等式符号
		if (strInequation.find("<=") != std::string::npos)
			symbol = "<=";
		else if (strInequation.find(">=") != std::string::npos)
			symbol = ">=";
		else if (strInequation.find(">") != std::string::npos)
			symbol = ">";
		else
			symbol = "<";

		//2.不等式参数
		list<string> flist = stringSplit(strInequation, symbol);

		list<string>::iterator iter2 = flist.begin();
		string tmp = *iter2;
		list<string> plist = stringSplit(tmp, "+");

		for (list<string>::iterator i = plist.begin(); i != plist.end(); i++) {
			pair<string, double> q(*i, 0);
			process.push_back(q);
		}
		++iter2;

		//3.不等式值
		value = atoi((*iter2).c_str());
	}

	//不等式参数赋值
	void assign(map<string, double> tmp){
		
		for (list<pair<string, double>>::iterator i = process.begin(); i != process.end(); i++) {
			string key = (*i).first;
			if (tmp.find(key) != tmp.end()){
				(*i).second = tmp.at(key);
			}
		}

		isOK = isEqual();
	}

	//清空
	void clear(){
		process.clear();
	}
};


class SupportClass {

private:
  int currentIndex;        //当前测试用例序号
  map<string, pair<string, string>> result; //存放时间结果 激励名,(代号,时间)
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
  void setCurTestCaseResult(string);
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
  

  /* 处理不等式*/
  RETURN_TYPE resultHandle();

  //返回测试完成后所有的时间
  string getProcess();

  //清空
  void clear();
};
#endif
