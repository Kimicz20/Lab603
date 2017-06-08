
// Implementation of the SupportClass class
#include "SupportClass.h"

#define    COLOR_NONE                    "\033[0m"
#define     FONT_COLOR_RED             "\033[0;31m"

using namespace std;
extern TestCase *currentTestCase = new TestCase(); //当前测试用例
shared_use_st *shared;                             //共享内存区
void *shm = NULL; //分配的共享内存的原始首地址

SupportClass::SupportClass() {
  this->createMem();
  this->currentIndex = shared->currentIndex;
}
/* 设置当前 测试用例实体类 从内存中获取*/
void SupportClass::setCurrentTestCase() { 
	if (processTimes.size() != 0)
		processTimes.clear();
	this->getTestCasesInMem(); 
}

/* 获取当前测试用例  */
TestCase *SupportClass::getCurrentTestCase() { return currentTestCase; }
/* 获取当前测试用例ID */
int SupportClass::getCurrentIndex() { return this->currentIndex; }

/* 获取所有的测试用例数目 */
int SupportClass::getTestCaseCount() { return shared->count; }

/* 设置当前 测试用例序号 同步到内存区域中*/
void SupportClass::setCurrentIndex() {
  shared->currentIndex = this->currentIndex;
}

/* 设置激励回写状态：包括 测试用例执行状态 以及测试用例结果状态
* 如果执行中没有 出现异常情况：
*	1. 正常执行
*	2. 按照用例执行不会出现死循环以及异常错误 但用例数据出现错误
*/
void SupportClass::setCurTestCaseResult(string exeSituation) {
  // 1. 设置当前测试用例执行状态
  if (exeSituation == "OK") {
	  exeSituation =
        this->ltos(atoi(currentTestCase->getFirstProcessTime().c_str()));
  }
  currentTestCase->setCurrenetTestCaseExecStatus(exeSituation);
  // 2. 设置当前测试用例结果状态
  currentTestCase->setCurrenetTestCaseResultStatus(exeSituation);
}

/* 在当前 测试用例 中根据激励名称以及参数名 获取参数值 */
int SupportClass::getParamValueWithNameAndKey(string processName, string key) {
	//没有查找到激励时 直接返回
	if (currentTestCase->isProcessExit(processName) == false){
		return NOTFIND;
	}
  string value = currentTestCase->getParamValueWithNameAndKey(processName, key);
  //数据值 处理：bool ,string -> int 类型
  if (value == "true" || value == "True") {
	  return 1;
  }
  if (value == "false" || value == "False") {
	  return 0;
  }
  return atoi(value.c_str());
}

/* 在当前 测试用例 中根据激励名称以及参数名 获取参数值 */
int SupportClass::getParamValueFormNamesWithKey(string processNames[], string key) {
	int value = NOTFIND;
	int size = atoi(processNames[0].c_str());
	
	//Fix修改V1.3
	for (int i = 1;i<=size; i++){
			value = getParamValueWithNameAndKey(processNames[i], key);
			if (value != NOTFIND){
				break;
			}
	}
	return value;
}
bool preIsNotOK(int index){
	if (index == 1)
		return false;
	return !currentTestCase->isExecOKWithIndex(index);
}

void SupportClass::Cout(string out){
	//cout << "--- " << out << " ---" << endl;
}

void SupportClass::timeHandle(string processName, int f){

	ProcessTime curProcessTime;
	switch (f){
		case 1:
			//如果key重复
			curProcessTime.init(1);
			//当前激励时间入map
			processTimes[processName] = curProcessTime;
			//curProcessTime.showPinterval(processName);
			break;
		case 2:
			//获取当前激励时间
			curProcessTime = processTimes[processName];
			//修改当前激励结束时间,并计算激励经过的时间
			curProcessTime.init(2);
			curProcessTime.showInterval("");
			//写入激励
			currentTestCase->setProcessStatusWithNameAndStatus(processName,
				this->ltos(curProcessTime.getInterval()));
			currentTestCase->setcurrentProcessExecStatus(processName);
			break;
	}
}

void SupportClass::setCurProcessResult(string processName, struct timeval startTime, struct timeval endTime) {
	double r = 1000.0 * (endTime.tv_sec - startTime.tv_sec)
		+ (endTime.tv_usec - startTime.tv_usec) / 1000.0;
	//写入激励
	currentTestCase->setProcessStatusWithNameAndStatus(processName,
		this->ltos(r));
	currentTestCase->setcurrentProcessExecStatus(processName);
}

void SupportClass::setCurProcessResult(string processName, long mtime,
                                       int flag) {
  //if (flag == 1) {
	 // timeHandle(processName,1);
  //}
  //if (flag == 2) {
	 // timeHandle(processName, 2);
  //}
}
/*	路径操作 */
void SupportClass::showTestExecPath() {
  //int index = 0;
  //string str = "";
  ////定义list的迭代器
  //list<string>::iterator iter;
  ////进行迭代遍历
  //for (iter = testExecPath.begin(); iter != testExecPath.end(); iter++) {
  //  cout << (*iter) << endl;
  //}
}
void SupportClass::cleanTestExecPath() { this->testExecPath.clear(); }

/*	类型转换 */
string SupportClass::ltos(double l) {
  ostringstream os;
  os << l;
  string result;
  istringstream is(os.str());
  is >> result;
  return result;
}

/*  信号处理以及设置定时器 */
void signal_handler(int signum) {
  static int flag = 0;

  switch (signum) {
  //用alarm函数设置的timer超时或setitimer函数设置的interval timer超时
  case SIGALRM:
    if (flag == 0) {
      cout << "程序异常或出现死循环，3秒后退出！\n";
      alarm(3);
    } else {
      //传送信号给指定的进程 ,SIGKILL:中止某个进程
      //对出错的用例进行操作处理
      string str = shared->result;
      string exeSituation = "ERROR";
      currentTestCase->setCurrenetTestCaseExecStatus(exeSituation);
      currentTestCase->setCurrenetTestCaseResultStatus(exeSituation);
      string tmp = currentTestCase->showTestCase();
      if (shared->currentIndex < shared->count)
        tmp += "\n*\n";
      strcat(shared->result, tmp.c_str());
      shared->currentIndex++;
	  shared->errorFlag = false;
      if (shmdt(shm) == -1) {
        cout << "共享内存从当前进程中分离失败!" << endl;
        exit(EXIT_FAILURE);
      }
      kill(getppid(), SIGKILL);
      //挂起 500 ms
      usleep(500);
      exit(EXIT_SUCCESS);
    }
    flag = 1;
    break;
  }
}

void SupportClass::setHandlerAndAlarm(int seconds) { alarm(seconds); signal(SIGALRM, signal_handler); }

//void SupportClass::setAlarm(int seconds) { alarm(seconds); }

/* 共享内存 创建并准备 */
void SupportClass::createMem() {
  int shmid; //共享内存标识符

  // 1.创建共享内存
  shmid = shmget((key_t)1111, sizeof(struct shared_use_st), 0666 | IPC_CREAT);

  if (shmid == -1) {
    cout << "打开共享内存失败!" << endl;
    exit(EXIT_FAILURE);
  } else {
    //cout << "打开共享内存成功，大小为" << sizeof(struct shared_use_st) << endl;
  }

  // 2.将共享内存连接到当前进程的地址空间
  shm = shmat(shmid, 0, 0);
  if (shm == (void *)-1) {
    cout << "共享内存连接到当前进程失败!" << endl;
    exit(EXIT_FAILURE);
  } else {
    //cout << "共享内存连接到当前进程成功!" << endl;
  }
  // 3.设置共享内存
  shared = (struct shared_use_st *)shm;
  // 4.统计所有用例的数目
  cout << "测试用例数目：" << shared->count << endl;
}

/* 将 测试用例实体集 放入 共享内存中 */
void SupportClass::putTestCasesInMem() {
  //更新 数据区域
  string str = shared->result, tmp = currentTestCase->showTestCase();
  //封装测试用例结果
  if (this->currentIndex < shared->count)
    tmp += "\n*\n";
  strcat(shared->result, tmp.c_str());
  // if(this->currentIndex < this->count)
  this->currentIndex++;
  //更新 共享内存区中 当前测试用例号
  this->setCurrentIndex();
}

/* 字符串 按某 字符 分割成list数组 */
list<string> SupportClass::stringSplit(string s, const char *str) {
  int l = 0;
  int r = 0;
  list<string> arr;
  string tmp(str);
  while (r != std::string::npos) {
    r = s.find_first_of(tmp, l);
    if (r != std::string::npos)
      arr.push_back(s.substr(l, r - l));
    else
      arr.push_back(s.substr(l, s.length() - l));
    l = r + tmp.length();
  }
  return arr;
}

/* 从 共享内存中 读写测试用例实体集 */
void SupportClass::getTestCasesInMem() {

  string testcase = shared->text, tmp = "testcCaseID:";

  //获取 当前测试用例的字符串
  int begin = testcase.find(tmp + to_string(this->currentIndex));
  //如果是最后一条测试用例则 是最大长度
  int len = 0;
  if (this->currentIndex == shared->count)
    len = testcase.size() - begin;
  else
    len = testcase.find(tmp + to_string(this->currentIndex + 1)) - begin;
  string tcStr = testcase.substr(begin, len - 1);
  //根据ID 构造测试用例实体
  currentTestCase = new TestCase(this->currentIndex);

  //构造激励链表
  tmp = "-->processList:[";
  begin = tcStr.find(tmp) + tmp.size() + 2;
  tmp = "-->execStatus:[";
  len = tcStr.find(tmp) - begin - 3;
  string pStr = tcStr.substr(begin, len);
  list<string> plist = this->stringSplit(pStr, "\n");

  //定义list的迭代器
  list<string>::iterator iter;
  //进行迭代遍历
  for (iter = plist.begin(); iter != plist.end(); iter++) {
    tmp = "ProcessName:";
    begin = (*iter).find(tmp) + tmp.size();
    tmp = "(";
    len = (*iter).find(tmp) - begin;
    string processName = (*iter).substr(begin, len - 1);
    tmp = "ProcessParameter:";
    begin = (*iter).find(tmp) + tmp.size();
    tmp = "ProcessStatus:";
    len = (*iter).find(tmp) - begin;
    string processParameter = (*iter).substr(begin, len);
    processParameter.erase(processParameter.find_last_not_of("\t")+1);
    currentTestCase->setProcessList(processName, processParameter, "");
  }
}

/* 分离 共享内存 */
void SupportClass::pullMem() {
  //把共享内存从当前进程中分离
  if (shmdt(shm) == -1) {
    cout << "共享内存从当前进程中分离失败!" << endl;
    exit(EXIT_FAILURE);
  }else{
    cout << "共享内存从当前进程中分离成功!" << endl;
  }
}