
// Implementation of the SupportClass class
#include "SupportClass.h"

using namespace std;
extern TestCase *currentTestCase = new TestCase(); //当前测试用例
shared_use_st *shared;                             //共享内存区
void *shm = NULL; //分配的共享内存的原始首地址
SupportClass::SupportClass() {
  this->createMem();
  this->currentIndex = shared->currentIndex;
}

/* 设置当前 测试用例实体类 从内存中获取*/
void SupportClass::setCurrentTestCase() { this->getTestCasesInMem(); }

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
* 如果执行中没有
*	1. 正常执行 
*		[x:x] 第一个表示所用激励执行情况 1 有误，2 无误
*			  第二个表示是否满足约束不等式 1 不满足 ，2 满足
*	2. 按照用例执行不会出现死循环以及异常错误 但用例数据出现错误
*/
void SupportClass::setCurTestCaseResult(string exe) {

	RETURN_TYPE t;
	//正常执行情况
	if (exe == "OK"){
		t = resultHandle();
		t.isERROR = false;
		currentTestCase->setCurrenetTestCaseStatus(t);
	}
}

/* 在当前 测试用例 中根据激励名称以及参数名 获取参数值 */
int SupportClass::getParamValueWithNameAndKey(string processName, string key) {

  string value = currentTestCase->getParamValueWithNameAndKey(processName, key);
  //数据值 处理：bool ,string -> int 类型
  if (value == ""){
	  return -1;
  }if (value == "true") {
    return 1;
  } else if (value == "false") {
    return 0;
  } else {
    return atoi(value.c_str());
  }
}

/*记录当前激励情况:路径记录以及状态回写
*	1、开始时间
*	2、结束时间
*	3、执行状态
*/
void SupportClass::setCurProcessResult(string processName, double mtime) {

    /* 根据激励名称以及对应状态 修改 */
	result[processName] = currentTestCase->setProcessStatusWithNameAndStatus(processName,
		this->d2s(mtime));
	/* 设置当前激励的执行状态 */
	currentTestCase->setcurrentProcessExecStatus(processName);
}

/*	类型转换 */
string SupportClass::d2s(double l) {
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
	  RETURN_TYPE t;
	  t.isERROR = true;
	  currentTestCase->setCurrenetTestCaseStatus(t);
      string tmp = currentTestCase->showTestCase();
      if (shared->currentIndex < shared->count)
        tmp += "\n*\n";
      strcat(shared->result, tmp.c_str());
      shared->currentIndex++;
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

void SupportClass::setHandler() { signal(SIGALRM, signal_handler); }

void SupportClass::setAlarm(int seconds) { alarm(seconds); }

/* 共享内存 创建并准备 */
void SupportClass::createMem() {
  int shmid; //共享内存标识符

  // 1.创建共享内存
  shmid = shmget((key_t)2222, sizeof(struct shared_use_st), 0666 | IPC_CREAT);

  if (shmid == -1) {
    cout << "打开共享内存失败!" << endl;
    exit(EXIT_FAILURE);
  }

  // 2.将共享内存连接到当前进程的地址空间
  shm = shmat(shmid, 0, 0);
  if (shm == (void *)-1) {
    cout << "共享内存连接到当前进程失败!" << endl;
    exit(EXIT_FAILURE);
  }
  // 3.设置共享内存
  shared = (struct shared_use_st *)shm;
  // 4.统计所有用例的数目
  cout << "ELS 测试用例数目：" << shared->count << endl;
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
	//1.激励名称
    tmp = "ProcessName:";
    begin = (*iter).find(tmp) + tmp.size();
    tmp = "(";
    len = (*iter).find(tmp) - begin;
    string processName = (*iter).substr(begin, len -1);
    
	//2.激励参数
	tmp = "ProcessParameter:";
    begin = (*iter).find(tmp) + tmp.size();
    tmp = "ProcessStatus:";
    len = (*iter).find(tmp) - begin;
    string processParameter = (*iter).substr(begin, len);
	processParameter.erase(processParameter.find_last_not_of("\t") + 1);

	//3.激励状态
	begin = (*iter).find(tmp) + tmp.size();
	tmp = ")";
	len = (*iter).find(tmp) - begin;
	string processStatus = (*iter).substr(begin, len-1);
	currentTestCase->setProcessList(processName, processParameter, processStatus);
  }

  //读取时间约束信息
  tmp = "timeLimit:";
  begin = tcStr.find(tmp) + tmp.size();
  tmp = "]";
  len = tcStr.rfind(tmp)- begin;
  string timeLimit = tcStr.substr(begin, len);
  currentTestCase->setResultStatus(timeLimit);

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

/* 记录处理时间 */
void SupportClass::timeHandle(string processName,int f,string preProcessName){
	
	processTime curProcessTime, preProcessTime;
	switch (f){
		case 0:
			if (!processTimes.empty())
				processTimes.clear();
			curProcessTime.init(0);
			processTimes[processName] = curProcessTime;
			break;
		case 1:
			//如果key重复
			curProcessTime.init(1);
			//计算当前激励与上一个激励之间的时间差值
			preProcessTime = processTimes[preProcessName];
			curProcessTime.pinterval = curProcessTime.setPinterval(preProcessTime);
			//当前激励时间入map
			processTimes[processName] = curProcessTime;
			curProcessTime.showPinterval(processName);
			//写入用例中
			setCurProcessResult(processName, curProcessTime.pinterval);
			break;
		case 2:
			//获取当前激励时间
			curProcessTime = processTimes[processName];
			//修改当前激励结束时间,并计算激励经过的时间
			curProcessTime.init(2);
			break;
	}
}

//三元组转换
map<string, double> map2map(map<string, pair<string, string>> res){
	map<string, double> result;
	for (map<string, pair<string, string>>::iterator mapIter = res.begin(); mapIter != res.end(); mapIter++) {
		
		pair<string, string> p = mapIter->second;
		//cout << mapIter->first << "," << p.first << "," << p.second << endl;
		// 有用元组
		if (p.first != "0")
			result[p.first] = atof(p.second.c_str());
	}
	return result;
}


/* 处理不等式*/
RETURN_TYPE SupportClass::resultHandle(){
	RETURN_TYPE finalResult;
	finalResult.isOK = true;
	list<string> slist = stringSplit(currentTestCase->getResultStatus(), ",");

	//解析不等式字符串
	for (list<string>::iterator iter = slist.begin(); iter != slist.end(); iter++) {
		inequation t;

		//不等式转换
		t.transfer(*iter, map2map(result));
		
		if (!t.isOK){
			finalResult.errorInfo.push_back(t.strInequation);
		}
	}

	if (finalResult.errorInfo.size() !=0)
		finalResult.isOK = false;
	return finalResult;
}
