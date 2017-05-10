#include "TestCase.h"
using namespace std;

/* 初始化操作 */
TestCase::TestCase(int testCaseID)
{
    this->testCaseID = testCaseID;
    this->processList = new ProcessList();
    this->execStatus = "";
    this->resultStatus = "";
}
TestCase::~TestCase()
{
    delete this->processList;
}

int TestCase::getTestCaseID(){
    return this->testCaseID;
}

int TestCase::getProcessNum(){
    return this->processList->ListLength();
}

string TestCase::getExecStatus(){
    return this->execStatus;
}

string TestCase::getResultStatus(){
    return this->resultStatus;
}

/* 获取当前激励链表 首位的时间*/
string TestCase::getFirstProcessTime(){
    return this->processList->getFirstProcessTime();
}

/* 设置当前激励执行状态 */
void TestCase::setcurrentProcessExecStatus(string processName){
    this->processList->setExecStatus(processName);
}

/* 打印测试用例 */
string TestCase::showTestCase(){
  string str = "";
  string testCaseID = to_string(this->testCaseID);
  str = str + "testcCaseID:"+testCaseID+"\n-->processList:[";
  if(this->processList->ListIsEmpty()){
      str += "NULL";
  }else {
      str += this->processList->ListTraverse();
  }
  str = str + "]\n-->execStatus:[";
  if(this->execStatus == ""){
      str +="NULL";
  }else{
      str += this->execStatus;
  }
  str +="]\n-->resultStatus:[";
  if(this->resultStatus == ""){
      str +="NULL";
  }else{
      str += this->resultStatus;
   }
  str +="]";
  return str;
}

/* 设置测试执行状态 */
void TestCase::setProcessList(string processName,string processParameter,string processStatus){
    //1.创建一个激励节点
    LinkList s = this->processList->createProcessNode(processName,processParameter,processStatus);
    //2.添加激励节点在激励链表中
    this->processList->ListInsert(s);
}

/* 根据激励名称 获取激励节点 */
LinkList TestCase::findProcessWithName(string processName){
    return this->processList->findProcessWithName(processName);
}
/* 根据激励ID 获取激励节点 */
LinkList TestCase::findProcessWithID(int ID){
    return this->processList->findProcessWithID(ID);
}

/* 从测试用例中 根据激励名称以及参数名 获取参数值*/
string TestCase::getParamValueWithNameAndKey(string processName,string key){
    return this->processList->findValueWithProcessNameAndKey(processName,key);
}

/* 根据激励名称以及对应状态 修改 */
void TestCase::setProcessStatusWithNameAndStatus(string processName,string status){
    this->processList->setProcessStatus(processName,status);
}

/* 设置当前测试用例的执行状态*/
void TestCase::setCurrenetTestCaseExecStatus(string execStatus){
    if(execStatus != "ERROR"){
        if(this->processList->findIsSuccessTest() == "失败"){
            execStatus = "1:"+execStatus;
        }else{
            execStatus = "2:"+execStatus;
        }
    }else{
        execStatus = "3";
    }
    this->execStatus = execStatus;
}
/* 设置结果状态 */
void TestCase::setCurrenetTestCaseResultStatus(string exeSituation){
    //正常执行
    if(exeSituation != "ERROR"){
        if(this->processList->findIsSuccessTest() == "失败" ){
            exeSituation = "1";
        }else{
            exeSituation = "2:"+exeSituation;
        }
    }else{
        exeSituation = "3";
    }
    this->resultStatus = exeSituation;
}

string TestCase::getProcessNameWithId(int index){
  return this->processList->getProcessNameWithId(index);
}
