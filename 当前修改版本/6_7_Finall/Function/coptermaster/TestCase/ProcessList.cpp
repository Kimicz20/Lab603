#include "ProcessList.h"
using namespace std;

ProcessList::ProcessList() { this->InitList(&(this->processList)); }

ProcessList::~ProcessList() {
  if (!this->processList) {
    delete this->processList;
  }
}

/* 设置执行状态 */
Status ProcessList::setExecStatus(string processName) {
  LinkList p = this->findProcessWithName(processName);
  //LinkList p = this->findProcessWithID(atoi(processName.c_str()));
  if (p) {
	//cout<<visit(p)<<endl;
    p->isExecOK = TRUE;
    return OK;
  }
  return ERROR;
}
/* 单链表初始化 */
Status ProcessList::InitList(LinkList *L) {
  /* 产生头结点，并使L指向次头结点 */
  *L = new Process{0, FALSE, "", "", "", NULL};
  if (!(*L)) /* 存储分配失败 */
  {
    return ERROR;
  }
  (*L)->next = NULL; /* 指针域为空 */

  return OK;
}

/* 返回当前链表 */
LinkList ProcessList::getProcessList() { return this->processList; }

/* 创建激励节点 */
LinkList ProcessList::createProcessNode(string processName,
                                        string processParameter,
                                        string processStatus) {
  //创建激励结构体,并赋值结构体数据,指针域为空
  int processID = this->ListLength() + 1;
  LinkList p = new Process{processID,        FALSE,         processName,
                           processParameter, processStatus, NULL};
  if (!p) {
    cout << "创建激励节点失败";
  }
  return p;
}

/* 根据名称 查找对应激励 */
LinkList ProcessList::findProcessWithName(string processName) {
  LinkList p = this->processList->next;
  while (p) {
    if (p->processName == processName && !(p->isExecOK)) {
      return p;
    }
    p = p->next;
  }
  return NULL;
}

/* 根据ID 查找对应激励 */
LinkList ProcessList::findProcessWithID(int ID) {
  LinkList p = this->processList->next;
  while (p) {
    if (p->processID == ID) {
      return p;
    }
    p = p->next;
  }
  return NULL;
}

/* 字符串 按某 字符 分割成list数组 */
list<string> stringSplit(string s, const char *str) {
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

string findValue(string str,string key){
	int pos = str.find("=");
	if (str.substr(0, pos) == key)
		return str.substr(pos + 1, str.size());
	return "";
}

/* 查找对应参数 的 数值 */
string ProcessList::findValueWithProcessNameAndKey(string processName,
                                                   string key) {
	
  LinkList p = this->findProcessWithName(processName);
	/*LinkList p = this->processList->next;
	while (p) {
		if (p->processName == processName) {
			break;
		}
		p = p->next;
	}*/
  if (p){
	  string tmp = p->processParameter;
	  //激励参数为空
	  if (tmp == "NULL")
		  return "";
	  
	  list<string> plist = stringSplit(tmp, ",");
	  //进行迭代遍历
	  for (list<string>::iterator iter = plist.begin(); iter != plist.end(); iter++) {
		  string t = findValue(*iter, key);
		  if (t != ""){
			  return t;
		  }
	  }
  }	
  return "";
}

/* 根据激励ID以及对应状态 修改 */
Status ProcessList::setProcessStatus(string processName, string status) {
	
  LinkList p = this->findProcessWithName(processName);
	//LinkList p = this->findProcessWithID(atoi(processName.c_str()));
  if (p) {
    p->processStatus = status;
    return OK;
  }
  return ERROR;
}

/* 操作结果：在L中插入新节点s，L的长度加1 */
Status ProcessList::ListInsert(LinkList s) {
  LinkList *L = &(this->processList);
  LinkList p;
  p = *L;
  while (p->next) /* 寻找最后一个结点 */
  {
    p = p->next;
  }
  s->next = p->next; /* 将p的后继结点赋值给s的后继结点 */
  p->next = s;       /* 将s赋值给p的后继结点 */
  return OK;
}

/* 初始条件：单链表已存在。操作结果：返回L中数据元素个数 */
int ProcessList::ListLength() {
  int i = 0;
  LinkList p = this->processList->next; /* p指向第一个结点 */
  while (p) {
    i++;
    p = p->next;
  }
  return i;
}

/* 遍历链表时显示函数 */
string ProcessList::visit(LinkList p) {
  string str = "";
  string processID = to_string(p->processID);
  str = str +"\n\t" + (p->isExecOK?"#":"&")+" ProcessID:"+processID;
  str = str +"\tProcessName:"+p->processName+" ( ";
  str = str +"\tProcessParameter:"+((p->processParameter == "")?"NULL":p->processParameter);
  str = str +"\tProcessStatus:"+((p->processStatus == "")?"NULL":p->processStatus)+" ) ";
  return str;
}

/* 操作结果：依次对L的每个数据元素输出 */
string ProcessList::ListTraverse() {
  string str = "";
  LinkList p = this->processList->next;
  while (p) {
    str += visit(p);
    p = p->next;
  }
  return str;
}

/* 操作结果：若L为空表，则返回TRUE,否则返回FALSE */
Status ProcessList::ListIsEmpty() {
  if (this->processList->next) {
    return FALSE;
  } else {
    return TRUE;
  }
}

/* 操作结果：将L重置为空表 */
Status ProcessList::ClearList() {
  LinkList *L = &(this->processList);
  LinkList p, q;
  p = (*L)->next; /* p指向第一个结点 */
  while (p)       /* 没到表尾 */
  {
    q = p->next;
    delete (p);
    p = q;
  }
  (*L)->next = NULL; /* 头结点指针域置为空 */

  return OK;
}

/* 循环查询链表中 每个激励的状态是否 为OK*/
string ProcessList::findIsSuccessTest() {
  LinkList p = this->processList->next;
  while (p) {
    if (!p->isExecOK) {
      return "失败";
    }
    p = p->next;
  }
  return "成功";
}

/* 获取当前激励链表 首位的时间*/
string ProcessList::getFirstProcessTime() {
  LinkList p = this->processList->next;
  return p->processStatus;
}

string ProcessList::getProcessNameWithId(int index)
{
    string str = "";
    int i=1;
    LinkList p = this->processList->next;
    while (p)
    {
        if(index == i )
          return p->processName;
        i++;
        p=p->next;
    }
    return str;
}
