#include "SupportClass.h"

using namespace std;
extern SupportClass *supt = new SupportClass();

int main(int argc,char *argv[]) {
     
    string file_name; 
    if(argv[1] != NULL){
    	cout<<"file_name:"<<argv[1]<<endl;
		file_name = argv[1];
    }

  	supt->type = "Function";

	//  1.测试用例放入共享内存中
	if (supt->putTestCasesInMem(file_name)){
	  //2.创建子程序
	  int ERRORNUM = 0, a[supt->getTestCaseCount()];
	  do {
		  //出现异常时 flag 为true
		  supt->flag = false;
		  supt->createPidAndPolling();
		  if (supt->flag){
			  a[ERRORNUM++] = supt->getCurrentIndex() - 1;
		  }
	  } while (supt->flag && (supt->getCurrentIndex() <=
		  supt->getTestCaseCount()));
	  
	  //write into file ,fullPath
	  supt->write2File();
	  
	  cout << "最终结果,测试的用例 统计: "
		  << "\n\t总条数：" << supt->getTestCaseCount()
		  << "\n\t错误总数：" << ERRORNUM;
	  if (ERRORNUM > 0){
		  cout << "\n\t出错序号：" << a[0];
		  for (int i = 1; i < ERRORNUM; ++i)
		  {
			  cout << " " << a[i];
		  }
	  }
	  supt->pullMem();
	}
  return 0;
}
