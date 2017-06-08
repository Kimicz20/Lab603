#include "ServerSocket.h"
#include "SupportClass.h"

using namespace std;

#define POST 5555
extern ServerSocket *serverSocket = NULL;
extern SupportClass *supt = new SupportClass();

int main() {
  
  int modelChoice=0;
  char isContinue;
  string file_name;

  do{
  	  int type;
  	  cout << "选择测试模式:1.功能 2.性能" << endl;
  	  cin >>type;
  	  switch(type){
  	  	case 1:
  	  		supt->type = "Function";
  	  		break;
  	  	case 2:
  	  		supt->type = "Performance";
  	  		break;
  	  	default:
		  	break;
  	  }
	  cout << "1.本地 2.远程" << endl;
	  cin >> modelChoice;
	  switch (modelChoice){
	  case 1:
		  cout << "测试文件名称：" << endl;
		  int num;
		  cin >> num;
		  switch (num){
			  case 1:
				  file_name = "testcase/1L.xml";
					  break;
			  case 2:
				  file_name = "testcase/1Z.xml";
					  break;
			  case 3:
				  file_name = "testcase/2L.xml";
					  break;
			  case 4:
				  file_name = "testcase/2Z.xml";
					  break;
			  default:
				  continue;
		  }
		  break;
	  case 2:
		   //1.创建连接Socket
		  serverSocket = new ServerSocket(POST);
		   //1.1文件接收
		  file_name = serverSocket->serverReceive();
		  break;
	  default:
		  continue;
	  }

	   //1.测试用例放入共享内存中
	  if (supt->putTestCasesInMem(file_name)){
		  //2.创建子程序
		  int ERRORNUM = 0, a[supt->getTestCaseCount()];
		  do {
			  //出现异常时 flag 为true
			  supt->flag = false;
			  supt->createPidAndPolling();

			  /*std::cout << "当前测试用例ID:" << supt->getCurrentIndex()
			  << "	所有测试用例数目:" << supt->getTestCaseCount()
			  << std::endl;
			  */
			  if (supt->flag){
				  a[ERRORNUM++] = supt->getCurrentIndex() - 1;
			  }
		  } while (supt->flag && (supt->getCurrentIndex() <=
			  supt->getTestCaseCount()));
		  if (serverSocket !=  NULL){
			  serverSocket->sendResult(supt->showResult());
		  }
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
		  supt->write2File();
		  //cout<<"\n\t成功用例结果:\n"<<supt->showResult()<<endl;
		  supt->pullMem();
	  }

	  cout << "是否继续执行：y/n" << endl;
	  cin >> isContinue;
  } while (isContinue == 'y');
  return 0;
}
