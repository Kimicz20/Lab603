#include "ServerSocket.h"
#include "SupportClass.h"

using namespace std;

#define POST 5555
extern ServerSocket *serverSocket = NULL;
extern SupportClass *supt = new SupportClass();

int main() {
  
  	string file_name;
	//1.Socket connect
	serverSocket = new ServerSocket(POST);
	//2.receive file
	file_name = serverSocket->serverReceive();
	supt->witType(file_name);
	//3.handle 
	if (supt->initPutIn2Men(file_name)){

		//4. calculate piece
		int count =  supt->getTestCaseCount();
		int t = count/COUNT_TS_ONCE;
		count = count%COUNT_TS_ONCE == 0?t:(t+1);
		int ERRORNUM = 0, a[COUNT_TS_ONCE];
		//5.execute each piece
		for (int i = 1; i <= count; ++i)
		{
			supt->Logger("######"+to_string(i)+"######");
			//7.put each piece data to mem
			if(supt->countTestCases(i)){
			  // supt->Logger(supt->showResult());
			 
			  do {
				  //exception exit ,flag is true
				  supt->flag = false;
				  supt->createPidAndPolling();
				  if (supt->flag){
					  a[ERRORNUM++] = supt->getCurrentIndex() - 1;
				  }
			  } while (supt->flag && supt->getCurrentIndex() <= COUNT_TS_ONCE); 
			  
			  supt->write2File(i);
			  serverSocket->sendResult("index#"+to_string(i));
			}  
		}// end for

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
	  serverSocket->sendResult("exit");
	  supt->pullMem();
	}
  return 0;
}
