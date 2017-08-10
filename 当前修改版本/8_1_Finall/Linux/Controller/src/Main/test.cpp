#include "ServerSocket.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
using namespace std;

#define POST 5555
extern ServerSocket *serverSocket = NULL;

/**
* 1. 建立连接
* 2. 发送http请求sftp服务
* 3. 文件发送
*/

/* 获取项目路径 */
string getProjectPath() {
  //获取程序路径
  char buf[1024];
  getcwd(buf, 1024);
  string fulldir(buf);
  string tmp = "/Controller";
  fulldir = fulldir.substr(0, fulldir.length() - tmp.length());
  return fulldir;
}

string readfile(string filename){
	ifstream ifile(filename);
	//将文件读入到ostringstream对象buf中
	ostringstream buf;
	char ch;
	while(buf&&ifile.get(ch))
		buf.put(ch);
	//返回与流对象buf关联的字符串
	return buf.str();
}

string buildURL(string file_name){
	string url = "http://192.168.0.130:6666/get?cmd=";
    string src = "root@192.168.0.131:";
    string fullPath = getProjectPath() + "/testcase/" + file_name;
    return url + "scp%20" + src + fullPath + "%20" + "/home/Test/testcase/";
}

int main() {
  
	//1.创建连接Socket
	serverSocket = new ServerSocket(POST);
	//2.文件接收
	string file_name = serverSocket->serverReceive();
	// string file_name = "2X.xml";
	//3.调用http请求
	execlp("curl","curl",buildURL(file_name).c_str());
	
	string filePath = "/home/result/result.txt";
	serverSocket->sendResult(readfile(filePath));
  return 0;
}
