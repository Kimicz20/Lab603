#ifndef Socket_class
#define Socket_class

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include "SocketException.h"

const int MAXHOSTNAME = 200;
const int MAXCONNECTIONS = 5;
const int MAXRECV = 500;
const int PACK_SIZE = 1024*512;

class Socket
{
 public:
  Socket();

  // 服务器初始化
  bool create();
  bool bind ( const int port );
  bool listen() const;
  bool accept ();

  // 数据传输
  int recv ( std::string& ) const;
    //从文件传输
  std::string recvWithFile()const;
  bool send ( const std::string ) const;
  bool is_valid() const { return listenfd != -1; }

 private:
  int listenfd,connectfd;                   //socket文件标志
  struct sockaddr_in server_addr;//socket地址
};


#endif
