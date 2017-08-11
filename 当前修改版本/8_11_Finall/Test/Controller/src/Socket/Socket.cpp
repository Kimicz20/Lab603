
#include "Socket.h"
using namespace std;

//通过初始化列表来出书画
Socket::Socket():listenfd ( -1 )
{
  memset (&server_addr,0,sizeof( server_addr ));
}

bool Socket::create()
{
  listenfd = socket ( AF_INET,SOCK_STREAM,0 );
  if ( ! is_valid() ){
    cout<<"Socket error:\n";
    return false;
  }
  return true;
}


bool Socket::bind ( const int port )
{
  if ( ! is_valid() )
    {
      return false;
    }
  /* 服务器端填充 sockaddr结构  */
  bzero(&server_addr,sizeof(struct sockaddr_in));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons ( port );

  int bind_return = ::bind ( listenfd,
			     ( struct sockaddr * ) (&server_addr),
			     sizeof ( server_addr ) );

  if ( bind_return == -1 )
    {
      cout<<"Bind error!\n";
      return false;
    }

  return true;
}


bool Socket::listen() const
{
  if ( ! is_valid() )
    {
      return false;
    }

  int listen_return = ::listen ( listenfd, MAXCONNECTIONS );

  if ( listen_return == -1 )
    {
    	cout<<"Listen error\n";
      return false;
    }

  return true;
}


bool Socket::accept () 
{
  int addr_length = sizeof (struct sockaddr_in);
  connectfd = ::accept ( listenfd, ( sockaddr * ) &server_addr, ( socklen_t * ) &addr_length );

  // TIME_WAIT - argh
  int on = 1;
  if ( setsockopt ( connectfd, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 )
    return false;

  if ( connectfd <= 0 ){
    cout<<"Accept error\n";
    return false;
  }
  else{
    cout<<"...客户端已经连接... "<<inet_ntoa(server_addr.sin_addr)<<"\n";
    return true;
  }
}


bool Socket::send ( const string s ) const
{
  int status = ::send ( connectfd, s.c_str(), s.size(), 0 );
  if ( status == -1 )
    {
		cout << "传输出错" << endl;
      return false;
    }
  else
    {
	    close(connectfd);
      close(listenfd);
      return true;
    }
}

int Socket::recv ( string& s ) const
{
  char buf [ MAXRECV + 1 ];

  s = "";

  memset ( buf, 0, MAXRECV + 1 );

  int status = ::recv ( connectfd, buf, MAXRECV, 0 );

  if ( status == -1 )
    {
      cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
      return 0;
    }
  else if ( status == 0 )
    {
      return 0;
    }
  else
    {
      s = buf;
      return status;
    }
}

vector<string> stringSplit(string s, const char *str) {
  int l = 0;
  int r = 0;
  vector<string> arr;
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

string Socket::recvWithFile()const
{
    string file_name="";
    Socket::recv(file_name);
    return file_name;
}
