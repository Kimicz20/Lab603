// Implementation of the ServerSocket class

#include "ServerSocket.h"

using namespace std;

ServerSocket::ServerSocket ( int port )
{

  sock = new Socket();

  if ( ! sock->create() )
    {
      throw SocketException ( "不能创建Socket连接." );
    }

  if ( ! sock->bind ( port ) )
    {
      throw SocketException ( "不能绑定该端口." );
    }

  if ( ! sock->listen() )
    {
      throw SocketException ( "不能监听该Socket端口." );
    }
    cout << "。。。等待链接。。。" <<endl;
    sock->accept ();
}

void ServerSocket::receive ( std::string& s )
{
  if ( ! sock->recv ( s ) )
    {
      throw SocketException ( "不能读取该Socket." );
    }
}
void ServerSocket::send (std::string s)
{
  if ( ! sock->send ( s ) )
    {
      throw SocketException ( "不能改写该Socket." );
    }
}

void ServerSocket::accept ()
{
  if ( ! sock->accept () )
    {
      throw SocketException ( "不能建立该Socket." );
    }
}

void ServerSocket::sendResult(std::string s){
    try
    {
      sock->send(s);
    }
    catch ( SocketException& e )
    {
      cout << "Exception was serversocket caught:" << e.description() << "\nExiting.\n";
    }
}

string ServerSocket::serverReceive(){
   try
    {
      return sock->recvWithFile();
    }
  catch ( SocketException& e )
    {
      cout << "Exception was serversocket caught:" << e.description() << "\nExiting.\n";
    }
}
