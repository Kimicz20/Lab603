// Definition of the ServerSocket class
#ifndef ServerSocket_class
#define ServerSocket_class

#include "Socket.h"

class ServerSocket : private Socket
{
 private:
 	 Socket *sock;
 public:
     ServerSocket ( int port );
     void receive ( std::string &s);
     void send (std::string s);
     void accept ();
     std::string serverReceive();
     void sendResult(std::string s);
};


#endif
