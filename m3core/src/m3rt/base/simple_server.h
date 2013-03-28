/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef  M3RT_SIMPLE_SERVER_H
#define  M3RT_SIMPLE_SERVER_H

#include <string>
#include <vector>
#include <sys/select.h>

namespace m3rt
{
	using namespace std;
class M3SimpleServer
{
public:
	M3SimpleServer():nb_rx(0),buf_rx(0),nb_tx(0),buf_tx(0),socket_fd(-1),listener(-1){}
	~M3SimpleServer();
	bool Startup(int port);//Non-blocking
	void Shutdown();
	int WriteStringToPort(string & s); //Non-blocking
	int  ReadStringFromPort(string & s, int & size);//Non-blocking
protected:
	bool HandleNewConnection();//Non-blocking
	bool IsActiveSocket(){return socket_fd!=-1;}
	int portno;
	unsigned char * buf_rx;
	int nb_rx;
	unsigned char * buf_tx;
	int nb_tx;
	fd_set master;   // master file descriptor list
	fd_set read_fds; // temp file descriptor list for select()
	fd_set write_fds; // temp file descriptor list for select()
	int fdmax;        // maximum file descriptor number
	int socket_fd;
	int listener;     // listening socket descriptor
	struct timeval tv;
};
}
#endif


