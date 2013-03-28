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


#include <m3rt/base/simple_server.h>
#include <m3rt/base/toolbox.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <netinet/in.h>
#include <arpa/inet.h>
#include "string.h"
#include <unistd.h>

namespace m3rt
{
	
M3SimpleServer::~M3SimpleServer()
{
	Shutdown();
}

void M3SimpleServer::Shutdown()
{
	if (buf_rx!=NULL)
		delete [] buf_rx;
	buf_rx=NULL;
	nb_rx=0;
	if (buf_tx!=NULL)
		delete [] buf_tx;
	buf_tx=NULL;
	nb_tx=0;
	if (socket_fd!=-1)
		close(socket_fd);
	if (listener!=-1)
		close(listener);

	fdmax=-1;
	socket_fd=-1;
	listener=-1;
	
}


//Return numbytes write, -1 if error, 0 if none
int M3SimpleServer:: WriteStringToPort(string & s)
{
	if (!IsActiveSocket())
		return 0;
	unsigned char * data = (unsigned char *) s.data();
	int size=s.size()+sizeof(int);
	//Grow buffer as needed
	if (nb_tx<size)
	{
		if (buf_tx!=NULL)
			delete [] buf_tx;
		buf_tx=new unsigned char [size];
		nb_tx=size;
	}
	
	((int*)buf_tx)[0]=s.size();
	memcpy(buf_tx+sizeof(int),data,s.size());
	write_fds = master; // copy it
	int nfd=select(fdmax+1, NULL, &write_fds, NULL, &tv);
	if (nfd== -1)//error 
	{
       M3_ERR("ERROR on select for Port %d\n",portno);
       return -1;
    }
	if (FD_ISSET(socket_fd, &write_fds))
	{
		int total = 0;        // how many bytes we've sent
		int bytesleft =size; // how many we have left to send
		//Send out port (possibly in multiple sends)
		while(total < size)
		{
			int n = send(socket_fd,buf_tx+total, bytesleft,0);
			if (n == -1) 
			{ 
				M3_ERR("ERROR writing to socket: %d\n",n);
				return false; 
			}
			total += n;
			bytesleft -= n;
		}
		return total;
	}
	return 0;	
}

#define MAX_STRING_SIZE 8192 //as a safety measure
//Return true if a packet is successfully recieved. Size is the data size minus the header.
// return -1 if error
// return -2 if no cmd data


int M3SimpleServer::ReadStringFromPort(string & s, int & size)
{
	//In theory can be more than one client writing to port. This shouldn't happen tho.
	int nr;
	int header;
	if (!IsActiveSocket())
	{
		HandleNewConnection();
		return 0;
	}
	tv.tv_sec=1; //0.5 second timeout
	//tv.tv_usec=500000;
	tv.tv_usec=0;
	read_fds = master; // copy it
	int nfd=select(fdmax+1, &read_fds, NULL, NULL, &tv);
	if (nfd== -1)//error 
	{
       M3_ERR("ERROR on select for Port %d\n",portno);
       return -1;
    }
	if (nfd && FD_ISSET(socket_fd, &read_fds))
	{	     	      	  		
		nr = recv(socket_fd, &header, sizeof(int), MSG_WAITALL);
		if (nr<= 0) 
		{// got error on client side
			if (nr == 0) 
			{
				M3_INFO("M3SimpleServer: socket %d port %d hung up\n", socket_fd,portno);
				close(socket_fd); // bye!
				FD_CLR(socket_fd, &master); // remove from master set
				socket_fd=-1;
				return -1;
			}			
			else
			{
				M3_ERR("ERROR reading from socket for Port %d\n",portno);
				return -1;
			}
		}
		
		if (nr != sizeof(int)) 
		{
			M3_ERR("Num bytes read not same as requested: %d %d\n",nr,sizeof(int));
			return -1;
		}
		if (header != 9999)
		{
		    M3_ERR("Header corrupted: %x nfd: %d\n", header, nfd);
		    //FD_CLR(socket_fd, &read_fds);
		    return -1;
		}
		nr = recv(socket_fd, &size, sizeof(int), MSG_WAITALL);
		if (size>MAX_STRING_SIZE||size<0)
		{
			M3_ERR("Packet size out of bounds, may be corrupted data: %x %d\n",size,MAX_STRING_SIZE);
			return -1;
		}
		if (size==0)
		{
			s.clear();
			return -2;
		}
		//Grow buffer as needed
		if (nb_rx<size)
		{
			if (buf_rx!=NULL)
				delete [] buf_rx;
			buf_rx=new unsigned char [size];
			nb_rx=size;
		}
		nr=recv(socket_fd,buf_rx,size, MSG_WAITALL);
		if (nr==size)
		{
			s.assign((const char *)buf_rx,(size_t)size);
			return (int)size;
		}
		else
		{
			M3_ERR("ERROR reading from socket. Read:  %d Expected: %d\n",nr,size);
			return -1;
		}
	}
	return 0;
}

bool M3SimpleServer::Startup(int port)
{
    struct sockaddr_in myaddr;     // server address
    int yes=1;        // for setsockopt() SO_REUSEADDR, below
	portno=port;
	socket_fd=-1;
	M3_INFO("Opening socket %d\n",(int)portno);
    FD_ZERO(&master);    // clear the master and temp sets
    FD_ZERO(&read_fds);
    if ((listener = socket(PF_INET, SOCK_STREAM, 0)) == -1) {
        M3_ERR("ERROR opening socket %d\n",portno);
        return false;
    }
	if (setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) 
	{
        M3_ERR("ERROR in setsockopt for Port %d\n",portno);
        return false;
    }
	myaddr.sin_family = AF_INET; // bind
    myaddr.sin_addr.s_addr = INADDR_ANY;
    myaddr.sin_port = htons(portno);
    memset(myaddr.sin_zero, '\0', sizeof myaddr.sin_zero);
	unlink(INADDR_ANY);
    if (bind(listener, (struct sockaddr *)&myaddr, sizeof myaddr) == -1) {
         M3_ERR("ERROR on bind for Port %d\n",portno);
        return false;
    }
	if (listen(listener, 10) == -1) {
        M3_ERR("ERROR on listen for Port %d\n",portno);
        return false;
    }
    FD_SET(listener, &master); // add the listener to the master set
	fdmax = listener; // keep track of the biggest file descriptor
    return true;
}




bool M3SimpleServer::HandleNewConnection()
{
	if (IsActiveSocket())
		return true;
	
	read_fds = master; // copy it
	tv.tv_sec=0; //0.5 second timeout
	tv.tv_usec=500000;
	int nfd=select(fdmax+1, &read_fds, NULL, NULL, &tv);
	if (nfd== -1)//error 
	{
       M3_ERR("ERROR on select for Port %d\n",portno);
       return false;
    }
	if (FD_ISSET(listener, &read_fds))
	{
		int newfd;        // newly accept()ed socket descriptor
		struct sockaddr_in remoteaddr; // client address
		socklen_t addrlen = sizeof remoteaddr;
		if ((newfd = accept(listener,(struct sockaddr *)&remoteaddr, &addrlen)) == -1) 
		{ 
			M3_ERR("ERROR on accept for Port %d\n",portno);
			return false;
		} 
		socket_fd=newfd;
		FD_SET(newfd, &master); // add to master set
		if (newfd > fdmax)    // keep track of the maximum
			fdmax = newfd;
		M3_INFO("M3SimpleServer: new connection from %s on socket %d\n",inet_ntoa(remoteaddr.sin_addr), newfd);
	}
	return true;
}

    
}