/**
  Implementation of the Client class. See the header file for
  more details.

  @file    tools/sdk/cpp/src/Client.cpp
  @author  Luke Tokheim, luke@motionnode.com
  @version 1.2

  (C) Copyright GLI Interactive LLC 2009. All rights reserved.

  The coded instructions, statements, computer programs, and/or related
  material (collectively the "Data") in these files contain unpublished
  information proprietary to GLI Interactive LLC, which is protected by
  US federal copyright law and by international treaties.

  The Data may not be disclosed or distributed to third parties, in whole
  or in part, without the prior written consent of GLI Interactive LLC.

  The Data is provided "as is" without express or implied warranty, and
  with no claim as to its suitability for any purpose.
*/

#include <Client.hpp>

#include <stdexcept>

// If you have the boost libraries installed make some compile time checks.
//#include <boost/static_assert.hpp>
#ifdef _WIN32
#  include <winsock2.h>
#  include <ws2tcpip.h>
#else
#  include <errno.h>
#  include <unistd.h>
#  include <sys/errno.h>
#  include <sys/types.h>
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <netinet/tcp.h>
#  include <arpa/inet.h>
#endif // _WIN32

#ifdef _WIN32
#  define SHUT_RD      SD_READ
#  define SHUT_WR      SD_SEND
#  define SHUT_RDWR    SD_BOTH
#  define EINTR        WSAEINTR
#  define ETIMEDOUT    WSAETIMEDOUT
#  define ECONNREFUSED WSAECONNREFUSED
#endif // _WIN32

// We assume that these contants match up with the BSD standard,
// then we do not need to use them directly in the code.
#ifdef BOOST_STATIC_ASSERT
#  ifdef _WIN32
    BOOST_STATIC_ASSERT((-1 == INVALID_SOCKET));
    BOOST_STATIC_ASSERT((-1 == SOCKET_ERROR));
#  endif // _WIN32
#endif

// On Linux, disable the SIGPIPE signal for send and recv system calls.
#ifndef MSG_NOSIGNAL
#  define MSG_NOSIGNAL 0
#endif

// Create a macro to access error code for system socket calls. 
#ifdef _WIN32
#  define ERROR_CODE WSAGetLastError()
#else
#  define ERROR_CODE errno
#endif // _WIN32


/**
  The maximum length of a single message.

  This is used as an extra level of safety. You could set the maximum
  message size to a very large value. The real hard limit on the size
  of the message is the maximum length of an std::string.

  In practice, we assume that someone screwed up if we receive a
  <b>huge</b> message.
*/
const std::size_t MaximumMessageLength = 65535;
/**
  The maximum size (in bytes) of a single send/receive to the underlying
  socket library. This socket will allocate this memory at instantiation
  time.
*/
const std::size_t ReceiveBufferSize = 1024;
/**
  Set the address to this value if we get an empty string.
*/
const std::string DefaultAddress = "127.0.0.1";
/**
  Default value, in seconds, for the socket receive time out
  in the Client#waitForData method. Zero denotes blocking receive.
*/
const std::size_t TimeOutWaitForData = 5;
/**
  Default value, in seconds, for the socket receive time out
  in the Client#readData method.
*/
const std::size_t TimeOutReadData = 1;
/**
  Default value, in seconds, for the socket send time out
  in the Client#writeData method.
*/
const std::size_t TimeOutWriteData = 1;
/**
  Detect XML message with the following header bytes.
*/
const std::string XMLMagic = "<?xml";


namespace MotionNode { namespace SDK {

Client::Client(const std::string & host, const unsigned & port)
  : m_socket(-1), m_host(), m_port(0), m_description(), m_xml_string(), m_intercept_xml(true),
    m_initialize(false), m_buffer(ReceiveBufferSize), m_nextMessage(),
    m_time_out_second(0), m_time_out_second_send(0)
{
  int socket = initialize();

  // Create the address structure to describe the remote host and
  // port we would like to connect to.
  sockaddr_in address;
  {
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    //::memset(&address.sin_zero, 0, 8);
    memset(&address.sin_zero, 0, 8);

    // Implement a default value for the remote host name.
    std::string ip_address = host;
    if (host.empty()) {
      ip_address = DefaultAddress;
    }

#ifdef _WIN32
    int result = ::inet_addr(ip_address.c_str());
    address.sin_addr.s_addr = result;
#else
    int result = ::inet_pton(AF_INET, ip_address.c_str(), &address.sin_addr);
#endif // _WIN32

    if (-1 == result) {
      throw std::runtime_error("failed conversion from host string to address structure");
    }
    // Zero return value indicates the address was not parseable.
    if (0 == result) {
      throw std::runtime_error("failed to parse host string");
    }
  }

  // Now open the actual connection.
  {
    int result = ::connect(socket, reinterpret_cast<sockaddr *>(&address), sizeof(sockaddr));

    // Check for connection failure. We may want to handle "connection refused" on
    // a different path than general function call failures.
    if (-1 == result) {
      const int error_code = ERROR_CODE;
      if (ECONNREFUSED == error_code) {
        // Connection refused. 
        // No connection could be made because the target computer actively refused
        // it. This usually results from trying to connect to a service  that is
        // inactive on the foreign host�that is, one with no server application running. 
        throw std::runtime_error("connection refused by remote host");
      }
      throw std::runtime_error("failed to connect to remote host");
    }
  }

  m_socket = socket;
  m_host = host;
  m_port = port;

  {
    // Read the first message from the service. It is a 
    // string description of the remote service.
    setReceiveTimeout(TimeOutWaitForData);
    (*this) >> m_description;
  }
}

Client::Client()
  : m_socket(-1), m_host(), m_port(0), m_description(), m_xml_string(), m_intercept_xml(true),
    m_initialize(false), m_buffer(ReceiveBufferSize), m_nextMessage(),
    m_time_out_second(0), m_time_out_second_send(0)
{
  m_socket = initialize();
}

Client::~Client()
{
  // Close an open socket. Do not throw exceptions in the destructor.
  if (m_socket > 0) {
    try {
      close();
    } catch (std::runtime_error &) {
    }
  }

#ifdef _WIN32
  if (m_initialize) {
    if (0 == WSACleanup()) {
      m_initialize = false;
    }
  }
#endif // _WIN32
}

void Client::close()
{
  // Is this an active socket connection?
  if (m_socket > 0) {
    // First, disable sends and receives on this socket. Notify the
    // other side of the connection that we are going away.
    int result = ::shutdown(m_socket, SHUT_RDWR);
    if (-1 == result) {
      throw std::runtime_error("failed to shutdown socket communication");
    }

#ifdef _WIN32
    result = ::closesocket(m_socket);
#else
    result = ::close(m_socket);
#endif // _WIN32
    if (-1 == result) {
      throw std::runtime_error("failed to close socket connection");
    }

    // Re-initialize local state.
    m_socket = -1;
    m_host.clear();
    m_port = 0;
    m_description.clear();
    m_xml_string.clear();

    std::fill(m_buffer.begin(), m_buffer.end(), 0);
    m_nextMessage.clear();
  } else {
    throw std::runtime_error("failed to close client, not connected");
  }
}

bool Client::waitForData(const int & time_out_second)
{
  bool result = false;

  // Is this an active socket connection?
  if (m_socket > 0) {
    // A default value of the time_out_second (-1)
    // indicates that we just want to use the default
    // implementation.
    if (time_out_second < 0) {
      if (TimeOutWaitForData != m_time_out_second) {
        setReceiveTimeout(TimeOutWaitForData);
      }
    } else {
      setReceiveTimeout(time_out_second);
    }

    std::string message;
    (*this) >> message;

    // Consume any incoming XML message.
    if ((message.size() >= XMLMagic.size()) && (message.substr(0, XMLMagic.size()) == XMLMagic)) {
      m_xml_string.assign(message);
    }
    
    if (!message.empty()) {
      result = true;
    }

  } else {
    throw std::runtime_error("failed to wait for incoming data, client is not connected");
  }

  return result;
}

bool Client::readData(data_type & data, const int & time_out_second)
{
  data.clear();

  bool result = false;

  // Is this an active socket connection?
  if (m_socket > 0) {
    // A default value of the time_out_second (-1)
    // indicates that we just want to use the default
    // implementation.
    if (time_out_second < 0) {
      if (TimeOutReadData != m_time_out_second) {
        setReceiveTimeout(TimeOutReadData);
      }
    } else {
      setReceiveTimeout(time_out_second);
    }

    std::string message;
    (*this) >> message;

    // Consume any incoming XML message.
    if (m_intercept_xml && (message.size() >= XMLMagic.size()) && (message.substr(0, XMLMagic.size()) == XMLMagic)) {
      m_xml_string.assign(message);

      (*this) >> message;
    }
    
    if (!message.empty()) {
      data.resize(message.size());

      std::copy(
        message.begin(), message.end(),
        data.begin());

      result = true;
    }

  } else {
    throw std::runtime_error("failed to read data, client is not connected");
  }

  return result;
}

bool Client::writeData(const data_type & data, const int & time_out_second)
{
  bool result = false;

  if (data.empty()) {
    return false;
  }

  // Is this an active socket connection?
  if (m_socket > 0) {
    // A default value of the time_out_second (-1)
    // indicates that we just want to use the default
    // implementation.
    if (time_out_second < 0) {
      if (TimeOutWriteData != m_time_out_second_send) {
        setSendTimeout(TimeOutWriteData);
      }
    } else {
      setSendTimeout(m_time_out_second_send);
    }

    {
      std::string message;
      {
        message.resize(data.size());
        std::copy(
          data.begin(), data.end(),
          message.begin());
      }

      (*this) << message;
    }
    
    result = true;
 
  } else {
    throw std::runtime_error("failed to read data, client is not connected");
  }

  return result;
}

bool Client::getXMLString(std::string & xml_string)
{
  // Note that this does not enforce the connection state.
  // This may return true even if we are not connected.
  xml_string.assign(m_xml_string);

  return !xml_string.empty();
}

Client & Client::operator>>(std::string & message)
{
  message.clear();
  
  // Use these intermediary values to pipe data into from our message buffer,
  // or from calls to recv. That way we can follow a unified path.
  std::string toRecv;
  std::size_t bytes = 0;
  bool receive_timed_out = false;
  
  // We may have saved a little something for ourselves from the buffer of another message.
  if (!m_nextMessage.empty()) {
    toRecv.assign(m_nextMessage);

    bytes = toRecv.length();
    m_nextMessage.clear();
  } else {
    bytes = receive(toRecv, receive_timed_out);
  }
  
  // This can indicate a graceful disconnection of the socket stream (for TCP).
  if (0 == bytes) {
    if (!receive_timed_out) {
      try {
        close();
      } catch (std::runtime_error &) {
      }
    }

    return *this;
  }

  // Read the length of the message from the message "header".
  unsigned length = 0;

  // Paranoia. We need at least 4 bytes to start this message. Let's give ourselves
  // every opportunity to get them before we give up.
  {
    unsigned count = 0;
    while ((bytes < sizeof(unsigned)) && (count < (sizeof(unsigned)-1))) {
      std::string temp;
      unsigned nextBytes = receive(temp, receive_timed_out);
      if (0 == nextBytes) {
        throw std::runtime_error("communication protocol error, failed to read message header");
      }
      toRecv.append(temp);
      bytes += nextBytes;
      count++;
    }
  }

  // Needs >= 4 bytes.
  if ((bytes < sizeof(unsigned)) || (bytes != toRecv.length())) {
    throw std::runtime_error("communication protocol error, failed to read full message header");
  }


  // Copy the network ordered message length header. Make sure it is a "reasonable" value.
  length = *reinterpret_cast<const unsigned *>(toRecv.c_str());
  length = ntohl(length);
  if ((0 == length) || (length > MaximumMessageLength)) {
    throw std::runtime_error("communication protocol error, message header specifies invalid length");
  }

  message = toRecv.substr(sizeof(unsigned), length);


  // We still may need more data out of the socket connection.
  if (message.length() < length) {

    unsigned endMessage = 0;
    std::string nextChunk;

    do {
      unsigned received = receive(nextChunk, receive_timed_out);
      if (0 == received) {
        throw std::runtime_error("communication protocol error, message interrupted");
      }

      endMessage = length - message.length();
      message.append(nextChunk, 0, endMessage);

    } while (message.length() < length);

    // Did we walk into the next logical message with the last recv?
    if (endMessage < nextChunk.length()) {
      m_nextMessage = nextChunk.substr(endMessage);
    }

  } else {
    // We have all that we need, but do we have extra?
    unsigned endMessage = length + sizeof(unsigned);
    if (toRecv.length() > endMessage) {
      m_nextMessage = toRecv.substr(endMessage);
    }
  }

  return *this;
}

Client & Client::operator<<(const std::string & message)
{
  if (message.length() > 0) {
    // If the input message is too long, give up now.
    if (message.length() > MaximumMessageLength) {
      throw std::runtime_error("communication protocol error, message too long to send");
    }

    // Actual outgoing message is copied into this buffer.
    std::string toSend;

    // Create a message length header.
    {
      // First thing. Dump 4 bytes of integer message length at the beginning of the message.
      // (In network order.)
      unsigned length = htonl(static_cast<unsigned>(message.length()));

      // Make the message the concatentaion of the length, and the input message.
      toSend.assign(reinterpret_cast<char *>(&length), sizeof(unsigned));
    }

    toSend.append(message);

    // Send as much of the message (with the length) as we can the first time.
    bool send_timed_out = false;
    unsigned bytes = send(toSend, send_timed_out);
    if (0 == bytes) {
      throw std::runtime_error("communication protocol error, failed to write message");
    }
    
    // If we couldn't send it all at once, finish the job now, Monster.
    while (bytes < toSend.length()) {
      std::string nextChunk = toSend.substr(bytes);
      unsigned sent = send(nextChunk, send_timed_out);
      if (0 == sent) {
        throw std::runtime_error("communication protocol error, message interrupted");
      }

      bytes += sent;
    }

    // Sanity check.
    if (bytes != toSend.length()) {
      throw std::runtime_error("communication protocol error, failed to write complete message");
    }
  }

  return *this;
}

unsigned Client::send(const std::string & data, bool & send_timed_out)
{
  send_timed_out = false;

  if (m_socket <= 0) {
    return 0;
  }

  int result = ::send(m_socket, data.c_str(), static_cast<int>(data.length()), MSG_NOSIGNAL);
  if (-1 == result) {
    const int error_code = ERROR_CODE;
    if (ETIMEDOUT == error_code) {
      // Connection timed out. 
      // A connection attempt failed because the connected party did not properly
      // respond after a period of time, or the established connection failed
      // because the connected host has failed to respond.

      // We set a time out for some receive calls. Do not throw an exception if we
      // expect to time out.
      result = 0;
      send_timed_out = true;
    } else if (EINTR == error_code) {
      // Interrupted function call. 
      // A blocking operation was interrupted by a call to WSACancelBlockingCall.

      // An external thread may have called the #close method for this instance.
      // This may be unsafe, but consider it to be a "graceful close" condition.
      result = 0;
    } else {
      throw std::runtime_error("failed to read data from socket");
    }
  }

  return static_cast<unsigned>(result);
}

unsigned Client::receive(std::string & data, bool & receive_timed_out)
{
  data.clear();
  receive_timed_out = false;

  if (m_socket <= 0) {
    return 0;
  }

  int result = ::recv(m_socket, &m_buffer[0], static_cast<int>(m_buffer.size()), MSG_NOSIGNAL);
  if (-1 == result) {
    const int error_code = ERROR_CODE;
    if (ETIMEDOUT == error_code) {
      // Connection timed out. 
      // A connection attempt failed because the connected party did not properly
      // respond after a period of time, or the established connection failed
      // because the connected host has failed to respond.

      // We set a time out for some receive calls. Do not throw an exception if we
      // expect to time out.
      result = 0;
      receive_timed_out = true;
    } else if (EINTR == error_code) {
      // Interrupted function call. 
      // A blocking operation was interrupted by a call to WSACancelBlockingCall.

      // An external thread may have called the #close method for this instance.
      // This may be unsafe, but consider it to be a "graceful close" condition.
      result = 0;
    } else {
      throw std::runtime_error("failed to read data from socket");
    }
  }
  
  // Process real data if we have any.
  if (result > 0) {
    data.assign(&m_buffer[0], result);
  } else {
    result = 0;
  }

  return static_cast<unsigned>(result);
}

bool Client::setReceiveTimeout(const std::size_t & second)
{
  bool result = false;

  // Is this an active socket connection?
  if (m_socket > 0) {

#ifdef _WIN32
    // Windows specifies this as an integer valued milliseconds.
    int optionval = static_cast<int>(second) * 1000;
#else
    // Linux specifies this as a timeval.
    timeval optionval;
    optionval.tv_sec = second;
    optionval.tv_usec = 0;
#endif // _WIN32

    int set_result = ::setsockopt(
      m_socket, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char *>(&optionval), sizeof(optionval));

    if (-1 == set_result) {
      throw std::runtime_error("failed to set client receive time out");
    }

    m_time_out_second = second;
    result = true;

  } else {
    throw std::runtime_error("failed to set client receive time out, socket is not connected");
  }

  return result;
}
bool Client::setSendTimeout(const std::size_t & second)
{
  bool result = false;

  // Is this an active socket connection?
  if (m_socket > 0) {

#ifdef _WIN32
    // Windows specifies this as an integer valued milliseconds.
    int optionval = static_cast<int>(second) * 1000;
#else
    // Linux specifies this as a timeval.
    timeval optionval;
    optionval.tv_sec = second;
    optionval.tv_usec = 0;
#endif // _WIN32

    int set_result = ::setsockopt(
      m_socket, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char *>(&optionval), sizeof(optionval));

    if (-1 == set_result) {
      throw std::runtime_error("failed to set client send time out");
    }

    m_time_out_second_send = second;
    result = true;

  } else {
    throw std::runtime_error("failed to set client send time out, socket is not connected");
  }

  return result;
}

int Client::initialize()
{
#ifdef _WIN32
  if (!m_initialize) {
    // Winsock API requires per application or DLL initialization.
    // We are allowed to make multiple calls to WSAStartup as long
    // as we make a cooresponding call to WSACleanup. Choose to do
    // this in the constructor and destructor, as this is the most
    // reliable way.
    WSADATA wsaData;
    if (0 == WSAStartup(MAKEWORD(1, 1), &wsaData)) {
      m_initialize = true;
    } else {
      throw std::runtime_error("failed to initialize Winsock at \"WSAStartup\"");
    }
  }
#endif // _WIN32

  int socket = m_socket;
  if (socket <= 0) {
    // Request a socket for a good old fashioned TCP connection.
    socket = static_cast<int>(::socket(AF_INET, SOCK_STREAM, 0));
    if (-1 == socket) {
      throw std::runtime_error("failed to create socket");
    }
  }

#ifdef SO_NOSIGPIPE
  // Mac OS X does not have the MSG_NOSIGNAL flag. It does have this
  // connections based version, however.
  if (socket > 0) {
    int set_option = 1;
    if (0 == setsockopt(socket, SOL_SOCKET, SO_NOSIGPIPE, &set_option, sizeof(set_option))) {
    } else {
      throw std::runtime_error("failed to set socket signal option");
    }
  }
#endif // SO_NOSIGPIPE

  return socket;
}

}} // namespace MotionNode::SDK
