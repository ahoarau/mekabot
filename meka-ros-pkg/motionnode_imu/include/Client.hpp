/*
  @file    tools/sdk/cpp/Client.hpp
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
#ifndef __MOTION_NODE_SDK_CLIENT_HPP_
#define __MOTION_NODE_SDK_CLIENT_HPP_
#include <cstring>
#include <string>
#include <vector>


namespace MotionNode { namespace SDK {

/**
  Implements a simple socket based data client and the MotionNode
  service binary message protocol. Provide a simple interface to
  develop external applications that can read real-time MotionNode
  data.

  This class only handles the socket connection and single message
  transfer. The @ref Format class implements interfaces to the
  service specific data formats.

  @code
  try {
    using MotionNode::SDK::Client;

    // Open connection to a MotionNode service on the localhost,
    // port 32079.
    Client client("", 32079);

    // This is application dependent. Use a loop to keep trying
    // to read data even after time outs.
    while (true) {
      // Wait until there is incoming data on the open connection,
      // timing out after 5 seconds.
      if (client.waitForData()) {
        // Read data samples in a loop. This will block by default
        // for 1 second. We can simply wait on an open connection
        // until a data sample comes in, or fall back to the
        // wait loop.
        Client::data_type data;
        while (client.readData(data)) {
          // Do something useful with the current real-time sample
        }
      }
    }

  } catch (std::runtime_error & e) {
    // The Client class with throw std::runtime_error for any error
    // conditions. Even if the connection to the remote host fails.
  }
  @endcode
*/
class Client {
public:
  /**
    Define the type of a single binary message. Buffer of bytes.
  */
  typedef std::vector<char> data_type;

  /**
    Create a client connection to a remote MotionNode service.

    @param   host IP address of remote host, the empty string
             defaults to "127.0.0.1"
    @param   port port name of remote host
    @pre     a service is accepting connections on the socket
             described by the host, port pair
    @post    this client is connected to the remote service described
             by the host, port pair
    @throws  std::runtime_error if the client connection fails for
             any reason
  */
  Client(const std::string & host, const unsigned & port);
  
  /**
    Does not throw any exceptions. Close this client connection
    if it is open.
  */
  virtual ~Client();

  /**
    Close the current client connection.

    @throws  std::runtime_error if this client is not connected
             or the close procedure fails for any reason
  */
  virtual void close();

  /**
    Wait until there is incoming data on this client
    connection and then returns true.

    @param   time_out_second time out and return false after
             this many seconds, 0 value specifies no time out,
             negative value specifies default time out
    @pre     this object has an open socket connection
    @throws  std::runtime_error if this client is not connected
             or for any communication error
  */
  virtual bool waitForData(const int & time_out_second=-1);

  /**
    Read a variable length binary message into the output
    vector. The output vector will be resized as necessary.

    @param   time_out_second time out and return false after
             this many seconds, 0 value specifies no time out,
             negative value specifies default time out
    @pre     this object has an open socket connection
    @throws  std::runtime_error if this client is not connected
             or for any communication error
  */
  virtual bool readData(data_type & data, const int & time_out_second=-1);

  /**
    Write a variable length binary message to the
    socket link.

    @param   time_out_second time out and return false after
             this many seconds, 0 value specifies no time out,
             negative value specifies default time out
    @pre     this object has an open socket connection
    @throws  std::runtime_error if this client is not connected
             or for any communication error
  */
  virtual bool writeData(const data_type & data, const int & time_out_second=-1);

  /**
    Return the most recent XML message that this client
    connection received. The message could be anything so
    client applications need to be robust.

    @param  xml_string will contain an XML tree string if there is
            one available
    @pre    returns true iff an XML message arrive in a previous
            call to waitForData or readData.
  */
  virtual bool getXMLString(std::string & xml_string);
protected:
  /** Raw socket file descriptor. */
  int m_socket;
  /** Current host name this object is connected to. */
  std::string m_host;
  /** Current port name this object is connected to. */
  unsigned m_port;
  /** String description of the remote service. */
  std::string m_description;
  /** Last XML message received. Only set if m_intercept_xml is true. */
  std::string m_xml_string;
  /** Grab XML messages and store them in the string buffer. */
  bool m_intercept_xml;

  /**
    Hide an empty constructor. Child classes can use this to skip
    the connection attempt in the public constructor.
  */
  Client();

  /**
    Read a single binary message defined by a length header.
    
    This will block until is receives a non-empty message. An empty result message
    indicates a graceful disconnection of the remote host.

    @param   message output storage for the next message, if <tt>message.empty()</tt>
             then the socket connection has been gracefully terminated
    @return  always a pointer to this
    @pre     this object has an open socket connection
    @post    message contains some complete domain specific binary message
    @throws  std::runtime_error for any errors in the message
             communication protocol
  */
  Client & operator>>(std::string & message);
  /**
    Write a single binary message defined by a length header.

    @param   message binary message to write to the socket link.
    @return  always a pointer to this
    @pre     this object has an open socket connection
    @pre     message is not empty
    @throws  std::runtime_error for any errors in the message
             communication protocol
  */
  Client & operator<<(const std::string & message);
  /**
    Low level socket receive command. Read a chunk into our object local buffer.
    This will block until it receives a non-empty message.

    @param   data output storage for the next message, if <tt>data.empty()</tt>
             after this call then the socket connection has been gracefully
             terminated while we were blocking for incoming data
    @param   receive_timed_out will be set to <code>true</code> if the
             system recv call timed out
    @return  the number of bytes read from the open socket connnection or 0 if
             the socket connection has been closed
    @pre     this object has an open socket connection
    @post    data contains N bytes of raw data from the socket connection
    @throws  std::runtime_error for any errors in the system socket recv call
  */
  unsigned receive(std::string & data, bool & receive_timed_out);
  /**
    Low level socket send command. Write a chunk of data to the remote
    socket endpoint.

    @param   data raw buffer of bytes to write to the socket
    @param   send_timed_out will be set to <code>true</code> if the
             system send call timed out
    @return  the number of bytes read from the open socket connnection or 0 if
             the socket connection has been closed
    @pre     this object has an open socket connection
    @pre     data is not empty
    @post    N bytes of data was written to the socket
    @throws  std::runtime_error for any errors in the system socket send call
  */  
  unsigned send(const std::string & data, bool & send_timed_out);
  /**
    Set the receive time out for this socket.

    @param   second specifies the number of seconds
    @pre     this object has an open socket connection
    @post    any calls to receive will time out after <code>second</code>
             seconds
    @throws  std::runtime_error for any errors in the system socket
             setsockopt call
  */
  bool setReceiveTimeout(const std::size_t & second);
  /**
    Set the send time out for this socket.

    @param   second specifies the number of seconds
    @pre     this object has an open socket connection
    @post    any calls to send will time out after <code>second</code>
             seconds
    @throws  std::runtime_error for any errors in the system socket
             setsockopt call
  */
  bool setSendTimeout(const std::size_t & second);

private:
  /** Initialization flag. Specific to the Winsock API. */
  bool m_initialize;
  /** Input buffer for receiving raw data. */
  std::vector<char> m_buffer;
  /** Temporary message storage spans multiple high level receive calls. */
  std::string m_nextMessage;
  /** Set this internal value to the current socket receive time out. */
  std::size_t m_time_out_second;
  /** Set this internal value to the current socket send time out. */
  std::size_t m_time_out_second_send;

  /**
    Initialize any network subsystems and create a socket descriptor.
    Set any basic communication flags. Return the socket descriptor.
  */
  int initialize();

  /**
    Disable the copy constructor.

    This is a resource object. Copy constructor semantics would be confusing
    at the very least. Disable it instead.
  */
  Client(const Client & rhs);
  /**
    Disable the assignment operator.

    @see Client#Client(const Client &)
  */
  const Client & operator=(const Client & lhs);
}; // class Client

}} // namespace MotionNode::SDK

#endif // __MOTION_NODE_SDK_CLIENT_HPP_
