#include "UdpStream.h"

UdpStream::UdpStream(uint16_t port) {
  _port = port;
  _socket = -1;
  memset( rdBuf, 0, sizeof(rdBuf) );
  rdCount = 0;
  rdPos = 0;
}


bool UdpStream::begin() {
  Serial.print("START of udpServer::Begin() _socket: "); Serial.println(_socket);
  // Open the socket if it isn't already open.
  if (_socket == -1) {
    // Create the UDP socket
    int soc = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (soc < 0) {
      Serial.println("socket() call failed");
      return false;
    }

    sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(_port);
    // JRB: Need to figure out how to set this to broadcast, or maybe a multicast address
    address.sin_addr.s_addr = 0;//0xC0A801C8;  // 192.168.1.255, broadcast addr
    socklen_t len = sizeof(address);
    if (bind(soc, (sockaddr*) &address, sizeof(address)) < 0) {
      Serial.println("bind() call failed");
      return false;
    }

    _socket = soc;
  }

  Serial.print("END of udpServer::begin() _socket: "); Serial.println(_socket);

  return true;
}

int UdpStream::available() {
//  Serial.println( "available: enter" );
  if ( rdCount == 0 )
  {
//    Serial.println( "rdCount was zero" );

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 5000; // JRB:  I may want to shorten this down.. don't really want to wait 5 ms for data.
    fd_set reads;
    FD_ZERO(&reads);
    FD_SET(_socket, &reads);
    
//    Serial.println( "Calling select" );
    select(_socket + 1, &reads, NULL, NULL, &timeout);
//    Serial.println( "select returned" );
    
    if (!FD_ISSET(_socket, &reads)) {
      // No data to read.
//      Serial.println("No data to read.");
      return false;
    }
  }

//  Serial.println( "available: returning true" );
  return true;
}

size_t UdpStream::write( uint8_t data)
{
  //Serial.println( "write: entry" );
  
  sockaddr_in address;
  memset(&address, 0, sizeof(address));
  address.sin_family = AF_INET;
  address.sin_port = htons(5005);
  // JRB: Need to figure out how to set this to broadcast, or maybe a multicast address
  //address.sin_addr.s_addr = 0xE1000025;  // multicast 225.0.0.37
  //address.sin_addr.s_addr = 0xC0A80164;  // 192.168.1.100... laptop addr.
  address.sin_addr.s_addr = 0x6401a8C0;  // 192.168.1.100... laptop addr.  Byte swapped correctly!!
  socklen_t len = sizeof(address);


//  Serial.print( "calling sendto... " );
  int n = sendto(_socket, &data, 1, 0, (sockaddr*)&address, len);
//  Serial.print( "sendto returned " );
//  Serial.println( n );
  if ( n < 0 )
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

int UdpStream::read()
{
//  Serial.println( "read: entry" );
  if ( rdCount == 0 )
  {
//    Serial.println( "rdCount was zero" );
    // Out of data, read in some more
    rdCount = readData( rdBuf, sizeof( rdBuf ) );
    rdPos = 0;

//    Serial.println( "readData returned" );
    if ( rdCount < 1 )
    {
      // There was an error, return the count to zero
      rdCount = 0;
    }
  }

  if ( rdCount > 0 )
  {
    // Feed some more data to the caller
    int retval = rdBuf[ rdPos ];
    rdPos++;
    rdCount--;
    return retval;
  }
  else
  {
    return -1;
  }
}

int UdpStream::peek()
{
  Serial.println( "peek: entry" );
  if ( rdCount == 0 )
  {
    // Out of data, read in some more
    rdCount = readData( rdBuf, sizeof( rdBuf ) );
    rdPos = 0;

    if ( rdCount < 1 )
    {
      // There was an error, return the count to zero
      rdCount = 0;
    }
  }

  if ( rdCount > 0 )
  {
    // Feed the next data to the caller
    int retval = rdBuf[ rdPos ];
    // But do not update the indexes, as this is a peek
    //rdPos++;      // this is peek, do not update
    //rdCount--;    // this is peek, do not update
    return retval;
  }
  else
  {
    return -1;
  }
}

void UdpStream::flush()
{
  Serial.println( "flush: entry" );
}

int UdpStream::readData(char *buffer, int bufferSize) {
  // If there is data, then stores it into buffer &
  // returns the length of buffer. (-1 if none)
//  Serial.println( "readData: entry" );
  if (available()) {  // Make sure data is really available - bummer because it again calls select and has that 5 ms wait.

//    Serial.println( "calling recv" );
    int n = recv(_socket, buffer, bufferSize, 0);
//    Serial.println( "recv returned" );

    if (n < 1) {
      // Error getting data.
      Serial.println("Error getting data");
      return -1;
    }

    return n;
  }

  return -1;
}

