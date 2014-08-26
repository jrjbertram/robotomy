#ifndef UDPSERVER_H
#define UDPSERVER_H

#include <Arduino.h>

#include "Adafruit_CC3000.h"
#include "utility/socket.h"

class UdpStream : public Stream {
  public:
    UdpStream( uint16_t port );
    
    // Requried by Stream  
    int available();
    size_t write( uint8_t data);
    int read();
    int peek();
    void flush();
    
    // Plus a "begin" function to kick off the socket connections (so that we can control when exactly
    // we try to initialize our sockets.  We don't want to do this before the main program has initialized
    // the networking hardware.)
    bool begin();


  private:
    uint16_t _port;
    int _socket;
    char rdBuf[ 128 ];
    int rdCount;
    int rdPos;

    int readData(char *buffer, int bufferSize);

};

#endif  // UDPSERVER_H

