
// I want to have a "seamless" console that works over the network or via the standard Arduino serial port interface.
// This class allows you to take input and output from two different Streams, but presents a standard Stream API to
// the user so that they're for the most part unaware that two Streams are involved.

class MuxStream : public Stream {  
  public:
  MuxStream( Stream * a, Stream * b ) { _a = a; _b = b;}
  MuxStream( Stream * a  ) { _a = a; _b = NULL;}  
  MuxStream() { _a = NULL; _b = NULL;}  
  MuxStream(const MuxStream& copy) : _a( copy._a ), _b( copy._b ) {}
  void operator=(const MuxStream& copy) { _a = copy._a; _b = copy._b; }
  
  int available() { 

    if( _a && _a->available() )
    {
      //Serial.println( "mux a availalbe" );
      return 1;
    }
    
    if( _b && _b->available() )
    {
      //Serial.println( "mux b availalbe" );
      return 1;
    }
  
    return 0;  
  }
  
  size_t write( uint8_t data) {
    if( _a ) { _a->write( data ); }
    if( _b ) { _b->write( data ); }
    return 1; 
  }
  
  int read(  ) { 
     if( _a && _a->available() )
     { 
       return _a->read( ); 
     }
     
     if( _b && _b->available() )
     { 
       return _b->read( ); 
     }
  }
  
  int peek( ) {

     if( _a && _a->available() )
     { 
       return _a->peek( ); 
     }
     
     if( _b && _b->available() )
     { 
       return _b->peek( ); 
     }
  } 
  
  void flush() {}

  
  private:
    Stream * _a;
    Stream * _b;
  
};

