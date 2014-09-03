#ifdef ENABLE_NET

class ClientStream : public Stream {
  public:
  ClientStream( Adafruit_CC3000_ClientRef * client ) { _client = client; haveCachedData = false;}
  
  int available() { return _client->available(); }
  size_t write( uint8_t data) { return _client->write( data ); }
  int read(  ) { 
     if( haveCachedData )
     { 
        haveCachedData = false;
        return cachedData;
     }
     else
     {
       return _client->read( ); 
     }
  }
  
  int peek( ) {
    if( haveCachedData )
    {
      Serial.print( "peek called when already have cached data.. need an array?" );
      while( 1 ) ;
    }
    else
    {
      cachedData = read();
      haveCachedData = 1;
    }
    return cachedData;
  } 
  
  void flush() {}

  
  private:
  Adafruit_CC3000_ClientRef * _client;
  bool haveCachedData;
  uint8_t cachedData;
  
};
#endif

// Class that will let me mux output on two streams
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

