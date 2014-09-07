#! /usr/bin/python3
import sys
import socket
import select
from time import sleep


PORT = 5005
addr = ( "192.168.1.200", 31000 )


sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_STREAM) # TCP
sock.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )
sock.bind(('', PORT))
sock.listen(1)

while True:
  conn, addr = sock.accept()
  print "Connection address: ", addr 
  conn.setblocking(0)

  while conn:
    # Check for input from either stdin or our socket
    sleep(0.001) # Time in seconds.
   
    activity =  select.select([sys.stdin, conn], [], [], 0)[0]
  
    if sys.stdin in activity:
      line = sys.stdin.readline()
      if line:
          #print "sending line ", line, " to ", addr, " port ", UDP_PORT
          conn.send( line )
    
    if conn in activity:
      data, addr = conn.recv(1024) # buffer size is 1024 bytes
      if not data: break
      #print "received message:", data, " from ", addr
      sys.stdout.write( data )
      sys.stdout.flush()

    conn.close()
