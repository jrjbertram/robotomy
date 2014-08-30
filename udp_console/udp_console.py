#! /usr/bin/python3
import sys
import socket
import select


UDP_IP = "0.0.0.0"
UDP_PORT = 5005
addr = ( "192.168.1.200", 31000 )


sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
  # Check for input from either stdin or our socket
 
  activity =  select.select([sys.stdin, sock], [], [], 0)[0]

  if sys.stdin in activity:
    line = sys.stdin.readline()
    if line:
        #print "sending line ", line, " to ", addr, " port ", UDP_PORT
        sock.sendto( line, addr )
  
  if sock in activity:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    #print "received message:", data, " from ", addr
    sys.stdout.write( data )
    sys.stdout.flush()
    

