#!/usr/bin/env python

"""
A basic, multiclient 'chat server' using Python's select module
with interrupt handling.

Entering any line of input at the terminal will exit the server.
"""

import select
import socket
import sys
import signal
from communication import send, receive

BUFSIZ = 1024


class ChatServer(object):
    """ Simple chat server using select """
    
    def __init__(self, port=5005, backlog=5):
        self.clients = 0
        # Client map
        self.clientmap = {}
        # Output socket list
        self.outputs = []
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('',port))
        print 'Listening to port',port,'...'
        self.server.listen(backlog)
        # Trap keyboard interrupts
        signal.signal(signal.SIGINT, self.sighandler)
        
    def sighandler(self, signum, frame):
        # Close the server
        print 'Shutting down server...'
        # Close existing client sockets
        for o in self.outputs:
            o.close()
            
        self.server.close()

    def getname(self, client):

        # Return the printable name of the
        # client, given its socket...
        info = self.clientmap[client]
        host, name = info[0][0], info[1]
        return '@'.join((name, host))
        
    def serve(self):
        
        inputs = [self.server,sys.stdin]
        self.outputs = []

        running = 1

        while running:

            try:
                #print "calling select..."
                inputready,outputready,exceptready = select.select(inputs, [], [])
                #print "select returned\r\n"
                #print "inputs: "
                #print inputready
                #print " outputs: "
                #print outputready
                #print " excepts: "
                #print exceptready
                #print "\n"
            except select.error, e:
                print "select error\r\n"
                break
            except socket.error, e:
                print "socket error\r\n"
                break

            for s in inputready:

                if s == self.server:
                    # handle the server socket
                    client, address = self.server.accept()
                    print 'chatserver: got connection %d from %s' % (client.fileno(), address)
                    # Compute client name and send back
                    self.clients += 1
                    cname = 'client%d' % self.clients
                    send(client, 'CLIENT: ' + str(address[0]))
                    inputs.append(client)

                    self.clientmap[client] = (address, cname)
                    # Send joining information to other clients
                    msg = '\n(Connected: New client (%d) from %s)' % (self.clients, self.getname(client))
                    print '%s' % msg
                    for o in self.outputs:
                        o.send(msg)
                        #send(o, msg)
                    
                    self.outputs.append(client)

                elif s == sys.stdin:
                    # handle standard input
                    msg = sys.stdin.readline()
                    print "sending \"%s\"" % msg
                    for o in self.outputs:
                        o.send(msg)
                        #send(o, msg)
                else:
                    # handle all other sockets
                    try:
                        #print 'attempting recieve\r\n'
                        data = s.recv(BUFSIZ)
                        #data = receive(s)
                        #print 'receive returned\r\n'
                        if data:
                            # Send as new client's message...
                            msg = data
                            # Send data to all except ourselves
                            print '\"%s\"' % msg
                            for o in self.outputs:
                                if o != s:
                                    o.send(msg)
                                    #send(o, msg)
                        else:
                            print 'chatserver: %d hung up' % s.fileno()
                            self.clients -= 1
                            s.close()
                            inputs.remove(s)
                            self.outputs.remove(s)

                            # Send client leaving information to others
                            msg = '\n(Hung up: Client from %s)' % self.getname(s)
                            print '%s' % msg
                            for o in self.outputs:
                                o.send(msg)
                                #send(o, msg)
                                
                    except socket.error, e:
                        # Remove
                        print 'chatserver: socket error %d: %s' % (socket.fileno(), self.getname(socket))
                        inputs.remove(s)
                        self.outputs.remove(s)
                        


        print "\r\nclosing server\r\n"
        self.server.close()

if __name__ == "__main__":
    ChatServer().serve()

