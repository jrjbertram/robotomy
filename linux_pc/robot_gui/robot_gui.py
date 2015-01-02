#!/usr/bin/env python

import select
import socket
import sys
import signal
import netConsole
import gui
import thread
import wx
import struct

GROUP = '224.0.2.0'
PORT = 5005

BUFSIZ = 1024

# --- here goes your event handlers ---

# --- gui2py designer generated code starts ---

with gui.Window(name='mywin', title=u'gui2py minimal app', resizable=True,
                height='496px', width='400px', image='', ):
    gui.Image(name=u'robot_background', height='302', left='0', top='-6',
              width='363', fgcolor=u'#4C4C4C', filename=u'robot.png', )
    gui.Label(name=u'ir_distance', height='17', left='170', top='120',
              width='60', bgcolor=u'#F2F1F0', fgcolor=u'#4C4C4C',
              text=u'0 inches', )
    gui.Label(name=u'heading', height='17', left='172', top='194',
              width='129', bgcolor=u'#F2F1F0', fgcolor=u'#4C4C4C',
              text=u'0 degrees', )
    gui.Label(name=u'mode', height='17', left='9', top='10', width='129',
              bgcolor=u'#F2F1F0', fgcolor=u'#4C4C4C', text=u'IDLE', )
    gui.Label(name=u'connstate', height='17', left='2', top='461',
              width='129', bgcolor=u'#F2F1F0', fgcolor=u'#4C4C4C',
              text=u'waiting for connection...', )
    gui.Label(name=u'left_vel', height='17', left='66', top='194',
              width='129', bgcolor=u'#F2F1F0', fgcolor=u'#4C4C4C', text=u'0', )
    gui.Label(name=u'right_vel', alignment='right', height='17', left='301',
              top='194', width='129', bgcolor=u'#F2F1F0', fgcolor=u'#4C4C4C',
              text=u'0', )
    gui.Label(name=u'plan', border='raised', height='17', left='10', top='34',
              width='0', bgcolor=u'#F2F1F0', fgcolor=u'#4C4C4C', text=u'', )

# --- gui2py designer generated code ends ---

mywin = gui.get("mywin")

# assign your event handlers:

mode_dict = { 'I' : "Idle", 'D': "Diagnostic", 'A': "Auto", 'U': "Unknown" }
plan_dict = { 'R' : "Reset", 'I': "Init", 'W': "Wander", 'H': "Hit", 'U': "Unknown" }

def parseMessage(msg):
    print "parsing message: %s" % msg

    if msg.startswith( "Md=" ):
	    params=dict(e.split('=') for e in msg.split(':'))

	    #print params
	    #print "setting heading = ", params["Heading"]
	    mywin['connstate'].text = "connected"
	    mywin['mode'].text = mode_dict[ params["Md"] ]
	    mywin['plan'].text = plan_dict[ params["Pl"] ]
	    mywin['heading'].text = params["Y"]
	    mywin['ir_distance'].text = params["IR"]
	    mywin['left_vel'].text = params["Lf"]
	    mywin['right_vel'].text = params["Rt"]
	    #print "success"


class netConsole:
    """ Simple chat server using select """

    def __init__(self, port=PORT, backlog=5):
        self.clients = 0
        # Client map
        self.clientmap = {}
        # Output socket list
        self.outputs = []
        #self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('',port))
        self.server.setblocking(0)
        # Get multicast group info
        addrinfo = socket.getaddrinfo(GROUP, None)[0]
        group_bin = socket.inet_pton(addrinfo[0], addrinfo[4][0])
        # Join group
        mreq = group_bin + struct.pack('=I', socket.INADDR_ANY)
        self.server.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)


#        print 'Listening to port',port,'...'
#        self.server.listen(backlog)
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
                inputready,outputready,exceptready = select.select(inputs, [], inputs)
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

#                if s == self.server:
#                    # handle the server socket
#                    client, address = self.server.accept()
#                    print 'chatserver: got connection %d from %s' % (client.fileno(), address)
#                    # Compute client name and send back
#                    self.clients += 1
#                    cname = 'client%d' % self.clients
#                    client.setblocking(0)
#                    #send(client, 'CLIENT: ' + str(address[0]))
#                    client.send('CLIENT: ' + str(address[0]))
#                    inputs.append(client)
#
#                    self.clientmap[client] = (address, cname)
#                    # Send joining information to other clients
#                    msg = '\n(Connected: New client (%d) from %s)' % (self.clients, self.getname(client))
#                    print '%s' % msg
#                    for o in self.outputs:
#                        o.send(msg)
#                        #send(o, msg)
#
#                    self.outputs.append(client)
#
#                elif s == sys.stdin:
                if s == sys.stdin:
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
                            #print '%s' % msg
                            #for o in self.outputs:
                            #    if o != s:
                            #        o.send(msg)
                            #        #send(o, msg)
                            wx.CallAfter( parseMessage, msg )
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

                        if hasattr( socket, 'fileno' ):
				print 'chatserver: socket %d error: %s' % ( socket.fileno(), self.getname(socket))
                        else:
				print 'chatserver: socket ? error: %s' % ( self.getname(socket))

                        # Remove
                        inputs.remove(s)
                        self.outputs.remove(s)



        print "\r\nclosing server\r\n"
        self.server.close()


#console = netConsole.netConsole()
console = netConsole()



if __name__ == "__main__":
    mywin.show()
    thread.start_new( gui.main_loop, () )
    console.serve()

