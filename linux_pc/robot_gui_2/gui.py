#!/usr/bin/python

import select
import socket
import sys
import signal
#import netConsole
import thread
import wx
import struct
import copy

GROUP = '224.0.2.0'
PORT = 5005

BUFSIZ = 1024


# 600 pixels
SCREEN_WIDTH=600
SCREEN_HEIGHT=SCREEN_WIDTH

# 60 inches
WORLD_WIDTH=60
WORLD_HEIGHT=WORLD_WIDTH

# pixels per inch
PX_PER_INCH_X=SCREEN_WIDTH/WORLD_WIDTH
PX_PER_INCH_Y=SCREEN_HEIGHT/WORLD_HEIGHT

class Visualization(wx.Frame):
    # public 
    ir_dist = 0

    # private
    screenRect = 0
    screenCenter = 0

    # ir bitmap
    ir_bitmap = 0

    def __init__(self, parent, title):
        super(Visualization, self).__init__(parent, title=title, 
            size=(600, 600))

        self.Bind(wx.EVT_PAINT, self.OnPaint)
        self.screenRect = wx.Rect( 0, 0, SCREEN_WIDTH, SCREEN_HEIGHT )
        self.screenCenter =  self.Midpoint( self.screenRect.TopLeft, self.screenRect.BottomRight )
        self.Centre()
        self.Show()

        self.ir_bitmap = wx.EmptyBitmap( SCREEN_WIDTH, SCREEN_HEIGHT )
        self.bmpdc = wx.MemoryDC()
        self.bmpdc.SelectObject( self.ir_bitmap )
        self.bmpdc.Clear()
        self.bmpdc.SetPen(wx.Pen('#808080'))
        self.bmpdc.SetBrush(wx.Brush('#808080'))
        self.bmpdc.SelectObject( wx.NullBitmap )

        # Create alpha channel data as an arry of bytes for each pixel in our bitmap
        self.alpha = chr(200) * ( SCREEN_WIDTH * SCREEN_HEIGHT )
        #img = self.ir_bitmap.ConvertToImage()
        #img.InitAlpha()
        #self.alpha = img.GetAlphaData()
        #print "alpha len: %d" % len(self.alpha)
        #for index in range(self.alpha):
        #    self.alpha[index] = chr(200)
        #    print ord(self.alpha[index])
        #
        #print self.alpha

    def Midpoint(self, p1, p2 ):
        x = ( p1.x + p2.x ) / 2.0
        y = ( p1.y + p2.y ) / 2.0
        return wx.Point( x, y )

    def InchesToPixelsX(self, inch):
        return PX_PER_INCH_X * inch

    def InchesToPixelsY(self, inch):
        return PX_PER_INCH_Y * inch

    def SetIrDist(self, distInches ):
        self.ir_dist = self.InchesToPixelsY( distInches  )
        self.Refresh( False )

    def DrawRobot(self, dc ):
        robot_top = wx.Rect(0,0, 10 * PX_PER_INCH_X, 5 * PX_PER_INCH_Y );
	robot_top = robot_top.CenterIn( self.screenRect )
        robot_top.OffsetXY( 0, robot_top.Height / 2.0 )
        

        botLft = robot_top.BottomLeft
        botRht = robot_top.BottomRight
        botMid = self.Midpoint( botLft, botRht )

        dc.DrawRectangleRect( robot_top )
	dc.DrawArcPoint(botLft, botRht, botMid )

    def DrawIrBlip(self, dc ):
        ir_dot = copy.copy(self.screenCenter)
        ir_dot.y = ir_dot.y - self.ir_dist
        #dc.CrossHairPoint(ir_dot)
        #bmpdc = wx.MemoryDC()
        img = self.ir_bitmap.ConvertToImage()

        #img.InitAlpha()
        #img.SetAlphaData( self.alpha )
        factor = 1.05
        img= img.AdjustChannels( factor,factor,factor ) 

        self.ir_bitmap = wx.BitmapFromImage( img )

        self.bmpdc.SelectObject( self.ir_bitmap )
        self.bmpdc.DrawCirclePoint(ir_dot, 2)

        #dc.Blit(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, self.bmpdc, 0, 0 )
        dc.DrawBitmap( self.ir_bitmap, 0, 0 )
        self.bmpdc.SelectObject( wx.NullBitmap )



    def OnPaint(self, e):
        dc = wx.PaintDC(self)
        #dc.DrawLine(50, 60, 190, 60)
        dc.SetPen(wx.Pen('#c56c00'))
        dc.SetBrush(wx.Brush('#c56c00'))

        self.DrawIrBlip( dc )
        self.DrawRobot( dc )


viz = 0  # will be the instance of the visualization class above

def parseMessage(msg):
    print "parsing message: %s" % msg, # comma prevents newline

    if msg.startswith( "Mode=" ):
        params=dict(e.split('=') for e in msg.split(':'))

        viz.SetIrDist( float(params["IR"]) )


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

console = netConsole()

if __name__ == '__main__':
    app = wx.App()
    viz = Visualization(None, 'Line')
    thread.start_new( app.MainLoop, () )
    console.serve()

