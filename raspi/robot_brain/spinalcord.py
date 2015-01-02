import sys

TTYDEV = "/dev/ttyAMA0"

class SpinalCord:

    state = {}

    def __init__(self):
        self.tty = open( TTYDEV, "r" )

    def GetStatus(self):
        msg = self.tty.readline()

        # Check for our status message, and not some other form of message
        # Open question right now for what to do with non status messages.
        if msg.startswith( "Mode=" ):
            self.state=dict(e.split('=') for e in msg.split(':'))

        return msg

    def State(self, variable):
        return self.state.get( variable, "" )

