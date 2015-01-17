#!/usr/bin/env python

# The raspi is the "brain" of our system, making all of the high level decisions and taking
# care of ethernet communication to the outside world.  Decisions are off loaded to an
# arduino which serves as the "spinal cord" of our system.


import time
import struct
import socket
import sys
import visualization

import spinalcord


def brain():

    viz = visualization.Status()
    spinal= spinalcord.SpinalCord()

    while True:
        arduinoState = spinal.GetStatus()

        print arduinoState,  # note trailing comma prevents newline print
        #print "Mode: " + spinal.State("Mode")


        viz.SendStatus( arduinoState )
        # Allow a rate of approx 10 times per second
        #time.sleep(.1)


if __name__ == '__main__':
    brain()

