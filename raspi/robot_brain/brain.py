#!/usr/bin/env python
#

import time
import struct
import socket
import sys
import visualization

import spinalcord


def brain():

    viz = vizualization.Status()
    spinal= spinalcord.SpinalCord()

    while True:
        arduinoState = spinal.GetStatus()

        print "Mode: " + spinal.State("Mode")


        viz.SendStatus( arduinoState )
        # Allow a rate of approx 10 times per second
        time.sleep(.1)


if __name__ == '__main__':
    brain()

