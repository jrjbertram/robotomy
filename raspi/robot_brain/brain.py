#!/usr/bin/env python
#

import time
import struct
import socket
import sys
import status

import spinalcord


def brain():

    stat = status.Status()
    spinal= spinalcord.SpinalCord()

    while True:
        arduinoState = spinal.GetStatus()
        stat.SendStatus( arduinoState )

        print "Mode: " + spinal.state["Mode"]


        # Allow a rate of approx 10 times per second
        time.sleep(.1)


if __name__ == '__main__':
    brain()

