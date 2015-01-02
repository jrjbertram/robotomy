import socket
import sys
import struct

PORT = 5005
GROUP = '224.0.2.0'
MYTTL = 1 # Increase to reach other networks

class Status:
    def __init__(self):
        self.addrinfo = socket.getaddrinfo(GROUP, None)[0]

        self.sock = socket.socket(self.addrinfo[0], socket.SOCK_DGRAM)

        # Set Time-to-live (optional)
        ttl_bin = struct.pack('@i', MYTTL)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl_bin)

    def SendStatus(self, message):
        self.sock.sendto(message + '\0', (self.addrinfo[4][0], PORT))
