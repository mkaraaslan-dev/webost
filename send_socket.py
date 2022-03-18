# -*- coding: utf-8 -*-
"""
Created on Mon Mar  7 13:14:12 2022

@author: MAHMUT
"""

import socket

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

while True:
    sock.sendto(("data").encode("utf-8"), ("127.0.0.1",5002))
    