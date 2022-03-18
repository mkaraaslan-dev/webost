# -*- coding: utf-8 -*-
"""
Created on Sat Mar 12 01:16:41 2022

@author: MAHMUT
"""
import cv2
import io
import socket
import struct
import time
import pickle
import zlib

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.246.20', 8485))
connection = client_socket.makefile('wb')

cam = cv2.VideoCapture(0)

cam.set(3, 320);
cam.set(4, 240);

img_counter = 0

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

while True:
    ret, frame = cam.read()
    result, frame = cv2.imencode('.jpg', frame, encode_param)
#    data = zlib.compress(pickle.dumps(frame, 0))
    data = pickle.dumps(frame, 0)
    size = len(data)
    print(frame)

    #print("{}: {}".format(img_counter, size))
    client_socket.sendall(struct.pack(">L", size) + data)
    img_counter += 1

cam.release()
# import socket
# import cv2 

# cap=cv2.VideoCapture(0)

# # read a test image
# while True:
#     img = cap.read()
#     # encode it to jpg format, you can do this without redundant file openings
#     retval, buf = cv2.imencode(".JPEG", img)
#     # get number of bytes
#     number_of_bytes = len(buf)
#     # create a null terminated string
#     header = "" + str(number_of_bytes) + "\0"
#     # encode it to utf-8 byte format
#     raw_header = bytes(header, "utf-8")
#     # create server socket
#     sock = socket.socket()
#     sock.bind(('localhost', 8000))
#     sock.listen()
#     conn, addr = sock.accept()
#     # send header first, reciever will use it to recieve image
#     conn.send(raw_header)
#     # send the rest of image
#     conn.send(buf)
#     cv2.waitKey(1)
    