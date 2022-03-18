# -*- coding: utf-8 -*-
"""
Created on Sat Mar 12 01:12:08 2022

@author: MAHMUT
"""
from io import BytesIO
from io import StringIO
from multiprocessing import Process

import cv2
import io
import socket
import struct
import time
import pickle
import zlib
import numpy as np
from PIL import Image
def func1():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('127.0.0.1', 8485))
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
    
    
        print("{}: {}".format(img_counter, size))
        client_socket.sendall(struct.pack(">L", size) + data)
        img_counter += 1
    
    cam.release()

def convert_string_to_bytes(string):
    bytes = b''
    for i in string:
        bytes += struct.pack("B", ord(i))
    return bytes


def func2():
    HOST=''
    PORT=5000
    
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    print('Socket created')
    
    s.bind((HOST,PORT))
    print('Socket bind complete')
    s.listen(10)
    print('Socket now listening')
    
    conn,addr=s.accept()
    
    data = b""
    payload_size = struct.calcsize(">L")
    print("payload_size: {}".format(payload_size))
    while True:
        while len(data) < payload_size:
            print("Recv: {}".format(len(data)))
            # data += conn.recv(1228800)
            data += conn.recv(1330425)
    
        print("Done Recv: {}".format(len(data)))
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        print("msg_size: {}".format(msg_size))
        while len(data) < msg_size:
            # data += conn.recv(1228800)
            data += conn.recv(1330425)
        frame_data = data[:msg_size]
        data = data[msg_size:]
    
        frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        # frame = np.fromstring(frame.getvalue(), dtype='uint8')
        print(type(frame)) 
        
        # frame = frame.astype(np.uint8)
        # frame = Image.frombytes("L", (3, 2), frame)
        #frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        # stream = StringIO(frame)

        # image = Image.open(stream).convert("RGBA")  
        # print(frame)
        # stream.close()
        img = Image.frombytes("RGBA", (640, 480), frame)
        # stream = BytesIO(bytes(frame))
        img = np.array(img)
        # image = Image.open(stream).convert("RGBA")
        # stream.close()
        # image.show()
        cv2.imshow('ImageWindow',img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break
        


func2()