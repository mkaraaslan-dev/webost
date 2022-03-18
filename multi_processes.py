# -*- coding: utf-8 -*-
"""
Created on Sat Mar 12 01:12:08 2022

@author: MAHMUT
"""

from multiprocessing import Process
import cv2
import io
import socket
import struct
# import time
import pickle
# import zlib

def func1():
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
        print(frame)    
        data = pickle.dumps(frame, 0)
        size = len(data)
    
    
        print("{}: {}".format(img_counter, size))
        client_socket.sendall(struct.pack(">L", size) + data)
        img_counter += 1
    
    cam.release()


def func2():
    HOST=''
    PORT=8485
    
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
            data += conn.recv(4096)
    
        print("Done Recv: {}".format(len(data)))
        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack(">L", packed_msg_size)[0]
        print("msg_size: {}".format(msg_size))
        while len(data) < msg_size:
            data += conn.recv(4096)
        frame_data = data[:msg_size]
        data = data[msg_size:]
    
        frame=pickle.loads(frame_data, fix_imports=True, encoding="bytes")
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        cv2.imshow('ImageWindow',frame)
        cv2.waitKey(1)


if __name__ == '__main__':
    p1 = Process(target=func1)
    p1.start()
    p2 = Process(target=func2)
    p2.start()
    p1.join() # Process1'in tamamlanmasını beklemek için kullanılır
    p2.join()

# func1: starting
# func2: starting
# func2: finishing
# func1: finishing