

import socket

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

address = ("127.0.0.1",5002)


sock.bind(address)


while True:
    data,adr=sock.recvfrom(1024)
    data=data.decode("utf-8")
    # data = data.split("[")
    # x,y,z = data[1].
    # accelerometer1,gyro1,accelerometer2,gyro2 = data[3],data[4],data[5],data[6]
    # print(f"x: {x} y: {y} z: {z}")
    # print(f"acc1: {accelerometer1} gyro1: {gyro1} acc2: {accelerometer2} gyro2: {gyro2}")
    print(data)