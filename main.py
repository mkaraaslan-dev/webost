# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:22:35 2022

@author: MAHMUT KARAASLAN
"""
from controller import Robot,InertialUnit
from controller import Motor,Gyro
from controller import Compass,GPS,Camera
from controller import Keyboard
from controller import Accelerometer

from multiprocessing import Process
import cv2
import io
import socket
import struct
import time
import pickle
import zlib

"Create robot"
robot = Robot()

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

"get the time step of the current world."
timestep = int(robot.getBasicTimeStep())



camera=robot.getCamera("camera")
Camera.enable(camera,timestep)

accelerometer1 = Accelerometer("accelerometer1")
imu=InertialUnit("inertial unit")
gyro=Gyro("gyro")
compass=Compass("compass")
gps=GPS("gps")
keyboard = Keyboard()

gyro.enable(timestep)
imu.enable(timestep)
compass.enable(timestep)
gps.enable(timestep)
accelerometer1.enable(timestep)
keyboard.enable(timestep)


front_left_motor=robot.getMotor("front left propeller")
front_right_motor=robot.getMotor("front right propeller")
rear_right_motor=robot.getMotor("rear right propeller")
rear_left_motor=robot.getMotor("rear left propeller")

camera_roll_motor=robot.getMotor("camera roll")
camera_pitch_motor=robot.getMotor("camera pitch")

front_left_motor.setPosition(float("inf"))
front_right_motor.setPosition(float("inf"))
rear_left_motor.setPosition(float("inf")) 
rear_right_motor.setPosition(float("inf"))

front_left_motor.setVelocity(1.0) 
front_right_motor.setVelocity(1.0) 
rear_left_motor.setVelocity(1.0) 
rear_right_motor.setVelocity(1.0)





def CLAMP(n,minn,maxn):
    if n<minn:
        return minn
    elif n>maxn:
        return maxn
    else :
        return n

def robot_main():    
    k_vertical_thrust = 68.5
    k_vertical_offset = 0.6
    k_vertical_p = 3.0
    k_roll_p = 50.0
    k_pitch_p = 30.0
    target_altitude = 1.0
    while robot.step(timestep) != -1:
        
        "Retrieve robot position using the sensors."
        roll=imu.getRollPitchYaw()[0]
        pitch=imu.getRollPitchYaw()[1]
        yaw=imu.getRollPitchYaw()[2]
        
        altitude=gps.getValues()[2]
            
        roll_acceleration = gyro.getValues()[0]
        pitch_acceleration = gyro.getValues()[1]
        yaw_acceleration = gyro.getValues()[2]
        
        "Stabilize the Camera by actuating the camera motors according to the gyro feedback."
        camera_roll_motor.setPosition(0.1*roll_acceleration)
        camera_pitch_motor.setPosition(0.1*pitch_acceleration)
        
        "Transform the keyboard input to disturbances on the stabilization algorithm."
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0
        key=keyboard.getKey()
        print(key)
        while key>0:
            if key == 315:
                pitch_disturbance = -2.0
                
            if key == 317:
                pitch_disturbance = 2.0
            "sağ 316"    
            if key == 316:
                yaw_disturbance = -1.3
            "sol 314"
            if key == 314:
                yaw_disturbance = 1.3
            
            if key == 65852:
                roll_disturbance = -1.0
                
            if key == 65850:
                roll_disturbance = 1.0
            if key == 65851:
                target_altitude += 0.05
            if key == 65853:
                target_altitude -= 0.05
            key=keyboard.getKey()
            
        "Compute the roll, pitch, yaw and vertical inputs."
        roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) + pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
        vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)
        
        "Actuate the motors taking into consideration all the computed inputs."
        
        front_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
        front_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
        rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
        rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
        
        front_left_motor.setVelocity(front_left_motor_input) 
        front_right_motor.setVelocity(-front_right_motor_input) 
        rear_left_motor.setVelocity(-rear_left_motor_input) 
        rear_right_motor.setVelocity(rear_right_motor_input) 
        
        
    
        print(f"gps:{gps.getValues()}")
        print(imu.getQuaternion())
        print(accelerometer1.getValues())
        #print(f"{qx} {qy} {qz} {qw}")        
        #print(f"roll:{roll} pitch:{yaw} yaw:{yaw}")
        #print(f"compass:{compass.getValues()}")
        #print(f"gyro:{gyro.getValues()}")
        
        #sock.sendto((f"gps:{gps.getValues()}").encode("utf-8"), ("127.0.0.1",5002))
        #sock.sendto((f"roll:{roll} pitch:{yaw} yaw:{yaw}").encode("utf-8"), ("127.0.0.1",5002))
        #sock.sendto((f"compass:{compass.getValues()}").encode("utf-8"), ("127.0.0.1",5002))
        #sock.sendto((f"gyro:{gyro.getValues()}").encode("utf-8"), ("127.0.0.1",5002))
        #sock.sendto((f"----------------------").encode("utf-8"), ("127.0.0.1",5002))
def send_photo():
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
    
    
        print("{}: {}".format(img_counter, size))
        client_socket.sendall(struct.pack(">L", size) + data)
        img_counter += 1

    cam.release()
def send_data():
    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    while True:
        sock.sendto((f"gps:{gps.getValues()}").encode("utf-8"), ("127.0.0.1",5002))
if __name__ == '__main__':    
    
    p1 = Process(target=robot_main)
    p1.start()
    p2 = Process(target=send_photo)
    p2.start()
    p3 = Process(target=send_data)
    p3.start()
    p1.join() # Process1'in tamamlanmasını beklemek için kullanılır
    p2.join()
    p3.join()       