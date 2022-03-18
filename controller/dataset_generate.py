# -*- coding: utf-8 -*-
"""
Created on Fri Mar  4 10:22:35 2022

@author: MAHMUT KARAASLAN
"""
from controller import Robot,InertialUnit
from controller import Motor,Gyro
from controller import Compass,GPS,Camera
from controller import Keyboard
import numpy as np
import os
import math
"Create robot"
robot = Robot()


"get the time step of the current world."
timestep = int(robot.getBasicTimeStep())
# RPY/Euler angles to Rotation Vector

def euler_to_rotVec(yaw, pitch, roll):
    # compute the rotation matrix
    Rmat = euler_to_rotMat(yaw, pitch, roll)
    
    theta = math.acos(((Rmat[0, 0] + Rmat[1, 1] + Rmat[2, 2]) - 1) / 2)
    sin_theta = math.sin(theta)
    if sin_theta == 0:
        rx, ry, rz = 0.0, 0.0, 0.0
    else:
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (Rmat[2, 1] - Rmat[1, 2]) * theta
        ry = multi * (Rmat[0, 2] - Rmat[2, 0]) * theta
        rz = multi * (Rmat[1, 0] - Rmat[0, 1]) * theta
    return rx, ry, rz

def euler_to_rotMat(yaw, pitch, roll):
    Rz_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]])
    Ry_pitch = np.array([
        [ np.cos(pitch), 0, np.sin(pitch)],
        [             0, 1,             0],
        [-np.sin(pitch), 0, np.cos(pitch)]])
    Rx_roll = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]])
    # R = RzRyRx
    rotMat = np.dot(Rz_yaw, np.dot(Ry_pitch, Rx_roll))
    return rotMat
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]
camera=robot.getCamera("camera")
Camera.enable(camera,timestep)


imu=InertialUnit("inertial unit")
gyro=Gyro("gyro")
compass=Compass("compass")
gps=GPS("gps")
keyboard = Keyboard()

gyro.enable(timestep)
imu.enable(timestep)
compass.enable(timestep)
gps.enable(timestep)
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


k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0
target_altitude = 1.0


def CLAMP(n,minn,maxn):
    if n<minn:
        return minn
    elif n>maxn:
        return maxn
    else :
        return n

file = open("trajectory.txt","w")
name = 0    
os.chdir("rgb")    
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
    qx, qy, qz, qw = get_quaternion_from_euler(roll_acceleration, pitch_acceleration, yaw_acceleration)
    #print(f"roll:{roll} {pitch} {yaw}")
    #print(f"roll_A:{roll_acceleration} {pitch_acceleration} {yaw_acceleration}")
    print(get_quaternion_from_euler(roll_acceleration, pitch_acceleration, yaw_acceleration))
    print(get_quaternion_from_euler(roll, pitch, yaw))
    print(f"for:{to_quaternion(roll, pitch, yaw)}")
    print(f"imu:{imu.getQuaternion()}")  
    #print(f"{name} {gps.getValues()[0]} {gps.getValues()[1]} {gps.getValues()[2]} {qw} {qx} {qy} {qz}\n")
    if key == 83:
        name = name +1
        line = f"{name} {gps.getValues()[0]} {gps.getValues()[1]} {gps.getValues()[2]} {qw} {qx} {qy} {qz}\n"
        
        file.write(line)
        Camera.getImage(camera)
        Camera.saveImage(camera,filename= f"{name}.jpg",quality=100)
    
    while key>0:
        if key == 315:
            pitch_disturbance = -2.0
            
        if key == 317:
            pitch_disturbance = 2.0
            
        if key == 316:
            yaw_disturbance = -1.3
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
    
    

    #print(f"gps:{gps.getValues()}")
    #print(f"{qx} {qy} {qz} {qw}")
    
    
    
    #print(f"roll:{roll} pitch:{yaw} roll:{yaw}")
    #print(f"compass:{compass.getValues()}")
    #print(f"gyro:{gyro.getValues()}")
#file.close()    