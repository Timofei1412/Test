import sys
import time
import random
from math import*

pi = 3.14159265

class motorPid:
  def __init__(self, motor, kP, dir):
    self.motor = motor      # Чем мы управляем
    self.encoderOld = 0     
    self.kP  = kP           
    self.dir = dir          # Направление
    self.baseSpeed = 0    
    self.COEF_A = 1        
    self.COEF_b = 1
    
  
  def smoothM (self, encoder, dt, addition = 0):
    Real_Speed = (encoder - self.encoderOld)/dt
    basePidSpeed = self.baseSpeed * self.COEF_A + self.COEF_B
    error = self.baseSpeed - Real_Speed
    speed = basePidSpeed + (error * self.kP * self.dir)
    self.encoderOld = encoder
    self.motor(speed + addition)


m1 = motorPid(brick.motor(M1).setPower,1,1)
m2 = motorPid(brick.motor(M2).setPower,1,-1)
m3 = motorPid(brick.motor(M3).setPower,1,1)
m4 = motorPid(brick.motor(M4).setPower,1,-1)
m1.targetSpeed = 50
m2.targetSpeed = 50
m3.targetSpeed = 50
m4.targetSpeed = 50


current_dir =1
last_color = 0
oldTime = 0
while True:
  dt = int((time.time()-oldTime)*1000)
  enc1 = brick.encoder(E1).read()
  enc2 = brick.encoder(E2).read()
  enc3 = brick.encoder(E3).read()
  enc4 = brick.encoder(E4).read()
  if(brick.sensor(D5).readRawData()-last_color>20):
    current_dir = 1 if(current_dir<0) else -1  
  m1.smoothM(enc1,dt,current_dir* 20)
  m2.smoothM(enc2,dt,-(current_dir* 20))
  m3.smoothM(enc3,dt,current_dir* 20)
  m4.smoothM(enc4,dt,-(current_dir* 20))
  
