import sys
import time
import random
from math import*

pi = 3.14159265

class motorPid:
  def __init__(self, Motor, kP, dir):
    self.Motor = Motor      # Чем мы управляем
    self.encoderOld = 0     
    self.kP  = kP           
    self.dir = dir          # Направление
    self.targetSpeed = 0    
    coefA = 1        
    coefB = 1
    
  
  def smoothM (self, encoder, dt):
    Real_Speed = (encoder - self.encoderOld)/dt
    speed = self.targetSpeed * self.coefA + self.coefB + ((self.targetSpeed - Real_Speed) * self.kP * self.dir)
    self.encoderOld = encoder
    self.Motor(speed)


oldTime = 0
m1 = motorPid(brick.motor(M1).setPower, 1, 1)
while True:
  dt = int((time.time()-oldTime)*1000)
  enc = brick.encoder(E1).read()

  m1.targetSpeed = 50
  m1.smoothM(enc, dt)
