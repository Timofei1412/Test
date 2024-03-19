import sys
import time
import random
from math import*
pi = 3.14159265
ticks_to_mm = (pi*100)/(6*45)
class Motor_pid:
  def __init__(self, Encoder, motor, kP, dir):
    self.last_encoder = 0
    self.last_time = 0
    self.motor = motor      # Чем мы управляем
    self.enc = Encoder    
    self.kP  = kP           
    self.dir = dir          # Направление
    self.target_speed = 0    
    self.coefA = 133
    self.coefB = -17
   
  def set_motor(self, speed):
    self.motor(speed)
    
  
  def motor_pid (self):
    cur_enc = self.enc.readRawData()
    real_speed = ((cur_enc - self.last_encoder)/((time.time()-self.last_time)*1000))*ticks_to_mm * self.dir
    
    self.last_encoder = cur_enc
    self.last_time = time.time()
    base_value = self.target_speed * self.coefA + self.coefB
    correction = (self.target_speed - real_speed)*self.kP
    speed =  base_value + correction
    if self.target_speed == 0:
      speed = 0
    #brick.display().addLabel(real_speed, 1, 1)
    #brick.display().addLabel(base_value, 1, 20)
    #brick.display().addLabel(speed, 1, 40)
    #brick.display().addLabel(speed, 1, 60)
    #brick.display().addLabel(self.targetSpeed, 1, 80)
    #brick.display().redraw()
    self.motor(speed)
oldTime = 0
a = brick.encoder(E1)
m1_pid = Motor_pid(a, brick.motor(M1).setPower, 0.3 , 1)
b = brick.encoder(E2)
m2_pid = Motor_pid(b, brick.motor(M2).setPower, 0.3 , 1)
c = brick.encoder(E3)
m3_pid = Motor_pid(c, brick.motor(M3).setPower, 0.3 , 1)
d = brick.encoder(E4)
m4_pid = Motor_pid(d, brick.motor(M4).setPower, 0.3 , 1)
last_color = 0
current_dir = 0
oldTime = 0
kP = 0.005
speed = 0.3
ligth = brick.sensor(A6).read()
dist = brick.sensor(D1).read()
wall = False
wallD = 50
while not wall:
  if(time.time()-oldTime>0.01):
    oldTime = time.time()
    ligth = brick.sensor(A6).read()
    dist = brick.sensor(D1).read()
    error = (36 - ligth) * kP
    m1_pid.target_speed = -speed
    m2_pid.target_speed = -speed 
    m3_pid.target_speed = -speed - error
    m4_pid.target_speed = -speed + error
    brick.display().addLabel(brick.sensor(A6).read(),1,10)
    brick.display().redraw()
    m1_pid.motor_pid()
    m2_pid.motor_pid()
    m3_pid.motor_pid()
    m4_pid.motor_pid()
    if(dist>wallD):
      wall = True
    
    
