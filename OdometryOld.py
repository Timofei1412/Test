import sys
import time
import random
from math import*

pi = 3.14159265

L = 202.5/2
K = 175/2
B = pi/2
D=100

D = D/2
l = (L**2 + K**2)**0.5
a = atan2(L, K)


class Motor_pid:
  
  
  def __init__(self, Encoder, Motor, kp, kd, ki, rev):
    self.last_encoder = 0
    self.last_time = 0
    self.target_speed = 600
    self.last_error = 0
    self.error_sum = 0
    self.motor = Motor
    self.enc = Encoder
    self.kP = kp
    self.kD = kd
    self.kI = ki
    self.rev = rev
    
  
  

  def motor_pid(self):
    current_encoder = self.enc.readRawData()
    current_time = time.time()
    real_speed = ((current_encoder - self.last_encoder)/(current_time*1000 - self.last_time*1000))*1000 *self.rev
    
    self.last_encoder = current_encoder
    self.last_time = current_time
    error = self.target_speed - real_speed
    
    dP = error*self.kP
    dD = (error-self.last_error)*self.kD
    dI = self.error_sum*self.kI
    
    if(self.target_speed < 0):
      signal = -50 + dP + dD + dI
    else:
      signal = 50 + dP + dD + dI
    
    if(signal > 100):
      signal = 100
    if(signal < -100):
      signal = -100
    
    #print(real_speed, self.target_speed, signal)
    #brick.display().addLabel(real_speed, 1, 1)
    #brick.display().addLabel(signal, 1, 20)
    #brick.display().redraw()
    
    
    self.motor(signal)
    
    self.last_error = error
    self.error_sum += error
    if (self.error_sum >= 1000):
      self.error_sum = 1000
    if (self.error_sum <= -1000):
      self.error_sum = -1000


class Odometry:
  def __init__(self, Encoder1, Encoder2, Encoder3, Encoder4):
    self.last_w1 = 0
    self.last_w2 = 0
    self.last_w3 = 0
    self.last_w4 = 0
    self.last_time = 0
    self.x = 0
    self.y = 0
    self.w = 0
    self.enc1 = Encoder1
    self.enc2 = Encoder2
    self.enc3 = Encoder3
    self.enc4 = Encoder4
   
  def tick(self):
     current_time = time.time()
     e1 = self.enc1.readRawData()
     e2 = self.enc2.readRawData()
     e3 = self.enc3.readRawData()
     e4 = self.enc4.readRawData()
     dw1 = ((e1 - self.last_w1)/(current_time*1000 - self.last_time*1000))
     dw2 = ((e2 - self.last_w2)/(current_time*1000 - self.last_time*1000))*-1
     dw3 = ((e3 - self.last_w3)/(current_time*1000 - self.last_time*1000))
     dw4 = ((e4 - self.last_w4)/(current_time*1000 - self.last_time*1000))
     self.last_time = current_time
     self.last_w1 = e1
     self.last_w2 = e2
     self.last_w3 = e3
     self.last_w4 = e4
     
     self.x += 0.5*D*(dw1+dw3)
     self.y += 0.5*D*tan(B)*(dw1-dw2)
     #self.w += ((-1)*D*(dw1-dw3))/(2*l*(cos(a)*cot(B)+sin(a)))
     


def spin(x,y,w):
  brick.display().addLabel("text", x, y)
  brick.display().redraw()
  m1 = brick.motor(M1).setPower
  m2 = brick.motor(M2).setPower
  m3 = brick.motor(M3).setPower
  m4 = brick.motor(M4).setPower
  w1 = ((x*1 + y/tan(-B) + w*((l*cos(-B+a))/sin(-B))))/D
  w2 = ((x*1 + y/tan(B) + w*((l*cos(B+(pi-a)))/sin(B))))/D
  w3 = ((x*1 + y/tan(-B) + w*((l*cos(-B+(-pi+a)))/sin(-B))))/D
  w4 = ((x*1 + y/tan(B) + w*((l*cos(B-a))/sin(B))))/D
  print(w1, w2, w3, w4)
  m1(w1*500)
  m2(w2*500)
  m3(w3*500)
  m4(w4*500)
  script.wait(1000)
  m1(0)
  m2(0)
  m3(0)
  m4(0)


a = brick.encoder(E1)
m1_pid = Motor_pid(a, brick.motor(M1).setPower, 0.3 , 0, 0,1)
b = brick.encoder(E2)
m2_pid = Motor_pid(b, brick.motor(M2).setPower, 0.3 , 0, 0,-1)

c = brick.encoder(E3)
m3_pid = Motor_pid(c, brick.motor(M3).setPower, 0.3 , 0, 0,1)

d = brick.encoder(E4)
m4_pid = Motor_pid(d, brick.motor(M4).setPower, 0.3 , 0, 0,-1)

odom = Odometry(a, b, c, d)

while(True):
  odom.tick()
  brick.display().addLabel(odom.x, 1, 1)
  brick.display().addLabel(odom.y, 1, 20)
  brick.display().addLabel(odom.w, 1, 40)
  brick.display().redraw()
  script.wait(50)