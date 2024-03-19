
import sys
import time
import random
from math import*

pi = 3.14159265


class Motor_pid:
  def __init__(self, Encoder, Motor, kp, kd, ki, rev):
    self.last_encoder = 0
    self.last_time = 0
    self.target_speed = 700
    self.last_error = 0
    self.error_sum = 0
    self.motor = Motor
    self.enc = Encoder
    self.kP = kp
    self.kD = kd
    self.kI = ki
    self.rev = rev
   
  def set_target_speed(self, speed):
    self.target_speed = speed
  
  

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
    
    signal = self.target_speed/9 + dP + dD + dI
    
    if(signal > 100):
      signal = 100
    if(signal < -100):
      signal = -100
    
    #print(real_speed, self.target_speed, signal)
    brick.display().addLabel(real_speed, 1, 1)
    brick.display().addLabel(signal, 1, 20)
    brick.display().redraw()
    
    
    self.motor(signal)
    
    self.last_error = error
    self.error_sum += error
    if (self.error_sum >= 1000):
      self.error_sum = 1000
    if (self.error_sum <= -1000):
      self.error_sum = -1000
    


def spin(x,y,w, m1_pid, m2_pid, m3_pid, m4_pid):
  L = 202.5/2
  K = 175/2
  B = pi/2

  D = 100

  brick.display().addLabel("text", x, y)
  brick.display().redraw()


  D = D/2
  l = (L**2 + K**2)**0.5
  a = atan2(L, K)
  w1 = ((x*1 + y/tan(-B) + w*((l*cos(-B+a))/sin(-B))))/D
  w2 = ((x*1 + y/tan(B) + w*((l*cos(B+(pi-a)))/sin(B))))/D
  w3 = ((x*1 + y/tan(-B) + w*((l*cos(-B+(-pi+a)))/sin(-B))))/D
  w4 = ((x*1 + y/tan(B) + w*((l*cos(B-a))/sin(B))))/D
  m1_pid.set_target_speed(w1*1000)
  m2_pid.set_target_speed(w2*1000)
  m3_pid.set_target_speed(w3*1000)
  m4_pid.set_target_speed(w4*1000)



class Odometry:
  def __init__(self, Encoder1, Encoder2, Encoder3, Encoder4):
    self.last_w1 = 0
    self.last_w2 = 0
    self.last_w3 = 0
    self.last_w4 = 0
    self.x = 0
    self.y = 0
    self.w = 0
    self.enc1 = Encoder1
    self.enc2 = Encoder2
    self.enc3 = Encoder3
    self.enc4 = Encoder4
   
  def tick(self):
     global col
     current_time = time.time()
     e1 = self.enc1.readRawData()
     e2 = self.enc2.readRawData()
     e3 = self.enc3.readRawData()
     e4 = self.enc4.readRawData()
     dw1 = (e1 - self.last_w1)*ticks_to_mm
     dw2 = (e2 - self.last_w2)*(-1)*ticks_to_mm
     dw3 = (e3 - self.last_w3)*ticks_to_mm
     dw4 = (e4 - self.last_w4)*(-1)*ticks_to_mm
     self.last_w1 = e1
     self.last_w2 = e2
     self.last_w3 = e3
     self.last_w4 = e4
     
     '''print("dw1", dw1)
     print("dw2", dw2)
     print("dw3", dw3)
     print("dw4", dw4)'''
     
     
     self.x += (dw1 + dw2 + dw3 + dw4)/4
     self.y += (dw1 - dw2 - dw3 + dw4)/4
     self.w += ((-dw1+dw2-dw3+dw4)/((l1**2+l2**2)**0.5))/4
     
     '''print("dx", (dw1 + dw2 + dw3 + dw4)/4)
     print("dy", (dw1 - dw2 - dw3 + dw4)/4)
     print("x", self.x)
     print("y", self.y)
     print("w", self.w)
     brick.display().redraw()
     col = 0'''


a = brick.encoder(E1)
m1_pid = Motor_pid(a, brick.motor(M1).setPower, 0.15 , 0, 0,1)
b = brick.encoder(E2)
m2_pid = Motor_pid(b, brick.motor(M2).setPower, 0.15 , 0, 0,-1)
c = brick.encoder(E3)
m3_pid = Motor_pid(c, brick.motor(M3).setPower, 0.15 , 0, 0,1)
d = brick.encoder(E4)
m4_pid = Motor_pid(d, brick.motor(M4).setPower, 0.15 , 0, 0,-1)

a.reset()
b.reset()
c.reset()
d.reset()






spin(0, 0, 10, m1_pid, m2_pid, m3_pid, m4_pid)

while(True):
  script.wait(100)
  m1_pid.motor_pid()
  m2_pid.motor_pid()
  m3_pid.motor_pid()
  m4_pid.motor_pid()