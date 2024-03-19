import sys
import time
import random
from math import*

pi = 3.14159265

L = 202.5/2
K = 175/2
B = pi/4
D=100

D = D/2
l = (L**2 + K**2)**0.5
A = atan2(L, K)

l1 = 101.25
l2 = 175/2

target_x = 0
target_y = -10000
target_w = 0

base_speed = 600

calculation_timer = 0
odometry_timer = 0

'''class Motor_pid:
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
    
    signal = self.target_speed/9 + dP + dD + dI
    
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
      self.error_sum = -1000'''
class Motor_pid:
  def __init__(self, Encoder, Motor, kp, kd, ki, rev):
    self.last_encoder = 0
    self.last_time = time.time()
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
    
  def pidCont(self, target, current):
    error = target- current
    P = error*self.kP
    self.motor(P)
  

  def motor_pid(self):
    current_enc = self.enc.read()
    _dt = int((time.time() - self.last_time) * 1000)
    self.last_time = time.time()
    currentSpeed = (current_enc-self.last_encoder)/_dt
    self.last_encoder = current_enc
    self.pidCont(self.target_speed, currentSpeed)

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
     _dt = (current_time - self.last_time)*1000
     dw1 = ((e1 - self.last_w1)/(_dt))
     dw2 = ((e2 - self.last_w2)/(_dt))*(-1)
     dw3 = ((e3 - self.last_w3)/(_dt))
     dw4 = ((e4 - self.last_w4)/(_dt))*(-1)
     self.last_time = current_time
     self.last_w1 = e1
     self.last_w2 = e2
     self.last_w3 = e3
     self.last_w4 = e4
     
     #brick.display().addLabel(dw1, 1, 0)
     #brick.display().addLabel(dw2, 1, 20)
     #brick.display().addLabel(dw3, 1, 40)
     #brick.display().redraw()
     
     self.x += ((dw1 + dw2 + dw3 + dw4)*D)/2
     self.y += ((dw1 - dw2 - dw3 + dw4)*D)/2
     self.w += (((-dw1+dw2-dw3+dw4)/(l1+l2))*D)/2

def spin(m1, m2, m3, m4, position):
  dx = target_x - position.x
  dy = target_y - position.y
  dw = target_w - position.w

  if(abs(dx) < 100 and abs(dy) <100 and abs(dw) <  0.5):
    m1.target_speed = 0
    m2.target_speed = 0
    m3.target_speed = 0
    m4.target_speed = 0
    return 
  
  brick.display().addLabel(dx, 1, 0)
  brick.display().addLabel(dy, 1, 20)
  brick.display().addLabel(dw, 1, 40)
 
  dw*=((dx*dy)/2)
    
  sum_d = abs(dx)+abs(dy)+abs(dw)
  
  vx = (dx/sum_d)*base_speed
  vy = (dy/sum_d)*base_speed
  vw = (dw/sum_d)*base_speed
  
  vx*=1
  
  brick.display().addLabel(vx, 1, 60)
  brick.display().addLabel(vy, 1, 80)
  brick.display().addLabel(vw, 1, 100)
  
  w1 = (vx + vy - vw*(l1+l2))/D
  w2 = (vx - vy + vw*(l1+l2))/D
  w3 = (vx - vy - vw*(l1+l2))/D
  w4 = (vx + vy + vw*(l1+l2))/D
  
  m1.target_speed = w1*40
  m2.target_speed = w2*40
  m3.target_speed = w3*40
  m4.target_speed = w4*40
  brick.display().addLabel(w1, 1, 120)
  brick.display().addLabel(w2, 1, 140)
  brick.display().addLabel(w3, 1, 160)
  brick.display().addLabel(w4, 1, 180)
  brick.display().redraw()

a = brick.encoder(E1)
m1_pid = Motor_pid(a, brick.motor(M1).setPower, 0.3 , 0, 0,1)

b = brick.encoder(E2)
m2_pid = Motor_pid(b, brick.motor(M2).setPower, 0.3 , 0, 0,-1)

c = brick.encoder(E3)
m3_pid = Motor_pid(c, brick.motor(M3).setPower, 0.3 , 0, 0,1)

d = brick.encoder(E4)
m4_pid = Motor_pid(d, brick.motor(M4).setPower, 0.3 , 0, 0,-1)

a.reset()
b.reset()
c.reset()
d.reset()

odom = Odometry(a, b, c, d)

while(True):
  if (time.time() - odometry_timer > 0.025):
   odometry_timer = time.time()
   odom.tick()
   
  if (time.time() - calculation_timer > 0.01):
    calculation_timer = time.time()
    spin(m1_pid, m2_pid, m3_pid, m4_pid, odom)
    m1_pid.motor_pid()
    m2_pid.motor_pid()
    m3_pid.motor_pid()
    m4_pid.motor_pid()

  #brick.display().addLabel(odometry_timer, 1, 200)
  #brick.display().addLabel(time.time(), 1, 220)
  #brick.display().addLabel(odom.w, 1, 240)
  #brick.display().redraw()
  '''brick.display().addLabel(odom.last_w1, 1, 60)
  brick.display().addLabel(odom.last_w2, 1, 80)
  brick.display().addLabel(odom.last_w3, 1, 100)
  brick.display().addLabel(odom.last_w4, 1, 120)
  brick.display().redraw()'''
