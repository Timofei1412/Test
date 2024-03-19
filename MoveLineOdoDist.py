import sys
import time
import random
from math import*
from trik import brick
from q import script

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

target_x = 1000
target_y = 0
target_w = 0

base_speed = 0.35

calculation_timer = 0
odometry_timer = 0

ticks_to_mm = (pi*100)/(12*45)

display_timer = 0

def sign(val):
  if val < 0:
    return -1
  if val:
    return 1
  return 0
  
col = 20
def print(s, val):
  global col
  brick.display().addLabel(s + " " + str(round(val, 2)), 1, col)
  col += 20
      
class Motor_pid:
  def __init__(self, Encoder, motor, kP, dir, coefA, coefB):
    self.last_encoder = 0
    self.last_time = 0
    self.motor = motor      # Чем мы управляем
    self.enc = Encoder    
    self.kP  = kP           
    self.dir = dir          # Направление
    self.target_speed = 0    
    self.coefA = coefA
    self.coefB = coefB
   
  def set_motor(self, speed):
    self.motor(speed)
    
  
  def motor_pid (self):
    cur_enc = self.enc.readRawData()
    real_speed = ((cur_enc - self.last_encoder)/((time.time()-self.last_time)*1000))*ticks_to_mm * self.dir
    
    self.last_encoder = cur_enc
    self.last_time = time.time()
    base_value = (abs(self.target_speed) * self.coefA + self.coefB)*sign(self.target_speed)
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


class Odometry:
  def __init__(self, Encoder1, Encoder2, Encoder3, Encoder4):
    self.last_w1 = 0
    self.last_w2 = 0
    self.last_w3 = 0
    self.last_w4 = 0
    self.x = 0
    self.y = 0
    self.x_glob = 0
    self.y_glob = 0
    self.w = 0
    self.enc1 = Encoder1
    self.enc2 = Encoder2
    self.enc3 = Encoder3
    self.enc4 = Encoder4
    self.dw1 = 0
    self.dw2 = 0
    self.dw3 = 0
    self.dw4 = 0
   
  def tick(self):
     global col
     current_time = time.time()
     e1 = self.enc1.readRawData()
     e2 = self.enc2.readRawData()
     e3 = self.enc3.readRawData()
     e4 = self.enc4.readRawData()
     self.dw1 = (e1 - self.last_w1)*ticks_to_mm
     self.dw2 = (e2 - self.last_w2)*(-1)*ticks_to_mm
     self.dw3 = (e3 - self.last_w3)*ticks_to_mm
     self.dw4 = (e4 - self.last_w4)*(-1)*ticks_to_mm
     self.last_w1 = e1
     self.last_w2 = e2
     self.last_w3 = e3
     self.last_w4 = e4
     
     dx = (self.dw1 + self.dw2 + self.dw3 + self.dw4)/4
     dy = (self.dw1 - self.dw2 - self.dw3 + self.dw4)/4
     
     self.x += dx
     self.y += dy
     self.x_glob += (dx*cos(self.w) - dy*sin(self.w))
     self.y_glob += (dx*sin(self.w) + dy*cos(self.w))
     self.w += (((-self.dw1+self.dw2-self.dw3+self.dw4)/((l1**2+l2**2)**0.5))/4)*0.702


a = brick.encoder(E1)
m1_pid = Motor_pid(a, brick.motor(M1).setPower, 60, -1, 133, -17)

b = brick.encoder(E2)
m2_pid = Motor_pid(b, brick.motor(M2).setPower, 60, -1, 150, -17)

c = brick.encoder(E3)
m3_pid = Motor_pid(c, brick.motor(M3).setPower, 60, -1, 133, -17)

d = brick.encoder(E4)
m4_pid = Motor_pid(d, brick.motor(M4).setPower, 60, -1, 133, -17)

a.reset()
b.reset()
c.reset()
d.reset()

odom = Odometry(a, b, c, d)

def goToLine(speed = 0.3, kP = 0.005):
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d
  oldTime = 0
  ligth1 = brick.sensor("A6").read()
  ligth2 = brick.sensor("A5").read()
  startDist = odom.x
  currentDist = odom.x    
  while brick.sensor("D2").read() < dist:
    odom.tick()
    if(time.time()-oldTime>0.01):
      currentDist = odom.x
      oldTime = time.time()
      ligth1 = brick.sensor("A6").read()
      ligth2 = brick.sensor("A5").read()
      error = (ligth2 - ligth1) * kP
      m1_pid.target_speed = speed - error
      m2_pid.target_speed = speed + error
      m3_pid.target_speed = speed 
      m4_pid.target_speed = speed 
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()


goToLine()

while(True):
  if (time.time() - odometry_timer > 0.05):
    odometry_timer = time.time()
    odom.tick()
  
  if (time.time() - display_timer > 0.1):
    display_timer = time.time()
    print("dw1", odom.dw1)
    print("dw2", odom.dw1)
    print("dw3", odom.dw1)
    print("dw4", odom.dw1)
    print("x", odom.x)
    print("y", odom.y)
    print("x_glob", odom.x_glob)
    print("y_glob", odom.y_glob)
    print("w", odom.w)
    brick.display().redraw()
    col = 0

