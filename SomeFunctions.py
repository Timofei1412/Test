from trik import brick
import script

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


a = brick.encoder("E1")
m1_pid = Motor_pid(a, brick.motor("M1").setPower, 60, -1, 133, -17)

b = brick.encoder("E2")
m2_pid = Motor_pid(b, brick.motor("M2").setPower, 60, -1, 150, -17)

c = brick.encoder("E3")
m3_pid = Motor_pid(c, brick.motor("M3").setPower, 60, -1, 133, -17)

d = brick.encoder("E4")
m4_pid = Motor_pid(d, brick.motor("M4").setPower, 60, -1, 133, -17)

a.reset()
b.reset()
c.reset()
d.reset()

odom = Odometry(a, b, c, d)



#______________________________________________________________________________________________________________________________
#______________________________________________________________________________________________________________________________
#_________________Code starts here_____________________________________________________________________________________________
#______________________________________________________________________________________________________________________________


def gyroTurn(angle):
  THRESHOLD = 5 # Окно
  startAngle = brick.gyroscope().read()[6]/1000 # Куда мы смотрели изначально
  currentAngle = brick.gyroscope().read()[6]/1000
  while (startAngle + angle  - currentAngle < THRESHOLD):
    odom.tick()
    if(time.time() - oldTime > 0.2):
      if(angle>0):                              # Крутимся по часовой стрелке
        m1_pid.target_speed = 50
        m2_pid.target_speed = -50 
        m3_pid.target_speed = 50
        m4_pid.target_speed = -50
      else:                                     # Крутимся против часовой стрелки
        m1_pid.target_speed = -50
        m2_pid.target_speed = 50
        m3_pid.target_speed = -50
        m4_pid.target_speed = 50
      currentAngle = brick.gyroscope().read()[6]/1000
      oldTime = time.time()

#______________________________________________________________________________________________________________________________
#______________________________________________________________________________________________________________________________

def lineFollowTime(tmr, speed = 0.3, kP = 0.005): 
  global m1_pid, m2_pid, m3_pid, m4_pid
  oldTime = 0 
  startTime = time.time()
  ligth = brick.sensor("A6").read()
  while time.time() - startTime < tmr:
    odom.tick()
    if(time.time()-oldTime>0.01):
      oldTime = time.time()
      ligth = brick.sensor("A6").read()
      error = (36 - ligth) * kP
      m1_pid.target_speed = -speed
      m2_pid.target_speed = -speed 
      m3_pid.target_speed = -speed - error
      m4_pid.target_speed = -speed + error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()


def lineFollowDist(dist, speed = 0.3, kP = 0.005):
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d
  oldTime = 0
  ligth = brick.sensor("A6").read()
  startDist = odom.x
  currentDist = odom.x    
  while abs(currentDist-startDist) < dist:
    odom.tick()
    if(time.time()-oldTime>0.01):
      currentDist = odom.x
      oldTime = time.time()
      ligth = brick.sensor("A6").read()
      error = (36 - ligth) * kP
      m1_pid.target_speed = -speed
      m2_pid.target_speed = -speed 
      m3_pid.target_speed = -speed - error
      m4_pid.target_speed = -speed + error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
      

def lineFollowUntilCross(speed = 0.3, kP = 0.005): 
  global m1_pid, m2_pid, m3_pid, m4_pid
  oldTime = 0 
  ligth1 = brick.sensor("A6").read()
  ligth2 = brick.sensor("A5").read()
  while ligth2 != 1 and ligth1 != 1:
    odom.tick()
    if(time.time()-oldTime>0.01):
      oldTime = time.time()
      ligth1 = brick.sensor("A6").read()
      ligth2 = brick.sensor("A5").read()
      error = (36 - ligth1) * kP
      m1_pid.target_speed = -speed
      m2_pid.target_speed = -speed 
      m3_pid.target_speed = -speed - error
      m4_pid.target_speed = -speed + error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()

#______________________________________________________________________________________________________________________________
#______________________________________________________________________________________________________________________________


def moveSideToLine(side, kZ, speed = 0.3, kP = 0.005): # Side  <- 1|0 ->
  oldTime = time.time()
  startZ = brick.gyroscope().read()[6]//100
  z = brick.gyroscope().read()[6]//100
  ligth1 = brick.sensor("A6").read()
  ligth2 = brick.sensor("A5").read()
  if side == 1:
    while ligth1 < 35:    # <---- side
      z = brick.gyroscope().read()[6]//100
      odom.tick()
      error = (startZ - z) * kZ
      m1_pid.target_speed = speed - error
      m2_pid.target_speed = -speed + error
      m3_pid.target_speed = -speed - error
      m4_pid.target_speed = speed + error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
  else:
    while ligth2 < 35:    # ----> side
      z = brick.gyroscope().read()[6]//100
      odom.tick()
      error = (startZ - z) * kZ
      m1_pid.target_speed = -speed + error
      m2_pid.target_speed = speed - error
      m3_pid.target_speed = speed + error
      m4_pid.target_speed = -speed - error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()



def moveSideTime(time, side, kZ, speed = 0.3, kP = 0.005): # Side  <- 1|0 ->
  oldTime = time.time()
  startTime = time.time()
  startZ = brick.gyroscope().read()[6]//100
  z = brick.gyroscope().read()[6]//100
  ligth1 = brick.sensor("A6").read()
  ligth2 = brick.sensor("A5").read()
  if side == 1:
    while time.time()-startTime < time:    # <---- side
      z = brick.gyroscope().read()[6]//100
      odom.tick()
      error = (startZ - z) * kZ
      m1_pid.target_speed = speed - error
      m2_pid.target_speed = -speed + error
      m3_pid.target_speed = -speed - error
      m4_pid.target_speed = speed + error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()

  else:
    while time.time()-startTime < time:    # ----> side
      z = brick.gyroscope().read()[6]//100
      odom.tick()
      error = (startZ - z) * kZ
      m1_pid.target_speed = -speed + error
      m2_pid.target_speed = speed - error
      m3_pid.target_speed = speed + error
      m4_pid.target_speed = -speed - error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()

def moveSideDist(dist, side, kZ, speed = 0.3, kP = 0.005): # Side  <- 1|0 ->
  oldTime = time.time()
  startY = odom.y
  startZ = brick.gyroscope().read()[6]//100
  z = brick.gyroscope().read()[6]//100
  ligth1 = brick.sensor("A6").read()
  ligth2 = brick.sensor("A5").read()
  if side == 1:
    while abs(odom.y-startY) < dist:    # <---- side
      z = brick.gyroscope().read()[6]//100
      odom.tick()
      error = (startZ - z) * kZ
      m1_pid.target_speed = speed - error
      m2_pid.target_speed = -speed + error
      m3_pid.target_speed = -speed - error
      m4_pid.target_speed = speed + error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()

  else:
    while abs(odom.y-startY) < dist:    # ----> side
      z = brick.gyroscope().read()[6]//100
      odom.tick()
      error = (startZ - z) * kZ
      m1_pid.target_speed = -speed + error
      m2_pid.target_speed = speed - error
      m3_pid.target_speed = speed + error
      m4_pid.target_speed = -speed - error
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
      
			


#______________________________________________________________________________________________________________________________
#______________________________________________________________________________________________________________________________

def turnForLine(speed):
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d
  oldTime = 0
  ligth1 = brick.sensor("A6").read()
  ligth2 = brick.sensor("A5").read()
  while ligth1 < 30 and ligth2 < 30:  # While there is no line
    odom.tick()
    if(time.time()-oldTime>0.01):
      ligth1 = brick.sensor("A6").read()
      ligth2 = brick.sensor("A5").read()
      m1_pid.target_speed = -speed
      m2_pid.target_speed = -speed 
      m3_pid.target_speed = -speed
      m4_pid.target_speed = -speed
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
  if ligth1 < 30 and ligth2 > 30: # if we see line with right sensor
    while ligth1 < 30 :
      odom.tick()
      if(time.time()-oldTime>0.01):
        ligth1 = brick.sensor("A6").read()
        ligth2 = brick.sensor("A5").read()
        m1_pid.target_speed = -(speed/4)
        m2_pid.target_speed = -speed 
        m3_pid.target_speed = -(speed/4)
        m4_pid.target_speed = -speed
        m1_pid.motor_pid()
        m2_pid.motor_pid()
        m3_pid.motor_pid()
        m4_pid.motor_pid()
  else:   # if we see line with left sensor
    while ligth2 < 30 :
      odom.tick()
      if(time.time()-oldTime>0.01):
        ligth1 = brick.sensor("A6").read()
        ligth2 = brick.sensor("A5").read()
        m1_pid.target_speed = -speed
        m2_pid.target_speed = -(speed/4)
        m3_pid.target_speed = -speed
        m4_pid.target_speed = -(speed/4)
        m1_pid.motor_pid()
        m2_pid.motor_pid()
        m3_pid.motor_pid()
        m4_pid.motor_pid()
  lineFollowTime(3)    # Lineup with the line
  

#______________________________________________________________________________________________________________________________
#______________________________________________________________________________________________________________________________

s1 = brick.motor("S1").setPower
s2 = brick.motor("S2").setPower
def armSetup():
  s1(-40)
  s2(-40)

def armGetCube():
  script.wait(1000)
  s2(90)
  script.wait(1000)
  s1(-10)
  script.wait(1000)
  s2(-40)
  script.wait(1000)

def armPutDown():
  script.wait(1000)
  s2(90)
  script.wait(1000)
  s1(-10)
  script.wait(1000)


def armPutCube():
  s2(90)
  script.wait(1000)
  s1(-50)
  script.wait(1000)
  s2(-40)
  script.wait(1000)
  

