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

base_speed_x = 1.4
base_speed_y = 1.7

calculation_timer = 0
odometry_timer = 0

ticks_to_mm = (pi*100)/(12*45)

display_timer = 0

delta_y = 0
delta_x = 0

brake_x = 70
brake_y = 50

def sign(val):
  if val < 0:
    return -1
  if val:
    return 1
  return 0
 
def sign_2(val):
  if val < 0:
    return -1
  return 1
  
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
    self.accel = 20
    self.last_speed = 0
   
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
    if abs(speed) - abs(self.last_speed) > self.accel:
      speed = self.last_speed + sign(speed)*self.accel
    self.motor(speed)
    self.last_speed = speed


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
    self.x_proj = 0
    self.y_proj = 0
    self.w = 0
    self.w_loc = 0
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
     self.x_glob += (dx*cos(self.w) + dy*sin(self.w))
     self.y_glob += (-dx*sin(self.w) + dy*cos(self.w))
     dw = (((-self.dw1+self.dw2-self.dw3+self.dw4)/((l1**2+l2**2)**0.5))/4)*0.702
     self.w += dw
     self.w_loc += dw
     self.x_proj = (self.x_glob*cos(-self.w) + self.y_glob*sin(-self.w))
     self.y_proj = (-self.x_glob*sin(-self.w) + self.y_glob*cos(-self.w))



a = brick.encoder(E1)
m1_pid = Motor_pid(a, brick.motor(M1).setPower, 60, -1, 133, -17)
b = brick.encoder(E2)
m2_pid = Motor_pid(b, brick.motor(M2).setPower, 60, -1, 133, -17)
c = brick.encoder(E3)
m3_pid = Motor_pid(c, brick.motor(M3).setPower, 60, -1, 133, -17)
d = brick.encoder(E4)
m4_pid = Motor_pid(d, brick.motor(M4).setPower, 60, -1, 133, -17)

a.reset()
b.reset()
c.reset()
d.reset()

odom = Odometry(a, b, c, d)

def goToLineSensor(MINDIST = 35, speed = 0.3, kP = 0.005):
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d
  oldTime = 0
  ligth1 = brick.sensor("A6").read()    # Значение датчика 1
  ligth2 = brick.sensor("A5").read()    # Значение датчика 2
  while brick.sensor("D2").read() > MINDIST:    # Пока мы дальше чем MINDIST от препядствия
    odom.tick()
    if(time.time() - oldTime > 0.1):
      oldTime = time.time()
      ligth1 = brick.sensor("A6").read()
      ligth2 = brick.sensor("A5").read()
      error = (ligth2 - ligth1) * kP    # Ошибка линии
      m1_pid.target_speed = speed - error   # Полутанковая схема (только передние)
      m2_pid.target_speed = speed + error
      m3_pid.target_speed = speed 
      m4_pid.target_speed = speed 
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
  odom.delta_x = odom.x_glob     # Насколько мы сместились (для одометрии)
  odom.delta_y = odom.y_glob     # Насколько мы сместились (для одометрии)


def goToLineDistance(distance, speed = 0.4, kP = 0.0035):
  # 0.6 0.005
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d
  dist_error = 355*speed - 104    # Расчет тормозного пути
  distance -= dist_error 
  oldTime = 0
  ligth1 = brick.sensor("A6").read()    # Значение датчика 1
  ligth2 = brick.sensor("A5").read()    # Значение датчика 2
  startX = odom.x   
  odom.tick()
  while odom.x - startX < distance:
    odom.tick()
    if(time.time() - oldTime>0.01):
      oldTime = time.time()
      ligth1 = brick.sensor("A6").read()
      ligth2 = brick.sensor("A5").read()
      error = (ligth1 - ligth2) * kP     # Ошибка линии
      m1_pid.target_speed = speed - error   # Полутанковая схема (только передние)
      m2_pid.target_speed = speed + error
      m3_pid.target_speed = speed 
      m4_pid.target_speed = speed 
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
  odom.delta_x = odom.x_glob     # Насколько мы сместились (для одометрии)
  odom.delta_y = odom.y_glob     # Насколько мы сместились (для одометрии)



def goToLineTime(Time, speed = 0.4, kP = 0.0035):
  # Для speed =  0.6 kP = 0.005
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d 
  oldTime = 0
  ligth1 = brick.sensor("A6").read()    # Значение датчика 1
  ligth2 = brick.sensor("A5").read()    # Значение датчика 2
  startTime = time.time()
  while time.time() - startTime < Time:
    odom.tick()
    if(time.time() - oldTime>0.1):
      oldTime = time.time()
      ligth1 = brick.sensor("A6").read()    # Значение датчика 1
      ligth2 = brick.sensor("A5").read()    # Значение датчика 2
      error = (ligth1 - ligth2) * kP    # Ошибка линии
      m1_pid.target_speed = speed - error   # Полутанковая схема (только передние)
      m2_pid.target_speed = speed + error
      m3_pid.target_speed = speed 
      m4_pid.target_speed = speed 
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
  odom.delta_x = odom.x_glob     # Насколько мы сместились (для одометрии)
  odom.delta_y = odom.y_glob     # Насколько мы сместились (для одометрии)





def doLineUp(lineUP = 5, size = 100, speed = 0.4, kP = 0.0035):
  # Для speed =  0.6 kP = 0.005
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d
  oldTime = 0
  ligth1 = brick.sensor("A6").read()    # Значение датчика 1
  ligth2 = brick.sensor("A5").read()    # Значение датчика 2
  error = 0
  arr = 0
  for i in range(size):    # Заполняем массив для бегущего среднего
    arr[i] = size
  ptr = 0
  while round(sum(arr)/size, 4) > lineUP:    # Пока средняя ошибка больше чем LineUp 
    odom.tick()
    if(time.time() - oldTime>0.1):
      oldTime = time.time()
      ligth1 = brick.sensor("A6").read()    # Значение датчика 1
      ligth2 = brick.sensor("A5").read()    # Значение датчика 2
      error = (ligth1 - ligth2) * kP    # Ошибка линии
      arr[ptr % size] = abs(error)  
      ptr += 1
      m1_pid.target_speed = speed - error   # Полутанковая схема (только передние)
      m2_pid.target_speed = speed + error
      m3_pid.target_speed = speed 
      m4_pid.target_speed = speed 
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
  odom.delta_x = odom.x_glob     # Насколько мы сместились (для одометрии)
  odom.delta_y = odom.y_glob     # Насколько мы сместились (для одометрии)


def bump(dirY = 1, speed = 0.4): # dirY => (-1;1) ; dirX => (-1;1)
  oldTime = time.time()
  while time.time() - oldTime < 1:    # Одну секунду движемся вбок (dirY выбирает направление)
    m1_pid.target_speed(speed * dirY)
    m2_pid.target_speed(-speed * dirY)
    m3_pid.target_speed(-speed * dirY)
    m4_pid.target_speed(speed * dirY)
  oldTime = time.time()
  while time.time() - oldTime < 1:    # Одну секунду движемся назад 
    m1_pid.target_speed(-speed)
    m2_pid.target_speed(-speed)
    m3_pid.target_speed(-speed)
    m4_pid.target_speed(-speed)
  odom.x = 0        # Обнуляем одометрию
  odom.y = 0
  odom.x_glob = 0
  odom.y_glob = 0
  odom.w = 0
  
  
  
def bumpDist(mode, dir = 1, speed = 0.4):    # Mode = "X","Y"  Dir = направление (1 = ->; -1 = <-)  
  if mode == "X":   # В стену по X (сзади)
    oldTime = time.time()
    while time.time() - oldTime < 1:    # Одну секунду движемся назад 
      m1_pid.target_speed(-speed)
      m2_pid.target_speed(-speed)
      m3_pid.target_speed(-speed)
      m4_pid.target_speed(-speed)    
    odom.x = 0        # Обнуляем одометрию
    odom.y = 0
    odom.x_glob = 0
    odom.y_glob = 0
    odom.w = 0
    return brick.sensor("D2").read()    # Вывод расстояния до ближайшего обьекта (для A*)
  else:
    oldTime = time.time()
    while time.time() - oldTime < 1:    # Одну секунду движемся вбок (dir выбирает направление)
      m1_pid.target_speed(speed * dirY)
      m2_pid.target_speed(-speed * dirY)
      m3_pid.target_speed(-speed * dirY)
      m4_pid.target_speed(speed * dirY)
    odom.x = 0        # Обнуляем одометрию
    odom.y = 0
    odom.x_glob = 0
    odom.y_glob = 0
    odom.w = 0
    return brick.sensor("D1").read()# Вывод расстояния до ближайшего обьекта (для A*)


def lineFollowUntilCross(val = 50, speed = 0.4, kP = 0.0035):
  # Для speed =  0.6 kP = 0.005
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d 
  oldTime = 0
  ligth1 = brick.sensor("A6").read()    # Значение датчика 1
  ligth2 = brick.sensor("A5").read()    # Значение датчика 2
  startTime = time.time()
  while ligth1 <= val and ligth2 <= val:    # Пока не перекресток
    odom.tick()
    if(time.time() - oldTime > 0.1):
      oldTime = time.time()
      ligth1 = brick.sensor("A6").read()    # Значение датчика 1
      ligth2 = brick.sensor("A5").read()    # Значение датчика 2
      error = (ligth1 - ligth2) * kP    # Ошибка линии
      m1_pid.target_speed = speed - error   # Полутанковая схема (только передние)
      m2_pid.target_speed = speed + error
      m3_pid.target_speed = speed 
      m4_pid.target_speed = speed 
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
  odom.delta_x = odom.x_glob     # Насколько мы сместились (для одометрии)
  odom.delta_y = odom.y_glob     # Насколько мы сместились (для одометрии)

goToLineDistance(500, 0.6, 0.005) 
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#-------TODO:----------------------------------------------------------------------------------------------------------------------------------------------------------------
  

def straightTime():
  pass
def straightDist():
  pass
def straightSensor():
  pass






