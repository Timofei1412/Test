import sys
import time
import random
from math import*

pi = 3.14159265
brick.gyroscope().calibrate(6000)
script.wait(6100)
brick.gyroscope().setCalibrationValues(brick.gyroscope().getCalibrationValues())

arr = brick.gyroscope().read()
# Милиградусы!!!

x = arr[4] # Крен
y = arr[5] # Тангаж
z = arr[6] # Рысканье
brick.display().addLabel(x/1000, 1, 20)
brick.display().redraw()
brick.display().addLabel(y/1000, 1, 50)
brick.display().redraw()
brick.display().addLabel(z/1000, 1, 100)
brick.display().redraw()
script.wait(100)
def getYaw():
  brick.display().addLabel(brick.gyroscope().read()[6]/1000, 1, 20) 
  brick.display().redraw()
  return
  
while True:
  z = brick.gyroscope().read()[6]/200
  brick.motor(M1).setPower(50-z)
  brick.motor(M3).setPower(50-z)
  brick.motor(M2).setPower(50+z)
  brick.motor(M4).setPower(50+z)
  script.wait(100)
   