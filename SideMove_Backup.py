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