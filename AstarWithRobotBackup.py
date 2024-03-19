import sys
import time
import random
from math import *
import heapq 


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

base_speed_x = 1.7
base_speed_y = 2.1

calculation_timer = 0
odometry_timer = 0

ticks_to_mm = (pi*100)/(12*45)

display_timer = 0

delta_y = 0
delta_x = 0

brake_x = 100
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
    self.x = 5000
    self.y = 5000
    self.x_glob = 0
    self.y_glob = 0
    self.x_proj = 0
    self.y_proj = 0
    self.w = 0
    self.w_loc = 2 * pi
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
m1_pid = Motor_pid(a, brick.motor(M1).setPower, 60, -1, 165, -17)

b = brick.encoder(E2)
m2_pid = Motor_pid(b, brick.motor(M2).setPower, 60, -1, 110, -17)

c = brick.encoder(E3)
m3_pid = Motor_pid(c, brick.motor(M3).setPower, 60, -1, 165, -17)

d = brick.encoder(E4)
m4_pid = Motor_pid(d, brick.motor(M4).setPower, 60, -1, 110, -17)

a.reset()
b.reset()
c.reset()
d.reset()

odom = Odometry(a, b, c, d)

s1 = brick.motor("S1").setPower
s2 = brick.motor("S2").setPower

def move_forward_dist(distance):
  distance_old = distance
  distance -= brake_x*sign(distance) 
  global delta_x
  global m1_pid, m2_pid, m3_pid, m4_pid, odom
  delay_time = time.time()
  m1_pid.last_speed = 0
  m2_pid.last_speed = 0  
  m3_pid.last_speed = 0  
  m4_pid.last_speed = 0
  last_x = odom.x
  last_y = odom.y
  last_w = (odom.w//((pi/2)*sign_2(odom.w)))*(pi/2)
  local_timer = time.time()
  
  target_x = (odom.x_proj) + distance
  target_y = (odom.y_glob - delta_y)
  
  while (1):
    odom.tick()
    if time.time() - local_timer > 0.1:
      local_timer = time.time()
      if (abs(odom.x_glob-delta_x) < abs(distance)):
        vx = base_speed_x * sign((distance+delta_x) - odom.x_glob)
        #print("vx", vx)
        #brick.display().redraw()
        #col = 0
        vy = (odom.y_glob - delta_y) / 170
        vy*=-1
        vw = -(odom.w-last_w) * base_speed_x * 10
        if(time.time() - delay_time <0.5):
          vy = 0
          vw = 0
        
        if vy > base_speed_y*0.7:
          vy = base_speed_y*0.7
        if vy < -base_speed_y*0.7:
          vy = -base_speed_y*0.7
        
        if vw > base_speed_x*0.7:
          vw = base_speed_x*0.7
        if vw < -base_speed_x*0.7:
          vw = -base_speed_x*0.7
        
        w1 = (vx + vy - vw)
        w2 = (vx - vy + vw)
        w3 = (vx - vy - vw)
        w4 = (vx + vy + vw)
        
        m1_pid.target_speed = w1
        m2_pid.target_speed = w2
        m3_pid.target_speed = w3
        m4_pid.target_speed = w4
      else:
        m1_pid.target_speed = 0
        m2_pid.target_speed = 0
        m3_pid.target_speed = 0
        m4_pid.target_speed = 0
        m1_pid.motor_pid()
        m2_pid.motor_pid()
        m3_pid.motor_pid()
        m4_pid.motor_pid()
        delta_x += distance_old
        
        return
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()


def move_sideways_dist(distance):
  distance_old = distance
  distance -= brake_y*sign(distance)
  global delta, col, delta_y
  global m1_pid, m2_pid, m3_pid, m4_pid, odom
  global display_timer
  
  delay_time = time.time()
  last_x = odom.x
  last_y = odom.y_proj
  last_w = (odom.w//((pi/2)*sign_2(odom.w)))*(pi/2)
  local_timer = time.time()
  
  target_x = (odom.x_proj) + distance
  target_y = (odom.y_proj) + distance
  
  m1_pid.last_speed = 0
  m2_pid.last_speed = 0  
  m3_pid.last_speed = 0  
  m4_pid.last_speed = 0  
  while (1):
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
    
    odom.tick()
    if time.time() - local_timer > 0.1:
      local_timer = time.time()
      if (abs(odom.y_glob-delta_y) < abs(distance)):
        vy = base_speed_y * sign((distance+delta_y) - odom.y_glob)
        vx = (odom.x_glob - delta_x)/150  
        vx*=-1
        vw = -(odom.w-last_w)*base_speed_x*15
        if(time.time() - delay_time <0.5):
          vy = 0
          vw = 0
        
        if vx > base_speed_x*0.7:
          vx = base_speed_x*0.7
        if vx < -base_speed_x*0.7:
          vx = -base_speed_x*0.7
         
        if vw > base_speed_y*0.5:
          vw = base_speed_y*0.5
        if vw < -base_speed_y*0.5:
          vw = -base_speed_y*0.5
        w1 = (vx + vy - vw)
        w2 = (vx - vy + vw)
        w3 = (vx - vy - vw)
        w4 = (vx + vy + vw)
        
        m1_pid.target_speed = w1
        m2_pid.target_speed = w2
        m3_pid.target_speed = w3
        m4_pid.target_speed = w4
      else:
        m1_pid.target_speed = 0
        m2_pid.target_speed = 0
        m3_pid.target_speed = 0
        m4_pid.target_speed = 0
        m1_pid.motor_pid()
        m2_pid.motor_pid()
        m3_pid.motor_pid()
        m4_pid.motor_pid()
        delta_y += distance_old
        odom.w += 0.09*sign(distance)*-1
        return
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()

def rotate_angle(rad):
  global m1_pid, m2_pid, m3_pid, m4_pid, odom
  last_w = (odom.w//((pi/2)*sign_2(odom.w)))*(pi/2)
  local_timer = time.time()
  while (1):
    odom.tick()
    if time.time() - local_timer > 0.1:
      local_timer = time.time()
      if (abs(odom.w - last_w) < abs(rad)):
        vw = ((last_w+rad)-odom.w)*10
        if vw > base_speed:
          vw = base_speed
        if vw < base_speed:
          vw = -base_speed
        vx = 0
        vy = 0
        w1 = (vx + vy - vw)
        w2 = (vx - vy + vw)
        w3 = (vx - vy - vw)
        w4 = (vx + vy + vw)
        
        m1_pid.target_speed = w1
        m2_pid.target_speed = w2
        m3_pid.target_speed = w3
        m4_pid.target_speed = w4
      else:
        m1_pid.target_speed = 0
        m2_pid.target_speed = 0
        m3_pid.target_speed = 0
        m4_pid.target_speed = 0
        m1_pid.motor_pid()
        m2_pid.motor_pid()
        m3_pid.motor_pid()
        m4_pid.motor_pid()
        return
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()

#----------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
class Astar: 
    def __init__(self, map_size, pad: int, move_x: float, move_y: float, robot_coords: list): 
        self.x, self.y = map_size 
        self.pad = pad 
        self.move_x, self.move_y = move_x, move_y 
        self.current_coords = tuple(robot_coords) 
 
        self.map_obs = [[1] * self.x for _ in range(self.y)]
#        w, h = self.map_obs.shape[:2] 
#        self.map_obs[pad:w-pad][pad:h-pad] = 0
        for i in range(pad, len(self.map_obs)-pad+1):
          for j in range(pad, len(self.map_obs[0])-pad+1):
              self.map_obs[i][j] = 0
        
 
    def create_obs(self, x, y): 
        pad = self.pad 
        self.map_obs[x][y] = 1 
        for i in range(x-pad, x+pad):
          for j in range(y-pad, y+pad):
            self.map_obs[i][j] = 1
        # self.map_obs[x-pad, y-pad] = 0 
        # self.map_obs[x-pad, y+pad] = 0 
        # self.map_obs[x+pad, y-pad] = 0 
        # self.map_obs[x+pad, y+pad] = 0s 
     
    def manhattan_distance(self, point1, point2): 
        return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1]) 
 
    def astar(self, goal): 
        grid = self.map_obs 
        start = self.current_coords 
        cost = {start: 0} 
        came_from = {start: None} 
 
        priority_queue = [(0, start)] 
 
        while priority_queue: 
            current_cost, current_point = heapq.heappop(priority_queue) 
 
            if current_point == goal: 
                break 
 
            for neighbor in [(1, 0), (-1, 0), (0, 1), (0, -1)]: 
                x, y = current_point[0] + neighbor[0], current_point[1] + neighbor[1] 
 
 
                new_cost = cost[current_point] + 1 
                if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0: 
                    if (x, y) not in cost or new_cost < cost[(x, y)]: 
                        cost[(x, y)] = new_cost 
                        priority = new_cost + self.manhattan_distance((x, y), goal) 
                        heapq.heappush(priority_queue, (priority, (x, y))) 
                        came_from[(x, y)] = current_point 
        path = [] 
        current = goal 
        while current: 
            path.append(current) 
            current = came_from[current] 
        path.reverse() 
        self.current_coords = tuple(path[-1]) 
        path = self.optimize_way(path) 
        return path 
     
    def optimize_way(self, path): 
        new_path = [path[0]] 
        for i in range(1, len(path) - 1): 
            if new_path[-1][0] == path[i][0] and new_path[-1][0] != path[i+1][0]: 
                new_path.append(path[i]) 
            if new_path[-1][1] == path[i][1] and new_path[-1][1] != path[i+1][1]: 
                new_path.append(path[i]) 
        new_path.append(path[-1]) 
        return new_path 
 
    def to_commad(self, way): 
        command_list = [] 
        for i in range(len(way) - 1): 
            if way[i][0] > way[i + 1][0]: 
                command_list.append(["left", (way[i][0] - way[i + 1][0]) * self.move_y]) 
            if way[i][0] < way[i + 1][0]:
                command_list.append(["right", (way[i + 1][0] - way[i][0]) * self.move_y]) 
            if way[i][1] > way[i + 1][1]: 
                command_list.append(["backward", (way[i][1] - way[i + 1][1]) * self.move_x]) 
            if way[i][1] < way[i + 1][1]: 
                command_list.append(["forward", (way[i + 1][1] - way[i][1]) * self.move_x]) 
        return command_list 
 
    def create_obs_lst(self, x1, y1, x2, y2): 
        obs_list = [[[x1 + i, y1] for i in range(x2-x1+1)], 
            [[x1, y1 + i]  for i in range(y2-y1+1)], 
            [[x1 + i, y2] for i in range(x2-x1+1)], 
            [[x2, y1 + i] for i in range(y2-y1+1)]] 
        for i in obs_list: 
            for point in i: 
                self.create_obs(point[0], point[1])      
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------

def goToLineDistance(distance, speed = 0.4, kP = 0.0035):
  global m1_pid, m2_pid, m3_pid, m4_pid, a, b, c, d
  dist_error = 355*speed - 104
  distance -= dist_error 
  oldTime = 0
  ligth1 = brick.sensor("A6").read()
  ligth2 = brick.sensor("A5").read()
  startX = odom.x
  odom.tick()
  brick.display().addLabel(odom.x,1,10)
  brick.display().redraw()
  while odom.x - startX < distance:
    odom.tick()
    if(time.time() - oldTime>0.01):
      oldTime = time.time()
      ligth1 = brick.sensor("A6").read()
      ligth2 = brick.sensor("A5").read()
      error = (ligth1 - ligth2) * kP
      brick.display().addLabel("L1 = " + str(ligth1),1,10)
      brick.display().addLabel("L2 = " + str(ligth2),1,20)
      brick.display().addLabel("error = " + str(error),1,30)
      brick.display().redraw()
      m1_pid.target_speed = speed - error
      m2_pid.target_speed = speed + error
      m3_pid.target_speed = speed 
      m4_pid.target_speed = speed 
      m1_pid.motor_pid()
      m2_pid.motor_pid()
      m3_pid.motor_pid()
      m4_pid.motor_pid()
      
      
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------
  
#-----------------------------------------------------------------------------------------------------------------------------------------------------------------
ast = Astar([40, 24], 2, 60, 50, [21, 2]) 
#
#print("fsdfds", 1210)
#brick.display().redraw()
#col=0

obs_list = [[11, 5, 21, 14]] 
for i in obs_list: 
    ast.create_obs_lst(i[0], i[1], i[2], i[3]) 

way = ast.astar((19, 21))
way = ast.to_commad(way)
for command in way:
    if(command[0] == "forward"):
        move_forward_dist(command[1])
        script.wait(500)
    elif(command[0] == "backward"):
        move_forward_dist(-command[1])
        script.wait(500)
    elif(command[0] == "right"):
        move_sideways_dist(command[1])
        script.wait(500)
    elif(command[0] == "left"):
        move_sideways_dist(-command[1])
        script.wait(500)
        
        
goToLineDistance(500, 0.6, 0.005) 
