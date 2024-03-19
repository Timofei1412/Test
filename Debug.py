col = 20
def print(s, val):
  global col
  brick.display().addLabel(s + " " + str(round(val, 2)), 1, col)
  col += 20
  
def odoDebug():
    global odom, col
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
        print("x_proj", odom.x_proj)
        print("y_proj", odom.y_proj)
        print("w", odom.w)
        brick.display().redraw()
        col = 0