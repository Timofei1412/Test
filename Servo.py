s1 = brick.motor("S1").setPower
s2 = brick.motor("S2").setPower


def armSetup():
  s1(-40)
  s2(-40)


def armGetCube():
  s2(-40)
  script.wait(1000)


def armPutCube():
  s2(90)
  script.wait(1000)


def armToBottom():
  s2(90)
  script.wait(1000)
  s1(-50)
  script.wait(1000)


def armToTop():
  s2(90)
  script.wait(1000)
  s1(-50)
  script.wait(1000)
 

def armHome():
  s1(-40)
  s2(-40)

