"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np
import math

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


def euler_from_quaternion(x, y, z ,w):
    t0 = +2.0*(w*x+y*z)
    t1 = +1.0-2.0*(x*x+y*y)
    roll_x = math.degrees(math.atan2(t0,t1))

    t2 = +2.0*(w*y-z*x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.degrees(math.asin(t2))

    t3 = +2.0*(w*z+x*y)
    t4 = +1.0-2.0*(y*y+z*z)
    yaw_z = math.degrees(math.atan2(t3,t4))

    return np.array([roll_x, pitch_y, yaw_z])

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    while(True):
    	position = cf.position()
    	angles = euler_from_quaternion(position[3], position[4], position[5], position[6])
    	print(angles)
    	timeHelper.sleep(2)

if __name__ == "__main__":
    main()
