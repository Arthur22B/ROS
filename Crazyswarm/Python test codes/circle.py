#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *


Z = 0.5
sleepRate = 20


def goCircle(timeHelper, cf, totalTime, radius):
        startPos  = cf.position()
        center_circle = startPos - np.array([radius, 0, 0])
        startTime = timeHelper.time()
        time = timeHelper.time() - startTime
        while (time < 4):
            time = timeHelper.time() - startTime
            omega = 2 * np.pi / totalTime
            desiredPos = center_circle + radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            cf.cmdPosition(desiredPos)
            timeHelper.sleepForRate(sleepRate)

        stopPos = cf.position()
        z_land = stopPos[2]

        while (z_land > 0.10):
          cf.cmdPosition(np.array([stopPos[0], stopPos[1], z_land]))
          z_land -= 0.01
          timeHelper.sleepForRate(sleepRate)

        cf.notifySetpointsStop(remainValidMillisecs = 100)

        cf.land(targetHeight=0.04, duration=1)
        timeHelper.sleep(1)  

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, cf, totalTime=4, radius=0.5) 