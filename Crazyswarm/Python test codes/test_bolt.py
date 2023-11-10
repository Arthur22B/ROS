from pycrazyswarm import Crazyswarm
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[1]

    startTime = timeHelper.time()
    time = timeHelper.time() - startTime

    while (time < 1):
        time = timeHelper.time() - startTime
        cf.cmdVel(0, 0, 0, 0)
        timeHelper.sleepForRate(20)

    startTime = timeHelper.time()
    time = timeHelper.time() - startTime

    while (time < 3):
        time = timeHelper.time() - startTime
        cf.cmdVel(0, 0, 0, 5000)
        timeHelper.sleepForRate(20)

    while (time < 5):
        time = timeHelper.time() - startTime
        cf.cmdVel(0, 0, 0, 10000)
        timeHelper.sleepForRate(20)

    while (time < 7):
        time = timeHelper.time() - startTime
        cf.cmdVel(0, 0, 0, 15000)
        timeHelper.sleepForRate(20)

    while (time < 9):
        time = timeHelper.time() - startTime
        cf.cmdVel(0, 0, 0, 20000)
        timeHelper.sleepForRate(20)

if __name__ == "__main__":
    main()
