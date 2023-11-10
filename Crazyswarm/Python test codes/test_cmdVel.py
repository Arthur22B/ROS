from pycrazyswarm import Crazyswarm
import numpy as np


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]


    startTime = timeHelper.time()
    time = timeHelper.time() - startTime

    while (time < 1):
        time = timeHelper.time() - startTime
        cf.cmdVel(0, 0, 0, 0)
        timeHelper.sleepForRate(20)

    initial_position = cf.position()
    Z = initial_position[2]
    # takeoff
    while Z < 0.5:
        pos = initial_position + np.array([0, 0, Z])
        cf.cmdPosition(pos, yaw = 0)
        timeHelper.sleep(0.1)
        Z += 0.01

    startTime = timeHelper.time()
    time = timeHelper.time() - startTime

    startPos  = cf.position()

    print("a")
    while (time < 1):
        time = timeHelper.time() - startTime
        cf.cmdPosition(startPos)
        timeHelper.sleepForRate(20)

    startTime = timeHelper.time()
    time = timeHelper.time() - startTime

    print("b")

    while (time < 1):
        time = timeHelper.time() - startTime
        cf.cmdVel(0, 0, 0, 45500)
        timeHelper.sleepForRate(20)

if __name__ == "__main__":
    main()
