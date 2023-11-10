from pycrazyswarm import Crazyswarm
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    startTime = timeHelper.time()
    time = timeHelper.time() - startTime

    while (time < 1):
        time = timeHelper.time() - startTime
        cf.cmdVelocityWorld(np.array([0.0, 0.0, 0.0]), yawRate=0)
        timeHelper.sleepForRate(20)

    timeHelper.sleep(1)

    while (time < 2):
        print(time)
        time = timeHelper.time() - startTime
        cf.cmdVelocityWorld(np.array([0, 0, 0.5]), yawRate=0)
        timeHelper.sleepForRate(20)

if __name__ == "__main__":
    main()
