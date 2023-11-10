"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    position = cf.position()
    goal = np.array([position[0]+0.5, position[1]-0.3, position[2]+0.5])
    print(goal)
    cf.goTo(goal, yaw = 0.0, duration = TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    print(cf.position())
    cf.land(targetHeight=0.04, duration=2.5)


if __name__ == "__main__":
    main()
