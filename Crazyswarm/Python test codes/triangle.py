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
    timeHelper.sleep(TAKEOFF_DURATION)
    position_initial = cf.position()
    initial = np.array([position_initial[0], position_initial[1], position_initial[2]])
    goal = initial + np.array([0.5, 0.25, 0])
    cf.goTo(goal, yaw = 0.0, duration = TAKEOFF_DURATION)
    timeHelper.sleep(HOVER_DURATION)
    goal = initial + np.array([0.5, -0.25, 0])
    cf.goTo(goal, yaw = 0.0, duration = TAKEOFF_DURATION)
    timeHelper.sleep(HOVER_DURATION)
    goal = initial
    cf.goTo(goal, yaw = 0.0, duration = TAKEOFF_DURATION)
    timeHelper.sleep(HOVER_DURATION)

    cf.land(targetHeight=0.04, duration=2.5)


if __name__ == "__main__":
    main()
