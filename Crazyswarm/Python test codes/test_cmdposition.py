from pycrazyswarm import *
import numpy as np

swarm = Crazyswarm()
timeHelper = swarm.timeHelper
cf = swarm.allcfs.crazyflies[0]


initial_position = cf.position()
Z = initial_position[2]
# takeoff
while Z < 1.0:
    pos = initial_position + np.array([0, 0, Z])
    cf.cmdPosition(pos, yaw = 0)
    timeHelper.sleep(0.1)
    Z += 0.05

# land
while Z > (initial_position[2]):
    pos = initial_position + np.array([0, 0, Z])
    cf.cmdPosition(pos, yaw = 0)
    timeHelper.sleep(0.1)
    Z -= 0.05

# turn-off motors
cf.cmdStop()
