from pycrazyswarm import Crazyswarm
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 2.0 

turtle_pose = PoseStamped()

def turtle_pose_callback(msg):
    global turtle_pose
    turtle_pose = msg

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    inital_time = timeHelper.time()

    sub = rospy.Subscriber('/vrpn_client_node/TURTLEBOT2/pose', PoseStamped, callback = turtle_pose_callback)

    while(timeHelper.time()-inital_time < 0.2):
        print('x= ', turtle_pose.pose.position.x, 'y= ', turtle_pose.pose.position.y)
        timeHelper.sleepForRate(10)

    pos_x = turtle_pose.pose.position.x
    pos_y = turtle_pose.pose.position.y

    print('x= ', pos_x, 'y= ', pos_y)

    cf.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION+HOVER_DURATION)
    position_initial = cf.position()
    goal = np.array([pos_x, pos_y, position_initial[2]])
    cf.goTo(goal, yaw = 0.0, duration = TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    goal = np.array([position_initial[0], position_initial[1], position_initial[2]])
    cf.goTo(goal, yaw = 0.0, duration = TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    cf.land(targetHeight=0.04, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
  # /vrpn_client_node/TURTLEBOT2/pose
  # geometry_msgs/PoseStamped


if __name__ == "__main__":

    main()