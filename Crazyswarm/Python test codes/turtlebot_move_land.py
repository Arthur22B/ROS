from pycrazyswarm import Crazyswarm
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

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

    sub = rospy.Subscriber('/vrpn_client_node/TURTLEBOT2/pose', PoseStamped, callback = turtle_pose_callback)


    cf.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION+HOVER_DURATION)

    position_initial = cf.position()

    pos_x = turtle_pose.pose.position.x
    pos_y = turtle_pose.pose.position.y

    inital_time = timeHelper.time()

    pos_x_bf = 1000
    pos_y_bf = 1000

    pos_x = turtle_pose.pose.position.x
    pos_y = turtle_pose.pose.position.y

    timeHelper.sleep(1)

    while(math.sqrt((pos_x-pos_x_bf)**2+(pos_y-pos_y_bf)**2) > 0.0001):
        pos_x_bf = pos_x
        pos_y_bf = pos_y
        pos_x = turtle_pose.pose.position.x
        pos_y = turtle_pose.pose.position.y
        goal = np.array([pos_x, pos_y, position_initial[2]])
        cf.cmdPosition(goal)
        timeHelper.sleepForRate(10)

    pos_x = turtle_pose.pose.position.x
    pos_y = turtle_pose.pose.position.y
    #q_x = turtle_pose.pose.orientation.x
    #q_y = turtle_pose.pose.orientation.y
    #q_z = turtle_pose.pose.orientation.z
    #q_w = turtle_pose.pose.orientation.w
    #t3 = 2.0*(q_w*q_z+q_x*q_y) 
    #t4 = 1.0-2.0*(q_y*q_y+q_z*q_z)
    #yaw_z = math.atan2(t3,t4)

    cf.notifySetpointsStop(remainValidMillisecs = 100)

    goal = np.array([pos_x, pos_y, position_initial[2]])
    cf.goTo(goal, yaw = 0.0, duration= 1.0)
    timeHelper.sleep(1.0)

    #goal = np.array([position_initial[0], position_initial[1], position_initial[2]])
    #cf.goTo(goal, yaw = 0.0, duration = TAKEOFF_DURATION)
    #timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)

    cf.land(targetHeight=0.16, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)
  # /vrpn_client_node/TURTLEBOT2/pose
  # geometry_msgs/PoseStamped


if __name__ == "__main__":

    main()