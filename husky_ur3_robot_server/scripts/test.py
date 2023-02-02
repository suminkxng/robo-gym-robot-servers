#!/usr/bin/env python3

import rospy
import copy
from leg_tracker.msg import LegArray, Leg, Person, PersonArray
import PyKDL
from std_msgs.msg import Int32, Float64
import numpy as np
from geometry_msgs.msg import Twist, Pose, Pose2D, PoseStamped
from pedsim_msgs.msg  import AgentStates


def callbackState(data):
    x = data.position.x
    y = data.position.y

    orientation = PyKDL.Rotation.Quaternion(data.orientation.x,
                                            data.orientation.y,
                                            data.orientation.z,
                                            data.orientation.w)

    euler_orientation = orientation.GetRPY()
    yaw = euler_orientation[2]

    # Update internal Pose variable
    husky_pose = copy.deepcopy([x, y, yaw])
    husky_coords = np.array([husky_pose[0], husky_pose[1]])

    print("husky_pose: ", husky_coords)
    # print("husky_pose_type: ", type(husky_pose))

    return husky_coords


def update_humans(msg):
    humans = list()
    ob = list()
    for p in msg.agent_states:
        o = p.pose.orientation
        orientation = PyKDL.Rotation.Quaternion(o.x, o.y, o.z, o.w)
        euler_orientation = orientation.GetRPY()
        yaw = euler_orientation[2]

        human = (p.pose.position.x, p.pose.position.y)
        # human = (p.pose.position.x, p.pose.position.y, yaw)
        humans.append(human)
        print("first: ", human)
        print("second: ", humans)
        print("second_length: ", len(humans))
        print("=====================")
    # for i, hum in enumerate(human):
    #     human_pose = ob.append(hum)
    #     print("human_pose: ", human_pose)
    # self.human_pose = copy.deepcopy(self.ob)
    # # return self.human_pose
    # print("human_pose: ", self.human_pose)
    # if len(humans) >= 4:

    if len(humans) >= 4:
        human_pose_0 = copy.deepcopy([humans[0][0], humans[0][1]])
        human_pose_1 = copy.deepcopy([humans[1][0], humans[1][1]])
        human_pose_2 = copy.deepcopy([humans[2][0], humans[2][1]])
        human_pose_3 = copy.deepcopy([humans[3][0], humans[3][1]])
        print("human_pose_0: ", human_pose_0)
        print("human_pose_1: ", human_pose_1)
        print("human_pose_2: ", human_pose_2)
        print("human_pose_3: ", human_pose_3)
        # human_pose = copy.deepcopy([humans[0][0], humans[0][1], humans[1][0], humans[1][1], humans[2][0], humans[2][1], humans[3][0], humans[3][1]])        
        human_pose = copy.deepcopy([human_pose_0, human_pose_1, human_pose_2, human_pose_3])        

        print("human_pose: ", human_pose, len(human_pose))
        print("human_pose[0]: ", human_pose[0])

    # for i, human in enumerate(humans):
    #     human_coords = np.array([human[0], human[1]])
    #     human_pose = copy.deepcopy(human_coords)
    #     # human_coords.append(human.get_observable_state())
    #     print("classss: ", human_pose)

    return human_pose


def distance(a, b):
    dis = np.linalg.norm(a - b, axis=-1)
    return dis
        

def main():
    rospy.init_node('test file', anonymous=True)
    # rospy.Subscriber('people_tracked', PersonArray, update_humans)
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, update_humans)
    rospy.Subscriber('robot_pose', Pose, callbackState)

    # human_pose = update_humans
    # husky_pose = callbackState
    # dis = distance(human_pose, husky_pose)
    # print("distance: ", dis)
    
    rospy.spin()


if __name__ == '__main__':
    main()