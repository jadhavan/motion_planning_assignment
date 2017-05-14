#!/usr/bin/env python
import time
import openravepy
from pathfinder import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    robot = env.GetRobots()[0]
    tuckarms(env,robot);
    raw_input("Press enter to start A* path planner...")
    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]
        #### YOUR CODE HERE ####

        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        heuristic = 'manhattan'
        connected = 4

        heuristic = 'euclidiean'
        connected = 8
        pathPlanner = astar(env,robot,goalconfig,heuristic,connected)
        path = pathPlanner.rebuildPath()
        raw_input("Press enter to execute trajectory...")

        traj = RaveCreateTrajectory(env, '')

        # get config spec for trajectory, choose linear interpolation method, add timestamp to config spec
        config = robot.GetActiveConfigurationSpecification('linear')
        config.AddDeltaTimeGroup()

        # initialize the trajectory
        traj.Init(config)

        # create C-space path with time stamps from path curve
        # set delta time to be 0.003
        finalPath = [[point[0], point[1], -pi/2, i*0.03] for i, point in enumerate(path)]

        # fill trajectory with waypoints
        for i, p in enumerate(finalPath):
           traj.Insert(i, p, config, True)

        robot.GetController().SetPath(traj)
        print robot.GetTransform()

        # print pathPlanner.path
        # robot.GetController().SetPath(pathPlanner.path)
    waitrobot(robot)
    print robot.GetTransform()


    raw_input("Press enter to exit...")
