#!/usr/bin/env python
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
lines = [];
def strtolist(path):
    path = path.split('\n')
    path = path[0:(len(path)-1)]
    for i in range(len(path)):
        path[i] = path[i].split(',')
        del path[i][-1]
        for j in range(len(path[i])):
            path[i][j]= float(path[i][j])
    return path
def drawLine(path,env,shortPath):
    robot = env.GetRobots()[0]
    line = zeros((1,3))
    if shortPath:
        color = array(((0,0,1)))
    else:
        color = array(((1,0,0)))
    for joint_val in path:
        robot.SetActiveDOFValues(joint_val)
        line = vstack((line,robot.GetLinks()[49].GetTransform()[0:3,3]))
    lines.append(env.drawlinestrip(points=line[1:,:],linewidth=3,colors=color))
#### END OF YOUR IMPORTS ####
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
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('build/rrt_plugin')
    rrt_module = RaveCreateModule(env,'rrt_module')
    ### END INITIALIZING YOUR PLUGIN ###


    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    #set start config
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
    startconfig = [-0.15,0.075,-1.008,0,0,-0.11,0]
    startconfig = [-0.15,0.075,-1.008,-0.11,0,-0.11,0] #changing joint 4 to max limits instead of out of joint limits
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    Stepsize = 0.2;
    goalbias = 0.16;
    with env:
        goalconfig = [0.449,-0.201,-0.151,0,0,-0.11,0]
        goalconfig = [0.449,-0.201,-0.151,-0.11,0,-0.11,0] #changing joint 4 to max limits instead of out of joint limits

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig

        append = startconfig + goalconfig+ [Stepsize, goalbias];
        start = time.time()
        path = rrt_module.SendCommand('rrtRun start %.3f %.3f %.3f %.3f %.3f %.3f %.3f goal %.3f %.3f %.3f %.3f %.3f %.3f %.3f Stepsize %.2f goalbias %0.2f'%tuple(append));
        end = time.time()
        path = strtolist(path)
        drawLine(path,env,False)
        Sstart = time.time()
        path = rrt_module.SendCommand('pathSmooth iter 200');
        Send = time.time()
        path = strtolist(path)
        drawLine(path,env,True)
        print '\ngoalbias, Stepsize',goalbias,Stepsize
        print "RRT : ",end-start," seconds"
        print "Smooth : ",Send-Sstart," seconds"
        robot.SetActiveDOFValues(startconfig);
        traj = RaveCreateTrajectory(env, '')

        # get config spec for trajectory, choose linear interpolation method, add timestamp to config spec
        config = robot.GetActiveConfigurationSpecification('linear')
        config.AddDeltaTimeGroup()

        # initialize the trajectory
        traj.Init(config)

        for i in range(len(path)):
            path[i] = path[i] + [i*0.001]
        # fill trajectory with waypoints
        for i, p in enumerate(path):
           traj.Insert(i, p, config, True)

        robot.GetController().SetPath(traj)
        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("\nPress enter to exit...")
