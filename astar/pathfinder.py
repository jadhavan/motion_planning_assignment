#!/usr/bin/env python
from Queue import PriorityQueue as pq
from numpy import *
import time
class Node(object):
    def __init__(self, robotConfig,parent,costFn,heuFn):
        self.description = robotConfig
        self.parentNode = parent
        self.cost = costFn
        self.heu = heuFn
        self.priority = heuFn + costFn
        # print 'New config:', robotConfig
        # print 'evalFn:', self.priority

    def __cmp__(self, other):
        # node with least value (here priority is a evaluation fn F(n))
        # will be on the top of the priority queue
        # F(n) = h(n) + g(n)
        # where, h(n) = heuristic fn, estimated cost from current node to and g(n) = cost fn from start node to current node
        return cmp(self.priority, other.priority)

    def __hash__(self):
        data_str = ''
        for i in range(0,len(self.description)):
            if i ==0:
                data_str = str(self.description[i])
                continue
            data_str = data_str+','+ str(self.description[i])
        # print "in hash ", data_str, "priority: ",self.priority
        return hash(data_str)


class astar:
    def __init__(self,env,robot,goal,heuristicType,noNeighors=4):
        # defining environment,robot and start ans goal configurations
        # reference: goalconfig = [2.6,-1.3,-pi/2] and startconfig = [-3.4,-1.4,0]
        self.env = env
        self.robot = robot
        self.goal = goal
        self.start = robot.GetTransform()[:2,3].tolist()
        self.start.append(0)

        self.robot.SetActiveDOFValues(self.start)

        # self.start.append(goal[2])
        self.startTransform = robot.GetTransform()
        # defining step size i.e, minimum dist robot can translate and rotate
        # fronter is defined.
        self.stepSize = 0.1
        self.angleStep = -pi/4
        # openlist is frontier and closedlist is explored
        self.frontier = pq(maxsize=0)
        self.explored = {}
        self.stepCost = 0.1
        self.noNeighors = noNeighors
        self.heuristicType = heuristicType

        self.cost_so_far = {}

        # defining start node
        self.startNode = Node(self.start,None,0,self._h(self.start));

        self.frontier.put(self.startNode)

        self.cost_so_far = {self.startNode:0}

        self.came_from = {self.startNode:None}

        self.algorithm()

        self.robot.SetActiveDOFValues(self.start)

    def euclidiean(self,currentNode):
        return ( ( ((self.goal[0]-currentNode[0])/self.stepSize)**2) + (abs((self.goal[1]-currentNode[1])/self.stepSize)**2) )**(0.5)
    def _h(self,currentNode):
        # input is list
        if self.heuristicType == 'manhattan':
            # manhattan:
            # heuristicCost =  ((abs(self.goal[0]-currentNode[0]) + abs(self.goal[1]-currentNode[1]))/self.stepSize)+ (abs(self.goal[2]-currentNode[2])/self.angleStep)
            heuristicCost =  ((abs(self.goal[0]-currentNode[0]) + abs(self.goal[1]-currentNode[1])))+ (abs(self.goal[2]-currentNode[2]))
        else:
            # euclidiean
            heuristicCost =  ( ( ((self.goal[0]-currentNode[0]))**2) + (abs((self.goal[1]-currentNode[1]))**2) + (abs((self.goal[2]-currentNode[2]))**2) )**(0.5)
        # print "h(n) = ", heuristicCost
        return heuristicCost

    def _g(self,currentNode):
        # input is object of Node
        actualCost = self.cost_so_far[currentNode]
        return actualCost

    def neighborsFour(self,currentNode):
        # input is object of Node
        currentNode = currentNode.description
        n = []
        n.append([currentNode[0],currentNode[1]+self.stepSize,currentNode[2]])
        n.append([currentNode[0]+self.stepSize,currentNode[1],currentNode[2]])
        n.append([currentNode[0],currentNode[1],(currentNode[2]+self.angleStep)%(2*pi)])
        n.append([currentNode[0],currentNode[1]-self.stepSize,currentNode[2]])
        n.append([currentNode[0]-self.stepSize,currentNode[1],currentNode[2]])
        n.append([currentNode[0],currentNode[1],(currentNode[2]-self.angleStep)%(2*pi)])
        return n

    def neighborsEight(self,currentNode):
        # input is object of Node
        currentNode = currentNode.description
        n = []
        for i in range(-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if (i==j==k==0):
                        continue
                    n.append([currentNode[0]+(i*self.stepSize),currentNode[1]+(j*self.stepSize), (currentNode[2]+(k*self.angleStep))%(2*pi)])
        return n

    def drawPoints(self,node,path=0):
        if path:
            for nde in node:
                self.handles.append(self.env.plot3(points=array(((nde[0],nde[1],0.2))),pointsize=0.05,colors=array(((0,0,0))),drawstyle=1))
        else:
            self.robot.SetActiveDOFValues(node)
            # self.robot.GetController().SetDesired(robot.GetDOFValues());
            collision = self.env.CheckCollision(self.robot)
            if collision:
                self.handles.append(self.env.plot3(points=array(((node[0],node[1],0.1))),pointsize=0.05,colors=array(((1,0,0))),drawstyle=1))
            else:
                self.handles.append(self.env.plot3(points=array(((node[0],node[1],0.15))),pointsize=0.05,colors=array(((0,0,1))),drawstyle=1))
            return collision

    def stepCostfn(self,node,currentNode):
        currentNode = currentNode.description
        return self.euclidiean(node)
        if self.noNeighors ==4:
            if round(node[0],1) == round(currentNode[0],1) and round(node[1],1) == round(currentNode[1],1):
                return 0.0
            return self.stepCost
        if self.noNeighors ==8:
            if round(node[0],1) == round(currentNode[0],1) and round(node[1],1) == round(currentNode[1],1):
                return 0.0
            elif round(node[0],1) != round(currentNode[0],1) and round(node[1],1) != round(currentNode[1],1):
                return self.stepCost + 0.05
            return self.stepCost


    def algorithm(self):
        print "\n\n\nStarting A*"
        self.handles=[]
        self.handles.append(self.env.plot3(points=array(((self.goal[0],self.goal[1],0.20))),pointsize=0.05,colors=array(((1,1,1))),drawstyle=1))
        while not self.frontier.empty():
            currentNode = self.frontier.get()
            self.explored.update({hash(currentNode):currentNode})
            if (((round(currentNode.description[0],1) == round(self.goal[0],1))) and ((round(currentNode.description[1],1) == round(self.goal[1],1)))):
                if (round(currentNode.description[2],1) == round(self.goal[2],1) or round(currentNode.description[2],1) == round((2*pi)+self.goal[2],1)):
                    self.goalNode = currentNode
                    break
                print "Goal Reached"
                self.goalNode = currentNode
                break

            if self.noNeighors == 4:
                nNodes =  self.neighborsFour(currentNode)
            elif self.noNeighors == 8:
                nNodes =  self.neighborsEight(currentNode)
            for node in nNodes:
                newCost = self._g(currentNode) + self.stepCost
                z = 0
                if self.drawPoints(node):
                    continue
                key = str(node[0])+','+str(node[1])+','+str(node[2])

                if self.explored.has_key(hash(key)):
                    node = self.explored[hash(key)]
                else:
                    node = Node(node,currentNode,newCost,self._h(node))
                if node not in self.cost_so_far or newCost < self.cost_so_far[node]:
                    self.cost_so_far[node] = newCost
                    node.priority = newCost + self._h(node.description)
                    node.parentNode = currentNode
                    self.frontier.put(node)
                    self.came_from[node] = currentNode
        if self.frontier.empty():
            print "No solution found"

    def rebuildPath(self):
        self.path = [self.goalNode.description]
        parent = self.goalNode.parentNode
        while (1):
            self.path.append(parent.description)
            parent = parent.parentNode
            if parent is None:
                break
        self.path.reverse()
        self.drawPoints(self.path,1)
        return self.path
