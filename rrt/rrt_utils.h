#ifndef RRT_UTILS_H
#define RRT_UTILS_H
#include <openrave/plugin.h>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>
#include <functional>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
using namespace OpenRAVE;
typedef double rrt_type;
int cSpaceDim = 7;
class RRTNode
{
  std::vector<rrt_type> config;
  RRTNode* parent;
public:
  RRTNode(std::vector<rrt_type>, RRTNode*); // constructor : sets config and parent
  RRTNode(std::vector<rrt_type>); // constructor : sets config
  std::vector<rrt_type> getcurConfig(); //gets config
  void curConfigstr(); //prints config
  void setParent(RRTNode*); //sets parent
  RRTNode* getParent(); //gets parent
  inline bool operator== (RRTNode);
  double distNode(RRTNode,std::vector<rrt_type>);
};

class NodeTree
{
  std::vector<RRTNode*> nodes;
public:
  NodeTree(); //constructor
  NodeTree(std::vector<RRTNode*>); //constructor
  void addNode(RRTNode &);
  void delNode(RRTNode &);
  std::vector<RRTNode*> getNodes();
  void printNodes();
  void printNodes(RRTNode*);
  RRTNode* nearestNode(RRTNode &,double &,std::vector<rrt_type>);
  void getpath(std::vector<std::vector<rrt_type> > &);
};

// fns for collision checking
bool collisionChecker(RRTNode*,EnvironmentBasePtr,RobotBasePtr);
bool collisionChecker(std::vector<rrt_type> ,EnvironmentBasePtr ,RobotBasePtr );
bool edgecollisioncheck(std::vector<rrt_type> ,std::vector<rrt_type>  ,double , OpenRAVE::EnvironmentBasePtr , RobotBasePtr ,std::vector<std::vector<rrt_type> >& );
// fns for generating random sample
std::vector<rrt_type> randConfigGen(std::vector<dReal>, std::vector<dReal>);
// fns to do basic ops in vector and disp fn
void printvec(std::vector<rrt_type>);
std::vector<rrt_type> unitvec(std::vector<rrt_type> );
std::vector<rrt_type> scalevec(std::vector<rrt_type>, rrt_type);
std::vector<rrt_type> addvec(std::vector<rrt_type> , std::vector<rrt_type>);
std::vector<rrt_type> distvec(std::vector<rrt_type>,std::vector<rrt_type>);
// fns pretaining to rrt algorithm
NodeTree* growTree(NodeTree* , std::vector<dReal> ,std::vector<dReal> , rrt_type ,std::vector<rrt_type> , rrt_type ,bool & ,EnvironmentBasePtr , RobotBasePtr , std::vector<rrt_type> );
std::vector<std::vector<rrt_type> > rrt_pathfind(std::vector<rrt_type>,std::vector<rrt_type>, rrt_type, rrt_type,EnvironmentBasePtr);
// fns related to shortcut smoothing
double pathDist(std::vector<std::vector<rrt_type> > );
// std::vector<std::vector<rrt_type> > shortPath(std::vector<std::vector<rrt_type> > path,RobotBasePtr pr2, OpenRAVE::EnvironmentBasePtr env ,int iter =200);


// std::vector<rrt_type> randConfigGene(std::vector<dReal>, std::vector<dReal>,rrt_type);
// bool injointspace(std::vector<rrt_type> ,std::vector<dReal>, std::vector<dReal>);
#endif // RRT_UTILS_H
