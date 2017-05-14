#ifndef RRT_UTILS_C
#define RRT_UTILS_C
#include "rrt_utils.h"
/*
//
// member function definitions for class RRTNode
//
*/
RRTNode::RRTNode(std::vector<rrt_type> newConfig, RRTNode* parentNode)
{
  config = newConfig;
  parent = parentNode;
}
RRTNode::RRTNode(std::vector<rrt_type> newConfig)
{
  config = newConfig;
  parent = NULL;
  // this->curConfigstr();
}

std::vector<rrt_type> RRTNode::getcurConfig()
{
  return config;
}
void RRTNode::curConfigstr()
{
  if(!config.empty())
  {
    std::ostringstream oss;
    std::copy(config.begin(), config.end()-1, std::ostream_iterator<float>(oss, ","));
    oss << config.back();
    std::cout<<"\n"<<oss.str();
  }
  else
    std::cout<<"empty config";
}
void RRTNode::setParent(RRTNode* parentNode)
{
  parent = parentNode;
}
RRTNode* RRTNode::getParent()
{
  return parent;
}
inline bool RRTNode::operator== (RRTNode rhs)
{
  int j=0;
  for (std::vector<rrt_type>::iterator i = config.begin(); i != config.end();i++,j++)
  {
    if (*i!=rhs.config[j])
    {
      return false;
    }
  }
  return true;
}

double RRTNode::distNode(RRTNode randConfig,std::vector<rrt_type> eucWeight)
{
  double dist = 0.0;
  int j=0;
  std::vector<rrt_type> ConfigRand = randConfig.getcurConfig();
  for (std::vector<rrt_type>::iterator i = config.begin(); i != config.end();i++,j++)
  {
    dist+=(eucWeight[j]*pow(*i-ConfigRand[j],2));
  }
  return sqrt(dist);
}
/*
//
// member function definitions for class NodeTree
//
*/
NodeTree::NodeTree()
{
  nodes.clear();
}
NodeTree::NodeTree(std::vector<RRTNode*> newNodes) //constructor
{
  nodes = newNodes;
}
void NodeTree::addNode(RRTNode &vertex)
{
  // std::cout<<"\nNode added:";vertex.curConfigstr();
  nodes.push_back(&vertex);
}
void NodeTree::delNode(RRTNode &vertex)
{
  // node.erase(vertex);
  std::cout<<"\nNode deleted:";
  // RRTNode* az1;
  int j=0;
  for (std::vector<RRTNode*>::iterator i = nodes.begin(); i != nodes.end();i++)
    if (**i == vertex)
    {
      // az1 = *i;
      (*i)->curConfigstr(); // or (**i).curConfigstr();
      nodes.erase(nodes.begin()+j);
      // std::cout<<"\n"<< az1->getcurConfig()[0];  //nodes.size();
    }
}
std::vector<RRTNode*> NodeTree::getNodes()
{
  return nodes;
}
void NodeTree::printNodes()
{
  RRTNode* parentsss;
  std::cout<<"\nNodes Size :"<<nodes.size();
  std::cout<<"\nNodes are :";
  nodes[nodes.size()-1]->curConfigstr();
  parentsss = nodes[nodes.size()-1]->getParent();
  for (unsigned int i=0;i<nodes.size();++i)
  {
    // std::cout<<"\n";
    if(parentsss == NULL){
      // std::cout<<"\nNULL detected in parent";
      break;
    }
    parentsss->curConfigstr();
    parentsss = parentsss->getParent();
  }
}

void NodeTree::getpath(std::vector<std::vector<rrt_type> > &path)
{
  RRTNode* parentsss;
  nodes[nodes.size()-1]->curConfigstr();
  parentsss = nodes[nodes.size()-1]->getParent();
  path.push_back(nodes[nodes.size()-1]->getcurConfig());

  for (unsigned int i=0;i<nodes.size();++i)
  {
    if(parentsss == NULL){
      break;
    }
    path.push_back(parentsss->getcurConfig());
    parentsss = parentsss->getParent();
  }
  std::cout<<"\nNode Explored "<<nodes.size()<<" No of nodes in Path  "<<path.size();
}

RRTNode* NodeTree::nearestNode(RRTNode &randConfig, double &leastDist, std::vector<rrt_type> eucWeight)
{
  leastDist = 0.0;
  double curDist = 0.0;
  bool first = true;
  RRTNode* closestNode;
  for (std::vector<RRTNode*>::iterator i = nodes.begin(); i != nodes.end();i++)
  {
    curDist = (*i)->distNode(randConfig, eucWeight);
    if((leastDist > curDist)| first)
    {
      leastDist = curDist;
      closestNode = *i;
      first =false;
    }
  }
  // std::cout<<"\nleast Dist :" <<leastDist;
  return closestNode;
}

//
//
// Other functions defined in the header
//
//

/*bool injointspace(std::vector<rrt_type> config,std::vector<dReal> minLimit, std::vector<dReal> maxLimit)
{
  for (int i = 0; i < cSpaceDim; i++)
    if((minLimit[i]>config[i]) | (config[i]<maxLimit[i]))
      return false;
  return true;
}*/

std::vector<rrt_type> randConfigGen(std::vector<dReal> minLimit, std::vector<dReal> maxLimit)
{
  // generates random config and checks if it is reachable by each joints
  std::vector<rrt_type> randConfig;
  randConfig.clear();
  // std::cout<<"\nSample config:";
  // printvec(randConfig);
  for(int i=0;i<cSpaceDim;i++)
  {
    rrt_type dim_config = ( ((rrt_type)rand()/(rrt_type)RAND_MAX) *(maxLimit[i]-minLimit[i]) ) +(minLimit[i]);
    // std::cout<<" dim "<<dim_config;
    randConfig.push_back(dim_config);
    // std::cout<<dim_config;
  }
  // printvec(randConfig);
  // RRTNode randConfigNode(randConfig);
  // return randConfigNode;
  return randConfig;
}

void printvec(std::vector<rrt_type> config)
{
  if(!config.empty())
  {
    std::ostringstream oss;
    std::copy(config.begin(), config.end()-1, std::ostream_iterator<float>(oss, ","));
    oss << config.back();
    std::cout<<"\n"<<oss.str();
  }
  else
    std::cout<<" empty vect";
}
std::vector<rrt_type> unitvec(std::vector<rrt_type> vec)
{

  double mag = 0.0;
  for(std::vector<rrt_type>::iterator v=vec.begin();v!=vec.end();v++)
    mag+=pow(*v,2);
  mag = sqrt(mag);

  // std::cout<<" mag "<<mag;
  for(std::vector<rrt_type>::iterator v=vec.begin();v!=vec.end();v++)
  {
    *v = *v/mag;
    // *v = *v/dist;
  }
  return vec;
}

std::vector<rrt_type> scalevec(std::vector<rrt_type> vec , rrt_type factor)
{
  std::vector<rrt_type> scaledvec;
  for(std::vector<rrt_type>::iterator v=vec.begin();v!=vec.end();v++)
    scaledvec.push_back((*v)*factor);
  return scaledvec;
}

std::vector<rrt_type> addvec(std::vector<rrt_type> vec , std::vector<rrt_type> vec1)
{
  std::vector<rrt_type> addvector;
  std::vector<rrt_type>::iterator v1=vec1.begin();
  for(std::vector<rrt_type>::iterator v=vec.begin();v!=vec.end();v++)
  {
    addvector.push_back((*v)+(*v1));
    v1++;
  }
  return addvector;
}

std::vector<rrt_type> distvec(std::vector<rrt_type> from ,std::vector<rrt_type> to)
{
  std::vector<rrt_type> dist,dist1;
  dist.clear();
  std::vector<rrt_type>::iterator t1=to.begin();
  // std::transform(to.begin(),to.end(),dist1.begin(),same);
  // printvec(dist1);
  // std::transform(from.begin(),from.end(),from.begin(),dist1.begin(),std::minus<rrt_type>());
  // int i=0;
  for(std::vector<rrt_type>::iterator f1=from.begin();f1!=from.end();f1++)
  {
    // std::cout<<" from "<<*f1<<" to "<<*t1;
    // std::cout<<" diff "<<(*f1)-(*t1)<<" ";//<<dist1[i];
    dist.push_back((*t1)-(*f1));
    t1++;
    // i++;
  }
  // std::cout<"\ndistvec fn: "
  // printvec(dist);
  return dist;
}

bool collisionChecker(RRTNode* chkNode,EnvironmentBasePtr env,RobotBasePtr pr2)
{
  pr2->SetActiveDOFValues(chkNode->getcurConfig());
  if(env->CheckCollision(pr2)||env->CheckSelfCollision(pr2))
    return true;
  return false;
}

bool collisionChecker(std::vector<rrt_type> chkConfig,EnvironmentBasePtr env,RobotBasePtr pr2)
{
  pr2->SetActiveDOFValues(chkConfig);
  if(env->CheckCollision(pr2)||env->CheckSelfCollision(pr2))
    return true;
  return false;
}

NodeTree* growTree(NodeTree* tree, std::vector<dReal> minLimit,std::vector<dReal> maxLimit, rrt_type stepsize,std::vector<rrt_type> goalconfig, rrt_type goalbias,bool & gflag,EnvironmentBasePtr env, RobotBasePtr pr2, std::vector<rrt_type> eucWeight)
{
  RRTNode* randNode;
  RRTNode* nearNode;
  RRTNode* newNode;
  // generating a rand node with some goal bias
  rrt_type prob = ((rrt_type)rand()/(rrt_type)RAND_MAX);
  // std::cout <<"\nGoal Bias "<<goalbias<<"  "<<prob;
  if (( prob < goalbias))
  {
    randNode = new RRTNode(goalconfig);
    gflag = true;
  }
  else
    randNode = new RRTNode(randConfigGen(minLimit,maxLimit));

  double dist = 1.0;
  while(true)
  {
    nearNode = tree->nearestNode(*randNode,dist,eucWeight);//dist is by reference
    // std::cout<<"\nnearNode  ";nearNode->curConfigstr();
    std::vector<rrt_type> vect = unitvec(distvec(nearNode->getcurConfig(),randNode->getcurConfig()));
    // printvec(vect);
    if((*nearNode) == (*randNode))
    {
      break;
    }
    if(stepsize < dist)
    {
      std::vector<rrt_type> newVect = scalevec(vect,stepsize);
      newVect = addvec(newVect,nearNode->getcurConfig());

      newNode = new RRTNode(newVect,nearNode);

      if(collisionChecker(newNode,env,pr2))
      {
        // std::cout<<"\ncollision detected";
        gflag = false;
        break;
      }

      tree->addNode(*newNode);
      // tree->printNodes();
    }
    else
    {
      randNode->setParent(nearNode);

      if(collisionChecker(randNode,env,pr2))
      {
        // std::cout<<"\ncollision detected";
        gflag = false;
        break;
      }

      tree->addNode(*randNode);
      break;
    }
  }
  return tree;
}
std::vector<std::vector<rrt_type> > rrt_pathfind(std::vector<rrt_type> startconfig,std::vector<rrt_type> goalconfig, rrt_type stepsize, rrt_type goalbias,EnvironmentBasePtr env)
{
  // srand(time(NULL));
  std::cout<<"\nRRT running...\n";
  bool gflag = false;
  NodeTree* startTree = new NodeTree;
  RRTNode* curNode;
  std::vector<RobotBasePtr> robots;
  std::vector<dReal> minLimit,maxLimit;
  // euclidean weightage:
  std::vector<rrt_type> eucWeight;
  eucWeight.clear();
  eucWeight.push_back(1);
  eucWeight.push_back(1);
  eucWeight.push_back(1);
  eucWeight.push_back(1);
  eucWeight.push_back(0);
  eucWeight.push_back(0);
  eucWeight.push_back(0);
  // adding start node to the tree
  curNode = new RRTNode(startconfig);
  startTree->addNode(*curNode);
  env->GetRobots(robots);
  // since the limits of joint 5 and  7 can rotate 360* making it -pi to pi
  robots[0]->GetActiveDOFLimits(minLimit,maxLimit);
  minLimit[4] = minLimit[6] = -PI;
  maxLimit[4] = maxLimit[6] = PI;
  int i =0;
  time_t now,start;
  time(&start);
  while(true)
  {
    ++i;
    startTree = growTree(startTree,minLimit,maxLimit,stepsize,goalconfig,goalbias,gflag,env,robots[0], eucWeight);
    if(gflag)
    {
      break;
    }
    time(&now);
    if(difftime(now,start)>(20*60))
    {
      std::cout<<"Took 20 mins..\n";
      break;
    }
  }
  std::cout<<"\nIters: "<<i;
  std::vector<std::vector<rrt_type> > path;
  startTree->getpath(path);
  return path;
}
double pathDist(std::vector<std::vector<rrt_type> > path)
{
  double Totmag = 0;
  double mag = 0;

  for(std::vector<std::vector<rrt_type> >::iterator v = path.begin();v!=(path.end()-1);v++)
  {
    mag = 0.0;
    for(int i=0;i<cSpaceDim;i++)
    {
      mag+=pow((*v)[i]-(*(v+1))[i],2);
    }
    Totmag += sqrt(mag);
  }
  return Totmag;
}
bool edgecollisioncheck(std::vector<rrt_type> minNode,std::vector<rrt_type> distDisp ,double mag, OpenRAVE::EnvironmentBasePtr env, RobotBasePtr pr2 ,std::vector<std::vector<rrt_type> >&  newbetNodes)
{
  newbetNodes.clear();
  double mag_new = 0.05; //stepsize
  newbetNodes.push_back(minNode);
  mag = 1.0;
  while(mag_new < mag)
  {
    std::vector<rrt_type> betNode = scalevec(distDisp,mag_new);
    betNode = addvec(betNode,minNode);
    newbetNodes.push_back(betNode);
    pr2->SetActiveDOFValues(betNode);
    if(env->CheckCollision(pr2)||env->CheckSelfCollision(pr2))
    {
      return true;
    }
    // if(collisionChecker(betNode,env,pr2))
      // return true;
    mag_new+=0.05; //stepsize
  }
  return false;
}

std::vector<std::vector<rrt_type> > shortPath(std::vector<std::vector<rrt_type> > path,RobotBasePtr pr2, OpenRAVE::EnvironmentBasePtr env ,int iter =200)
{
  std::vector<RobotBasePtr> robots;
  std::vector<std::vector<rrt_type> > shortPath = path;


  for(int i=0;i<iter;i++)
  {
    // std::cout<<"\n iteration  "<<i;
    // std::cout<<" path length  "<<pathDist(shortPath);
    // std::cout<<" path size  "<<shortPath.size();

    int node1,node2;
    while(true)
    {

    node1 = rand()%(shortPath.size());
    node2 = rand()%(shortPath.size());
    if(node1!=node2)
      break;
    }

    std::vector<rrt_type> maxNode = shortPath[std::max(node1,node2)];
    std::vector<rrt_type> minNode = shortPath[std::min(node1,node2)];
    std::vector<rrt_type> distDisp = distvec(minNode,maxNode);
    std::vector<std::vector<rrt_type> > newbetNodes;

    double magDisp = 0.0;

    for(std::vector<rrt_type>::iterator v=distDisp.begin();v!=distDisp.end();v++)
      magDisp+=pow(*v,2);
    magDisp = sqrt(magDisp);

    bool check = edgecollisioncheck(minNode,distDisp,magDisp,env,pr2,newbetNodes);
    if (check)
      continue;
    shortPath.erase(shortPath.begin()+(std::min(node1,node2)+1),shortPath.begin()+std::max(node1,node2));
    shortPath.insert(shortPath.begin()+(std::min(node1,node2)+1),newbetNodes.begin(),newbetNodes.end());
  }
  return shortPath;
}
#endif //RRT_UTILS_C
