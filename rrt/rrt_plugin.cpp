#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include"rrt_utils.cpp"
using namespace OpenRAVE;

class rrt_module : public ModuleBase
{
  std::vector< std::vector<rrt_type> > path;
public:
    rrt_module(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("rrtRun",boost::bind(&rrt_module::rrt,this,_1,_2),
                        "This is command runs rrt algorithm");
        RegisterCommand("pathSmooth",boost::bind(&rrt_module::smoothing,this,_1,_2),
                        "This is command smoothens the path produced by rrt algorithm");
    }
    virtual ~rrt_module() {}
    bool smoothing(std::ostream& sout, std::istream& sinput);
    bool rrt(std::ostream& sout, std::istream& sinput);
    void pathStr(std::vector<std::vector<rrt_type> > path,std::ostream& sout);
};
void rrt_module::pathStr(std::vector<std::vector<rrt_type> > path,std::ostream& sout)
{
  // converts the path from vector format to string format to intrface with pyton
  for (unsigned int i=0;i<path.size();++i)
  {
    for (unsigned int j=0;j<path[i].size();++j)
    {
      sout<<path[i][j]<<",";
    }
    sout<<"\n";
  }
}

bool rrt_module::smoothing(std::ostream& sout, std::istream& sinput)
{
  OpenRAVE::EnvironmentBasePtr env = OpenRAVE::InterfaceBase::GetEnv();
  std::vector<RobotBasePtr> robots;
  env->GetRobots(robots);

  std::string input;
  int iter = 200;
  sinput>>input;
  if(input=="iter")
    sinput>>iter;
  // printvec(path[0]);

  std::cout<<"\nPath length before smoohing "<<pathDist(path);
  std::vector< std::vector<rrt_type> > shorterpath = shortPath(path,robots[0],GetEnv(),iter);
  std::cout<<"\nPath length after smoohing "<<pathDist(shorterpath);
  pathStr(shorterpath,sout);
  return true;
}
bool rrt_module::rrt(std::ostream& sout, std::istream& sinput)
{
  std::string input;
  rrt_type data,Stepsize=0.0,goalbias=0.11;
  std::vector<rrt_type> startconfig,goalconfig;
  std::vector<rrt_type>* configPtr;
  OpenRAVE::EnvironmentBasePtr env = OpenRAVE::InterfaceBase::GetEnv();
  // gets start and goal configs.
  for(int iter = 0;iter <2;iter++)
  {
    sinput>>input;
    if(input=="start")
      configPtr = &startconfig;
    else if(input=="goal")
      configPtr = &goalconfig;
    configPtr->clear();
    for(int i=0;i<cSpaceDim;i++)
    {
      sinput>>data;
      configPtr->push_back(data);
    }
    if(input == "goal")
      break;
  }
  sinput>>input;
  if (input=="Stepsize")
  {
    sinput>>Stepsize;
  }
  sinput>>input;
  if (input=="goalbias")
  {
    sinput>>goalbias;
  }
  path = rrt_pathfind(startconfig,goalconfig,Stepsize,goalbias,env);
  std::reverse(path.begin(),path.end());
  pathStr(path,sout);
  return true;
}

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrt_module" ) {
        return InterfaceBasePtr(new rrt_module(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("rrt_module");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
