#include <ros/ros.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/collision_detection/collision_common.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <queue>

#include <moveit_recorder/utils.h>
#include <moveit_recorder/trajectory_video_lookup.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "video_survey/features.h"

using namespace std;

ros::Publisher ps_pub;

struct Cell
{
  int x, y, z;
  Cell(int X, int Y, int Z) : x(X), y(Y), z(Z) {}
};

struct Index
{
  const int dimx, dimy, dimxy;
  Index(int dx, int dy, int dz) : dimx(dx), dimy(dy), dimxy(dy*dx) {}
  int operator()(const Cell& c) {return (*this)(c.x,c.y,c.z); }
  int operator()(int x, int y, int z) { return x + (y*dimx) + (z*dimxy); }
};

struct Grid2World
{
  const double inc;
  const Point3 ctr;
  Grid2World(double increment, Point3 center, double maxdist) : inc(increment), ctr(center.x-maxdist,
      center.y-maxdist,
      center.z-maxdist){}
  Point3 operator()(int x, int y, int z){ return (*this)(Cell(x,y,z)); }
  Point3 operator()(const Cell& c) { return Point3( c.x*inc+ctr.x, 
      c.y*inc+ctr.y, 
      c.z*inc+ctr.z ); }
  Cell operator()(const Point3& p) { return Cell( int((p.x-ctr.x)/inc),
      int((p.y-ctr.y)/inc),
      int((p.z-ctr.z)/inc));}
};

struct SetCube
{
  planning_scene::PlanningScenePtr scene;
  SetCube(planning_scene::PlanningScenePtr& sc) : scene(sc) {}
  map<string,double> config;
  void operator()(const Point3& p)
  {
    config["world_joint/trans_x"] = p.x;
    config["world_joint/trans_y"] = p.y;
    config["world_joint/trans_z"] = p.z;
    scene->getCurrentStateNonConst().setVariablePositions(config); 
  }
};

struct InCollision
{
  planning_scene::PlanningScenePtr scene;
  SetCube set_cube;
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  InCollision(planning_scene::PlanningScenePtr& sc) : scene(sc), set_cube(sc) {}
  bool operator()(const Point3& p)
  {
    set_cube(p);
    res.clear();
    scene->checkCollision(req,res);
    return res.collision;
  }
};

void do_bfs_3d( const Point3& start, 
    const Point3& goal, 
    const Point3& center,
    const double maxdist,
    const double increment, 
    planning_scene::PlanningScenePtr& cube_scene, 
    vector<Point3>& path)
{
  SetCube set_cube(cube_scene);
  
  // check if contained
  assert(start.x < center.x+maxdist); 
  assert(start.y < center.y+maxdist); 
  assert(start.z < center.z+maxdist); 
  assert(goal.x  < center.x+maxdist); 
  assert(goal.y  < center.y+maxdist); 
  assert(goal.z  < center.z+maxdist); 
  assert(start.x > center.x-maxdist); 
  assert(start.y > center.y-maxdist); 
  assert(start.z > center.z-maxdist); 
  assert(goal.x  > center.x-maxdist); 
  assert(goal.y  > center.y-maxdist); 
  assert(goal.z  > center.z-maxdist); 

  //allocate the search space
  int dimx = int( (center.x+maxdist)/increment ) + 3;
  int dimy = int( (center.y+maxdist)/increment ) + 3;
  int dimz = int( (center.z+maxdist)/increment ) + 3;

  Index index(dimx,dimy,dimz);
  Grid2World grid_to_world(increment, center, maxdist);
  InCollision is_in_collision(cube_scene);

  // intialize
  int* grid = new int[dimx * dimy * dimz];
  const int WALL = -1000000;
  const int UNDISCOVERED = 1;
  size_t num_walls = 0;
  for(int x=0; x<dimx; ++x)
  {
    for(int y=0; y<dimy; ++y)
    {
      for(int z=0; z<dimz; ++z)
      {
        if(is_in_collision( grid_to_world(x,y,z) )) 
        {
          grid[ index(x,y,z) ] = WALL;
          num_walls++;
        }
        else
          grid[ index(x,y,z) ] = UNDISCOVERED;
      }
    }
  }
  ROS_INFO("%d cells in grid (%dx%dx%d), %d are walls.", dimx*dimy*dimz, dimx,dimy,dimz, (int)num_walls);

  //extents
  //yz face
  for(int y=0; y<dimy; ++y)
  {
    for(int z=0; z<dimz; ++z)
    {
      grid[ index(0,y,z) ] = WALL;
      grid[ index(dimx-1,y,z) ] = WALL;
    }
  }
  //xy face
  for(int y=0; y<dimy; ++y)
  {
    for(int x=0; x<dimx; ++x)
    {
      grid[ index(x,y,0) ] = WALL;
      grid[ index(x,y,dimz-1) ] = WALL;
    }
  }
  //xz face
  for(int x=0; x<dimx; ++x)
  {
    for(int z=0; z<dimz; ++z)
    {
      grid[ index(x,0,z) ] = WALL;
      grid[ index(x,dimy-1,z) ] = WALL;
    }
  }

  // prepare to do the search
  Cell start_cell = grid_to_world(start);
  Cell goal_cell = grid_to_world(goal);

  if( grid[ index(start_cell) ] == WALL )
    ROS_WARN("Start Cell = (%d,%d,%d), in collision!", start_cell.x, start_cell.y, start_cell.z);
  else
    ROS_INFO("Start Cell = (%d,%d,%d)", start_cell.x, start_cell.y, start_cell.z);
  if( grid[ index(goal_cell) ] == WALL )
    ROS_WARN("Goal Cell = (%d,%d,%d), in collision!", goal_cell.x, goal_cell.y, goal_cell.z);
  else
    ROS_INFO("Goal Cell = (%d,%d,%d)", goal_cell.x, goal_cell.y, goal_cell.z);

  // BFS
  queue<Cell> frontier;
  frontier.push(start_cell);
  grid[ index(start_cell) ] = 0;
  bool goal_found = false;
  int goal_value;
  while(!goal_found && !frontier.empty())
  {
    // expand
    Cell expand = frontier.front();
    moveit_msgs::PlanningScene ps_msg;
    set_cube( grid_to_world(expand) );
    cube_scene->getPlanningSceneMsg(ps_msg);
    ps_pub.publish(ps_msg);
    sleep(.1);
    ros::spinOnce();

    frontier.pop();
    if( index(expand) == index(goal_cell) )
    {
      ROS_INFO("Finished! Path Found");
      goal_found = true;
      goal_value = grid[ index(expand) ];
      ROS_WARN("(%d,%d,%d)=%d",expand.x,expand.y,expand.z,grid[ index(expand) ]);
    }
    else
    {
      // successors (26 succs)
      vector<Cell> succs;
      for(int dx=-1; dx<2; ++dx){
        for(int dy=-1; dy<2; ++dy){
          for(int dz=-1; dz<2; ++dz){
            if(!(dx==0 && dy==0 && dz==0)){
              Cell nbr(expand.x+dx, expand.y+dy, expand.z+dz);
              if( grid[ index(nbr) ] == UNDISCOVERED ){
                frontier.push(nbr);
                grid[ index(nbr) ] = index(expand) ; // for reconstruct
              }
            }
          }
        }
      }
    } //endif
  }// end search
  if(goal_found)
  {
    Cell current = goal_cell;
    int current_val = goal_value;
    vector<Cell> cell_path;
    while(grid[ index(current) ]!=0)
    {
      //find current_val around current
      bool break_flag = false;
      for(int dx=-1; dx<2; ++dx){
        for(int dy=-1; dy<2; ++dy){
          for(int dz=-1; dz<2; ++dz){
            if(!(dx==0 && dy==0 && dz==0)){
              Cell nbr(current.x+dx, current.y+dy, current.z+dz);
              //if I generated you, your grid value is my index
              // if nbr generated current, grid[ index(current) ] == index(nbr)
              if( grid[ index(current) ] == index(nbr) ){
                cell_path.push_back(current);
                ROS_INFO("(%d,%d,%d) -> %d to %d",current.x,current.y,current.z,current_val,index(nbr));
                current = nbr;
                break_flag = true;
                break;
              }
              if(break_flag) { break;}
            }
            if(break_flag) { break;}
          }
          if(break_flag) { break;}
        }
        if(break_flag) { break;}
      } // end loop
    }// end unroll
    assert( index(current) == index(start_cell) );
    reverse(cell_path.begin(), cell_path.end());
    ROS_INFO("Path length: %d", cell_path.size());
    for(int c=0; c<cell_path.size(); ++c)
      path.push_back( grid_to_world( cell_path[c] ) );
  }
  
  ROS_INFO("BFS TERMINATED");
  std::cin.get();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "split_screen_creator");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("path_dir",boost::program_options::value<string>(), "Directory for paths to plan")
    ("save_dir",boost::program_options::value<string>(), "Directory for videos");
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);
  if (vm.count("help") || argc == 1) // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }
  string save_dir = utils::get_option(vm,"save_dir","");
  string path_dir = utils::get_option(vm,"path_dir","");

  boost::filesystem::path save_directory(save_dir);
  boost::filesystem::path path_directory(path_dir);

  // load scenes & query list
  TrajectoryVideoLookup tvl;
  tvl.loadFromBag(save_directory.string());
  // load problem sets
  ifstream problems_file;
  problems_file.open( (path_directory/"problemset.path").string().c_str() ); 

  // publisher to scene
  string scene_topic = "cube_planning_scene";
  ps_pub = node_handle.advertise<moveit_msgs::PlanningScene>(scene_topic,1, true);
  utils::rostopic::waitOnSubscribersToTopic(ps_pub, scene_topic);

  // load robot & scene
  planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
  planning_scene::PlanningScenePtr cube_scene;
  cube_scene = psm.getPlanningScene();

  // for every problem
  string traj_id;
  double sx, sy, sz, gx, gy, gz, tx, ty, tz, max_dist;
  while( problems_file >> traj_id >> sx >> sy >> sz >> gx >> gy >> gz >> tx >> ty >> tz >> max_dist)
  {
    ROS_INFO("Loading %s", traj_id.c_str());
    moveit_msgs::PlanningScene ps_msg = tvl.getPlanningScene(traj_id);
    ifstream scene_stream;
    string scene_file = (path_directory/(traj_id+".scene")).string();

    scene_stream.open( scene_file.c_str());
    cube_scene->loadGeometryFromStream(scene_stream);
    scene_stream.close();

    //config bfs
    vector<Point3> path;
    Point3 start(sx,sy,sz);
    Point3 goal(gx,gy,gz);
    Point3 cent(tx,ty,tz);
    double increment = 0.1;

    do_bfs_3d(start,goal,cent,max_dist,increment,cube_scene,path);
    cube_scene->getPlanningSceneMsg( ps_msg );
    ps_pub.publish(ps_msg);
    ros::spinOnce();
    ROS_WARN("Press enter to continue");
    cin.get();
  }

  return 0;
}
