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
  Cell operator()(int i)
  {
    int z = i / dimxy;
    int xy = i % dimxy;
    int y = xy / dimx;
    int x = xy % dimx;
    return Cell(x,y,z);
  }
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

void display_cube( const Point3& p, SetCube& sc, planning_scene::PlanningScenePtr scene, ros::Publisher pub)
{
  moveit_msgs::PlanningScene ps_msg;
  sc(p);
  scene->getPlanningSceneMsg(ps_msg);
  ps_pub.publish(ps_msg);
  ros::spinOnce();
  usleep(1000);
}

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
  int dimx = 100;//int( (center.x+2*maxdist)/increment ) + 3;
  int dimy = 100;//int( (center.y+2*maxdist)/increment ) + 3;
  int dimz = 100;//int( (center.z+2*maxdist)/increment ) + 3;

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

  if( grid[ index(start_cell) ] == WALL || start_cell.x>=dimx || start_cell.y>=dimy || start_cell.z>=dimz)
  {
    ROS_WARN("Start Cell = (%d,%d,%d), in collision!", start_cell.x, start_cell.y, start_cell.z);
    display_cube( grid_to_world(start_cell), set_cube, cube_scene, ps_pub );
    cin.get();
  }
  else
  {
    ROS_INFO("Start Cell = (%d,%d,%d)", start_cell.x, start_cell.y, start_cell.z);
    display_cube( grid_to_world(start_cell), set_cube, cube_scene, ps_pub );
  }
  if( grid[ index(goal_cell) ] == WALL  || goal_cell.x>=dimx || goal_cell.y>=dimy || goal_cell.z>=dimz)
  {
    ROS_WARN("Goal Cell = (%d,%d,%d), in collision!", goal_cell.x, goal_cell.y, goal_cell.z);
    display_cube( grid_to_world(goal_cell), set_cube, cube_scene, ps_pub );
    cin.get();
  }
  else
  {
    ROS_INFO("Goal Cell = (%d,%d,%d)", goal_cell.x, goal_cell.y, goal_cell.z);
    display_cube( grid_to_world(goal_cell), set_cube, cube_scene, ps_pub );
  }

  const int U=1, Z=0, D=-1;
  const int num_motions = 26;
  //                     one cell step  two cell steps---------->  three cell step
  int dx[num_motions] = {U,D, Z,Z, Z,Z, U,U,D,D, Z,Z,Z,Z, U,D,U,D, U,U,U,U,D,D,D,D};
  int dy[num_motions] = {Z,Z, U,D, Z,Z, U,D,U,D, U,U,D,D, Z,Z,Z,Z, U,U,D,D,U,U,D,D};
  int dz[num_motions] = {Z,Z, Z,Z, U,D, Z,Z,Z,Z, U,D,U,D, U,U,D,D, U,D,U,D,U,D,U,D};
  
  /*
  const int num_motions = 26;
  int dx[num_motions] = {U,U,U, U,U,U, U,U,U, D,D,D, D,D,D, D,D,D, Z,Z,Z, Z,Z,Z, Z,Z};
  int dy[num_motions] = {U,U,U, D,D,D, Z,Z,Z, U,U,U, D,D,D, Z,Z,Z, U,U,U, D,D,D, Z,Z};
  int dz[num_motions] = {U,D,Z, U,D,Z, U,D,Z, U,D,Z, U,D,Z, U,D,Z, U,D,Z, U,D,Z, U,D};
  */

  // BFS
  ROS_INFO("Searching...");
  queue<Cell> frontier;
  frontier.push(start_cell);
  grid[ index(start_cell) ] = 0;
  bool goal_found = false;
  int goal_value;
  while(!goal_found && !frontier.empty())
  {
    // expand
    Cell expand = frontier.front();
    //display_cube( grid_to_world(expand), set_cube, cube_scene, ps_pub );
    frontier.pop();
    if( index(expand) == index(goal_cell) )
    {
      ROS_INFO("Finished! Path Found");
      goal_found = true;
    }
    else
    {
      // successors (26 succs)
      vector<Cell> succs;
      for(int m=0; m<num_motions;++m)
      {
        Cell nbr(expand.x+dx[m], expand.y+dy[m], expand.z+dz[m]);
        if( grid[ index(nbr) ] == UNDISCOVERED )
        {
          frontier.push(nbr);
          grid[ index(nbr) ] = index(expand); // for reconstruct
        }
      }
      /*
      for(int dx=-1; dx<2; ++dx){
        for(int dy=-1; dy<2; ++dy){
          for(int dz=-1; dz<2; ++dz){
            if(!(dx==0 && dy==0 && dz==0)){
              //valid motion
              if(expand.x==0 && dx==-1)
                continue;
              if(expand.y==0 && dy==-1)
                continue;
              if(expand.z==0 && dz==-1)
                continue;
              if(expand.x==(dimx-1) && dx==1)
                continue;
              if(expand.y==(dimy-1) && dy==1)
                continue;
              if(expand.z==(dimz-1) && dz==1)
                continue;
              //valid cell
              Cell nbr(expand.x+dx, expand.y+dy, expand.z+dz);
              if( grid[ index(nbr) ] == UNDISCOVERED )
              {
                frontier.push(nbr);
                grid[ index(nbr) ] = index(expand); // for reconstruct
              }
            }
          }
        }
      } // loop over motions
      */
    } //endif
  }// end search
  
  if(goal_found)
  {
    Cell current = goal_cell;
    vector<Cell> cell_path;
    while(grid[ index(current) ]!=0)
    {
      cell_path.push_back(current);
      // grid[ index(child) ] = index(parent);
      // -->
      // parent = inverse_index( grid[ index( current ) ] )
      // display_cube( grid_to_world(current), set_cube, cube_scene, ps_pub );
      current = index( grid[ index(current) ] );
    }
    assert( index(current) == index(start_cell) );
    cell_path.push_back(current);

    reverse(cell_path.begin(), cell_path.end());
    ROS_INFO("Path length: %d", (int)cell_path.size());
    for(int c=0; c<cell_path.size(); ++c)
      path.push_back( grid_to_world( cell_path[c] ) );

    //replay
    //for(int i=0; i<path.size(); ++i)
    //{
    //  display_cube( path[i], set_cube, cube_scene, ps_pub );
    //  usleep(100000); //0.1s
    //}
  }
  ROS_INFO("Search terminated.");
  delete[] grid;
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
    double increment = 0.02;

    do_bfs_3d(start,goal,cent,max_dist,increment,cube_scene,path);

    ofstream solution_file;
    solution_file.open( (path_directory/(traj_id+".path")).string().c_str(), ios::out);
    for(vector<Point3>::iterator i=path.begin(); i!=path.end(); ++i)
      solution_file << i->x << " " << i->y << " " << i->z << endl;
    solution_file.close();

    moveit_msgs::PlanningScene empty_ps_msg;
    ps_pub.publish(empty_ps_msg);
    cube_scene->setPlanningSceneMsg( empty_ps_msg );
    ros::spinOnce();
  }
  
  ROS_INFO("Completed compute the path solutions.");
  return 0;
}
