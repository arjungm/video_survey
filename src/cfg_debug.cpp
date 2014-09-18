#include <string>
#include <iostream>
#include <fstream>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

using namespace std;

int main(int argc, char** argv)
{
  boost::program_options::options_description desc;
  desc.add_options()
    ("help", "Show help message")
    ("trials",boost::program_options::value<int>(), "trials")
    ("timeout",boost::program_options::value<int>(), "timeout")
    ("debug",boost::program_options::value<string>(), "debug file")
    ("template",boost::program_options::value<string>(), "template file");
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options po = boost::program_options::parse_command_line(argc, argv, desc);
  boost::program_options::store(po, vm);
  boost::program_options::notify(vm);
  
  if (vm.count("help") || argc == 1) // show help if no parameters passed
  {
    std::cout << desc << std::endl;
    return 1;
  }
  
  string debugfilename = vm.count("debug") ? vm["debug"].as<std::string>() : "";
  if(debugfilename=="")
  {
    cout << "No file. "; 
    return 0;
  }
  string templatefilename = vm.count("template") ? vm["template"].as<std::string>() : "";
  if(templatefilename=="")
  {
    cout << "No file. "; 
    return 0;
  }

  int timeout = vm.count("timeout") ? vm["timeout"].as<int>() : 30;
  int trials = vm.count("trials") ? vm["trials"].as<int>() : 30;

  string name;
  int num=0;

  std::string error, start, goal, planner, previous;

  ifstream debugfile( debugfilename.c_str() );
  
  ifstream templatefile( templatefilename.c_str() );
  templatefile >> name;
  
  boost::filesystem::path filepath(templatefilename);
  string path = filepath.parent_path().string();
  string logpath = (filepath.parent_path() / "../log").string();
  cout << "Name=" << name << endl;
  cout << "Path=" << path << endl;
  cout << "Log=" << logpath << endl;

  while(debugfile >> error >> start >> goal >> planner >> previous)
  {
    ofstream outfile;
    string outfilename = boost::str(boost::format("%s/%s_%d_debug.cfg") % path % name % num);
    cout << boost::str(boost::format("roslaunch moveit_recorder run_benchmark_ompl.launch cfg:=%s") % outfilename) << endl;
    outfile.open( outfilename.c_str() , ios::out );
    
    outfile << boost::str(boost::format("[scene]\n"));
    outfile << boost::str(boost::format("name=%s\n") % name);
    outfile << boost::str(boost::format("output=%s/%s.log\n") % logpath % name);
    outfile << boost::str(boost::format("runs=1\n"));
    outfile << boost::str(boost::format("record=1\n"));
    outfile << boost::str(boost::format("start=%s\n") % start);
    outfile << boost::str(boost::format("goal=%s\n") % goal);
    outfile << boost::str(boost::format("query=xxxxxxxx\n"));
    outfile << boost::str(boost::format("group=right_arm\n"));
    outfile << boost::str(boost::format("timeout=%d\n") % timeout);
    outfile << boost::str(boost::format("num_trials=%d\n") % trials);
    outfile << boost::str(boost::format("\n"));
    outfile << boost::str(boost::format("[plugin]\n"));
    outfile << boost::str(boost::format("name=ompl_interface/OMPLPlanner\n"));
    outfile << boost::str(boost::format("planners=%s\n") % planner);
    outfile << boost::str(boost::format("runs=1\n"));

    outfile.close();
    ++num;
  }
  debugfile.close();
  templatefile.close();
  
  // break for reading
  ofstream debugfilewrite;
  debugfilewrite.open(debugfilename.c_str(), ios::out | ios::app);
  debugfilewrite << endl;
  debugfilewrite.close();

  cout << "Files=" << num << endl;
  return 0;
}

