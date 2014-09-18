#include "video_survey/experiment_utils.h"

#include <cstdio>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

std::string exp_utils::system::runCommand(const std::string& command)
{
  char* buffer= new char[255];
  FILE *stream = popen(command.c_str(),"r");
  while ( fgets(buffer, 255, stream) != NULL ) { }
  pclose(stream);
  std::string response_str(buffer);
  delete buffer;

  return response_str;
}

bool exp_utils::youtube::isYoutubeLink(const std::string& str)
{
  if(str.length() < 22)
    return false;
  else
  {
    std::string head = str.substr(0,22);
    if(head=="http://www.youtube.com")
      return true;
    else
      return false;
  }
}

std::string exp_utils::youtube::getYoutubeVideoID(std::string url)
{
  //TODO check is valid youtube URL
  return url.erase(0, url.find_first_of('=')+1);
}

std::string exp_utils::youtube::getYoutubeEmbedURL(const std::string& url)
{
  std::string id = getYoutubeVideoID(url);
  return boost::str(boost::format("%s%s")
                                  % "//www.youtube.com/embed/"
                                  % id);
}
