#ifndef EXP_UTILS_H
#define EXP_UTILS_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>

namespace exp_utils
{
  namespace system
  {
    std::string runCommand(const std::string& command);
  }

  namespace youtube
  {
    bool isYoutubeLink(const std::string& str);
    std::string getYoutubeVideoID(std::string url);
    std::string getYoutubeEmbedURL(const std::string& url);
  }
}
#endif
