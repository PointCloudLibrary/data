/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * @author: Cedric Cagniart, Koen Buys
 */

#ifndef PCL_PEOPLE_OPTIONPARSER_H_
#define PCL_PEOPLE_OPTIONPARSER_H_

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <cstdio>

inline std::string buildFilename(const std::string &baseName, int id)
{
  char cfilename[1024]; //buffer for the filename
  sprintf(cfilename, baseName.c_str(), id);
  return std::string(cfilename);
}

inline std::string buildFilename(const std::string &baseName, int id1, int id2)
{
  char cfilename[1024]; //buffer for the filename
  sprintf(cfilename, baseName.c_str(), id1, id2);
  return std::string(cfilename);
}

inline std::string buildFilename(const std::string &baseName, int id0, int id1, int id2)
{
  char cfilename[1024]; //buffer for the filename
  sprintf(cfilename, baseName.c_str(), id0, id1, id2);
  return std::string(cfilename);
}

/*
class OptionParser
{
    public :
    OptionParser(int argc, char **argv);

    template <class T>
    const T getOption(const char *optName, const char *plugName = 0);

    protected :
    std::map<std::string, std::vector<std::string> > mArguments;
};

OptionParser::OptionParser(int argc, char**argv)
{
    std::vector t;
    for(int i=0;i<argc;++i)
        t.push_back(std::string(argv[i]));
}

*/

/// current restriction, flags have to be separated from the argument
class OptionParser
{
public :
  inline OptionParser(int argc, char **argv);

  template <class T>
  inline const T getOption(const char *optName);

protected :
  std::vector<std::string > mArgV;
};

inline OptionParser::OptionParser(int argc, char **argv)
{
  for (int i = 0; i < argc; ++i)
  {
    mArgV.push_back(std::string(argv[i]));
  }
}

template <class T>
inline const T OptionParser::getOption(const char *optName)
{
  T r;
  for (unsigned int i = 0; i < mArgV.size() - 1; ++i)
  {
    //size_t pos = mArgV[i].find(optName);
    //if(pos != std::string::npos)
    if (mArgV[i].compare(optName) == 0)
    {
      std::stringstream ss(mArgV[i + 1]);
      ss >> r;
      break;
    }
  }
  return r;
}

#endif
