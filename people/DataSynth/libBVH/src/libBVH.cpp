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

#include <libBVH/libBVH.h>


#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <stack>


namespace pcl
{
namespace people
{
namespace BVH
{


std::string bvh_ErrString(const std::string &msg, const std::string filename, int linenr)
{
  std::ostringstream oss;
  oss << "(E) bvhParse : " << msg << " ||  file " << filename << ":" << linenr;
  return oss.str();
}


// #################################
// #################################
// Helper functions
// #################################
// #################################
bool bvhparse_CONSTLINE(const std::string &line, const std::string &constline)
{
  std::istringstream ss(line);
  std::string CONSTLINE;
  ss >> CONSTLINE;
  return (!ss.fail()) && (CONSTLINE == constline);
}

bool bvhparse_JOINT(const std::string &line, bvhJointType &jointtype, std::string  &name)
{
  std::istringstream ss(line);
  std::string JOINTTYPE;
  ss >> JOINTTYPE >> name;
  if (ss.fail())
  {
    return false;
  }
  if (JOINTTYPE == "ROOT")
  {
    jointtype = BVH_ROOT;
    return true;
  }
  else if (JOINTTYPE == "JOINT")
  {
    jointtype = BVH_JOINT;
    return true;
  }
  else if (JOINTTYPE == "End")
  {
    jointtype = BVH_END;
    name = "End";
    return true;
  }
  return false;
}


bool bvhparse_OFFSET(const std::string &line, float &x, float &y, float &z)
{
  std::istringstream ss(line);
  std::string OFFSET;
  ss >> OFFSET;
  if (ss.fail() || OFFSET != "OFFSET")
  {
    return false;
  }
  ss >> x >> y >> z;
  return !(ss.fail());
}

bool bvhparse_CHANNELS(const std::string &line, bvhChannelType &type)
{
  std::istringstream ss(line);
  std::string CHANNELS;
  ss >> CHANNELS;
  if (ss.fail() || CHANNELS != "CHANNELS")
  {
    return false;
  }
  int numChans;
  ss >> numChans;
  if (ss.fail())
  {
    return false;
  }
  switch (numChans)
  {
  case 6 :
  {
    std::string channames[6];
    for (int i = 0; i < 6; ++i)
    {
      ss >> channames[i];
    }
    if (!ss.fail()
        && channames[0] == "Xposition"
        && channames[1] == "Yposition"
        && channames[2] == "Zposition"
        && channames[3] == "Zrotation"
        && channames[4] == "Yrotation"
        && channames[5] == "Xrotation")
    {
      type = BVH_XYZ_ZYX;
      return true;
    }
    return false;
    break;
  }
  case 3 :
  {
    std::string channames[3];
    for (int i = 0; i < 3; ++i)
    {
      ss >> channames[i];
    }
    if (!ss.fail()
        && channames[0] == "Zrotation"
        && channames[1] == "Yrotation"
        && channames[2] == "Xrotation")
    {
      type = BVH_ZYX;
      return true;
    }
    return false;
    break;
  }
  default :
    return false;
  }
}


bool bvhParse_Frames(const std::string &line, int &numFrames)
{
  std::istringstream iss(line);
  std::string Frames;
  iss >> Frames;
  if (iss.fail() || Frames != "Frames:")
  {
    return false;
  }
  iss >> numFrames;
  if (iss.fail())
  {
    numFrames = 0;
    return false;
  }
  return true;
}

bool bvhParse_FrameTime(const std::string &line, float &frameTime)
{
  std::istringstream iss(line);
  std::string buffer;
  iss >> buffer;
  if (iss.fail() || buffer != "Frame")
  {
    return false;
  }
  iss >> buffer;
  if (iss.fail() || buffer != "Time:")
  {
    return false;
  }
  iss >> frameTime;
  if (iss.fail())
  {
    frameTime = 1.0;
    return false;
  }
  return true;
}

bool bvhParse_FrameValues(const std::string &line, int numValues, std::vector<float> &values)
{
  values.resize(numValues);

  std::istringstream iss(line);
  for (int vi = 0; vi < numValues; ++vi)
  {
    iss >> values[vi];
  }
  if (iss.fail())
  {
    return false;
  }
  return true;
}

#define BVHEXCEPT(s) \
  throw ( std::runtime_error( bvh_ErrString(s, filename, lineCounter)) );

// #################################
// #################################
// Main Parsing function
// #################################
// #################################
bool bvhparseFile(const std::string                 &filename,
                  std::vector<bvhJoint>             &skel,
                  float                             &period,
                  std::vector<std::vector<float> >  &values)
{
  int lineCounter = 0;
  std::ifstream fin(filename.c_str());
  //if( !fin.is_open() ) BVHEXCEPT("could not open")
  if (!fin.is_open())
  {
    std::cout << std::endl << "(E): bvhparseFile() : could not open : " << filename << std::endl;
    return false;
  }
  skel.clear();

  // the buffers
  std::string  line;
  std::stack<int> parentStack;
  parentStack.push(-1);

  // Hierarchy
  getline(fin, line);
  lineCounter++;
  if (!bvhparse_CONSTLINE(line, "HIERARCHY"))
    BVHEXCEPT("malformed line: expected HIERARCHY")


    // ##############################
    // PARSING THE HIERACHY
    // ##############################
    while (true)
    {
      // ###############
      // we are expecting root or motion.. it can not be } or else parentstack
      // would be 2
      if (parentStack.size() == 1)
      {
        getline(fin, line);
        lineCounter++;
        // if its motion we break out of the loop and move to the next
        // ( Motion ) parsing section
        if (bvhparse_CONSTLINE(line, "MOTION"))
        {
          break;
        }
        // ---------------
        // Load the root
        bvhJointType type;
        bvhJoint     joint;
        if (!bvhparse_JOINT(line, type, joint.name) || type != BVH_ROOT)
          BVHEXCEPT("malformed line: expected ROOT")
          getline(fin, line);
        lineCounter++;
        if (!bvhparse_CONSTLINE(line, "{"))
          BVHEXCEPT("malformed line: expected {")
          getline(fin, line);
        lineCounter++;
        if (!bvhparse_OFFSET(line, joint.ox, joint.oy, joint.oz))
          BVHEXCEPT("malformed line: expected OFFSET x y z")
          getline(fin, line);
        lineCounter++;
        if (!bvhparse_CHANNELS(line, joint.ctype))
          BVHEXCEPT("malformed line: expected CHANNEL definition")
          // ---------------
          // save the root
          joint.parent = parentStack.top();
        parentStack.push(skel.size());
        skel.push_back(joint);
      }

      // ###############
      // we are expecting a joint or end site or a }
      else
      {
        // ---------------
        // if we are closing braces pop the parentstack
        getline(fin, line);
        lineCounter++;
        if (bvhparse_CONSTLINE(line, "}"))
        {
          parentStack.pop();
          continue;
        }
        // ---------------
        // Make sure we can handle the joint
        bvhJoint     joint;
        bvhJointType type;
        if (!bvhparse_JOINT(line, type, joint.name)
            || (type != BVH_JOINT && type != BVH_END))
          BVHEXCEPT("malformed line: expected JOINT or End Site")
          // ---------------
          // Process the joint depending on its type
          if (type == BVH_END)
          {
            getline(fin, line);
            lineCounter++;
            if (!bvhparse_CONSTLINE(line, "{"))
              BVHEXCEPT("malformed line: expected {")
              getline(fin, line);
            lineCounter++;
            if (!bvhparse_OFFSET(line, joint.ox, joint.oy, joint.oz))
              BVHEXCEPT("malformed line: expected OFFSET x y z")
              getline(fin, line);
            lineCounter++;
            if (!bvhparse_CONSTLINE(line, "}"))
              BVHEXCEPT("malformed line: expected }")
              joint.ctype = BVH_EMPTY;
            // write the endpoint
            joint.parent = parentStack.top();
            skel.push_back(joint);
          }
          else if (type == BVH_JOINT)
          {
            getline(fin, line);
            lineCounter++;
            if (!bvhparse_CONSTLINE(line, "{"))
              BVHEXCEPT("malformed line: expected {")
              getline(fin, line);
            lineCounter++;
            if (!bvhparse_OFFSET(line, joint.ox, joint.oy, joint.oz))
              BVHEXCEPT("malformed line: expected OFFSET x y z")
              getline(fin, line);
            lineCounter++;
            if (!bvhparse_CHANNELS(line, joint.ctype))
              BVHEXCEPT("malformed line: expected CHANNEL definition")
              // write the joint
              joint.parent = parentStack.top();
            parentStack.push(skel.size());
            skel.push_back(joint);
          }
      }
    }



  // ##############################
  // PARSING THE MOTION
  // ##############################
  int numJoints = skel.size();
  // 1 - evaluate the total size of parameters
  int paramSize = 0;
  for (int ji = 0; ji < numJoints; ++ji)
  {
    paramSize += bvhChannelSize[ skel[ji].ctype ];
  }

  // 2 - hopefully get motion
  if (!bvhparse_CONSTLINE(line, "MOTION"))
    BVHEXCEPT("malformed line: expected MOTION")

    // 3 - get number of frames
    int numFrames;
  getline(fin, line);
  lineCounter++;
  if (!bvhParse_Frames(line, numFrames))
    BVHEXCEPT("malformed line, expected Frames: numFrames")

    // 4- get rate
    getline(fin, line);
  lineCounter++;
  if (!bvhParse_FrameTime(line, period))
    BVHEXCEPT("malformed line, expected Frame Time: period")


    // 5 - getParams
    values.resize(numFrames);
  for (int fi = 0; fi < numFrames; ++fi)
  {
    getline(fin, line);
    lineCounter++;
    if (!bvhParse_FrameValues(line, paramSize, values[fi]))
      BVHEXCEPT("malformed value line")
    }

  return true;
}

}
}
} // end namespace BVH


#undef BVHEXCEPT

