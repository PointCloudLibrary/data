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

#include <libBVH/BVHMesh_smooth.h>
#include <fstream>
#include <stdexcept>
#include <sstream>
#include <map>



namespace pcl
{
namespace people
{
namespace BVH
{


void loadMesh_VJ_smooth(const char                  *filename,
                        const std::vector<bvhJoint> &joints,
                        std::vector<vertex_joint>   &vjs,          // sorted by vertices
                        std::vector<int>            &vjs_bounds)   // NVertices+1 long, vjs_bounds[i] is begin offset for vertex i, vjs_bounds[i+1] is end offset
{

  std::ifstream fin(filename);
  if (!fin.is_open())
  {
    throw (std::runtime_error(std::string("(E)could not open: ") + std::string(filename)));
  }


  // prepare the name map
  int numJoints = joints.size();
  std::map<std::string, int> name_2_id;
  for (int ji = 0; ji < numJoints; ++ji)
  {
    name_2_id[joints[ji].name] = ji;
  }




  std::string line;
  int numVertices;
  // get the first vertex
  {
    getline(fin, line);
    std::istringstream iss(line);
    iss >> numVertices;
  }

  // resize the arrays
  vjs.clear();
  vjs_bounds.resize(numVertices + 1);
  vjs_bounds[0] = 0;

  // read from the file
  for (int vi = 0; vi < numVertices; ++vi)
  {
    getline(fin, line);
    std::istringstream iss(line);
    while (1)
    {
      vertex_joint vj;
      std::string name;
      float weight;
      iss >> name >> weight;
      if (iss.fail())
      {
        break;  // we hit the end of the line
      }

      std::map<std::string, int>::const_iterator f_itr = name_2_id.find(name);
      if (f_itr == name_2_id.end())
      {
        throw (std::runtime_error(std::string("(E) apparently mismatched joints and .bones files") + std::string(filename)));
      }
      vj.boneId = f_itr->second; // the index
      vj.weight = weight;
      vjs.push_back(vj);
    }
    if (int(vjs.size()) == vjs_bounds[vi])
    {
      throw (std::runtime_error(std::string("(E) malformed .bones file, a vertex has no parent: ") + std::string(filename)));
    }
    vjs_bounds[vi + 1] = vjs.size();
  }



  // check that it makes sense
  for (int vi = 0; vi < numVertices; ++vi)
  {
    std::vector<vertex_joint>::const_iterator vj_itr = vjs.begin() + vjs_bounds[vi];
    std::vector<vertex_joint>::const_iterator vj_end = vjs.begin() + vjs_bounds[vi + 1];
    float sumW = 0.;
    while (vj_itr != vj_end)
    {
      sumW += vj_itr->weight;
      vj_itr++;
    }

    if (fabs(1.0 - sumW) > 0.001)
    {
      throw (std::runtime_error(std::string("(E) malformed .bones file, a vertex has weights that do not sum to 1: ") + filename));
    }
  }
}







void transformMesh_smooth(const std::vector<Transform3>     &transforms,
                          const std::vector<vertex_joint>   &vjs,
                          const std::vector<int>            &vjs_bounds,
                          const std::vector<Vec3>           &X0,
                          std::vector<Vec3>                 &X)
{
  int numVertices = X0.size();

  // resize target
  X.resize(numVertices);
  Vec3 zero(0, 0, 0);
  std::fill(X.begin(), X.end(), zero);

  for (int vi = 0; vi < numVertices; ++vi)
  {
    std::vector<vertex_joint>::const_iterator       vj_itr = vjs.begin() + vjs_bounds[vi];
    const std::vector<vertex_joint>::const_iterator vj_end = vjs.begin() + vjs_bounds[vi + 1];

    while (vj_itr != vj_end)
    {
      X[vi] += transforms[vj_itr->boneId] * X0[vi] * vj_itr->weight ;
      vj_itr++;
    }
  }
}


















// ######################################
// ######################################
//  The duplicate border mess
// ######################################
// ######################################


/*



void duplicateLabelBorders( const std::vector<Triangle>&    tris0,
                            const std::vector<int>&         labels,
                            std::vector<int>&               ids_d, //ids of the new vertices in the old array
                            std::vector<Triangle>&          tris0_d )
{
  const int numVertices = labels.size();
  const int numTriangles = tris0.size();

  tris0_d = tris0;
  ids_d = std::vector<int>(numVertices);
  for(int vi=0;vi<numVertices;++vi) ids_d[vi] = vi; // simply copy the indices for the original vertices ( that will stay)

  // go through the triangles
  int numTriangles = tris0.size();
  for(int ti=0;ti<numTriangles;++ti)
  {
    const Triangle& t = tris0[ti];

  }

}


void duplicateLabelBorders( const std::vector<Vec3>&        X0,
                            const std::vector<Triangle>&    tris0,
                            const std::vector<int>&         labels,
                            const std::vector<vertex_joint> vjs,
                            const std::vector<int>          vjs_bounds,
                            std::vector<Vec3>&              X0_d, // d stands for duplicated
                            std::vector<Vec3>&              tris0_d,
                            std::vector<Vec3>&              labeld_d,
                            std::vector<vertex_joint>&      vjs_d,
                            std::vector<int>&               vjs_bounds_d)
{
  // we start by copying everything
  X0_d     = X0;
  tris0_d  = tris0;
  labels_d = labels;
  vjs_d    = vjs
  vjs_bounds_d = vjs_bounds;


  // now we split triangles where needed



}

*/


}
}
}// end namespace BVH
