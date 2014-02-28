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

/*

#include <libBVH/libBVH.h>
#include <fstream>


namespace BVH
{


void buildChildVec( const std::vector<bvhJoint>& joints,
                    std::vector<int>             child,
                    std::vector<int>             child_bounds )
{

  int numJoints = joints.size();
  child_bounds.resize(numJoints+1);

  // compute the bounds vector
  std::fill(child_bounds.begin(), child_bounds.end(), 0);
  for(int ji=0;ji<numJoints;++ji) {
    int parent = joints[ji].parent;
    if( parent != -1 ) child_bounds[parent+1]++;
  }
  std::vector<int> child_bounds(numJoints+1, 0);
    for(int ji=0;ji<numJoints;++ji) {
    child_bounds[pi+1] += child_bounds[pi];
  }


  // compute the child vector
  child.resize( child_bounds[numJoints] );
  std::vector<int> offset(child_bounds);
  for(int ji=0;ji<numJoints;++ji) {
    int parent = joint[ji].parent;
    if( parent != -1 ) {
      child[offset[parent]] = ji;
      offset[parent]++;
    }
  }
}


std::vector<int> findRoots( const std::vector<bvhJoint>& joints ) {
  int numJoints = joints.size();
  std::vector<int> res;
  for(int ji=0;ji<numJoints;++ji) {
    if( joints[ji].parent == -1 ) res.push_back(ji);
  }
}








bool bvhwriteHierarchy( const std::string&            filename,
                        const std::vector<bvhJoint>&  joints )
{

  std::ofstream fout(filename.c_str() );
  if( !fout.is_open() ) throw std::runtime_error( std::string("could not open: ") + filename );

  int numJoints = joints.size();

  // another representation of the graph
  std::vector<int> child;
  std::vector<int> child_bounds;
  buildChildVec(joints, child, child_bounds);

  // find the roots
  std::vector<int> roots = findRoots(joints);


  for( std::vector<int>::const_iterator root_itr = roots.begin(); root_itr != roots.end(); ++roots ) {

  }

  return true;
}



} // end namespace BVH

*/
