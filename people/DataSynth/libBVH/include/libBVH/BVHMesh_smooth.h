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

#ifndef PCL_PEOPLE_BVHMESH_SMOOTH_H_
#define PCL_PEOPLE_BVHMESH_SMOOTH_H_

#include "BVHTransforms.h"

namespace pcl
{
namespace people
{
namespace BVH
{

struct vertex_joint
{
  int boneId;
  float weight;
};

void loadMesh_VJ_smooth(const char                  *filename,
                        const std::vector<bvhJoint> &joints,
                        std::vector<vertex_joint>   &vjs,          // sorted by vertices
                        std::vector<int>            &vjs_bounds);  // NVertices+1 long, vjs_bounds[i] is begin offset for vertex i, vjs_bounds[i+1] is end offset


void transformMesh_smooth(const std::vector<Transform3>     &transforms,
                          const std::vector<vertex_joint>   &vjs,
                          const std::vector<int>            &vjs_bounds,
                          const std::vector<Vec3>           &X0,
                          std::vector<Vec3>                 &X);



/**
 * duplicate label borders.. so that the boundaries of labels will be nice and clean
 */
/*
void duplicateLabelBorders( const std::vector<Vec3>&        X0,
                      const std::vector<Triangle>&    tris0,
                      const std::vector<int>&         labels,
                      const std::vector<vertex_joint> vjs,
                      const std::vector<int>          vjs_bounds,
                      std::vector<Vec3>&              X0_d, // d stands for duplicated
                      std::vector<Vec3>&              tris0_d,
                      std::vector<Vec3>&              labeld_d,
                      std::vector<vertex_joint>&      vjs_d,
                      std::vector<int>&               vjs_bounds_d);

 */
} // End namespace BVH
} // End namespace people
} // End namespace pcl






















#endif
