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

#ifndef PCL_PEOPLE_BVHMESH_H_
#define PCL_PEOPLE_BVHMESH_H_

#include "BVHTransforms.h"

namespace pcl
{
namespace people
{
namespace BVH
{
struct Triangle
{
  int v0, v1, v2;
};

/**
 * \brief just load a triangle list
 */
void loadMesh_Tri(const char            *filename,
                  std::vector<Triangle> &tris);

/**
 * \brief load the vertex-joint associations
 */
void loadMesh_VJ(const char                  *filename,
                 const std::vector<bvhJoint> &joints,
                 std::vector<int>            &vjs);


/**
 * \brief load the vertex labels
 */
void loadMesh_labels(const char              *filename,
                     std::vector<int>        &labels);


/**
 * \brief load the vertices rest positions
 */
void loadMesh_Vertices(const char          *filename,
                       std::vector<Vec3>   &X0);

/**
 * \brief transform the vertices
 */
void transformMesh(const std::vector<Transform3> &transforms,
                   const std::vector<int>        &VJ,
                   const std::vector<Vec3>       &X0,
                   std::vector<Vec3>             &X);

} // end namespace BVH
} // End namespace people
} // End namespace pcl
#endif

