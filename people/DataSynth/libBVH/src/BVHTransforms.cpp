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

#include <libBVH/BVHTransforms.h>

namespace pcl
{
namespace people
{
namespace BVH
{

using Eigen::AngleAxisf;
using Eigen::Translation3f;

void computeRestStateJointPos(const std::vector<bvhJoint> &joints,
                              std::vector<Vec3>           &jointPos)
{
  int numJoints = joints.size();
  jointPos.resize(numJoints);

  for (int ji = 0; ji < numJoints; ++ji)
  {
    const bvhJoint &joint = joints[ji];
    const int parent      = joints[ji].parent;
    const Vec3 offset(joint.ox, joint.oy, joint.oz);
    if (parent == -1)
    {
      jointPos[ji] = offset;
    }
    else
    {
      jointPos[ji] = jointPos[parent] + offset;
    }
  }
}

void computeTis(const std::vector<bvhJoint> &joints,
                const std::vector<Vec3>     &jointRestPos,
                const std::vector<float>    &values,
                std::vector<Transform3>     &Tis)
{
  int numJoints = joints.size();
  Tis.resize(numJoints);

  // iterate through values
  std::vector<float>::const_iterator values_itr = values.begin();
  for (int ji = 0; ji < numJoints; ++ji)
  {
    const bvhJoint &joint = joints[ji];
    const Vec3 offset(joint.ox, joint.oy, joint.oz);
    switch (joints[ji].ctype)
    {
    case BVH_EMPTY:
    {
      Tis[ji] = Transform3::Identity();
      break;
    }
    case BVH_XYZ_ZYX:
    {
      float tx = *values_itr++;
      float ty = *values_itr++;
      float tz = *values_itr++;
      Vec3 t = Vec3(tx, ty, tz);
      float rz = (3.159 / 180.) **values_itr++;
      float ry = (3.159 / 180.) **values_itr++;
      float rx = (3.159 / 180.) **values_itr++;
      Quat R = AngleAxisf(rz, Vec3(0, 0, 1)) * AngleAxisf(ry, Vec3(0, 1, 0)) * AngleAxisf(rx, Vec3(1, 0, 0)) ;
      Tis[ji] = Translation3f(t) * Translation3f(jointRestPos[ji]) * R * Translation3f(-jointRestPos[ji]);
      break;
    }
    case BVH_ZYX:
    {
      float rz = (3.159 / 180.) **values_itr++;
      float ry = (3.159 / 180.) **values_itr++;
      float rx = (3.159 / 180.) **values_itr++;
      Quat R = AngleAxisf(rz, Vec3(0, 0, 1)) * AngleAxisf(ry, Vec3(0, 1, 0)) * AngleAxisf(rx, Vec3(1, 0, 0)) ;
      Tis[ji] = Translation3f(jointRestPos[ji]) * R * Translation3f(-jointRestPos[ji]);
      break;
    }
    default :
      assert(0);
      break;
    }
  }
}


void computeTTis(const std::vector<bvhJoint>   &joints,
                 const std::vector<Transform3> &Tis,
                 std::vector<Transform3>       &TTis)
{
  int numJoints = joints.size();
  assert(int(Tis.size()) == numJoints);
  TTis.resize(numJoints);

  for (int ji = 0; ji < numJoints; ++ji)
  {
    const int parent = joints[ji].parent;
    assert(parent < ji);
    if (parent == -1)
    {
      TTis[ji] = Tis[ji];
    }
    else
    {
      TTis[ji] = TTis[parent] * Tis[ji];
    }
  }
}

}
}
} // end namespace BVH
