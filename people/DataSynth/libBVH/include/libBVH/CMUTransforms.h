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

// THIS IS KIND OF AN UGLY HACK
// This should be data...
// For now please make sure that you include libBVH before calling this
#ifndef PCL_PEOPLE_BVHTRANSFORMS_H_
#error "Please include BVHTransforms.h before this header"
#endif

#include <Eigen/Core>
#include <Eigen/Geometry> // for the transforms and stuff

namespace pcl
{
namespace people
{
namespace BVH
{
using namespace Eigen;

/**
 * \brief applies the stupid scaling factor for the CMU Mocap database.
 * grounds the origin of skel to world origin.
 * orients the hips so that Z is up and they point to Y-.
 */
void CMUScaleResetHips(const std::vector<bvhJoint>   &joints,
                       std::vector<Transform3>       &Tis)
{
  for (size_t ji = 0; ji < joints.size(); ++ji)
  {
    if (joints[ji].parent == -1)
    {
      Tis[ji] = Transform3::Identity();
      Tis[ji].scale(0.056444);
    }
  }
}

bool CMUisStanding(const std::vector<bvhJoint>   &joints,
                   const std::vector<Transform3> &Tis)
{
  for (size_t ji = 0; ji < joints.size(); ++ji)
  {
    if (joints[ji].parent == -1)
    {
      Vec3 upward = Tis[ji].linear() * Vec3(0, 1, 0);
      if (upward[1] > 0.85)
      {
        return true;  // 0.85 is 30 deg
      }
    }
  }
  return false;
}

/**
 * \brief applies the stupid scaling factor for the CMU Mocap database.
 * grounds the origin of skel to world origin.
 * orients the hips so that they mostly point to Y-
 */
static inline void CMU_scaleRotateHips(const std::vector<bvhJoint>   &joints,
                                       std::vector<Transform3>       &Tis,
                                       double                         angle = 0.)
{
  for (size_t ji = 0; ji < joints.size(); ++ji)
  {
    if (joints[ji].parent == -1)
    {
      // important to take the linear part so we dont translate
      Vec3 Z = Tis[ji].linear() * Vec3(0, 0, 1);
      float norm = sqrtf(Z[0] * Z[0] + Z[2] * Z[2]);
      float costheta = Z[2] / norm;
      float sintheta = Z[0] / norm;
      float theta = atan2(sintheta, costheta);


      // this will rotate the model in its original pose
      // ( head up Y, Hips facing a vector in X,Z plane )
      // so that it points towards the Z plane
      // then add "angle" to it
      Transform3 R2(AngleAxisf(-theta + angle, Vec3(0, 1, 0)));
      Transform3 S(Eigen::UniformScaling<float>(0.056444f));
      Tis[ji] = R2 * S * Tis[ji];

      // we do not want to create an offset in the
      // vertical direction ( Y+ is up in cmu)
      Vec3 centeroffset = Tis[ji].translation();
      centeroffset(1) = 0;
      Tis[ji] = Translation3f(-centeroffset) * Tis[ji];
    }
  }
}
} // End namespace BVH
} // End namespace people
} // End namespace pcl
