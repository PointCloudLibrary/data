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

#ifndef PCL_PEOPLE_BVHTRANSFORMS_H_
#define PCL_PEOPLE_BVHTRANSFORMS_H_

#include "libBVH.h"

#include <Eigen/Core>
#include <Eigen/Geometry> // for the transforms and stuff

namespace pcl
{
namespace people
{
namespace BVH
{
typedef Eigen::Matrix3f     Mat3;
typedef Eigen::Vector3f     Vec3;
typedef Eigen::Affine3f     Transform3;
typedef Eigen::Quaternionf  Quat;

/**
 * \brief Computes the positions of joints in the rest state
 */
void computeRestStateJointPos(const std::vector<bvhJoint> &joints,
                              std::vector<Vec3>           &jointPos);

/**
 * \brief Computes for each joint its 3D Rigid Transform given the value vector
 */
void computeTis(const std::vector<bvhJoint> &joints,
                const std::vector<Vec3>     &jointRestPos,
                const std::vector<float>    &values,
                std::vector<Transform3>     &Tis);

/**
 * \brief Accumulates for each joint its transform with that of its parent
 */
void computeTTis(const std::vector<bvhJoint>   &joints,
                 const std::vector<Transform3> &Tis,
                 std::vector<Transform3>       &TTis);

} // end namespace BVH
} // End namespace people
} // End namespace pcl
#endif
