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
 */

/** \author Jia Pan */

#ifndef COLLISION_CHECKING_RSS_H
#define COLLISION_CHECKING_RSS_H

#include "collision_checking/BVH_defs.h"

namespace collision_checking
{

class RSS
{
public:
  /** \brief Orientation of RSS */
  Vec3f axis[3];

  /** \brief position of rectangle */
  Vec3f Tr;

  /** \brief side lengths of rectangle */
  BVH_REAL l[2];

  /** \brief radius of sphere summed with rectangle to form RSS */
  BVH_REAL r;

  RSS() {}

  /** \brief Check collision between two RSS */
  bool overlap(const RSS& other) const;

  /** \brief Check collision between two RSS and return the overlap part.
   * For RSS, we return nothing, as the overlap part of two RSSs usually is not a RSS
   */
  bool overlap(const RSS& other, RSS& overlap_part) const
  {
    return overlap(other);
  }

  /** \brief Check whether the RSS contains a point */
  inline bool contain(const Vec3f& p) const;

  /** \brief A simple way to merge the RSS and a point, not compact. */
  inline RSS& operator += (const Vec3f& p);

  /** \brief Merge the RSS and another RSS */
  inline RSS& operator += (const RSS& other)
  {
    *this = *this + other;
    return *this;
  }

  /** \brief Return the merged RSS of current RSS and the other one */
  inline RSS operator + (const RSS& other) const;

  /** \brief Width of the RSS */
  inline BVH_REAL width() const
  {
    return l[0] + 2 * r;
  }

  /** \brief Height of the RSS */
  inline BVH_REAL height() const
  {
    return l[1] + 2 * r;
  }

  /** \brief Depth of the RSS */
  inline BVH_REAL depth() const
  {
    return 2 * r;
  }

  /** \brief Volume of the RSS */
  inline BVH_REAL volume() const
  {
    return (l[0] * l[1] * 2 * r + 4 * 3.1415926 * r * r * r);
  }

  /** \brief Size of the RSS, for split order */
  inline BVH_REAL size() const
  {
    return (sqrt(l[0] * l[0] + l[1] * l[1]) + 2 * r);
  }

  /** \brief The RSS center */
  inline Vec3f center() const
  {
    return Tr;
  }
};

}


#endif
