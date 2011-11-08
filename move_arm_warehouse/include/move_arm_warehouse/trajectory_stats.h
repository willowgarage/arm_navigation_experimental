/*********************************************************************
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 *********************************************************************/

/* \author: Ken Anderson */

#ifndef TRAJECTORY_STATS_H
#define TRAJECTORY_STATS_H

#include <move_arm_warehouse/move_arm_utils.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace planning_scene_utils
{

////
/// class TrajectoryStats
/// @PlanningStats analyzes a trajectory and returns statistics.
/// Some of these statistics require extra information, such as the planning scene, and/or the motion plan request.
/// All new statistics should be added to this class (or subclasses).
////
class TrajectoryStats
{
protected:
  trajectory_msgs::JointTrajectory& trajectory_;

public:
  TrajectoryStats(trajectory_msgs::JointTrajectory& trajectory) : trajectory_(trajectory) {}

  /// @brief returns the (estimated/actual) duration of the trajectory (time between start and goal points).
  /// Note, if the trajectory does not have valid timestamps, the return value will probably be zero duration.
  ros::Duration getExecutionDuration();

  /// @brief returns the sum of the angular movements (in radians) by every joint in the arm as it moves through the trajectory.
  /// Linear interpolation is used to calculate this metric.
  double getAngularDistance();

  /// @brief Returns the cartesian distance (in meters) travelled by the end effector as it moves through the trajectory.
  /// Linear interpolation is used to calculate this metric.
  double getCartesianDistance(planning_scene_utils::MotionPlanRequestData& motion_plan_req);

  // / @brief returns the minimal clearance distance (in meters) of the arm as it moves through the trajectory.
  // / Linear interpolation is used to calculate this metric.
  //double getClearanceDistance(planning_scene_utils::PlanningSceneData& scene);

  /// @brief returns the maximum angular error over all joints for all points on the trajectory
  /// This is useful for determining how well the controller executed the trajectory.
  double getMaxAngularError(trajectory_msgs::JointTrajectory& trajectory_error);

};

};

#endif // TRAJECTORY_STATS_H
