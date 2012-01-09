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

#include <cmath>
#include <move_arm_warehouse/move_arm_utils.h>
#include <move_arm_warehouse/trajectory_stats.h>

using namespace std;
using namespace planning_scene_utils;
using namespace planning_models;

ros::Duration TrajectoryStats::getExecutionDuration()
{
  size_t tsize = trajectory_.points.size();
  if( tsize < 1 )
  {
    return ros::Duration(0,0);
  }

  trajectory_msgs::JointTrajectoryPoint& point1 = trajectory_.points[0];
  trajectory_msgs::JointTrajectoryPoint& point2 = trajectory_.points[tsize-1];
  return point2.time_from_start - point1.time_from_start;
}

double TrajectoryStats::getAngularDistance()
{
  double angular_diff_sum = 0.0;
  size_t tsize = trajectory_.points.size();

  // Loop through trajectory points
  for(unsigned int i=1; i<tsize; i++)
  {
    trajectory_msgs::JointTrajectoryPoint point1 = trajectory_.points[i-1];
    trajectory_msgs::JointTrajectoryPoint point2 = trajectory_.points[i];

    if(point1.positions.size() != point2.positions.size())
    {
      ROS_ERROR("Invalid Trajectory, the number of joints is inconsistent");
      return 0.0;
    }

    // Loop through all joints
    size_t num_pos = point1.positions.size();
    for(unsigned int j=0; j<num_pos; j++)
    {
      double angular_diff = point1.positions[j] - point2.positions[j];
      angular_diff_sum += abs(angular_diff);
    }
  }

  return angular_diff_sum;
}

double TrajectoryStats::getCartesianDistance(planning_scene_utils::MotionPlanRequestData& motion_plan_req)
{
  double cartesian_distance_sum = 0.0;
  planning_models::KinematicState* kin_state = motion_plan_req.getStartState();
  KinematicState::JointStateGroup* joint_state_group = kin_state->getJointStateGroup(motion_plan_req.getGroupName());

  // Loop through trajectory points
  size_t tsize = trajectory_.points.size();
  for(unsigned int i=1; i<tsize; i++)
  {
    // Get position of end-effector at the first point of the trajectory.
    trajectory_msgs::JointTrajectoryPoint point1 = trajectory_.points[i-1];
    if( !joint_state_group->setKinematicState(point1.positions) )
    {
      ROS_ERROR("Mismatch in number of joints.");
      return 0.0;
    }
    joint_state_group->updateKinematicLinks();
    KinematicState::LinkState* end_effector_state1 = kin_state->getLinkState(motion_plan_req.getEndEffectorLink());
    const tf::Transform& end_effector_transform1 = end_effector_state1->getGlobalLinkTransform();
    const tf::Vector3 end_effector_location1 = end_effector_transform1.getOrigin();

    // Get position of end-effector at the next point of the trajectory.
    trajectory_msgs::JointTrajectoryPoint point2 = trajectory_.points[i];
    if( !joint_state_group->setKinematicState(point2.positions) )
    {
      ROS_ERROR("Mismatch in number of joints.");
      return 0.0;
    }
    joint_state_group->updateKinematicLinks();
    KinematicState::LinkState* end_effector_state2 = kin_state->getLinkState(motion_plan_req.getEndEffectorLink());
    const tf::Transform& end_effector_transform2 = end_effector_state2->getGlobalLinkTransform();
    const tf::Vector3 end_effector_location2 = end_effector_transform2.getOrigin();

    // Calculate
    double cartesian_distance = end_effector_location1.distance(end_effector_location2);
    cartesian_distance_sum += abs(cartesian_distance);
  }

  return cartesian_distance_sum;
}

/*
double TrajectoryStats::getClearanceDistance(planning_scene_utils::PlanningSceneData& scene)
{
  return 0;
}
*/

double TrajectoryStats::getMaxAngularError(trajectory_msgs::JointTrajectory& trajectory_error)
{
  double max_error = 0.0;

  // Loop through trajectory points
  size_t tsize = trajectory_error.points.size();
  for(unsigned int i=0; i<tsize; i++)
  {
    trajectory_msgs::JointTrajectoryPoint& point = trajectory_error.points[i];

    // Loop through all joints
    size_t num_pos = point.positions.size();
    for(unsigned int j=0; j<num_pos; j++)
    {
      double error = abs(point.positions[j]);
      if(error>max_error)
      {
        max_error = error;
      }
    }
  }

  return max_error;
}
