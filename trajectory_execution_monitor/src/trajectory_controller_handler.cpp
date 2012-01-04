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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

/** \author E. Gil Jones */

#include <trajectory_execution_monitor/trajectory_controller_handler.h>

using namespace trajectory_execution_monitor;

bool TrajectoryControllerHandler::addNewStateToRecordedTrajectory(const ros::Time& time,
                                                                  const std::map<std::string, double>& joint_positions,
                                                                  const std::map<std::string, double>& joint_velocities)
{

  // FIXME-- hack-- replace > XX with actual condition
  if( controller_state_ == OVERSHOOTING && overshoot_trajectory_.points.size() > 10 )
  {
    controller_state_ = IDLE;
    recorder_->delayedDeregisterCallback(group_controller_combo_name_);
    trajectory_finished_callback_(success_);
    return false;
  }

  if( controller_state_ == EXECUTING )
  {
    return _addNewStateToTrajectory(time, joint_positions, joint_velocities, recorded_trajectory_);
  }
  else if( controller_state_ == OVERSHOOTING )
  {
    return _addNewStateToTrajectory(time, joint_positions, joint_velocities, overshoot_trajectory_);
  }
  return false;
}

bool TrajectoryControllerHandler::_addNewStateToTrajectory(const ros::Time& time,
                                                           const std::map<std::string, double>& joint_positions,
                                                           const std::map<std::string, double>& joint_velocities,
                                                           trajectory_msgs::JointTrajectory& trajectory)
{
  ros::Time start_time;

  trajectory_msgs::JointTrajectoryPoint p;
  for(unsigned int i = 0; i < trajectory.joint_names.size(); i++) {
    const std::string& jn = trajectory.joint_names[i];
    if(joint_positions.find(jn) == joint_positions.end()) {
      return false;
    }
    p.positions.push_back(joint_positions.at(jn));
    if(joint_velocities.find(jn) == joint_velocities.end()) {
      p.velocities.push_back(joint_velocities.at(jn));
    }
    p.time_from_start = time-trajectory.header.stamp;
  }
  trajectory.points.push_back(p);

  return true;
}

void TrajectoryControllerHandler::initializeRecordedTrajectory(const trajectory_msgs::JointTrajectory& goal_trajectory)
{
  goal_trajectory_ = goal_trajectory;

  recorded_trajectory_.header.stamp = ros::Time::now();
  recorded_trajectory_.joint_names = goal_trajectory.joint_names;
  recorded_trajectory_.points.clear();

  controller_state_ = EXECUTING;
}

void TrajectoryControllerHandler::initializeOvershootTrajectory()
{
  overshoot_trajectory_.header.stamp = ros::Time::now();
  overshoot_trajectory_.joint_names = goal_trajectory_.joint_names;
  overshoot_trajectory_.points.clear();

  controller_state_ = OVERSHOOTING;
}
