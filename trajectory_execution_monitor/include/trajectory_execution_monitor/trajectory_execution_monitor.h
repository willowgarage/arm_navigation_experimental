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

#include <ros/ros.h>
#include <boost/function.hpp>

#include <trajectory_msgs/JointTrajectory.h>

#include <trajectory_execution_monitor/trajectory_recorder.h>
#include <trajectory_execution_monitor/trajectory_controller_handler.h>

#include <planning_environment/models/collision_models.h>

namespace trajectory_execution_monitor
{

struct TrajectoryExecutionRequest {
  
  std::string group_name_;
  std::string controller_name_;
  std::string recorder_name_;

  bool failure_ok_;
  bool test_for_close_enough_;

  double max_settling_velocity_epsilon_;
  double max_settling_time_;
  double max_joint_distance_;
  double failure_time_factor_;
  
  trajectory_msgs::JointTrajectory trajectory_;
  boost::function<void(const std::string& group_name)> callback_function_;
};

enum TrajectoryExecutionResult {

  NOT_ATTEMPTED = 0,
  SUCCEEDED,
  NO_RECORDER,
  NO_HANDLER,
  ALREADY_AT_GOAL,
  HANDLER_FAILED_ENTIRELY,
  HANDLER_REPORTS_FAILURE,
  HANDLER_REPORTS_FAILURE_BUT_OK,
  HANDLER_REPORTS_FAILURE_BUT_CLOSE_ENOUGH
};

struct TrajectoryExecutionData {

  TrajectoryExecutionResult result_;

  // stats
  ros::Duration time_;							// recorded
  ros::Duration time_to_settle_;		// overshoot
  double angular_distance_;					// recorded

  // trajectories
  trajectory_msgs::JointTrajectory recorded_trajectory_;
  trajectory_msgs::JointTrajectory overshoot_trajectory_;
};

struct TrajectoryExecutionDataVector : public std::vector<TrajectoryExecutionData> 
{

  void reset() {
    clear();
    last_attempted_trajectory_index_ = 0;
  }

  unsigned int last_attempted_trajectory_index_;
};

class TrajectoryExecutionMonitor
{

public:
  
  TrajectoryExecutionMonitor() : cm_("robot_description"){};

  void addTrajectoryRecorder(boost::shared_ptr<TrajectoryRecorder>& trajectory_recorder);

  void addTrajectoryControllerHandler(boost::shared_ptr<TrajectoryControllerHandler>& trajectory_controller_handler);

  void executeTrajectories(const std::vector<TrajectoryExecutionRequest>& to_execute,
                           const boost::function<bool(TrajectoryExecutionDataVector)>& done_callback);

protected:
  
  bool sendTrajectory(const TrajectoryExecutionRequest& ter);
  
  void trajectoryFinishedCallbackFunction(bool ok);

  bool closeEnough(const TrajectoryExecutionRequest& ter,
                   const TrajectoryExecutionData& ted);
  

  void compareLastRecordedToStart(const TrajectoryExecutionRequest& last_ter,
                                  const TrajectoryExecutionRequest& next_ter,
                                  const TrajectoryExecutionData& ted);


  boost::function<bool(TrajectoryExecutionDataVector)> result_callback_;
  TrajectoryExecutionDataVector execution_result_vector_;
  const std::vector<TrajectoryExecutionRequest>* execution_data_;
  unsigned int current_trajectory_index_;

  boost::shared_ptr<TrajectoryControllerHandler> last_requested_handler_;
  std::map<std::string, boost::shared_ptr<TrajectoryRecorder> > trajectory_recorder_map_;
  std::map<std::string, boost::shared_ptr<TrajectoryControllerHandler> > trajectory_controller_handler_map_;

  planning_environment::CollisionModels cm_;
  
};

}
