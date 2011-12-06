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

#include <trajectory_execution_monitor/trajectory_execution_monitor.h>

using namespace trajectory_execution_monitor;

void TrajectoryExecutionMonitor::addTrajectoryRecorder(boost::shared_ptr<TrajectoryRecorder>& trajectory_recorder) {
  trajectory_recorder_map_[trajectory_recorder->getName()] = trajectory_recorder;
}

void TrajectoryExecutionMonitor::addTrajectoryControllerHandler(boost::shared_ptr<TrajectoryControllerHandler>& trajectory_controller_handler) {
  trajectory_controller_handler_map_[trajectory_controller_handler->getGroupControllerComboName()] = 
    trajectory_controller_handler;
}

void TrajectoryExecutionMonitor::executeTrajectories(const std::vector<TrajectoryExecutionRequest>& to_execute,
                                                     const boost::function<bool(TrajectoryExecutionDataVector)>& done_callback) {
  
  execution_data_ = &to_execute;
  execution_result_vector_.reset();
  result_callback_ = done_callback;
  
  current_trajectory_index_ = 0;
  
  if(!sendTrajectory((*execution_data_)[current_trajectory_index_])) {
    result_callback_(execution_result_vector_);
  }
  
  //also register timer in case controller doesn't return
};

bool TrajectoryExecutionMonitor::sendTrajectory(const TrajectoryExecutionRequest& ter) {
  
  execution_result_vector_.resize(execution_result_vector_.size()+1);
  
  std::string recorder_name = ter.recorder_name_;
  if(trajectory_recorder_map_.size() == 1) {
    recorder_name = trajectory_recorder_map_.begin()->second->getName();
  }
     
  if(trajectory_recorder_map_.find(ter.recorder_name_) == trajectory_recorder_map_.end() && 
     trajectory_recorder_map_.size() > 1) {
    execution_result_vector_.back().result_ = NO_RECORDER;
    return false;
  } 
  
  std::string combo_name = TrajectoryControllerHandler::combineGroupAndControllerNames(ter.group_name_,
                                                                                       ter.controller_name_);
  
  if(trajectory_controller_handler_map_.find(combo_name) == trajectory_controller_handler_map_.end()) {
    execution_result_vector_.back().result_ = NO_HANDLER;
    return false;
  }
  
  boost::shared_ptr<TrajectoryRecorder>& requested_recorder = trajectory_recorder_map_.find(recorder_name)->second;
  last_requested_handler_ = trajectory_controller_handler_map_.find(combo_name)->second;
  
  trajectory_msgs::JointTrajectory traj = ter.trajectory_;
  traj.header.stamp = ros::Time::now();
  if(!last_requested_handler_->executeTrajectory(traj,
                                                 requested_recorder,
                                                 boost::bind(&TrajectoryExecutionMonitor::trajectoryFinishedCallbackFunction, this, _1))) {
    execution_result_vector_.back().result_ = HANDLER_FAILED_ENTIRELY;
  }
  return true;
}
  
void TrajectoryExecutionMonitor::trajectoryFinishedCallbackFunction(bool ok) {
  //adding this in any case
  execution_result_vector_.back().recorded_trajectory_ = last_requested_handler_->getLastRecordedTrajectory();
  if(ok || (*execution_data_)[current_trajectory_index_].failure_ok_ ||
     ((*execution_data_)[current_trajectory_index_].test_for_close_enough_ &&
      closeEnough((*execution_data_)[current_trajectory_index_],
                  execution_result_vector_.back())))
  {
    ROS_INFO_STREAM("Trajectory finished with ok");
    if(!ok) {
      if((*execution_data_)[current_trajectory_index_].failure_ok_) {
        execution_result_vector_.back().result_ = HANDLER_REPORTS_FAILURE_BUT_OK;
      } else {
        execution_result_vector_.back().result_ = HANDLER_REPORTS_FAILURE_BUT_CLOSE_ENOUGH;
      }
    } else {
      execution_result_vector_.back().result_ = SUCCEEDED;
    }
    current_trajectory_index_++;
    if(current_trajectory_index_ >= execution_data_->size()) {
      result_callback_(execution_result_vector_);
      return;
    }
    if(!sendTrajectory((*execution_data_)[current_trajectory_index_])) {
      result_callback_(execution_result_vector_);
    }
  } else {
    ROS_INFO_STREAM("Trajectory finished with failure");
    execution_result_vector_.back().result_ = HANDLER_REPORTS_FAILURE;
    result_callback_(execution_result_vector_);
  }
};

bool TrajectoryExecutionMonitor::closeEnough(const TrajectoryExecutionRequest& ter,
                                             const TrajectoryExecutionData& ted) {
  if(ted.recorded_trajectory_.points.size() == 0) {
    ROS_WARN_STREAM("No points in recorded trajectory");
    return false;
  }
  double total_distance = 0.0;
  for(unsigned int i = 0; i < ter.trajectory_.points.back().positions.size(); i++) {
    ROS_DEBUG_STREAM("Distance for " << ter.trajectory_.joint_names[i] << " is " << fabs(ter.trajectory_.points.back().positions[i]-ted.recorded_trajectory_.points.back().positions[i]));
    total_distance += fabs(ter.trajectory_.points.back().positions[i]-ted.recorded_trajectory_.points.back().positions[i]);
  }
  if(total_distance < ter.max_joint_distance_) {
    ROS_INFO_STREAM("Allowing because max distance low " << total_distance);
    return true;
  }
  ROS_INFO_STREAM("Not allowing because max distance high " << total_distance);
  return false;
}
