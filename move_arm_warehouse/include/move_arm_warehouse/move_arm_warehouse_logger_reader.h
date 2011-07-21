/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *
 *  \author E. Gil Jones
 *********************************************************************/

#ifndef MOVE_ARM_WAREHOUSE_LOGGER_H_
#define MOVE_ARM_WAREHOUSE_LOGGER_H_

#include <mongo_ros/message_collection.h>

#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/MotionPlanRequest.h>
#include <arm_navigation_msgs/ArmNavigationErrorCodes.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <head_monitor_msgs/HeadMonitorFeedback.h>

namespace move_arm_warehouse
{

class MoveArmWarehouseLoggerReader {
  
public:

  MoveArmWarehouseLoggerReader();

  ~MoveArmWarehouseLoggerReader();

  ///
  /// LOGGING FUNCTIONS
  ///

  void pushPlanningSceneToWarehouse(const arm_navigation_msgs::PlanningScene planning_scene);

  void pushMotionPlanRequestToWarehouse(const arm_navigation_msgs::PlanningScene& planning_scene,
                                        const std::string& stage_name,
                                        const arm_navigation_msgs::MotionPlanRequest& motion_plan_request,
                                        const std::string& ID = "MPR 1");
    
  void pushJointTrajectoryToWarehouse(const arm_navigation_msgs::PlanningScene& planning_scene,
                                      const std::string& trajectory_source,
                                      const ros::Duration& production_time, 
                                      const trajectory_msgs::JointTrajectory& trajectory,
                                      const std::string& ID = "Trajectory 1",
                                      const std::string& motion_plan_ID = "MPR 1");
    
  void pushOutcomeToWarehouse(const arm_navigation_msgs::PlanningScene& planning_scene,
                              const std::string& pipeline_stage,
                              const arm_navigation_msgs::ArmNavigationErrorCodes& error_codes,
                              const std::string& trajectory_ID = "Trajectory 1");
  
  void pushPausedStateToWarehouse(const arm_navigation_msgs::PlanningScene& planning_scene,
                                  const head_monitor_msgs::HeadMonitorFeedback& feedback);

  ///
  /// READING FUNCTIONS
  ///

  void getAvailablePlanningSceneList(const std::string& hostname, std::vector<ros::Time>& creation_times);

  bool getPlanningScene(const std::string& hostname, const ros::Time& time, 
                        arm_navigation_msgs::PlanningScene& planning_scene,std::string& hostnameOut);

  bool getAssociatedOutcomes(const std::string& hostname,
                             const ros::Time& time,
                             std::vector<std::string>& pipeline_names,
                             std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& error_codes,
                             std::vector<std::string>& trajectory_IDs);

  bool getAssociatedMotionPlanRequestsStageNames(const std::string& hostname, 
                                                 const ros::Time& time,
                                                 std::vector<std::string>& stage_names);

  bool getAssociatedMotionPlanRequest(const std::string& hostname, 
                                      const ros::Time& time,
                                      const std::string& stage_name,
                                      arm_navigation_msgs::MotionPlanRequest& request,
                                      std::string& ID_out);

  bool getAssociatedMotionPlanRequests(const std::string& hostname,
                                       const ros::Time& time,
                                       std::vector<std::string>& stage_names,
                                       std::vector<std::string>& IDs,
                                       std::vector<arm_navigation_msgs::MotionPlanRequest>& requests);

  bool getAssociatedJointTrajectorySources(const std::string& hostname, 
                                           const ros::Time& time,
                                           std::vector<std::string>& trajectory_sources);

  bool getAssociatedJointTrajectory(const std::string& hostname, 
                                    const ros::Time& time,
                                    const std::string& trajectory_source,
                                    const unsigned int& trajectory_index,
                                    ros::Duration& processing_time, 
                                    trajectory_msgs::JointTrajectory& joint_trajectory);

  bool getAssociatedJointTrajectories(const std::string& hostname,
                                      const ros::Time& time,
                                      const std::string& motion_plan_ID,
                                      std::vector<trajectory_msgs::JointTrajectory>& trajectories,
                                      std::vector<std::string>& sources,
                                      std::vector<std::string>& IDs,
                                      std::vector<ros::Duration>& durations);

  bool getAssociatedPausedStates(const std::string& hostname, 
                                 const ros::Time& time,
                                 std::vector<ros::Time>& paused_times);

  bool getAssociatedPausedState(const std::string& hostname, 
                                const ros::Time& planning_time, 
                                const ros::Time& paused_time,
                                head_monitor_msgs::HeadMonitorFeedback& paused_state);

protected:

  mongo_ros::Metadata initializeMetadataWithHostname();

  void addPlanningSceneTimeToMetadata(const arm_navigation_msgs::PlanningScene& planning_scene, mongo_ros::Metadata& metadata);

  mongo_ros::Query makeQueryForPlanningSceneTime(const ros::Time& time);

  mongo_ros::MessageCollection<arm_navigation_msgs::PlanningScene>* planning_scene_collection_;
  mongo_ros::MessageCollection<arm_navigation_msgs::MotionPlanRequest>* motion_plan_request_collection_;
  mongo_ros::MessageCollection<trajectory_msgs::JointTrajectory>* trajectory_collection_;
  mongo_ros::MessageCollection<arm_navigation_msgs::ArmNavigationErrorCodes>* outcome_collection_;
  mongo_ros::MessageCollection<head_monitor_msgs::HeadMonitorFeedback>* paused_state_collection_;
  
  std::string hostname_;
  
};

}
#endif
