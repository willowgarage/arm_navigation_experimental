/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
*
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <arm_kinematics_constraint_aware/multi_arm_kinematics_constraint_aware.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <planning_environment/models/collision_models_interface.h>


namespace interpolated_ik_motion_planner 
{

class InterpolatedIKMotionPlanner
{
public:
  InterpolatedIKMotionPlanner(const std::vector<std::string> &group_names, 
                              const std::vector<std::string> &kinematics_solver_names,
                              const std::vector<std::string> &end_effector_link_names,
                              planning_environment::CollisionModelsInterface *collision_models_interface);
  InterpolatedIKMotionPlanner();

  ~InterpolatedIKMotionPlanner()
  {
    delete collision_models_interface_;
    delete kinematics_solver_;
  }

private:


  bool getConfigurationParams(std::vector<std::string> &group_names,
                              std::vector<std::string> &kinematics_solver_names,
                              std::vector<std::string> &end_effector_names);

  bool getPath(const std::vector<geometry_msgs::Pose> &start,
               const std::vector<geometry_msgs::Pose> &end,
               const arm_navigation_msgs::PlanningScene& planning_scene,
               const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
               const ros::Duration &max_time,
               trajectory_msgs::JointTrajectory &trajectory);

  bool getInterpolatedIKPath(const std::vector<std::vector<geometry_msgs::Pose> > &path,
                             const ros::Duration &max_time,
                             trajectory_msgs::JointTrajectory &trajectory);

  bool interpolateCartesian(const geometry_msgs::Pose &start,
                            const geometry_msgs::Pose &end,
                            const double &translation_resolution,
                            const double &rotation_resolution,
                            std::vector<geometry_msgs::Pose> &path,
                            const int &num_points_path);

  bool getNumPoints(const geometry_msgs::Pose &start,
                    const geometry_msgs::Pose &end,
                    const double &translation_resolution,
                    const double &rotation_resolution,
                    unsigned int &num_points);

  bool getNumPoints(const tf::Point &start_tf,
                    const tf::Point &end_tf,
                    const tf::Quaternion &start_rot_tf,
                    const tf::Quaternion &end_rot_tf,
                    const double &translation_resolution,
                    const double &rotation_resolution,
                    unsigned int &num_points);

  bool addToJointTrajectory(trajectory_msgs::JointTrajectory &trajectory,
                            const std::vector<std::vector<double> > &solution);
  
  planning_environment::CollisionModelsInterface *collision_models_interface_;
  bool collision_models_interface_generated_;
  ros::NodeHandle node_handle_;
  arm_kinematics_constraint_aware::MultiArmKinematicsConstraintAware* kinematics_solver_;

  unsigned int num_groups_;

  double pos_spacing_,rot_spacing_,consistent_angle_;
  int num_steps_,collision_check_resolution_,steps_before_abort_,collision_aware_,start_from_end_;

  double max_distance_;

};

}
