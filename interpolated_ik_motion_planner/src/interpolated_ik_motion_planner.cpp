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

#include <interpolated_ik_motion_planner/interpolated_ik_motion_planner.h>

namespace interpolated_ik_motion_planner 
{

InterpolatedIKMotionPlanner::InterpolatedIKMotionPlanner(const std::vector<std::string> &group_names, 
                                                         const std::vector<std::string> &kinematics_solver_names,
                                                         const std::vector<std::string> &end_effector_link_names,
                                                         planning_environment::CollisionModelsInterface *collision_models_interface):collision_models_interface_(collision_models_interface),collision_models_interface_generated_(false),node_handle_("~")
{
  ROS_DEBUG("Initializing interpolated ik motion planner");
  kinematics_solver_ = new arm_kinematics_constraint_aware::MultiArmKinematicsConstraintAware(group_names,kinematics_solver_names,end_effector_link_names);
  ROS_INFO("Initialized interpolated ik motion planner");
}

InterpolatedIKMotionPlanner::InterpolatedIKMotionPlanner():node_handle_("~")
{
  planning_environment::CollisionModelsInterface* collision_models_interface = new planning_environment::CollisionModelsInterface("robot_description");
  std::vector<std::string> group_names, kinematics_solver_names, end_effector_link_names;
  getConfigurationParams(group_names,kinematics_solver_names,end_effector_link_names);
  InterpolatedIKMotionPlanner(group_names,kinematics_solver_names,end_effector_link_names,collision_models_interface);
  collision_models_interface_generated_ = true;
}

bool InterpolatedIKMotionPlanner::getConfigurationParams(std::vector<std::string> &group_names,
                                                         std::vector<std::string> &kinematics_solver_names,
                                                         std::vector<std::string> &end_effector_names)
{
  XmlRpc::XmlRpcValue group_list;
  if(!node_handle_.getParam("groups", group_list))
  {
    ROS_ERROR("Could not find groups on param server");
    return false;
  }
  if(group_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
 {
    ROS_ERROR("Group list should be of XmlRpc Array type");
    return false;
  } 
  for (int32_t i = 0; i < group_list.size(); ++i) 
  {
    if(group_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      ROS_ERROR("Group names should be strings");
      return false;
    }
    group_names.push_back(static_cast<std::string>(group_list[i]));
    ROS_DEBUG("Adding group: %s",group_names.back().c_str());
    std::string kinematics_solver_name;
    if(!node_handle_.hasParam(group_names.back()+"/kinematics_solver"))
    {
      ROS_ERROR("Kinematics solver not defined for group %s in namespace %s",group_names.back().c_str(),node_handle_.getNamespace().c_str());
      // throw new OMPLROSException();
    }
    node_handle_.getParam(group_names.back()+"/kinematics_solver",kinematics_solver_name);
    kinematics_solver_names.push_back(kinematics_solver_name);

    std::string tip_name;
    if(!node_handle_.hasParam(group_names.back()+"/tip_name"))
    {
      ROS_ERROR("End effector not defined for group %s in namespace %s",group_names.back().c_str(),node_handle_.getNamespace().c_str());
      // throw new OMPLROSException();
    }
    node_handle_.getParam(group_names.back()+"/tip_name",tip_name);
    end_effector_names.push_back(tip_name);
  }
  node_handle_.param("num_steps",num_steps_,6);
  node_handle_.param("consistent_angle",consistent_angle_,M_PI/9.0);
  node_handle_.param("collision_check_resolution",collision_check_resolution_,1);
  node_handle_.param("steps_before_abort",steps_before_abort_,0);
  node_handle_.param("pos_spacing",pos_spacing_,0.01);
  node_handle_.param("rot_spacing",rot_spacing_,0.1);
  node_handle_.param("collision_aware",collision_aware_,1);
  node_handle_.param("start_from_end",start_from_end_,0);
  max_distance_ = consistent_angle_;
  return true;
};

bool InterpolatedIKMotionPlanner::getPath(arm_navigation_msgs::GetMotionPlan::Request &request,
                                          arm_navigation_msgs::GetMotionPlan::Response &response)
{
  std::vector<geometry_msgs::Pose> start, goal;
  arm_navigation_msgs::PlanningScene planning_scene;
  arm_navigation_msgs::OrderedCollisionOperations collision_operations;

  if(!getStart(request.motion_plan_request.start_state,start))
    return false;
  if(!getGoal(request.goal_constraints,goal))
    return false;

  if(!getPath(start,end,planning_scence,collision_operations,request.motion_planning_request.allowed_planning_time,response.trajectory))
    return false;

}

bool InterpolatedIKMotionPlanner::getStart(const arm_navigation_msgs::RobotState &robot_state,
                                           std::vector<geometry_msgs::Pose> start)
{
  for (unsigned int i=0; i < num_groups_; i++)
  {
    for(unsigned int j=0; j < robot_state.multi_dof_joint_state.poses.size(); j++)
    {
      if(robot_state.multi_dof_joint_state.child_frame_ids[j] == group_names_[i])
        start.push_back(robot_state.multi_dof_joint_state.poses[j]);
    }
    if(start.size() < (i+1))
    {
      ROS_ERROR("Could not find start state for group %s",group_names_[i].c_str());
      return false;
    }
  }
  return true;
}

bool InterpolatedIKMotionPlanner::getStart(const arm_navigation_msgs::RobotState &robot_state,
                                           std::vector<geometry_msgs::Pose> start)
{
  for (unsigned int i=0; i < num_groups_; i++)
  {
    for(unsigned int j=0; j < robot_state.multi_dof_joint_state.poses.size(); j++)
    {
      if(robot_state.multi_dof_joint_state.child_frame_ids[j] == group_names_[i])
        start.push_back(robot_state.multi_dof_joint_state.poses[j]);
    }
    if(start.size() < (i+1))
    {
      ROS_ERROR("Could not find start state for group %s",group_names_[i].c_str());
      return false;
    }
  }
  return true;
}

bool InterpolatedIKMotionPlanner::getPath(const std::vector<geometry_msgs::Pose> &start,
                                          const std::vector<geometry_msgs::Pose> &end,
                                          const arm_navigation_msgs::PlanningScene& planning_scene,
                                          const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                                          const ros::Duration &max_time,
                                          trajectory_msgs::JointTrajectory &solution)
{
  ros::Duration timeout(max_time);
  std::vector<std::vector<geometry_msgs::Pose> > path;
  path.resize(num_groups_);
  unsigned int max_num_points(0);
  for(unsigned int i=0; i < num_groups_; i++)
  {
    unsigned int num_points;
    getNumPoints(start[i],end[i],pos_spacing_,rot_spacing_,num_points);    
    max_num_points = std::max<unsigned int>(num_points,max_num_points);
  }

  for(unsigned int i=0; i < num_groups_; i++)
    interpolateCartesian(start[i],end[i],pos_spacing_,rot_spacing_,path[i],max_num_points);

  kinematics_solver_->setup(planning_scene,collision_operations);

  while(timeout.toSec() >= 0.0)
  {
    ros::Time start_time = ros::Time::now();
    if(getInterpolatedIKPath(path,timeout,solution))
    {
      timeout -= (ros::Time::now()-start_time);
      kinematics_solver_->clear();
      return true;
    }
    timeout -= (ros::Time::now()-start_time);
  }
  kinematics_solver_->clear();
  return false;
}                                      

bool InterpolatedIKMotionPlanner::getInterpolatedIKPath(const std::vector<std::vector<geometry_msgs::Pose> > &path,
                                                        const ros::Duration &max_time,
                                                        trajectory_msgs::JointTrajectory &trajectory)
{
  double timeout = max_time.toSec();
  std::vector<int> error_codes;
  trajectory.points.clear();
  while(timeout >= 0.0)
  {
    // Find initial collision free solutions for both arms
    std::vector<std::vector <double> > seed_states, solution_states;
    std::vector<geometry_msgs::Pose> poses;
    seed_states.resize(num_groups_);
    for(unsigned int i=0; i < num_groups_; i++)
      poses.push_back(path[i][0]);

    if(!(kinematics_solver_->searchConstraintAwarePositionIK(poses,timeout,seed_states,error_codes)))
      return false;

    //Now use those collision free solutions to try and find solutions that are close
    for(unsigned int i=1; i < path.size(); i++)
    {
      for(unsigned int j=0; j < num_groups_; j++)
        poses.push_back(path[j][i]);
      if(kinematics_solver_->searchConstraintAwarePositionIK(poses,seed_states,timeout,solution_states,error_codes,max_distance_))
      {
        addToJointTrajectory(trajectory,solution_states);
        /*if(checkMotion(seed_states,solution_states))
          {
           seed_states = solution_states;
           continue;
        }
        else
          return false;
        */
      }
      else
        return false;
    }
    return true;
  }
  return false;
}

bool InterpolatedIKMotionPlanner::addToJointTrajectory(trajectory_msgs::JointTrajectory &trajectory,
                                                       const std::vector<std::vector<double> > &solution)
{
  if(trajectory.joint_names.empty())
    trajectory.joint_names = kinematics_solver_->getJointNames();

  trajectory_msgs::JointTrajectoryPoint point;
  for(unsigned int i=0; i < solution.size(); i++)
    for(unsigned int j=0; j < solution[i].size(); j++)
      point.positions.push_back(solution[i][j]);

  return true;
}

bool InterpolatedIKMotionPlanner::interpolateCartesian(const geometry_msgs::Pose &start,
                                                       const geometry_msgs::Pose &end,
                                                       const double &translation_resolution,
                                                       const double &rotation_resolution,
                                                       std::vector<geometry_msgs::Pose> &path,
                                                       const int &num_points_path)
{
  tf::Point start_tf, end_tf, point_tf;
  tf::Quaternion start_rot_tf, end_rot_tf, point_rot_tf;

  tf::pointMsgToTF(start.position,start_tf);
  tf::pointMsgToTF(end.position,end_tf);
  tf::quaternionMsgToTF(start.orientation,start_rot_tf);
  tf::quaternionMsgToTF(end.orientation,end_rot_tf);

  geometry_msgs::Pose pose;
  path.clear();

  unsigned int num_points;
  if(num_points_path > 0)
    num_points = num_points_path;
  else
    getNumPoints(start_tf,end_tf,start_rot_tf,end_rot_tf,translation_resolution,rotation_resolution,num_points);

  for(unsigned int i=0; i < num_points; i++)
  {
    double fraction = ((double) i)/(num_points-1);
    point_tf = start_tf.lerp(end_tf,fraction);
    point_rot_tf = start_rot_tf.slerp(end_rot_tf,fraction);
    tf::pointTFToMsg(point_tf,pose.position);
    tf::quaternionTFToMsg(point_rot_tf,pose.orientation);
    path.push_back(pose);
  }
  return true;
}

bool InterpolatedIKMotionPlanner::getNumPoints(const geometry_msgs::Pose &start,
                                             const geometry_msgs::Pose &end,
                                             const double &translation_resolution,
                                             const double &rotation_resolution,
                                             unsigned int &num_points)
{
  tf::Point start_tf, end_tf, point_tf;
  tf::Quaternion start_rot_tf, end_rot_tf, point_rot_tf;

  tf::pointMsgToTF(start.position,start_tf);
  tf::pointMsgToTF(end.position,end_tf);
  tf::quaternionMsgToTF(start.orientation,start_rot_tf);
  tf::quaternionMsgToTF(end.orientation,end_rot_tf);

  getNumPoints(start_tf,end_tf,start_rot_tf,end_rot_tf,translation_resolution,rotation_resolution,num_points);
  return true;
}

bool InterpolatedIKMotionPlanner::getNumPoints(const tf::Point &start_tf,
                                             const tf::Point &end_tf,
                                             const tf::Quaternion &start_rot_tf,
                                             const tf::Quaternion &end_rot_tf,
                                             const double &translation_resolution,
                                             const double &rotation_resolution,
                                             unsigned int &num_points)
{
  double distance = (end_tf-start_tf).length();
  double angle = end_rot_tf.angleShortestPath(start_rot_tf);

  unsigned int num_steps_distance = distance/translation_resolution+1;
  unsigned int num_steps_angle = angle/rotation_resolution+1;

  num_points = std::max<unsigned int>(num_steps_distance,num_steps_angle)+1;
  return true;
}
}
