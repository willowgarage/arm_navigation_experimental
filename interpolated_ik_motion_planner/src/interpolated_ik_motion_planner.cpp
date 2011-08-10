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

static const std::string DISPLAY_PATH_PUB_TOPIC  = "display_path";

InterpolatedIKMotionPlanner::InterpolatedIKMotionPlanner(const std::vector<std::string> &group_names, 
                                                         const std::vector<std::string> &kinematics_solver_names,
                                                         const std::vector<std::string> &end_effector_link_names,
                                                         planning_environment::CollisionModelsInterface *collision_models_interface):node_handle_("~"),planning_visualizer_(DISPLAY_PATH_PUB_TOPIC)
{
  if(!initialize(group_names,kinematics_solver_names,end_effector_link_names,collision_models_interface))
    throw new MultiArmKinematicsException();
  collision_models_interface_generated_ = false;
}

InterpolatedIKMotionPlanner::InterpolatedIKMotionPlanner():node_handle_("~"),planning_visualizer_(DISPLAY_PATH_PUB_TOPIC)
{
  planning_environment::CollisionModelsInterface* collision_models_interface = new planning_environment::CollisionModelsInterface("robot_description");
  std::vector<std::string> group_names, kinematics_solver_names, end_effector_link_names;
  getConfigurationParams(group_names,kinematics_solver_names,end_effector_link_names);
  if(!initialize(group_names,kinematics_solver_names,end_effector_link_names,collision_models_interface))
    throw new MultiArmKinematicsException();
  collision_models_interface_generated_ = true;
}

bool InterpolatedIKMotionPlanner::initialize(const std::vector<std::string> &group_names, 
                                             const std::vector<std::string> &kinematics_solver_names,
                                             const std::vector<std::string> &end_effector_link_names,
                                             planning_environment::CollisionModelsInterface *collision_models_interface)
{
  collision_models_interface_ = collision_models_interface;
  group_names_ = group_names;
  end_effector_link_names_ = end_effector_link_names;
  ROS_DEBUG("Initializing interpolated ik motion planner");
  try
  {  
    kinematics_solver_ = new arm_kinematics_constraint_aware::MultiArmKinematicsConstraintAware(group_names,kinematics_solver_names,end_effector_link_names,collision_models_interface);
  }
  catch (MultiArmKinematicsException &e)
  {
    ROS_ERROR("Could not initialize kinematics solver");
    return false;
  }
  plan_path_service_ = node_handle_.advertiseService("plan_kinematic_path", &InterpolatedIKMotionPlanner::computePlan, this);
  end_effector_pose_publisher_= node_handle_.advertise<visualization_msgs::Marker>("end_effector_pose", 128);
  end_effector_pose_array_publisher_= node_handle_.advertise<visualization_msgs::MarkerArray>("end_effector_pose_array", 128);
  ROS_INFO("Initialized interpolated ik motion planner");
  return true;
}

bool InterpolatedIKMotionPlanner::getConfigurationParams(std::vector<std::string> &group_names,
                                                         std::vector<std::string> &kinematics_solver_names,
                                                         std::vector<std::string> &end_effector_link_names)
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
    end_effector_link_names.push_back(tip_name);
  }
  num_groups_ = group_names.size();
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

bool InterpolatedIKMotionPlanner::computePlan(arm_navigation_msgs::GetMotionPlan::Request &request,
                                              arm_navigation_msgs::GetMotionPlan::Response &response)
{
  std::vector<geometry_msgs::Pose> start, goal;
  arm_navigation_msgs::PlanningScene planning_scene;
  arm_navigation_msgs::OrderedCollisionOperations collision_operations;

  if(!collision_models_interface_->isPlanningSceneSet()) {
    ROS_WARN("Planning scene not set");
    return false;
  } 

  if(!setRobotState(request,response))
    return false;

  if(!convertToBaseFrame(request,response))
    return true;

  if(!getStart(request,response,start))
    return true;
  if(!getGoal(request,response,goal))
    return true;

  ros::Duration allowed_planning_time = request.motion_plan_request.allowed_planning_time;
  if(getPath(start,goal,planning_scene,collision_operations,allowed_planning_time,response))
  {
    response.error_code.val = response.error_code.SUCCESS;
    arm_navigation_msgs::RobotState robot_state;
    planning_environment::convertKinematicStateToRobotState(*collision_models_interface_->getPlanningSceneState(),
                                                            ros::Time::now(),
                                                            collision_models_interface_->getWorldFrameId(),
                                                            robot_state);
    
    planning_visualizer_.visualizePlan(response.trajectory.joint_trajectory, robot_state);
  }
  return true;//services always return true (otherwise rospy chokes?)
}

bool InterpolatedIKMotionPlanner::setRobotState(arm_navigation_msgs::GetMotionPlan::Request &request,
                                                arm_navigation_msgs::GetMotionPlan::Response &response)
{
  planning_models::KinematicState* kinematic_state = collision_models_interface_->getPlanningSceneState();
  planning_environment::setRobotStateAndComputeTransforms(request.motion_plan_request.start_state,
                                                          *kinematic_state);
  return true;
}

bool InterpolatedIKMotionPlanner::convertToBaseFrame(arm_navigation_msgs::GetMotionPlan::Request &request,arm_navigation_msgs::GetMotionPlan::Response &response)
{
  if(!collision_models_interface_->convertConstraintsGivenNewWorldTransform(*collision_models_interface_->getPlanningSceneState(),request.motion_plan_request.goal_constraints,kinematics_solver_->getBaseFrame()))
  {
    response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
    return false;
  }
  return true;
}

bool InterpolatedIKMotionPlanner::getStart(const arm_navigation_msgs::GetMotionPlan::Request &request,
                                           arm_navigation_msgs::GetMotionPlan::Response &response,
                                           std::vector<geometry_msgs::Pose> &start)
{
  for (unsigned int i=0; i < num_groups_; i++)
  {
    for(unsigned int j=0; j < request.motion_plan_request.start_state.multi_dof_joint_state.poses.size(); j++)
    {
      if(request.motion_plan_request.start_state.multi_dof_joint_state.child_frame_ids[j] == end_effector_link_names_[i])
      {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = request.motion_plan_request.start_state.multi_dof_joint_state.frame_ids[j];
        pose_stamped.header.stamp = request.motion_plan_request.start_state.multi_dof_joint_state.stamp;
        pose_stamped.pose = request.motion_plan_request.start_state.multi_dof_joint_state.poses[j];

        if(!collision_models_interface_->convertPoseGivenWorldTransform(*collision_models_interface_->getPlanningSceneState(),
                                                                        kinematics_solver_->getBaseFrame(),
                                                                        pose_stamped.header,
                                                                        pose_stamped.pose,
                                                                        pose_stamped))
        {
          response.error_code.val = response.error_code.FRAME_TRANSFORM_FAILURE;
          return false;
        }   
        start.push_back(pose_stamped.pose);
        break;
      }
    }
    if(start.size() < (i+1))
    {
      ROS_ERROR("Could not find start state for group %s",group_names_[i].c_str());
      response.error_code.val = response.error_code.INVALID_ROBOT_STATE;
      return false;
    }
  }
  return true;
}

bool InterpolatedIKMotionPlanner::getGoal(const arm_navigation_msgs::GetMotionPlan::Request &request,
                                           arm_navigation_msgs::GetMotionPlan::Response &response,
                                          std::vector<geometry_msgs::Pose> &goal)
{
  for (unsigned int i=0; i < num_groups_; i++)
  {
    arm_navigation_msgs::PositionConstraint position_constraint;
    arm_navigation_msgs::OrientationConstraint orientation_constraint;
    if(!getConstraintsForGroup(request.motion_plan_request.goal_constraints,
                               group_names_[i],
                               end_effector_link_names_[i],
                               position_constraint,
                               orientation_constraint))
    {
      response.error_code.val = response.error_code.INVALID_GOAL_POSITION_CONSTRAINTS;
      return false;
    }
    geometry_msgs::PoseStamped desired_pose = arm_navigation_msgs::poseConstraintsToPoseStamped(position_constraint,orientation_constraint);
    goal.push_back(desired_pose.pose);
  }
  return true;
}

bool InterpolatedIKMotionPlanner::getConstraintsForGroup(const arm_navigation_msgs::Constraints &constraints,
                                                         const std::string &group_name,
                                                         const std::string &end_effector_link_name,
                                                         arm_navigation_msgs::PositionConstraint &position_constraint,
                                                         arm_navigation_msgs::OrientationConstraint &orientation_constraint,
                                                         const bool &need_both_constraints)
{
  int position_index = -1;
  int orientation_index = -1;
  for(unsigned int i=0; i < constraints.position_constraints.size(); i++)
  {
    if(constraints.position_constraints[i].link_name == end_effector_link_name)
    {
      position_index = i;
      break;
    }
  }
  for(unsigned int i=0; i < constraints.orientation_constraints.size(); i++)
  {
    if(constraints.orientation_constraints[i].link_name == end_effector_link_name)
    {
      orientation_index = i;
      break;
    }
  }
  if((position_index < 0 || orientation_index < 0) && need_both_constraints)
  {
    ROS_ERROR("Need at least one position and orientation constraint to be specified in the message");
    return false;
  }
  if(position_index >= 0)
    position_constraint = constraints.position_constraints[position_index];
  if(orientation_index >= 0)
    orientation_constraint = constraints.orientation_constraints[orientation_index];
  return true;
}

bool InterpolatedIKMotionPlanner::getPath(const std::vector<geometry_msgs::Pose> &start,
                                          const std::vector<geometry_msgs::Pose> &end,
                                          const arm_navigation_msgs::PlanningScene& planning_scene,
                                          const arm_navigation_msgs::OrderedCollisionOperations &collision_operations,
                                          const ros::Duration &max_time,
                                          arm_navigation_msgs::GetMotionPlan::Response &response)
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
  {
    ROS_DEBUG("%s Start Quaternion: %f %f %f %f",group_names_[i].c_str(),start[i].orientation.x,start[i].orientation.y,start[i].orientation.z,start[i].orientation.w);
    ROS_DEBUG("%s End Quaternion: %f %f %f %f",group_names_[i].c_str(),end[i].orientation.x,end[i].orientation.y,end[i].orientation.z,end[i].orientation.w);
    interpolateCartesian(start[i],end[i],pos_spacing_,rot_spacing_,path[i],max_num_points);
  }
  
  while(timeout.toSec() >= 0.0)
  {
    ros::Time start_time = ros::Time::now();
    if(getInterpolatedIKPath(path,timeout,response))
    {
      timeout -= (ros::Time::now()-start_time);
      return true;
    }
    timeout -= (ros::Time::now()-start_time);
  }
  return false;
}                                      

bool InterpolatedIKMotionPlanner::getInterpolatedIKPath(const std::vector<std::vector<geometry_msgs::Pose> > &path,
                                                        const ros::Duration &max_time,
                                                        arm_navigation_msgs::GetMotionPlan::Response &response)
{
  double timeout = max_time.toSec();
  std::vector<int> error_codes;
  response.trajectory.joint_trajectory.points.clear();
  arm_navigation_msgs::Constraints empty_constraints;
  int error_code;
  while(timeout >= 0.0)
  {
    // Find initial collision free solutions for both arms
    std::vector<std::vector <double> > seed_states, solution_states;
    std::vector<geometry_msgs::Pose> poses;
    seed_states.resize(num_groups_);
    solution_states.resize(num_groups_);
    error_codes.resize(num_groups_);

    for(unsigned int i=0; i < num_groups_; i++)
      poses.push_back(path[i][0]);

    if(kinematics_solver_->searchConstraintAwarePositionIK(poses,timeout,seed_states,error_codes))
    {
      ROS_DEBUG("Got solution for initial pose");
      addToJointTrajectory(response.trajectory.joint_trajectory,seed_states);
    }
    else
    {
      response.error_code = getArmNavigationErrorCode(error_codes);
      ROS_ERROR("Could not find solution for initial pose. Error code: %s",arm_navigation_msgs::armNavigationErrorCodeToString(response.error_code).c_str());
      visualizeEndEffectorPoses(poses);
      return false;
    }
    ROS_DEBUG("Num groups in path: %d",(int)path.size());
    //Now use those collision free solutions to try and find solutions that are close
    for(unsigned int i=1; i < path[0].size(); i++)
    {
      poses.clear();
      for(unsigned int j=0; j < num_groups_; j++)
        poses.push_back(path[j][i]);
      if(kinematics_solver_->searchConstraintAwarePositionIK(poses,seed_states,timeout,solution_states,error_codes,max_distance_))
      {
        if(checkMotion(seed_states,solution_states,empty_constraints,timeout,error_code))
        {
          addToJointTrajectory(response.trajectory.joint_trajectory,solution_states);
          seed_states = solution_states;
        }
        else
        {
          response.error_code = kinematicsErrorCodeToArmNavigationErrorCode(error_code);
          ROS_ERROR("Motion check failed");
          return false;
        }
      }
      else
      {
        response.error_code = getArmNavigationErrorCode(error_codes);
        ROS_ERROR("IK check failed");
        return false;
      }
    }
    ROS_INFO("Succeeded");
    return true;
  }
  return false;
}

bool InterpolatedIKMotionPlanner::visualizeEndEffectorPoses(const std::vector<geometry_msgs::Pose> &poses)
{
  visualization_msgs::MarkerArray mk;
  mk.markers.resize(poses.size());
  for(unsigned int i=0; i < poses.size(); i++)
  {
    mk.markers[i].header.stamp = ros::Time::now();
    mk.markers[i].header.frame_id = kinematics_solver_->getBaseFrame();
    mk.markers[i].pose = poses[i];

    mk.markers[i].scale.x = 1.0;
    mk.markers[i].scale.y = mk.markers[i].scale.z = 0.1;
    mk.markers[i].color.a = 1.0;
    mk.markers[i].color.r = 0.04;
    mk.markers[i].color.g = 1.0;
    mk.markers[i].color.b = 0.04;
  }
  end_effector_pose_array_publisher_.publish(mk);
  return true;
}

bool InterpolatedIKMotionPlanner::checkMotion(const std::vector<std::vector<double> > &start, 
                                              const std::vector<std::vector<double> >&end,
                                              const arm_navigation_msgs::Constraints &constraints,
                                              double &timeout,
                                              int &error_code)
{
  return kinematics_solver_->checkMotion(start,end,constraints,timeout,error_code);
}

arm_navigation_msgs::ArmNavigationErrorCodes InterpolatedIKMotionPlanner::kinematicsErrorCodeToArmNavigationErrorCode(const int& error_code)
{
  arm_navigation_msgs::ArmNavigationErrorCodes ec;
  switch(error_code)
  {
  case kinematics::SUCCESS:
    ec.val = ec.SUCCESS;
    break;
  case kinematics::TIMED_OUT:
    ec.val = ec.TIMED_OUT;
    break;
  case kinematics::NO_IK_SOLUTION:
    ec.val = ec.NO_IK_SOLUTION;
    break;
  case kinematics::FRAME_TRANSFORM_FAILURE:
    ec.val = ec.FRAME_TRANSFORM_FAILURE;
    break;
  case kinematics::IK_LINK_INVALID:
    ec.val = ec.INVALID_LINK_NAME;
    break;
  case kinematics::IK_LINK_IN_COLLISION:
    ec.val = ec.IK_LINK_IN_COLLISION;
    break;
  case kinematics::STATE_IN_COLLISION:
    ec.val = ec.KINEMATICS_STATE_IN_COLLISION;
    break;
  case kinematics::INVALID_LINK_NAME:
    ec.val = ec.INVALID_LINK_NAME;
    break;
    //  case kinematics::GOAL_CONSTRAINTS_VIOLATED:
    //    ec.val = ec.GOAL_CONSTRAINTS_VIOLATED;
    //    break;
  case kinematics::INACTIVE:
    ec.val = ec.NO_IK_SOLUTION;
    break;
  default:
    ec.val = ec.PLANNING_FAILED;
    break;
  }
  return ec;
}

arm_navigation_msgs::ArmNavigationErrorCodes InterpolatedIKMotionPlanner::getArmNavigationErrorCode(const std::vector<int> &error_codes)
{
  arm_navigation_msgs::ArmNavigationErrorCodes error_code;
  error_code.val = error_code.SUCCESS;
  for(unsigned int j=0; j < error_codes.size(); j++)
  {
    if(error_codes[j] != kinematics::SUCCESS)
    {
      ROS_DEBUG("Failed with error code on point %d: %d",j,error_codes[j]);
      error_code = kinematicsErrorCodeToArmNavigationErrorCode(error_codes[j]);
      break;
    }
  } 
  return error_code;
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

  trajectory.points.push_back(point);
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

  ROS_DEBUG("Interpolating cartesian path with num points: %d",num_points);

  for(unsigned int i=0; i < num_points; i++)
  {
    double fraction = ((double) i)/(num_points-1);
    point_tf = start_tf.lerp(end_tf,fraction);
    point_rot_tf = start_rot_tf.slerp(end_rot_tf,fraction);
    tf::pointTFToMsg(point_tf,pose.position);
    tf::quaternionTFToMsg(point_rot_tf,pose.orientation);
    ROS_DEBUG("Interpolate point (%d)",i);
    ROS_DEBUG("Position : %f, %f, %f",pose.position.x,pose.position.y,pose.position.z);
    ROS_DEBUG("Orientation : %f, %f, %f, %f",pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
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
  num_points = std::max<unsigned int>(num_points,2);
  return true;
}
}
