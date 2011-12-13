/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

/** \author Mrinal Kalakrishnan, E. Gil Jones */

#include <chomp_motion_planner/chomp_planner_node.h>
#include <chomp_motion_planner/chomp_trajectory.h>
#include <chomp_motion_planner/chomp_utils.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <chomp_motion_planner/chomp_optimizer.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <spline_smoother/cubic_trajectory.h>
#include <arm_navigation_msgs/FilterJointTrajectory.h>
#include <planning_environment/models/model_utils.h>
#include <spline_smoother/fritsch_butland_spline_smoother.h>

#include <map>
#include <vector>
#include <string>

using namespace std;
using namespace planning_models;
using namespace collision_proximity;

namespace chomp
{

ChompPlannerNode::ChompPlannerNode(ros::NodeHandle node_handle, CollisionProximitySpace* space) : node_handle_(node_handle), collision_proximity_space_(space)
                                                                  //filter_constraints_chain_("arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request")
{

}

bool ChompPlannerNode::init()
{
  // load in some default parameters
  node_handle_.param("trajectory_duration", trajectory_duration_, 3.0);
  node_handle_.param("trajectory_discretization", trajectory_discretization_, 0.03);
  node_handle_.param("use_additional_trajectory_filter", use_trajectory_filter_, false);
  node_handle_.param("minimum_spline_points", minimum_spline_points_, 40);
  node_handle_.param("maximum_spline_points", maximum_spline_points_, 100);
  if(node_handle_.hasParam("joint_velocity_limits")) {
    XmlRpc::XmlRpcValue velocity_limits;
    
    node_handle_.getParam("joint_velocity_limits", velocity_limits);

    if(velocity_limits.getType() ==  XmlRpc::XmlRpcValue::TypeStruct) {
      if(velocity_limits.size() > 0) {
        for(XmlRpc::XmlRpcValue::iterator it = velocity_limits.begin();
            it != velocity_limits.end();
            it++) {
          joint_velocity_limits_[it->first] = it->second;
          ROS_DEBUG_STREAM("Vel limit for " << it->first << " is " << joint_velocity_limits_[it->first]);
        }
      }
    }  
  } 

  //filter_constraints_chain_.configure("filter_chain",node_handle_);


  collision_models_ = collision_proximity_space_->getCollisionModelsInterface();

  if(!collision_models_->loadedModels()) {
    ROS_ERROR("Collision models could not load models");
    return false;
  }



  reference_frame_ = collision_proximity_space_->getCollisionModelsInterface()->getWorldFrameId();


  robot_model_= (planning_models::KinematicModel*)collision_proximity_space_->getCollisionModelsInterface()->getKinematicModel();

  // load chomp parameters:
  chomp_parameters_.initFromNodeHandle();
  
  collision_proximity_space_->setCollisionTolerance(.05);

  // initialize the visualization publisher:
  vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 0 );
  vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  // advertise the planning service
  plan_kinematic_path_service_ = root_handle_.advertiseService("chomp_planner_longrange/plan_path", &ChompPlannerNode::planKinematicPath, this);

  filter_joint_trajectory_service_ = root_handle_.advertiseService("chomp_planner_longrange/filter_trajectory_with_constraints", &ChompPlannerNode::filterJointTrajectory, this);

  if(use_trajectory_filter_) {
    filter_trajectory_client_ = root_handle_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>("trajectory_filter/filter_trajectory_with_constraints");    
  
    ros::service::waitForService("trajectory_filter/filter_trajectory_with_constraints");
  }

  ROS_INFO("Initalized CHOMP planning service...");

  return true;
}

ChompPlannerNode::~ChompPlannerNode()
{
  delete collision_models_;
}

int ChompPlannerNode::run()
{
  ros::spin();
  return 0;
}

bool ChompPlannerNode::planKinematicPath(arm_navigation_msgs::GetMotionPlan::Request &reqIn, arm_navigation_msgs::GetMotionPlan::Response &res)
{
  arm_navigation_msgs::GetMotionPlan::Request req = reqIn;
  if (!(req.motion_plan_request.goal_constraints.position_constraints.empty() && req.motion_plan_request.goal_constraints.orientation_constraints.empty()))
  {
    ROS_ERROR("CHOMP cannot handle pose contraints yet.");
    res.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GOAL_POSITION_CONSTRAINTS;
    return false;
  }

  sensor_msgs::JointState joint_goal_chomp = arm_navigation_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints);
  ROS_INFO("Chomp goal");

  if(joint_goal_chomp.name.size() != joint_goal_chomp.position.size())
  {
    ROS_ERROR("Invalid chomp goal");
    res.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::INVALID_GOAL_JOINT_CONSTRAINTS;
    return false;
  }

  for(unsigned int i=0; i<joint_goal_chomp.name.size(); i++)
  {
    ROS_INFO("%s %f",joint_goal_chomp.name[i].c_str(),joint_goal_chomp.position[i]);
  }

  ros::WallTime start_time = ros::WallTime::now();
  ROS_INFO("Received planning request...");

  string group_name;
  group_name = req.motion_plan_request.group_name;

  vector<string> linkNames;
  vector<string> attachedBodies;
  ros::WallTime start = ros::WallTime::now();
  collision_proximity_space_->setupForGroupQueries(group_name,
                                                   req.motion_plan_request.start_state,
                                                   linkNames,
                                                   attachedBodies);
  ROS_INFO_STREAM("Setting up for queries time is " << (ros::WallTime::now() - start));
  //collision_proximity_space_->visualizeObjectSpheres(collision_proximity_space_->getCurrentLinkNames());
  ChompTrajectory trajectory(robot_model_, trajectory_duration_, trajectory_discretization_, group_name);

  ROS_INFO("Initial trajectory has %d points", trajectory.getNumPoints());
  // set the start state:
  jointStateToArray(req.motion_plan_request.start_state.joint_state, group_name, trajectory.getTrajectoryPoint(0));

  ROS_INFO_STREAM("Joint state has " << req.motion_plan_request.start_state.joint_state.name.size() << " joints");

  // set the goal state equal to start state, and override the joints specified in the goal
  // joint constraints
  int goal_index = trajectory.getNumPoints()- 1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);
  jointStateToArray(arm_navigation_msgs::jointConstraintsToJointState(req.motion_plan_request.goal_constraints.joint_constraints), group_name, trajectory.getTrajectoryPoint(goal_index));


  map<string, KinematicModel::JointModelGroup*> groupMap = robot_model_->getJointModelGroupMap();
  KinematicModel::JointModelGroup* modelGroup = groupMap[group_name];


  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (size_t i = 0; i < modelGroup->getJointModels().size(); i++)
  {
    const KinematicModel::JointModel* model = modelGroup->getJointModels()[i];
    const KinematicModel::RevoluteJointModel* revoluteJoint = dynamic_cast<const KinematicModel::RevoluteJointModel*>(model);

    if (revoluteJoint != NULL)
    {
      if(revoluteJoint->continuous_)
      {
        double start = (trajectory)(0, i);
        double end = (trajectory)(goal_index, i);
        (trajectory)(goal_index, i) = start + angles::shortest_angular_distance(start, end);
      }
    }
  }

  // fill in an initial quintic spline trajectory
  trajectory.fillInMinJerk();

  // set the max planning time:
  chomp_parameters_.setPlanningTimeLimit(req.motion_plan_request.allowed_planning_time.toSec());

  // optimize!
  ros::WallTime create_time = ros::WallTime::now();
  ChompOptimizer optimizer(&trajectory, robot_model_, group_name, &chomp_parameters_,
      vis_marker_array_publisher_, vis_marker_publisher_, collision_proximity_space_);
  ROS_INFO("Optimization took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  optimizer.optimize();
  ROS_INFO("Optimization actually took %f sec to run", (ros::WallTime::now() - create_time).toSec());
  create_time = ros::WallTime::now();
  // assume that the trajectory is now optimized, fill in the output structure:

  ROS_INFO("Output trajectory has %d joints", trajectory.getNumJoints());
  vector<double> velocity_limits(trajectory.getNumJoints(), numeric_limits<double>::max());

  // fill in joint names:
  res.trajectory.joint_trajectory.joint_names.resize(trajectory.getNumJoints());
  for (size_t i = 0; i < modelGroup->getJointModels().size(); i++)
  {
    res.trajectory.joint_trajectory.joint_names[i] = modelGroup->getJointModels()[i]->getName();
    // try to retrieve the joint limits:
    if (joint_velocity_limits_.find(res.trajectory.joint_trajectory.joint_names[i])==joint_velocity_limits_.end())
    {
      joint_velocity_limits_[res.trajectory.joint_trajectory.joint_names[i]] = numeric_limits<double>::max();
    }
    velocity_limits[i] = joint_velocity_limits_[res.trajectory.joint_trajectory.joint_names[i]];
  }

  res.trajectory.joint_trajectory.header = req.motion_plan_request.start_state.joint_state.header; // @TODO this is probably a hack

  // fill in the entire trajectory
  res.trajectory.joint_trajectory.points.resize(trajectory.getNumPoints());
  for (int i=0; i < trajectory.getNumPoints(); i++)
  {
    res.trajectory.joint_trajectory.points[i].positions.resize(trajectory.getNumJoints());
    for (size_t j=0; j < res.trajectory.joint_trajectory.points[i].positions.size(); j++)
    {
      res.trajectory.joint_trajectory.points[i].positions[j] = trajectory.getTrajectoryPoint(i)(j);
    }
    // Setting invalid timestamps.
    // Further filtering is required to set valid timestamps accounting for velocity and acceleration constraints.
    res.trajectory.joint_trajectory.points[i].time_from_start = ros::Duration(0.0);
  }

  ROS_INFO("Bottom took %f sec to create", (ros::WallTime::now() - create_time).toSec());
  ROS_INFO("Serviced planning request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.joint_trajectory.points[goal_index].time_from_start.toSec());
  res.error_code.val = arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS;
  res.planning_time = ros::Duration((ros::WallTime::now() - start_time).toSec());
  return true;
}

bool ChompPlannerNode::filterJointTrajectory(arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request &request, arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response &res)
{
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request req = request;
  ros::WallTime start_time = ros::WallTime::now();
  ROS_INFO_STREAM("Received filtering request with trajectory size " << req.trajectory.points.size());

  if(req.path_constraints.joint_constraints.size() > 0 ||
     req.path_constraints.position_constraints.size() > 0 ||
     req.path_constraints.orientation_constraints.size() > 0 ||
     req.path_constraints.visibility_constraints.size() > 0) {
    if(use_trajectory_filter_) {
      ROS_INFO("Chomp can't handle path constraints, passing through to other trajectory filters");
      if(!filter_trajectory_client_.call(req,res)) {
        ROS_INFO("Pass through failed");
      } else {
        ROS_INFO("Pass through succeeded");
      }
    } else {
      ROS_INFO("Chomp can't handle path constraints, and not set up to use additional filter");
    }
    return true;
  } 
  for (unsigned int i=0; i< req.trajectory.points.size(); i++)
  {
    req.trajectory.points[i].velocities.resize(req.trajectory.joint_names.size(),0.0);
  }

  getLimits(req.trajectory, req.limits);

  trajectory_msgs::JointTrajectory jtraj;

  int num_points = req.trajectory.points.size();
  if(num_points > maximum_spline_points_) {
    num_points = maximum_spline_points_;
  } else if(num_points < minimum_spline_points_) {
    num_points = minimum_spline_points_;
  }


  //create a spline from the trajectory
  spline_smoother::CubicTrajectory trajectory_solver;
  spline_smoother::SplineTrajectory spline;

  planning_environment::setRobotStateAndComputeTransforms(req.start_state,
                                                          *collision_proximity_space_->getCollisionModelsInterface()->getPlanningSceneState());
  
  trajectory_solver.parameterize(req.trajectory,req.limits,spline);  
  
  double smoother_time;
  spline_smoother::getTotalTime(spline, smoother_time);
  
  ROS_INFO_STREAM("Total time given is " << smoother_time);
  
  double t = 0.0;
  vector<double> times(num_points);
  for(int i = 0; i < num_points; i++,t += smoother_time/(1.0*(num_points-1))) {
    times[i] = t;
  }
    
  spline_smoother::sampleSplineTrajectory(spline, times, jtraj);
  
  //double planner_time = req.trajectory.points.back().time_from_start.toSec();
  
  t = 0.0;
  for(unsigned int i = 0; i < jtraj.points.size(); i++, t += smoother_time/(1.0*(num_points-1))) {
    jtraj.points[i].time_from_start = ros::Duration(t);
  }
  
  ROS_INFO_STREAM("Sampled trajectory has " << jtraj.points.size() << " points with " << jtraj.points[0].positions.size() << " joints");


  string group_name;
  group_name = req.group_name;

  vector<string> linkNames;
  vector<string> attachedBodies;
  collision_proximity_space_->setupForGroupQueries(group_name,
                                                   req.start_state,
                                                   linkNames,
                                                   attachedBodies);

  ChompTrajectory trajectory(robot_model_, group_name, jtraj);

  //configure the distance field - this should just use current state
  arm_navigation_msgs::RobotState robot_state = req.start_state;

  jointStateToArray(arm_navigation_msgs::createJointState(req.trajectory.joint_names, jtraj.points[0].positions), group_name, trajectory.getTrajectoryPoint(0));

  //set the goal state equal to start state, and override the joints specified in the goal
  //joint constraints
  int goal_index = trajectory.getNumPoints()-1;
  trajectory.getTrajectoryPoint(goal_index) = trajectory.getTrajectoryPoint(0);

  sensor_msgs::JointState goal_state = arm_navigation_msgs::createJointState(req.trajectory.joint_names, jtraj.points.back().positions);

  jointStateToArray(goal_state, group_name,trajectory.getTrajectoryPoint(goal_index));
  
  map<string, KinematicModel::JointModelGroup*> groupMap = robot_model_->getJointModelGroupMap();
  KinematicModel::JointModelGroup* modelGroup = groupMap[group_name];

  // fix the goal to move the shortest angular distance for wrap-around joints:
  for (size_t i = 0; i < modelGroup->getJointModels().size(); i++)
  {
    const KinematicModel::JointModel* model = modelGroup->getJointModels()[i];
    const KinematicModel::RevoluteJointModel* revoluteJoint = dynamic_cast<const KinematicModel::RevoluteJointModel*>(model);

    if (revoluteJoint != NULL)
    {
      if(revoluteJoint->continuous_)
      {
        double start = trajectory(0, i);
        double end = trajectory(goal_index, i);
        trajectory(goal_index, i) = start + angles::shortest_angular_distance(start, end);
      }
    }
  }

  //sets other joints
  trajectory.fillInMinJerk();
  trajectory.overwriteTrajectory(jtraj);
  
  // set the max planning time:
  chomp_parameters_.setPlanningTimeLimit(req.allowed_time.toSec());
  chomp_parameters_.setFilterMode(true);
  // optimize!
  ChompOptimizer optimizer(&trajectory, robot_model_, group_name, &chomp_parameters_,
      vis_marker_array_publisher_, vis_marker_publisher_, collision_proximity_space_);
  optimizer.optimize();
  
  // assume that the trajectory is now optimized, fill in the output structure:

  vector<double> velocity_limits(trajectory.getNumJoints(), numeric_limits<double>::max());
  res.trajectory.points.resize(trajectory.getNumPoints());
  // fill in joint names:
  res.trajectory.joint_names.resize(trajectory.getNumJoints());
  for (size_t i = 0; i < modelGroup->getJointModels().size(); i++)
  {
    res.trajectory.joint_names[i] = modelGroup->getJointModels()[i]->getName();
    velocity_limits[i] = joint_limits_[res.trajectory.joint_names[i]].max_velocity;
  }
  
  res.trajectory.header.stamp = ros::Time::now();
  res.trajectory.header.frame_id = reference_frame_;

  // fill in the entire trajectory

  for (size_t i = 0; i < (unsigned int)trajectory.getNumPoints(); i++)
  {
    res.trajectory.points[i].positions.resize(trajectory.getNumJoints());
    res.trajectory.points[i].velocities.resize(trajectory.getNumJoints());
    for (size_t j=0; j < res.trajectory.points[i].positions.size(); j++)
    {
      res.trajectory.points[i].positions[j] = trajectory(i, j);
    }
    if (i==0)
      res.trajectory.points[i].time_from_start = ros::Duration(0.0);
    else
    {
      double duration = trajectory.getDiscretization();
      // check with all the joints if this duration is ok, else push it up
      for (int j=0; j < trajectory.getNumJoints(); j++)
      {
        double d = fabs(res.trajectory.points[i].positions[j] - res.trajectory.points[i-1].positions[j]) / velocity_limits[j];
        if (d > duration)
          duration = d;
      }
      try {
        res.trajectory.points[i].time_from_start = res.trajectory.points[i-1].time_from_start + ros::Duration(duration);
      } catch(...) {
        ROS_INFO_STREAM("Potentially weird duration of " << duration);
      }
    }
  }
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request  next_req;
  arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response next_res;

  if(use_trajectory_filter_) {
    next_req = req;
    next_req.trajectory = res.trajectory;  
    next_req.allowed_time=ros::Duration(1.0);//req.allowed_time/2.0;
    
    if(filter_trajectory_client_.call(next_req, next_res)) {
      ROS_INFO_STREAM("Filter call ok. Sent trajectory had " << res.trajectory.points.size() << " points.  Returned trajectory has " << next_res.trajectory.points.size() << " points ");
    } else {
      ROS_INFO("Filter call not ok");
    }
    
    res.trajectory = next_res.trajectory;
    res.error_code = next_res.error_code;
    res.trajectory.header.stamp = ros::Time::now();
    res.trajectory.header.frame_id = reference_frame_;
  } else {
    res.error_code.val = res.error_code.val = res.error_code.SUCCESS;
  }

  // for every point in time:
  for (unsigned int i=1; i<res.trajectory.points.size()-1; ++i)
  {
    double dt1 = (res.trajectory.points[i].time_from_start - res.trajectory.points[i-1].time_from_start).toSec();
    double dt2 = (res.trajectory.points[i+1].time_from_start - res.trajectory.points[i].time_from_start).toSec();

    // for every (joint) trajectory
    for (int j=0; j < trajectory.getNumJoints(); ++j)
    {
      double dx1 = res.trajectory.points[i].positions[j] - res.trajectory.points[i-1].positions[j];
      double dx2 = res.trajectory.points[i+1].positions[j] - res.trajectory.points[i].positions[j];

      double v1 = dx1/dt1;
      double v2 = dx2/dt2;

      res.trajectory.points[i].velocities[j] = 0.5*(v1 + v2);
    }
  }

  ROS_INFO("Serviced filter request in %f wall-seconds, trajectory duration is %f", (ros::WallTime::now() - start_time).toSec(), res.trajectory.points.back().time_from_start.toSec());
  return true;

}


void ChompPlannerNode::getLimits(const trajectory_msgs::JointTrajectory& trajectory, 
                                 vector<arm_navigation_msgs::JointLimits>& limits_out)
{
  int num_joints = trajectory.joint_names.size();
  limits_out.resize(num_joints);
  for (int i=0; i<num_joints; ++i)
  {
    map<string, arm_navigation_msgs::JointLimits>::const_iterator limit_it = joint_limits_.find(trajectory.joint_names[i]);
    arm_navigation_msgs::JointLimits limits;
    if (limit_it == joint_limits_.end())
    {
      // load the limits from the param server
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/min_position", limits.min_position, -numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_position", limits.max_position, numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_velocity", limits.max_velocity, numeric_limits<double>::max());
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/max_acceleration", limits.max_acceleration, numeric_limits<double>::max());
      bool boolean;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_position_limits", boolean, false);
      limits.has_position_limits = boolean?1:0;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_velocity_limits", boolean, false);
      limits.has_velocity_limits = boolean?1:0;
      node_handle_.param("joint_limits/"+trajectory.joint_names[i]+"/has_acceleration_limits", boolean, false);
      limits.has_acceleration_limits = boolean?1:0;
      joint_limits_.insert(make_pair(trajectory.joint_names[i], limits));
    }
    else
    {
      limits = limit_it->second;
    }
    limits_out[i] = limits;
  }
}

} // namespace chomp

int main(int argc, char** argv)
{
  ros::init(argc, argv, "chomp_planner_node");

  //ros::AsyncSpinner spinner(1); // Use 1 thread
  //spinner.start();

  ros::NodeHandle node_handle("~");
  string robotDescription;
  ros::param::param<string>("robot_description_file_name", robotDescription, "");
  bool use_signed_environment_field = false;
  bool use_signed_self_field = false;
  ros::param::param<bool>("use_signed_environment_field", use_signed_environment_field, false);
  ros::param::param<bool>("use_signed_self_field", use_signed_self_field, false);
  chomp::ChompPlannerNode chomp_planner_node(node_handle, new CollisionProximitySpace("robot_description",true,use_signed_environment_field, use_signed_self_field));
  if (!chomp_planner_node.init())
    return 1;
  return chomp_planner_node.run();
  //ros::waitForShutdown();
}
