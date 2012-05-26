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

/** \author Sachin Chitta */

#include <ros/ros.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <planning_environment/monitors/monitor_utils.h>
#include <planning_environment/models/model_utils.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <planning_environment/models/collision_models_interface.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>


#include <boost/thread/condition.hpp>
#include <boost/scoped_ptr.hpp>
#include <algorithm>
#include <string>
#include <limits>

namespace collision_free_arm_trajectory_controller
{
static const std::string TRAJECTORY_FILTER = "/trajectory_filter_server/filter_trajectory_with_constraints";
static const std::string TRAJECTORY_CONTROLLER = "/joint_trajectory_action";
static const double MIN_DELTA = 0.01;

enum ControllerState{
  IDLE,
  START_CONTROL,
  MONITOR
};


class CollisionFreeArmTrajectoryController
{

public:
  CollisionFreeArmTrajectoryController(): private_handle_("~")
  {
    std::string robot_description_name = node_handle_.resolveName("robot_description", true);

    collision_models_interface_ = new planning_environment::CollisionModelsInterface(robot_description_name);
    kmsm_ = new planning_environment::KinematicModelStateMonitor(collision_models_interface_, &tf_);

    kmsm_->addOnStateUpdateCallback(boost::bind(&CollisionFreeArmTrajectoryController::jointStateCallback, this, _1));

    collision_models_interface_->addSetPlanningSceneCallback(boost::bind(&CollisionFreeArmTrajectoryController::setPlanningSceneCallback, this, _1));

    ros::service::waitForService("filter_trajectory");

    filter_trajectory_client_ = node_handle_.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>("filter_trajectory",true);      

    private_handle_.param<std::string>("group_name", group_name_, "");

    traj_action_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(TRAJECTORY_CONTROLLER, true);
    while(ros::ok() && !traj_action_client_->waitForServer(ros::Duration(1.0))){
      ROS_INFO("Waiting for the arm trajectory controller to come up");
    }

    action_server_.reset(new actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction>(node_handle_, "collision_free_arm_trajectory_action_" + group_name_, false));

    action_server_->registerGoalCallback(boost::bind(&CollisionFreeArmTrajectoryController::executeTrajectory, this));
    action_server_->start();
    state_ = IDLE;
  }

  ~CollisionFreeArmTrajectoryController()
  {
    delete traj_action_client_;
    delete collision_models_interface_;
    delete kmsm_;
  }

  void setPlanningSceneCallback(const arm_navigation_msgs::PlanningScene& scene) {
    collision_models_interface_->bodiesLock();
    collision_models_interface_->disableCollisionsForNonUpdatedLinks(group_name_);
    collision_models_interface_->bodiesUnlock();
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state) {

    collision_models_interface_->bodiesLock();
    if(!collision_models_interface_->getPlanningSceneState()) {
      if(state_ == MONITOR) {
        ROS_WARN("Should be monitoring, but no planning scene set");
        traj_action_client_->cancelAllGoals();
        action_server_->setAborted();
        state_ = IDLE;
      }
      collision_models_interface_->bodiesUnlock();
      return;
    } 
    kmsm_->setStateValuesFromCurrentValues(*collision_models_interface_->getPlanningSceneState());
    if(state_ != MONITOR || current_joint_trajectory_.points.size() == 0) {
      collision_models_interface_->bodiesUnlock();
      return;
    }
    trajectory_msgs::JointTrajectory joint_trajectory_subset;
    planning_environment::removeCompletedTrajectory(collision_models_interface_->getParsedDescription(),
                                                    current_joint_trajectory_,
                                                    *joint_state,
                                                    joint_trajectory_subset,
                                                    false);
    current_joint_trajectory_ = joint_trajectory_subset;
    arm_navigation_msgs::Constraints emp;
    arm_navigation_msgs::ArmNavigationErrorCodes error_code;
    std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> error_code_vec;
    if(current_joint_trajectory_.points.size() > 0) {
      if(!collision_models_interface_->isJointTrajectoryValid(*collision_models_interface_->getPlanningSceneState(),
                                                              current_joint_trajectory_,
                                                              emp, emp,
                                                              error_code,
                                                              error_code_vec,
                                                              false)) {
        ROS_WARN_STREAM("Collision at point " << error_code_vec.size() << " of remaining trajectory points " << current_joint_trajectory_.points.size());
        traj_action_client_->cancelAllGoals();
        action_server_->setAborted();
        state_ = IDLE;
      }
    }
    collision_models_interface_->bodiesUnlock();
  }

  void executeTrajectory()
  {
    if(state_ != IDLE) {
      ROS_INFO_STREAM("Preempted, so stopping");
      traj_action_client_->cancelAllGoals();
    }
    pr2_controllers_msgs::JointTrajectoryGoal goal(*(action_server_->acceptNewGoal()));

    ROS_INFO("Got trajectory with %d points and %d joints",(int)goal.trajectory.points.size(),(int)goal.trajectory.joint_names.size());
    
    collision_models_interface_->bodiesLock();
    if(!collision_models_interface_->isPlanningSceneSet()) {
      ROS_WARN_STREAM("Can't execute safe trajectory control without planning scene");
      action_server_->setAborted();
      collision_models_interface_->bodiesUnlock();
      return;
    }
    
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request  req;
    arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response res;
    
    planning_environment::convertKinematicStateToRobotState(*collision_models_interface_->getPlanningSceneState(),
                                                            ros::Time::now(),
                                                            collision_models_interface_->getWorldFrameId(),
                                                            req.start_state);
    std::map<std::string, double> current_values;
    collision_models_interface_->getPlanningSceneState()->getKinematicStateValues(current_values);
    req.trajectory = goal.trajectory;

    trajectory_msgs::JointTrajectoryPoint jtp;
    for(unsigned int i = 0; i < req.trajectory.joint_names.size(); i++) {
      ROS_DEBUG_STREAM("Setting joint " << req.trajectory.joint_names[i] << " value " << current_values[req.trajectory.joint_names[i]]);
      jtp.positions.push_back(current_values[req.trajectory.joint_names[i]]);
    }
    req.trajectory.points.insert(req.trajectory.points.begin(), jtp);
    req.group_name = group_name_;
    if(filter_trajectory_client_.call(req,res))
    {
      if(res.error_code.val == res.error_code.SUCCESS)
      {
        current_joint_trajectory_ = res.trajectory;
        pr2_controllers_msgs::JointTrajectoryGoal goal;  
        goal.trajectory = current_joint_trajectory_;
        goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.2);
        traj_action_client_->sendGoal(goal,boost::bind(&CollisionFreeArmTrajectoryController::controllerDoneCallback, this, _1, _2));
        state_ = MONITOR;
        collision_models_interface_->bodiesUnlock();
        return;
      }
      else
      {
        ROS_INFO_STREAM("Filter rejects trajectory based on current state with status " << res.error_code.val);
        action_server_->setAborted();
        collision_models_interface_->bodiesUnlock();
        return;
      }
    } else {
      ROS_WARN_STREAM("Filter trajectory call failed entirely");
      action_server_->setAborted();
      collision_models_interface_->bodiesUnlock();
      return;
    }
  }


  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const pr2_controllers_msgs::JointTrajectoryResultConstPtr& result)
  {
    ROS_INFO_STREAM("Trajectory reported done with state " << state.toString());
    state_ = IDLE;
    action_server_->setSucceeded();
  }

private:
  ros::NodeHandle node_handle_, private_handle_;
  
  std::string group_name_;

  planning_environment::CollisionModelsInterface* collision_models_interface_;
  planning_environment::KinematicModelStateMonitor* kmsm_;

  ros::ServiceClient filter_trajectory_client_;

  tf::TransformListener tf_;

  boost::shared_ptr<actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> > action_server_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>* traj_action_client_;

  ControllerState state_;
  trajectory_msgs::JointTrajectory current_joint_trajectory_;
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_free_arm_trajectory_controller");
  collision_free_arm_trajectory_controller::CollisionFreeArmTrajectoryController cf;
  ROS_INFO("Collision free arm trajectory controller started");
  ros::spin();    
  return 0;
}

