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

#ifndef _PR2_GRIPPER_TRAJECTORY_CONTROLLER_HANDLER_H_
#define _PR2_GRIPPER_TRAJECTORY_CONTROLLER_HANDLER_H_

#include <trajectory_execution_monitor/trajectory_controller_handler.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

class Pr2GripperTrajectoryControllerHandler : public trajectory_execution_monitor::TrajectoryControllerHandler {

public:
  
  static const double GRIPPER_OPEN = 0.086;
  static const double GRIPPER_CLOSED = 0.0;
  static const double DEFAULT_GRIPPER_OBJECT_PRESENCE_THRESHOLD = 0.0021;

  Pr2GripperTrajectoryControllerHandler(const std::string& group_name, 
                                        const std::string& controller_name) : 
    TrajectoryControllerHandler(group_name, controller_name),
    pr2_gripper_action_client_(controller_name, true)
  {
    while(ros::ok() && !pr2_gripper_action_client_.waitForServer(ros::Duration(5.0))){
      ROS_INFO_STREAM("Waiting for the pr2_gripper action for group " << group_name << " on the topic " << controller_name << " to come up");
    }
  }

  // Gripper does not monitor overshoot.
  bool enableOvershoot( double max_overshoot_velocity_epsilon,
                        ros::Duration min_overshoot_time,
                        ros::Duration max_overshoot_time )
  {
    return false;
  }

  bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                         boost::shared_ptr<trajectory_execution_monitor::TrajectoryRecorder>& recorder,
                         const trajectory_execution_monitor::TrajectoryFinishedCallbackFunction& traj_callback)
  {
    recorder_ = recorder;
    trajectory_finished_callback_ = traj_callback;

    initializeRecordedTrajectory(trajectory);
    
    pr2_controllers_msgs::Pr2GripperCommandGoal gripper_command;
      gripper_command.command.max_effort = 10000;

    double jval = trajectory.points[0].positions[0];
    if(jval != 0.0) {
      ROS_INFO_STREAM("Commanding gripper open");
      gripper_command.command.position = GRIPPER_OPEN;
    } else {
      ROS_INFO_STREAM("Commanding gripper closed");
      gripper_command.command.position = GRIPPER_CLOSED;
    }
    
    pr2_gripper_action_client_.sendGoal(gripper_command,
                                        boost::bind(&Pr2GripperTrajectoryControllerHandler::controllerDoneCallback, this, _1, _2),
                                        boost::bind(&Pr2GripperTrajectoryControllerHandler::controllerActiveCallback, this),
                                        boost::bind(&Pr2GripperTrajectoryControllerHandler::controllerFeedbackCallback, this, _1));

    recorder_->registerCallback(group_controller_combo_name_, 
                                boost::bind(&Pr2GripperTrajectoryControllerHandler::addNewStateToRecordedTrajectory, this, _1, _2, _3));
    return true;
  }

  void cancelExecution() {
  }

  void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                              const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr& result)
  {
    ROS_INFO_STREAM("Gripper controller is done with state " << state.toString());

    if( controller_state_ == trajectory_execution_monitor::TrajectoryControllerStates::EXECUTING ||
        controller_state_ == trajectory_execution_monitor::TrajectoryControllerStates::OVERSHOOTING )
    {
      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        controller_state_ = trajectory_execution_monitor::TrajectoryControllerStates::SUCCESS;
      }
      else
      {
        ROS_WARN_STREAM("Failed state is " << state.toString() );
        controller_state_ = trajectory_execution_monitor::TrajectoryControllerStates::EXECUTION_FAILURE;
      }

      // We don't record overshoot on the gripper
      done();
    }
  }

  void controllerActiveCallback() 
  {
    ROS_DEBUG_STREAM("Controller went active");
  }

  void controllerFeedbackCallback(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
  {
    ROS_DEBUG_STREAM("Got feedback");
  }
    

protected:

  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> pr2_gripper_action_client_;
}; 

#endif
