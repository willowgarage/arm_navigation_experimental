/*********************************************************************
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
*********************************************************************/

/** \author E. Gil Jones */

#include <ros/ros.h>

#include <planning_environment/models/collision_models_interface.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

static const ros::Duration MARKER_DUR(.2);

class CurrentStateValidator {

public:

  CurrentStateValidator(void)
  {
    std::string robot_description_name = root_handle_.resolveName("robot_description", true);
    
    collision_models_interface_ = new planning_environment::CollisionModelsInterface(robot_description_name);
    kmsm_ = new planning_environment::KinematicModelStateMonitor(collision_models_interface_, &tf_);

    ros::NodeHandle("~").param<std::string>("group_name_1", group_name_1_, "");
    ros::NodeHandle("~").param<std::string>("group_name_2", group_name_2_, "");

    vis_marker_publisher_ = root_handle_.advertise<visualization_msgs::Marker>("current_state_validator", 128);
    vis_marker_array_publisher_ = root_handle_.advertise<visualization_msgs::MarkerArray>("current_state_validator_array", 128);
    
    kmsm_->addOnStateUpdateCallback(boost::bind(&CurrentStateValidator::jointStateCallback, this, _1));
  }

  void sendMarkersForGroup(const std::string& group) {

    collision_models_interface_->bodiesLock();
    
    collision_space::EnvironmentModel::AllowedCollisionMatrix acm = collision_models_interface_->getCurrentAllowedCollisionMatrix();

    if(!group.empty()) {
      collision_models_interface_->disableCollisionsForNonUpdatedLinks(group);
    }    

    planning_models::KinematicState state(collision_models_interface_->getKinematicModel());
    kmsm_->setStateValuesFromCurrentValues(state);
    
    std_msgs::ColorRGBA good_color;
    good_color.a = 1.0;
    good_color.r = 0.1;
    good_color.g = 0.8;
    good_color.b = 0.3;

    std_msgs::ColorRGBA bad_color;
    bad_color.a = 1.0;
    bad_color.r = 1.0;
    bad_color.g = 0.0;
    bad_color.b = 0.0;

    std_msgs::ColorRGBA col;
    if(collision_models_interface_->isKinematicStateInCollision(state)) {
      col = bad_color; 
    } else {
      col = good_color;
    }
    
    visualization_msgs::MarkerArray arr;
    if(group.empty()) {
      collision_models_interface_->getRobotMarkersGivenState(state,
                                                             arr,
                                                             col,
                                                             "validator",
                                                             ros::Duration(0.0));

    } else {
      const planning_models::KinematicModel::JointModelGroup* joint_model_group = collision_models_interface_->getKinematicModel()->getModelGroup(group);
      if(joint_model_group == NULL) {
        ROS_INFO_STREAM("No joint group " << group);
      }
      std::vector<std::string> group_links = joint_model_group->getGroupLinkNames();
      collision_models_interface_->getRobotMarkersGivenState(state,
                                                             arr,
                                                             col,
                                                             group,
                                                             ros::Duration(MARKER_DUR),
                                                             &group_links);      
    }
    vis_marker_array_publisher_.publish(arr);

    if(!group.empty()) {
      collision_models_interface_->setAlteredAllowedCollisionMatrix(acm);
    }    

    collision_models_interface_->bodiesUnlock();
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state) {
    if((ros::Time::now()-last_update_time_).toSec() < (MARKER_DUR.toSec()/2.0)) {
      return;
    }
    last_update_time_ = ros::Time::now();
    collision_models_interface_->bodiesLock();
    if(group_name_1_.empty() && group_name_2_.empty()) {
      sendMarkersForGroup("");
    } 
    if(!group_name_1_.empty()) {
      sendMarkersForGroup(group_name_1_);
    } 
    if(!group_name_2_.empty()) {
      sendMarkersForGroup(group_name_2_);
    }
    collision_models_interface_->bodiesUnlock();
  }
protected:
    
  planning_environment::CollisionModelsInterface* collision_models_interface_;
  planning_environment::KinematicModelStateMonitor* kmsm_;

  std::string group_name_1_;
  std::string group_name_2_;

  ros::Time last_update_time_;

  ros::NodeHandle root_handle_;

  ros::Publisher vis_marker_publisher_;
  ros::Publisher vis_marker_array_publisher_;

  tf::TransformListener tf_;
};  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clear_known_objects");

  ros::AsyncSpinner spinner(1); // Use 2 threads
  spinner.start();
  
  CurrentStateValidator cko;
  ros::waitForShutdown();
  
  return 0;
}

