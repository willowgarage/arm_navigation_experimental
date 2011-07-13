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
 *   * Neither the name of Willow Garage nor the names of its
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
 *
 *********************************************************************/

/* \author: Matthew Klingensmith */

#include <move_arm_warehouse/move_arm_utils.h>

using namespace std;
using namespace arm_navigation_msgs;
using namespace planning_scene_utils;
using namespace collision_space;
using namespace kinematics_msgs;
using namespace arm_navigation_msgs;
using namespace head_monitor_msgs;
using namespace move_arm_warehouse;
using namespace planning_environment;
using namespace planning_models;
using namespace std_msgs;
using namespace trajectory_msgs;
using namespace visualization_msgs;
using namespace arm_navigation_msgs;


std_msgs::ColorRGBA makeRandomColor(float brightness, float alpha)
{
  std_msgs::ColorRGBA toReturn;
  toReturn.a = alpha;

  toReturn.r = ((float)(random()) / (float) RAND_MAX)*(1.0f - brightness) + brightness;
  toReturn.g = ((float)(random()) / (float) RAND_MAX)*(1.0f - brightness) + brightness;
  toReturn.b = ((float)(random()) / (float) RAND_MAX)*(1.0f - brightness) + brightness;

  toReturn.r = min(toReturn.r, 1.0f);
  toReturn.g = min(toReturn.g, 1.0f);
  toReturn.b = min(toReturn.b, 1.0f);

  return toReturn;
}

///////////////////////
// PLANNING SCENE DATA
//////////////////////

PlanningSceneData::PlanningSceneData()
{
  setName("");
  setTimeStamp(ros::Time::now());
}

PlanningSceneData::PlanningSceneData(string name, ros::Time timestamp, PlanningScene scene)
{
  setName(name);
  setPlanningScene(scene);
  setTimeStamp(timestamp);
}

void PlanningSceneData::getRobotState(KinematicState* state)
{
  setRobotStateAndComputeTransforms(getPlanningScene().robot_state, *state);
}

////////////////////
//TRAJECTORY DATA
///////////////////

TrajectoryData::TrajectoryData()
{
  setCurrentState(NULL);
  setSource("");
  setGroupName("");
  setColor(makeRandomColor(0.3f, 0.6f));
  reset();
  setID("");
  showCollisions();
  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_counter_ = 0;
  trajectory_error_code_.val = ArmNavigationErrorCodes::SUCCESS;
}

TrajectoryData::TrajectoryData(string ID, string source, string groupName, JointTrajectory trajectory)
{
  setCurrentState(NULL);
  setID(ID);
  setSource(source);
  setGroupName(groupName);
  setTrajectory(trajectory);
  setColor(makeRandomColor(0.3f, 0.6f));
  reset();
  showCollisions();
  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_counter_ = 0;
  trajectory_error_code_.val = ArmNavigationErrorCodes::SUCCESS;
}

void TrajectoryData::moveThroughTrajectory(int step)
{
  unsigned int tsize = getTrajectorySize();

  if(tsize == 0 || getCurrentState() == NULL)
  {
    return;
  }

  if((int)getCurrentPoint() + step < 0)
  {
    setCurrentPoint(0);
  }
  else
  {
    setCurrentPoint((int)getCurrentPoint() + step);
  }
  if(getCurrentPoint() >= tsize - 1)
  {
    setCurrentPoint(tsize - 1);
    stop();
  }
  updateCurrentState();
}

void TrajectoryData::updateCurrentState()
{
  map<string, double> joint_values;
  for(unsigned int i = 0; i < getTrajectory().joint_names.size(); i++)
  {
    joint_values[getTrajectory().joint_names[i]] = getTrajectory().points[getCurrentPoint()].positions[i];
  }

  getCurrentState()->setKinematicState(joint_values);
}

void TrajectoryData::updateCollisionMarkers(CollisionModels* cm_, MotionPlanRequestData& motionPlanRequest, ros::ServiceClient& distance_state_validity_service_client_)
{
  if(areCollisionsVisible())
  {
    const KinematicState* state = getCurrentState();
    collision_markers_.markers.clear();
    if(state == NULL)
    {
      return;
    }
    std_msgs::ColorRGBA bad_color;
    bad_color.a = 1.0f;
    bad_color.r = 1.0f;
    bad_color.g = 0.0f;
    bad_color.b = 0.0f;
    cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(.2));
    const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
    Constraints empty_constraints;
    cm_->isKinematicStateValid(*state, jsg->getJointNames(), trajectory_error_code_, empty_constraints,
                               motionPlanRequest.getMotionPlanRequest().path_constraints, true);

    GetStateValidity::Request val_req;
    GetStateValidity::Response val_res;
    convertKinematicStateToRobotState(*state, ros::Time::now(), cm_->getWorldFrameId(), val_req.robot_state);

    if(!distance_state_validity_service_client_.call(val_req, val_res))
    {
      ROS_INFO_STREAM("Something wrong with distance server");
    }
  }
}

//////////////////////////////
// MOTION PLAN REQUEST DATA
////////////////////////////

MotionPlanRequestData::MotionPlanRequestData(KinematicState* robot_state)
{
  setSource("");
  setStartColor(makeRandomColor(0.3f, 0.6f));
  setEndColor(makeRandomColor(0.3f, 0.6f));
  setStartEditable(false);
  setEndEditable(false);
  setHasGoodIKSolution(true);
  setID("");
  show();
  showCollisions();

  start_state_ = new KinematicState(*robot_state);
  end_state_ = new KinematicState(*robot_state);
  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_counter_ = 0;
}

MotionPlanRequestData::MotionPlanRequestData(string ID, string source, MotionPlanRequest request, KinematicState* robot_state)
{

  start_state_ = new KinematicState(*robot_state);
  end_state_ = new KinematicState(*robot_state);

  setID(ID);
  setSource(source);
  setMotionPlanRequest(request);

  setStartColor(makeRandomColor(0.3f, 0.6f));
  setEndColor(makeRandomColor(0.3f, 0.6f));
  setStartEditable(false);
  setEndEditable(false);
  setHasGoodIKSolution(true);
  show();
  showCollisions();

  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_counter_ = 0;
}

void MotionPlanRequestData::updateGoalState()
{
  vector<JointConstraint>& constraints = getMotionPlanRequest().goal_constraints.joint_constraints;

  map<string, double> jointValues;
  for(size_t i = 0; i < constraints.size(); i++)
  {
    JointConstraint& constraint = constraints[i];
    jointValues[constraint.joint_name] = constraint.position;
  }

  end_state_->setKinematicState(jointValues);
}

void MotionPlanRequestData::updateStartState()
{
  setRobotStateAndComputeTransforms(getMotionPlanRequest().start_state, *start_state_);
}

void MotionPlanRequestData::setStartStateValues(std::map<std::string, double>& joint_values)
{
  setStateChanged(true);
  start_state_->setKinematicState(joint_values);
}

void MotionPlanRequestData::setGoalStateValues(std::map<std::string, double>& joint_values)
{
  setStateChanged(true);
  end_state_->setKinematicState(joint_values);

  int constraints = 0;
  for(std::map<std::string, double>::iterator it = joint_values.begin(); it != joint_values.end(); it++)
  {
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].joint_name = it->first;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].position= it->second;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].tolerance_above = 0.001;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].tolerance_below = 0.001;
    constraints++;
  }
}

void MotionPlanRequestData::updateCollisionMarkers(CollisionModels* cm_, ros::ServiceClient& distance_state_validity_service_client_)
{
  if(areCollisionsVisible())
  {
    const KinematicState* state = getStartState();
    collision_markers_.markers.clear();
    if(state == NULL)
    {
      return;
    }

    std_msgs::ColorRGBA bad_color;
    bad_color.a = 1.0f;
    bad_color.r = 1.0f;
    bad_color.g = 0.0f;
    bad_color.b = 0.0f;
    cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(.2));
    const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
    ArmNavigationErrorCodes code;
    Constraints empty_constraints;
    cm_->isKinematicStateValid(*state, jsg->getJointNames(), code, empty_constraints,
                               getMotionPlanRequest().path_constraints, true);

    GetStateValidity::Request val_req;
    GetStateValidity::Response val_res;
    convertKinematicStateToRobotState(*state, ros::Time::now(), cm_->getWorldFrameId(), val_req.robot_state);

    if(!distance_state_validity_service_client_.call(val_req, val_res))
    {
      ROS_INFO_STREAM("Something wrong with distance server");
    }

    {
      state = getGoalState();
      collision_markers_.markers.clear();
      if(state == NULL)
      {
        return;
      }
      std_msgs::ColorRGBA bad_color;
      bad_color.a = 1.0f;
      bad_color.r = 1.0f;
      bad_color.g = 0.0f;
      bad_color.b = 0.0f;
      cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(.2));
      const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
      ArmNavigationErrorCodes code;
      Constraints empty_constraints;
      cm_->isKinematicStateValid(*state, jsg->getJointNames(), code, empty_constraints,
                                 getMotionPlanRequest().path_constraints, true);

      GetStateValidity::Request val_req;
      GetStateValidity::Response val_res;
      convertKinematicStateToRobotState(*state, ros::Time::now(), cm_->getWorldFrameId(), val_req.robot_state);

      if(!distance_state_validity_service_client_.call(val_req, val_res))
      {
        ROS_INFO_STREAM("Something wrong with distance server");
      }
    }
  }
}

////////////////////////////////
// PLANNING SCENE EDITOR
///////////////////////////////


PlanningSceneEditor::PlanningSceneEditor()
{
  setRobotState(NULL, false);
  setCollisionModel(NULL, false);
  interactive_marker_server_ = NULL;
  planning_scene_map_ = NULL;
  trajectory_map_ = NULL;
  motion_plan_map_ = NULL;
  collision_aware_ik_services_ = NULL;
  non_collision_aware_ik_services_ = NULL;
  selectable_objects_ = NULL;
  ik_controllers_ = NULL;
  state_monitor_ = NULL;
}

PlanningSceneEditor::PlanningSceneEditor(PlanningSceneParameters& params)
{
  params_ = params;

  planning_scene_map_ = new map<string, PlanningSceneData>();
  trajectory_map_ = new map<string, TrajectoryData>();
  motion_plan_map_ = new map<string, MotionPlanRequestData>();
  collision_aware_ik_services_ = new map<string, ros::ServiceClient*>();
  non_collision_aware_ik_services_ = new map<string, ros::ServiceClient*>();
  selectable_objects_ = new map<string, SelectableObject>();
  ik_controllers_ = new map<string, IKController>();

  string robot_description_name = nh_.resolveName("robot_description", true);
  cm_ = new CollisionModels(robot_description_name);
  robot_state_ = new KinematicState(cm_->getKinematicModel());
  robot_state_->setKinematicStateToDefault();

  interactive_marker_server_ = new interactive_markers::InteractiveMarkerServer("planning_scene_warehouse_viewer_controls", "", false);

  collision_object_movement_feedback_ptr_ = boost::bind(&PlanningSceneEditor::collisionObjectMovementCallback, this, _1);
  collision_object_selection_feedback_ptr_ = boost::bind(&PlanningSceneEditor::collisionObjectSelectionCallback, this, _1);
  ik_control_feedback_ptr_ = boost::bind(&PlanningSceneEditor::IKControllerCallback, this, _1);

  joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState> ("joint_states", 10);
  vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker> (params.vis_topic_name_, 128);
  vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray> (params.vis_topic_name_ + "_array", 128);
  move_arm_warehouse_logger_reader_ = new MoveArmWarehouseLoggerReader();

  ros::service::waitForService(params.left_ik_name_);
  ros::service::waitForService(params.right_ik_name_);
  ros::service::waitForService(params.planner_service_name_);
  ros::service::waitForService(params.proximity_space_service_name_);
  ros::service::waitForService(params.proximity_space_validity_name_);

  left_ik_service_client_ = nh_.serviceClient<GetConstraintAwarePositionIK> (params.left_ik_name_, true);
  right_ik_service_client_ = nh_.serviceClient<GetConstraintAwarePositionIK> (params.right_ik_name_, true);
  non_coll_left_ik_service_client_ = nh_.serviceClient<GetPositionIK> (params.non_coll_left_ik_name_, true);
  non_coll_right_ik_service_client_ = nh_.serviceClient<GetPositionIK> (params.non_coll_right_ik_name_, true);
  planning_service_client_ = nh_.serviceClient<GetMotionPlan> (params.planner_service_name_, true);
  right_interpolate_service_client_ = nh_.serviceClient<GetMotionPlan> (params.right_interpolate_service_name_, true);
  left_interpolate_service_client_ = nh_.serviceClient<GetMotionPlan> (params.left_interpolate_service_name_, true);
  trajectory_filter_service_client_
      = nh_.serviceClient<FilterJointTrajectoryWithConstraints> (params.trajectory_filter_service_name_);
  distance_aware_service_client_ = nh_.serviceClient<GetMotionPlan> (params.proximity_space_service_name_, true);
  distance_state_validity_service_client_ = nh_.serviceClient<GetStateValidity> (params.proximity_space_validity_name_,
                                                                                 true);
  collision_proximity_planner_client_ = nh_.serviceClient<GetMotionPlan> (params.proximity_space_planner_name_, true);
  set_planning_scene_diff_client_ = nh_.serviceClient<SetPlanningSceneDiff> (params.set_planning_scene_diff_name_);

  (*collision_aware_ik_services_)[params.left_ik_link_] = &left_ik_service_client_;
  (*collision_aware_ik_services_)[params.right_ik_link_] = &right_ik_service_client_;
  (*non_collision_aware_ik_services_)[params.left_ik_link_] = &non_coll_left_ik_service_client_;
  (*non_collision_aware_ik_services_)[params.right_ik_link_] = &non_coll_right_ik_service_client_;

  menu_entry_maps_["Collision Object"] = MenuEntryMap();
  menu_entry_maps_["Collision Object Selection"] = MenuEntryMap();
  menu_entry_maps_["IK Control"] = MenuEntryMap();
  registerMenuEntry("Collision Object Selection", "Delete", collision_object_selection_feedback_ptr_);
  registerMenuEntry("Collision Object Selection", "Select", collision_object_selection_feedback_ptr_);
  registerMenuEntry("Collision Object", "Delete", collision_object_movement_feedback_ptr_);
  registerMenuEntry("Collision Object", "Deselect", collision_object_movement_feedback_ptr_);
  registerMenuEntry("IK Control", "Go To Last Good State", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Randomly Perturb", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Plan New Trajectory", ik_control_feedback_ptr_);
  registerMenuEntry("IK Control", "Filter Last Trajectory", ik_control_feedback_ptr_);

  if(params.use_robot_data_)
  {
    state_monitor_ = new KinematicModelStateMonitor(cm_, &transform_listenter_);
    state_monitor_->addOnStateUpdateCallback(boost::bind(&PlanningSceneEditor::jointStateCallback, this, _1));
  }
}

void PlanningSceneEditor::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  state_monitor_->setStateValuesFromCurrentValues(*robot_state_);
}

PlanningSceneEditor::~PlanningSceneEditor()
{
  setRobotState(NULL, true);
}


void PlanningSceneEditor::setCurrentPlanningScene(std::string ID, bool loadRequests, bool loadTrajectories)
{
  current_planning_scene_ID_ = ID;

  if(ID == "")
  {
    return;
  }

  for(map<string, SelectableObject>::iterator it = (*selectable_objects_).begin(); it != (*selectable_objects_).end(); it++)
  {
    interactive_marker_server_->erase(it->second.selection_marker_.name);
    interactive_marker_server_->erase(it->second.control_marker_.name);
  }

  (*selectable_objects_).clear();

  for(map<string, IKController>::iterator it = (*ik_controllers_).begin(); it != (*ik_controllers_).end(); it++)
  {
    interactive_marker_server_->erase(it->second.end_controller_.name);
    interactive_marker_server_->erase(it->second.start_controller_.name);
  }

  (*ik_controllers_).clear();

  deleteKinematicStates();
  motion_plan_map_->clear();
  trajectory_map_->clear();

  if((*planning_scene_map_).find(ID) != (*planning_scene_map_).end())
  {
    ROS_INFO("Getting requests associated with scene %s", ID.c_str());
    PlanningSceneData& scene = (*planning_scene_map_)[ID];
    ros::Time time = scene.getTimeStamp();
    for(size_t i = 0; i < scene.getPlanningScene().collision_objects.size(); i++)
    {
      std::stringstream ss;
      ss << "collision_object_";
      ss << i;
      createSelectableMarkerFromCollisionObject(scene.getPlanningScene().collision_objects[i], ss.str(), "");
    }

    if(loadRequests)
    {
      // Get the motion plan requests
      vector<string> IDs;
      vector<string> stageNames;
      vector<MotionPlanRequest> requests;
      getAllAssociatedMotionPlanRequests(time, IDs, stageNames, requests);
      createMotionPlanRequestData(ID, IDs, stageNames, requests);

      for(size_t j = 0; j < IDs.size(); j++)
      {
        MotionPlanRequest req;
        std::string motionID = IDs[j];

        MotionPlanRequestData& motionData = (*motion_plan_map_)[motionID];
        motionData.setPlanningSceneName(ID);

        std::vector<JointTrajectory> trajs;
        std::vector<string> sources;
        std::vector<string> IDs;
        std::vector<ros::Duration> durations;
        move_arm_warehouse_logger_reader_->getAssociatedJointTrajectories("", time, motionData.getID(), trajs, sources,
                                                                          IDs, durations);

        max_request_ID_++;

        if(loadTrajectories)
        {
          for(size_t k = 0; k < trajs.size(); k++)
          {
            TrajectoryData trajectoryData;
            trajectoryData.setTrajectory(trajs[k]);
            trajectoryData.setSource(sources[k]);
            trajectoryData.setID(IDs[k]);
            trajectoryData.setMotionPlanRequestID(motionData.getID());
            trajectoryData.setPlanningSceneName(ID);
            trajectoryData.setVisible(true);
            trajectoryData.setGroupName(motionData.getGroupName());
            trajectoryData.setDuration(durations[k]);
            trajectoryData.trajectory_error_code_ = error_map_[trajectoryData.getID()];

            scene.getTrajectories().push_back(trajectoryData.getID());
            motionData.getTrajectories().push_back(trajectoryData.getID());
            (*trajectory_map_)[IDs[k]] = trajectoryData;
            playTrajectory(motionData, trajectoryData);

            max_trajectory_ID_++;
          }

        }

      }

    }

  }
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::getTrajectoryMarkers(visualization_msgs::MarkerArray& arr)
{
  trajectory_map_->erase("");
  for(map<string, TrajectoryData>::iterator it = (*trajectory_map_).begin(); it != (*trajectory_map_).end(); it++)
  {

    if(it->second.isPlaying())
    {
      it->second.moveThroughTrajectory(5);
    }

    if(it->second.getCurrentState() == NULL)
    {
      continue;
    }

    it->second.updateCurrentState();
    if(it->second.shouldRefreshColors())
    {
      it->second.refresh_counter_ ++;

      if(it->second.refresh_counter_ > 1)
      {
        it->second.setHasRefreshedColors(true);
        it->second.refresh_counter_ = 0;
      }
    }
    else
    {
      if(it->second.isVisible())
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            cm_->getKinematicModel()->getModelGroup(it->second.getGroupName())->getUpdatedLinkModels();

        vector<string> lnames;
        lnames.resize(updated_links.size());
        for(unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }
        cm_->getRobotMarkersGivenState(*(it->second.getCurrentState()), arr, it->second.getColor(),
                                       it->first + "_trajectory", ros::Duration(.2), &lnames);
        cm_->getAttachedCollisionObjectMarkers(*(it->second.getCurrentState()), arr, it->first + "_trajectory",
                                               it->second.getColor(), ros::Duration(.2));
      }
    }

    if(it->second.areCollisionsVisible() && it->second.isVisible())
    {

      if(it->second.hasStateChanged())
      {
        it->second.updateCollisionMarkers(cm_, (*motion_plan_map_)[it->second.getMotionPlanRequestID()], distance_state_validity_service_client_);
        it->second.setStateChanged(false);
      }

      for(size_t i = 0; i < it->second.getCollisionMarkers().markers.size(); i++)
      {
        collision_markers_.markers.push_back(it->second.getCollisionMarkers().markers[i]);
      }
    }
  }
}

void PlanningSceneEditor::getMotionPlanningMarkers(visualization_msgs::MarkerArray& arr)
{
  for(map<string, MotionPlanRequestData>::iterator it = (*motion_plan_map_).begin(); it != (*motion_plan_map_).end(); it++)
  {
    MotionPlanRequestData& data = it->second;

    if(data.shouldRefreshColors())
    {
      data.refresh_counter_ ++;

      if(data.refresh_counter_ > 2)
      {
        data.setHasRefreshedColors(true);
        data.refresh_counter_ = 0;
      }
    }
    else
    {
      std_msgs::ColorRGBA fail_color;
      fail_color.a = 0.9;
      fail_color.r = 1.0;
      fail_color.g = 0.0;
      fail_color.b = 0.0;
      if(data.isStartVisible())
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            cm_->getKinematicModel()->getModelGroup(data.getMotionPlanRequest().group_name)->getUpdatedLinkModels();

        vector<string> lnames;
        lnames.resize(updated_links.size());
        for(unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }

        if(data.hasGoodIKSolution())
        {
          cm_->getRobotMarkersGivenState(*(data.getStartState()), arr, data.getStartColor(),
                                       it->first + "_start", ros::Duration(.2), &lnames);
        }
        else
        {
          cm_->getRobotMarkersGivenState(*(data.getStartState()), arr, fail_color,
                                         it->first + "_start", ros::Duration(.2), &lnames);
        }

      }

      if(data.isEndVisible())
      {
        const vector<const KinematicModel::LinkModel*>& updated_links =
            cm_->getKinematicModel()->getModelGroup(data.getMotionPlanRequest().group_name)->getUpdatedLinkModels();

        vector<string> lnames;
        lnames.resize(updated_links.size());
        for(unsigned int i = 0; i < updated_links.size(); i++)
        {
          lnames[i] = updated_links[i]->getName();
        }

        if(data.hasGoodIKSolution())
        {
          cm_->getRobotMarkersGivenState(*(data.getGoalState()), arr, data.getEndColor(),
                                         it->first + "_end", ros::Duration(.2), &lnames);
        }
        else
        {
          cm_->getRobotMarkersGivenState(*(data.getGoalState()), arr, fail_color,
                                         it->first + "_end", ros::Duration(.2), &lnames);
        }
      }
    }

    if(it->second.areCollisionsVisible() && (it->second.isStartVisible() || it->second.isEndVisible()))
    {
      if(it->second.hasStateChanged())
      {
        it->second.updateCollisionMarkers(cm_, distance_state_validity_service_client_);
        it->second.setStateChanged(false);
      }
      for(size_t i = 0; i < it->second.getCollisionMarkers().markers.size(); i++)
      {
        collision_markers_.markers.push_back(it->second.getCollisionMarkers().markers[i]);
      }
    }

  }
}

void PlanningSceneEditor::createMotionPlanRequest(planning_models::KinematicState& start_state,
                                                  planning_models::KinematicState& end_state, std::string group_name,
                                                  std::string end_effector_name, bool constrain, std::string planning_scene_ID,
                                                  std::string& motionPlan_ID_Out)
{
  MotionPlanRequest motion_plan_request;
  motion_plan_request.group_name = group_name;
  motion_plan_request.num_planning_attempts = 1;
  motion_plan_request.allowed_planning_time = ros::Duration(1);
  const KinematicState::JointStateGroup* jsg = end_state.getJointStateGroup(group_name);
  motion_plan_request.goal_constraints.joint_constraints.resize(jsg->getJointNames().size());

  vector<double> joint_values;
  jsg->getKinematicStateValues(joint_values);
  for(unsigned int i = 0; i < jsg->getJointNames().size(); i++)
  {
    motion_plan_request.goal_constraints.joint_constraints[i].joint_name = jsg->getJointNames()[i];
    motion_plan_request.goal_constraints.joint_constraints[i].position = joint_values[i];
    motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.001;
    motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.001;
  }
  if(constrain)
  {
    motion_plan_request.group_name += "_cartesian";
    motion_plan_request.goal_constraints.position_constraints.resize(1);
    motion_plan_request.goal_constraints.orientation_constraints.resize(1);
    geometry_msgs::PoseStamped end_effector_wrist_pose;
    tf::poseTFToMsg(end_state.getLinkState(end_effector_name)->getGlobalLinkTransform(), end_effector_wrist_pose.pose);
    end_effector_wrist_pose.header.frame_id = cm_->getWorldFrameId();
    poseStampedToPositionOrientationConstraints(end_effector_wrist_pose, end_effector_name,
                                                motion_plan_request.goal_constraints.position_constraints[0],
                                                motion_plan_request.goal_constraints.orientation_constraints[0]);
    motion_plan_request.path_constraints.orientation_constraints.resize(1);
    determinePitchRollConstraintsGivenState(end_state, end_effector_name,
                                            motion_plan_request.goal_constraints.orientation_constraints[0],
                                            motion_plan_request.path_constraints.orientation_constraints[0]);
  }
  convertKinematicStateToRobotState(start_state, ros::Time::now(), cm_->getWorldFrameId(), motion_plan_request.start_state);

  std::string id = generateNewMotionPlanID();
  MotionPlanRequestData data(id, "planner", motion_plan_request, robot_state_);
  data.setGroupName(motion_plan_request.group_name);
  data.setEndEffectorLink(end_effector_name);
  StateRegistry start;
  start.state = data.getStartState();
  start.source = "Motion Plan Request Data Start from line 731";
  StateRegistry end;
  end.state = data.getGoalState();
  end.source = "Motion Plan Request Data End from line 734";
  states_.push_back(start);
  states_.push_back(end);
  (*motion_plan_map_)[id] = data;
  data.setPlanningSceneName(planning_scene_ID);

  PlanningSceneData& planningSceneData = (*planning_scene_map_)[planning_scene_ID];
  planningSceneData.getRequests().push_back(data.getID());

  motionPlan_ID_Out = data.getID();

  createIkControllersFromMotionPlanRequest(data);

  sendPlanningScene(planningSceneData);
}

bool PlanningSceneEditor::planToKinematicState(KinematicState& state, string group_name, string end_effector_name,
                                               bool constrain, std::string& trajectoryID_Out, string& planning_scene_ID)
{
  std::string motionPlanID;
  createMotionPlanRequest(*robot_state_, state, group_name, end_effector_name, constrain, planning_scene_ID, motionPlanID);
  MotionPlanRequestData& data = (*motion_plan_map_)[motionPlanID];
  return planToRequest(data, trajectoryID_Out);
}

bool PlanningSceneEditor::planToRequest(std::string requestID, std::string& trajectoryID_Out)
{
  return planToRequest((*motion_plan_map_)[requestID], trajectoryID_Out);
}

bool PlanningSceneEditor::planToRequest(MotionPlanRequestData& data, std::string& trajectoryID_Out)
{
  GetMotionPlan::Request plan_req;
  plan_req.motion_plan_request = data.getMotionPlanRequest();
  GetMotionPlan::Response plan_res;

  if(!planning_service_client_.call(plan_req, plan_res))
  {
    ROS_INFO("Something wrong with planner client");
    return false;
  }

  std::string ID = generateNewTrajectoryID();
  std::string source = "planner";

  TrajectoryData& trajectoryData = (*trajectory_map_)[trajectoryID_Out];
  trajectoryData.setTrajectory(plan_res.trajectory.joint_trajectory);
  trajectoryData.setGroupName(data.getMotionPlanRequest().group_name);
  trajectoryData.setMotionPlanRequestID(data.getID());
  trajectoryID_Out = ID;
  trajectoryData.setPlanningSceneName(data.getPlanningSceneName());
  trajectoryData.setID(trajectoryID_Out);
  trajectoryData.setSource("planner");
  trajectoryData.setDuration(plan_res.planning_time);
  trajectoryData.setVisible(true);
  trajectoryData.setPlaying(true);

  PlanningSceneData& planningSceneData = (*planning_scene_map_)[data.getPlanningSceneName()];
  planningSceneData.getTrajectories().push_back(trajectoryID_Out);

  if(plan_res.error_code.val != plan_res.error_code.SUCCESS)
  {
    trajectoryData.trajectory_error_code_ = plan_res.error_code;
    ROS_INFO_STREAM("Bad planning error code " << plan_res.error_code.val);
    trajectoryData.trajectory_error_code_ = plan_res.error_code;
    (*trajectory_map_)[trajectoryID_Out] = trajectoryData;
    data.getTrajectories().push_back(trajectoryData.getID());

    return false;
  }
  trajectoryData.trajectory_error_code_ = plan_res.error_code;
  (*trajectory_map_)[trajectoryID_Out] = trajectoryData;
  data.getTrajectories().push_back(trajectoryData.getID());
  return true;
}

void PlanningSceneEditor::determinePitchRollConstraintsGivenState(const KinematicState& state,
                                                                  std::string& end_effector_link,
                                                                  OrientationConstraint& goal_constraint,
                                                                  OrientationConstraint& path_constraint)
{
  btTransform cur = state.getLinkState(end_effector_link)->getGlobalLinkTransform();
  //btScalar roll, pitch, yaw;
  //cur.getBasis().getRPY(roll,pitch,yaw);
  goal_constraint.header.frame_id = cm_->getWorldFrameId();
  goal_constraint.header.stamp = ros::Time::now();
  goal_constraint.link_name = end_effector_link;
  tf::quaternionTFToMsg(cur.getRotation(), goal_constraint.orientation);
  goal_constraint.absolute_roll_tolerance = 0.04;
  goal_constraint.absolute_pitch_tolerance = 0.04;
  goal_constraint.absolute_yaw_tolerance = M_PI;
  path_constraint.header.frame_id = cm_->getWorldFrameId();
  path_constraint.header.stamp = ros::Time::now();
  path_constraint.link_name = end_effector_link;
  tf::quaternionTFToMsg(cur.getRotation(), path_constraint.orientation);
  path_constraint.type = path_constraint.HEADER_FRAME;
  path_constraint.absolute_roll_tolerance = 0.1;
  path_constraint.absolute_pitch_tolerance = 0.1;
  path_constraint.absolute_yaw_tolerance = M_PI;
}

void PlanningSceneEditor::printTrajectoryPoint(const vector<string>& joint_names, const vector<double>& joint_values)
{
  for(unsigned int i = 0; i < joint_names.size(); i++)
  {
    ROS_INFO_STREAM("Joint name " << joint_names[i] << " value " << joint_values[i]);
  }
}

bool PlanningSceneEditor::filterTrajectory(MotionPlanRequestData& requestData, TrajectoryData& trajectory, std::string& filter_ID)
{
  FilterJointTrajectoryWithConstraints::Request filter_req;
  FilterJointTrajectoryWithConstraints::Response filter_res;

  convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(), filter_req.start_state);

  filter_req.trajectory = trajectory.getTrajectory();
  filter_req.group_name = trajectory.getGroupName();
  filter_req.goal_constraints = requestData.getMotionPlanRequest().goal_constraints;
  filter_req.path_constraints = requestData.getMotionPlanRequest().path_constraints;
  filter_req.allowed_time = ros::Duration(2.0);


  if(!trajectory_filter_service_client_.call(filter_req, filter_res))
  {
   ROS_INFO("Problem with trajectory filter");
   return false;
  }

  TrajectoryData data(generateNewTrajectoryID(), "filter", trajectory.getGroupName(), filter_res.trajectory);
  data.setPlanningSceneName(requestData.getPlanningSceneName());
  data.setMotionPlanRequestID(requestData.getID());
  data.setDuration(ros::Duration(2));
  requestData.getTrajectories().push_back(data.getID());
  (*planning_scene_map_)[requestData.getPlanningSceneName()].getTrajectories().push_back(data.getID());
  if(filter_res.error_code.val != filter_res.error_code.SUCCESS)
  {
   ROS_INFO_STREAM("Bad trajectory_filter error code " << filter_res.error_code.val);
   return false;
  }
  data.trajectory_error_code_ = filter_res.error_code;
  playTrajectory(requestData, data);

  (*trajectory_map_)[data.getID()] = data;

  filter_ID = data.getID();

  return true;
}

void PlanningSceneEditor::updateJointStates()
{

  if(params_.use_robot_data_)
  {
    return;
  }

  sensor_msgs::JointState msg;
  msg.header.frame_id = cm_->getWorldFrameId();
  msg.header.stamp = ros::Time::now();

  vector<KinematicState::JointState*> jointStates = getRobotState()->getJointStateVector();

  map<string, double> stateMap;
  getRobotState()->getKinematicStateValues(stateMap);
  getRobotState()->setKinematicState(stateMap);

  for(size_t i = 0; i < jointStates.size(); i++)
  {
    KinematicState::JointState* state = jointStates[i];
    msg.name.push_back(state->getName());

    // Assume that joints only have one value.
    if(state->getJointStateValues().size() > 0)
    {
      msg.position.push_back(state->getJointStateValues()[0]);
    }
    else
    {
      msg.position.push_back(0.0f);
    }
  }
  joint_state_publisher_.publish(msg);

}

void PlanningSceneEditor::sendMarkers()
{
  lockScene();
  sendTransformsAndClock();
  visualization_msgs::MarkerArray arr;

  getTrajectoryMarkers(arr);
  getMotionPlanningMarkers(arr);

  vis_marker_array_publisher_.publish(arr);
  vis_marker_array_publisher_.publish(collision_markers_);
  collision_markers_.markers.clear();
  unlockScene();
}

bool PlanningSceneEditor::getPlanningSceneOutcomes(const ros::Time& time, vector<string>& pipeline_stages,
                                                   vector<ArmNavigationErrorCodes>& error_codes, map<std::string, ArmNavigationErrorCodes>& error_map)
{
  std::vector<string> names;
  if(!move_arm_warehouse_logger_reader_->getAssociatedOutcomes("", time, pipeline_stages, error_codes, names))
  {
    ROS_WARN_STREAM("No outcome associated with planning scene");
    return false;
  }

  for(size_t i = 0; i < error_codes.size(); i++)
  {
    error_map[names[i]] = error_codes[i];
  }

  return true;
}

std::string PlanningSceneEditor::createNewPlanningScene()
{
  lock_scene_.lock();


  string robot_description_name = nh_.resolveName("robot_description", true);
  cm_ = new CollisionModels(robot_description_name);

  if(robot_state_ == NULL)
  {
    robot_state_ = new KinematicState(cm_->getKinematicModel());
    robot_state_->setKinematicStateToDefault();
  }

  PlanningSceneData data;
  data.setName(generateNewPlanningSceneID());
  data.setTimeStamp(ros::Time::now());

  convertKinematicStateToRobotState(*robot_state_, data.getTimeStamp(), cm_->getWorldFrameId(),
                                    data.getPlanningScene().robot_state);
  //end_effector_state_ = planning_state_;

  vector<arm_navigation_msgs::CollisionObject> objects;
  data.getPlanningScene().set_collision_objects_vec(objects);
  data.getPlanningScene().set_collision_objects_size(0);

  sendPlanningScene(data);

  lock_scene_.unlock();

  updateJointStates();

  return data.getName();
}

void PlanningSceneEditor::loadAllWarehouseData()
{
  max_planning_scene_ID_ = 0;
  max_request_ID_ = 0;
  max_trajectory_ID_= 0;
  max_collision_object_ID_ = 0;

  motion_plan_map_->clear();
  trajectory_map_->clear();
  planning_scene_map_->clear();
  states_.clear();
  vector<ros::Time> planningSceneTimes;
  getAllAvailableWarehousePlanningScenes(planningSceneTimes);

  // For each planning scene
  for(size_t i = 0; i < planningSceneTimes.size(); i++)
  {
    ros::Time& time = planningSceneTimes[i];
    std::string ID;
    // Load it
    loadPlanningScene(time, ID);

    ROS_INFO("Got planning scene %s from warehouse.", ID.c_str());
    PlanningSceneData& data = (*planning_scene_map_)[ID];
    getPlanningSceneOutcomes(time, data.getPipelineStages(), data.getErrorCodes(), error_map_);
  }
}

void PlanningSceneEditor::savePlanningScene(PlanningSceneData& data)
{
  move_arm_warehouse_logger_reader_->pushPlanningSceneToWarehouse(data.getPlanningScene());

  ROS_INFO("Saving Planning Scene %s", data.getName().c_str());

  for(size_t i = 0; i < data.getRequests().size(); i++)
  {
    MotionPlanRequestData& req = (*motion_plan_map_)[data.getRequests()[i]];
    move_arm_warehouse_logger_reader_->pushMotionPlanRequestToWarehouse(data.getPlanningScene(),req.getSource(), req.getMotionPlanRequest(),req.getID());
    ROS_INFO("Saving Request %s", req.getID().c_str());
    for(size_t j = 0; j < req.getTrajectories().size(); j++)
    {
      TrajectoryData& traj = (*trajectory_map_)[req.getTrajectories()[j]];
      move_arm_warehouse_logger_reader_->pushJointTrajectoryToWarehouse(data.getPlanningScene(), traj.getSource(), traj.getDuration(), traj.getTrajectory(),traj.getID(), traj.getMotionPlanRequestID());
      move_arm_warehouse_logger_reader_->pushOutcomeToWarehouse(data.getPlanningScene(), traj.getSource(), traj.trajectory_error_code_,traj.getID());
      ROS_INFO("Saving Trajectory %s", traj.getID().c_str());
    }
  }
}

bool PlanningSceneEditor::getAllAvailableWarehousePlanningScenes(vector<ros::Time>& planning_scene_times)
{
  move_arm_warehouse_logger_reader_->getAvailablePlanningSceneList("", last_creation_time_query_);
  planning_scene_times = last_creation_time_query_;
  return true;
}

bool PlanningSceneEditor::loadPlanningScene(const ros::Time& time, std::string& ID)
{
  assert(planning_scene_map_ != NULL);
  PlanningSceneData data;
  data.setTimeStamp(time);
  data.setName(generateNewPlanningSceneID());
  if(!move_arm_warehouse_logger_reader_->getPlanningScene("", time, data.getPlanningScene()))
  {
    return false;
  }

  std::pair<string, PlanningSceneData> p(data.getName(),data);
  planning_scene_map_->insert(p);
  ID = data.getName();
  return true;
}

bool PlanningSceneEditor::getAllAssociatedMotionPlanRequests(const ros::Time& time, vector<string>& IDs, vector<string>& stages, vector<MotionPlanRequest>& requests)
{
  move_arm_warehouse_logger_reader_->getAssociatedMotionPlanRequests("", time, stages, IDs, requests);
  return true;
}

void PlanningSceneEditor::deleteKinematicStates()
{
  lockScene();
  assert(trajectory_map_ != NULL);
  std::vector<KinematicState*> removals;
  for(map<string, TrajectoryData>::iterator it = (*trajectory_map_).begin(); it != (*trajectory_map_).end(); it++)
  {
    removals.push_back(it->second.getCurrentState());
    it->second.reset();
  }

  for(map<string, MotionPlanRequestData>::iterator it = (*motion_plan_map_).begin(); it != (*motion_plan_map_).end(); it++)
  {
    removals.push_back(it->second.getStartState());
    removals.push_back(it->second.getGoalState());
    it->second.reset();
  }

  for(size_t i = 0; i < states_.size(); i++)
  {
    if(states_[i].state != NULL)
    {
      bool shouldBreak = false;
      for(size_t j = 0; j < removals.size(); j++)
      {
        if(states_[i].state == removals[j])
        {
          shouldBreak = true;
          break;
        }
      }

      if(shouldBreak)
      {
        continue;
      }
      ROS_INFO("Missed a state from %s!", states_[i].source.c_str());
      delete states_[i].state;
      states_[i].state = NULL;
    }
  }
  states_.clear();
  unlockScene();
}

bool PlanningSceneEditor::sendPlanningScene(PlanningSceneData& data)
{
  lock_scene_.lock();
  last_collision_set_error_code_.val = 0;


  SetPlanningSceneDiff::Request planning_scene_req;
  SetPlanningSceneDiff::Response planning_scene_res;

  planning_scene_req.planning_scene_diff = data.getPlanningScene();
  planning_scene_req.planning_scene_diff.collision_objects.clear();
  planning_scene_req.planning_scene_diff.set_collision_objects_size(0);
  convertKinematicStateToRobotState(*robot_state_, ros::Time::now(), cm_->getWorldFrameId(),
                                    planning_scene_req.planning_scene_diff.robot_state);

  deleteKinematicStates();

  if(robot_state_ != NULL)
  {
    cm_->revertPlanningScene(robot_state_);
    robot_state_ = NULL;
  }


  vector<string> removals;
  // Handle additions and removals of planning scene objects.
  for(map<string, SelectableObject>::const_iterator it = (*selectable_objects_).begin(); it
      != (*selectable_objects_).end(); it++)
  {
    string name = it->first;
    arm_navigation_msgs::CollisionObject object = it->second.collision_object_;

    // Add or remove objects.
    if(object.operation.operation != arm_navigation_msgs::CollisionObjectOperation::REMOVE)
    {
      planning_scene_req.planning_scene_diff.collision_objects.push_back(object);
    }
    else
    {
      removals.push_back(it->first);
      interactive_marker_server_->erase(it->first);
    }
  }


  if(!set_planning_scene_diff_client_.call(planning_scene_req, planning_scene_res))
  {
    ROS_WARN("Can't get planning scene");
    return false;
  }

  // Delete collision poles from the map which were removed.
  for(size_t i = 0; i < removals.size(); i++)
  {
    (*selectable_objects_).erase(removals[i]);
  }

  data.setPlanningScene(planning_scene_res.planning_scene);

  setRobotState(cm_->setPlanningScene(data.getPlanningScene()));

  if(getRobotState() == NULL)
  {
    ROS_ERROR("Something wrong with planning scene");
    lock_scene_.unlock();
    return false;
  }
  robot_state_joint_values_.clear();
  robot_state_->getKinematicStateValues(robot_state_joint_values_);

  for(map<string, MotionPlanRequestData>::iterator it = motion_plan_map_->begin(); it != motion_plan_map_->end(); it++)
  {
    it->second.setStartState(new KinematicState(*robot_state_));
    it->second.setGoalState(new KinematicState(*robot_state_));
    StateRegistry start;
    start.state = it->second.getStartState();
    start.source = "Motion Plan Start After Sending Scene";
    states_.push_back(start);
    StateRegistry end;
    end.state = it->second.getGoalState();
    end.source = "Motion Plan End After Sending Scene";
    states_.push_back(end);
    it->second.updateStartState();
    it->second.updateGoalState();
  }


  //setShowCollisions(false);

  lock_scene_.unlock();
  return true;
}

void PlanningSceneEditor::createMotionPlanRequestData(std::string planning_scene_ID,std::vector<std::string>& IDs,
                                 std::vector<std::string>& stages,
                                 std::vector<arm_navigation_msgs::MotionPlanRequest>& requests)
{
  for(size_t i = 0; i < requests.size(); i++)
  {
    MotionPlanRequest& mpr = requests[i];
    cm_->disableCollisionsForNonUpdatedLinks(mpr.group_name);

    lock_scene_.lock();
    setRobotStateAndComputeTransforms(mpr.start_state, *robot_state_);

    GetMotionPlan::Request plan_req;
    plan_req.motion_plan_request = mpr;
    GetMotionPlan::Response plan_res;

    if(!distance_aware_service_client_.call(plan_req, plan_res))
    {
      ROS_INFO("Something wrong with distance client");
    }

    MotionPlanRequestData data(robot_state_);
    data.setID(IDs[i]);
    data.setMotionPlanRequest(mpr);
    data.setPlanningSceneName(planning_scene_ID);
    data.setGroupName(mpr.group_name);
    data.setSource("planner");

    StateRegistry start;
    start.state = data.getStartState();
    start.source = "Motion Plan Request Data Start from createRequest";
    StateRegistry end;
    end.state = data.getGoalState();
    end.source = "Motion Plan Request Data End from createRequest";
    states_.push_back(start);
    states_.push_back(end);

    const KinematicModel::GroupConfig& config = cm_->getKinematicModel()->getJointModelGroupConfigMap().at(mpr.group_name);
    std::string tip = config.tip_link_;
    data.setEndEffectorLink(tip);

    PlanningSceneData& planningSceneData = (*planning_scene_map_)[planning_scene_ID];
    planningSceneData.getRequests().push_back(data.getID());

    (*motion_plan_map_)[data.getID()] = data;
    if(send_collision_markers_)
    {
      updateCurrentCollisionSet(collision_marker_state_, mpr.group_name, data);
    }
    lock_scene_.unlock();

    createIkControllersFromMotionPlanRequest(data);

  }
}



bool PlanningSceneEditor::getMotionPlanRequest(const ros::Time& time, const string& stage, MotionPlanRequest& mpr,
                                               string& ID, string& planning_scene_ID)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedMotionPlanRequest("", time, stage, mpr, ID))
  {
    ROS_INFO_STREAM("No request with stage " << stage);
    return false;
  }

  cm_->disableCollisionsForNonUpdatedLinks(mpr.group_name);

  lock_scene_.lock();
  setRobotStateAndComputeTransforms(mpr.start_state, *robot_state_);

  GetMotionPlan::Request plan_req;
  plan_req.motion_plan_request = mpr;
  GetMotionPlan::Response plan_res;

  if(!distance_aware_service_client_.call(plan_req, plan_res))
  {
    ROS_INFO("Something wrong with distance client");
  }

  MotionPlanRequestData data(robot_state_);
  data.setID(generateNewMotionPlanID());

  data.setMotionPlanRequest(mpr);
  data.setPlanningSceneName(planning_scene_ID);
  data.setGroupName(mpr.group_name);
  data.setSource("planner");
  StateRegistry start;
  start.state = data.getStartState();
  start.source = "Motion Plan Request Data Start from loadRequest";
  StateRegistry end;
  end.state = data.getGoalState();
  end.source = "Motion Plan Request Data End from line loadRequest";
  states_.push_back(start);
  states_.push_back(end);

  const KinematicModel::GroupConfig& config = cm_->getKinematicModel()->getJointModelGroupConfigMap().at(mpr.group_name);
  std::string tip = config.tip_link_;
  data.setEndEffectorLink(tip);

  PlanningSceneData& planningSceneData = (*planning_scene_map_)[planning_scene_ID];
  planningSceneData.getRequests().push_back(data.getID());

  (*motion_plan_map_)[data.getID()] = data;
  ID = data.getID();
  if(send_collision_markers_)
  {
    updateCurrentCollisionSet(collision_marker_state_, mpr.group_name, data);
  }
  lock_scene_.unlock();

  createIkControllersFromMotionPlanRequest(data);
  return true;
}

bool PlanningSceneEditor::getAllAssociatedTrajectories(const ros::Time& time, vector<string>& trajectory_sources)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedJointTrajectorySources("", time, trajectory_sources))
  {
    return false;
  }
  return true;
}

bool PlanningSceneEditor::getAllAssociatedPausedStates(const ros::Time& time, vector<ros::Time>& paused_times)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedPausedStates("", time, paused_times))
  {
    return false;
  }
  return true;
}

bool PlanningSceneEditor::getPausedState(const ros::Time& time, const ros::Time& paused_time,
                                         HeadMonitorFeedback& paused_state)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedPausedState("", time, paused_time, paused_state))
  {
    return false;
  }
  return true;
}

bool PlanningSceneEditor::loadAndPlayTrajectory(MotionPlanRequestData& requestData, const ros::Time& time,
                                                const string& source_name, unsigned int num, std::string& ID)
{
  JointTrajectory traj;
  ros::Duration dur;

  if(!move_arm_warehouse_logger_reader_->getAssociatedJointTrajectory("", time, source_name, num, dur, traj))
  {
    ROS_INFO_STREAM("No source trajectory with name " << source_name);
    return false;
  }

  TrajectoryData data;
  data.setDuration(dur);
  data.setSource(source_name);
  data.setTrajectory(traj);
  data.setID(generateNewTrajectoryID());
  data.setPlanningSceneName(requestData.getPlanningSceneName());
  data.setMotionPlanRequestID(requestData.getID());
  data.setGroupName(requestData.getGroupName());
  data.setBadPoint(-1);
  requestData.getTrajectories().push_back(data.getID());

  (*planning_scene_map_)[requestData.getPlanningSceneName()].getTrajectories().push_back(data.getID());

  (*trajectory_map_)[data.getID()] = data;
  ID = data.getID();
  return playTrajectory(requestData, data);
}

bool PlanningSceneEditor::playTrajectory(MotionPlanRequestData& requestData, TrajectoryData& data)
{
  lock_scene_.lock();
  for(size_t i = 0; i < states_.size(); i++)
  {
    if(states_[i].state == data.getCurrentState())
    {
      states_[i].state = NULL;
    }
  }

  data.reset();

  data.play();
  data.setVisible(true);
  if(data.getTrajectory().points.size() == 0)
  {
    lock_scene_.unlock();
    return false;
  }
  if(data.getCurrentState() == NULL)
  {
    data.setCurrentState(new KinematicState(*robot_state_));
    StateRegistry currentState;
    currentState.state = data.getCurrentState();
    currentState.source = "Trajectory from play trajectory";
    states_.push_back(currentState);
  }
  data.setCurrentPoint(0);
  vector<ArmNavigationErrorCodes> trajectory_error_codes;
  cm_->isJointTrajectoryValid(*data.getCurrentState(), data.getTrajectory(),
                              requestData.getMotionPlanRequest().goal_constraints,
                              requestData.getMotionPlanRequest().path_constraints, data.trajectory_error_code_,
                              trajectory_error_codes, false);

  if(data.trajectory_error_code_.val != data.trajectory_error_code_.SUCCESS)
  {
    if(trajectory_error_codes.size() > 0)
    {
      data.setBadPoint(trajectory_error_codes.size() - 1);
    }
    else
    {
      data.setBadPoint(0);
    }
  }
  else
  {
    data.setBadPoint(-1);
  }

  data.moveThroughTrajectory(0);
  lock_scene_.unlock();
  return true;
}

void PlanningSceneEditor::setShowCollisions(bool send_collision_markers, const string& csd,
                                            std::string& arm_group_name, MotionPlanRequestData& data)
{
  lock_scene_.lock();
  send_collision_markers_ = send_collision_markers;
  if(send_collision_markers_)
  {
    updateCurrentCollisionSet(csd, arm_group_name, data);
  }
  else
  {
    collision_markers_.markers.clear();
  }
  lock_scene_.unlock();
}

void PlanningSceneEditor::updateCurrentCollisionSet(const string& csd, std::string& arm_group_name,
                                                    MotionPlanRequestData& data)
{
  lock_scene_.lock();
  const KinematicState* state = NULL;
  if((*trajectory_map_).find(csd) != (*trajectory_map_).end())
  {
    state = (*trajectory_map_)[csd].getCurrentState();
  }
  else if(csd == "paused")
  {
    state = paused_collision_state_;
  }
  else
  {
    state = robot_state_;
  }

  collision_marker_state_ = csd;
  collision_markers_.markers.clear();
  if(state == NULL)
  {
    lock_scene_.unlock();
    last_collision_set_error_code_.val = 0;
    return;
  }
  if(csd != "none")
  {
    cm_->getAllCollisionPointMarkers(*state, collision_markers_, point_color_, ros::Duration(.2));
    const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(arm_group_name);
    Constraints empty_constraints;
    cm_->isKinematicStateValid(*state, jsg->getJointNames(), last_collision_set_error_code_, empty_constraints,
                               data.getMotionPlanRequest().path_constraints, true);

    GetStateValidity::Request val_req;
    GetStateValidity::Response val_res;
    convertKinematicStateToRobotState(*state, ros::Time::now(), cm_->getWorldFrameId(), val_req.robot_state);

    if(!distance_state_validity_service_client_.call(val_req, val_res))
    {
      ROS_INFO_STREAM("Something wrong with distance server");
    }
  }
  else
  {
    last_collision_set_error_code_.val = 0;
  }
  lock_scene_.unlock();
}

void PlanningSceneEditor::createSelectableMarkerFromCollisionObject(CollisionObject& object, string name, string description)
{
  SelectableObject selectable;
  selectable.ID_ = name;
  selectable.collision_object_ = object;
  selectable.control_marker_.pose = object.poses[0];
  selectable.control_marker_.header.frame_id = "/" + cm_->getWorldFrameId();
  selectable.control_marker_.header.stamp = ros::Time::now();

  selectable.selection_marker_.pose = object.poses[0];
  selectable.selection_marker_.header.frame_id = "/" + cm_->getWorldFrameId();
  selectable.selection_marker_.header.stamp = ros::Time::now();

  InteractiveMarkerControl button;
  button.name = name;
  button.interaction_mode = InteractiveMarkerControl::BUTTON;
  button.description = description;
  float maxScale = 0.0f;

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    arm_navigation_msgs::Shape& shape = object.shapes[i];
    Marker mark;
    mark.color.a = 1.0;
    mark.color.r = 0.6;
    mark.color.g = 0.6;
    mark.color.b = 0.6;
    //mark.pose = object.poses[i];


    switch(shape.type)
    {
      case arm_navigation_msgs::Shape::BOX:
        mark.type = Marker::CUBE;
        mark.scale.x = shape.dimensions[0] * 1.01f;
        mark.scale.y = shape.dimensions[1] * 1.01f;
        mark.scale.z = shape.dimensions[2] * 1.01f;
        break;

      case arm_navigation_msgs::Shape::CYLINDER:
        mark.type = Marker::CYLINDER;
        mark.scale.x = shape.dimensions[0] * 2.01f;
        mark.scale.y = shape.dimensions[0] * 2.01f;
        mark.scale.z = shape.dimensions[1] * 1.01f;
        break;

      case arm_navigation_msgs::Shape::SPHERE:
        mark.type = Marker::SPHERE;
        mark.scale.x = shape.dimensions[0] * 2.01f;
        mark.scale.y = shape.dimensions[0] * 2.01f;
        mark.scale.z = shape.dimensions[0] * 2.01f;
        break;
      case arm_navigation_msgs::Shape::MESH:
        mark.type = Marker::CUBE;
        ROS_WARN("Attempting to get selectable marker as mesh resource, but mesh resources are not supported!");
        break;
    }

    button.markers.push_back(mark);

    if(mark.scale.x > maxScale)
    {
      maxScale = mark.scale.x * 1.05f;
    }

    if(mark.scale.y > maxScale)
    {
      maxScale = mark.scale.y * 1.05f;
    }

    if(mark.scale.z > maxScale)
    {
      maxScale = mark.scale.z * 1.05f;
    }
  }

  selectable.selection_marker_.controls.push_back(button);

  InteractiveMarkerControl sixDof;
  sixDof.orientation.w = 1;
  sixDof.orientation.x = 1;
  sixDof.orientation.y = 0;
  sixDof.orientation.z = 0;
  sixDof.always_visible = false;
  sixDof.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);
  sixDof.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);

  sixDof.orientation.w = 1;
  sixDof.orientation.x = 0;
  sixDof.orientation.y = 1;
  sixDof.orientation.z = 0;
  sixDof.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);
  sixDof.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);

  sixDof.orientation.w = 1;
  sixDof.orientation.x = 0;
  sixDof.orientation.y = 0;
  sixDof.orientation.z = 1;
  sixDof.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);
  sixDof.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  selectable.control_marker_.controls.push_back(sixDof);

  selectable.control_marker_.controls.push_back(button);

  selectable.control_marker_.name = name + "_control";
  selectable.selection_marker_.name = name + "_selection";

  selectable.control_marker_.scale = maxScale;
  interactive_marker_server_->insert(selectable.selection_marker_, collision_object_selection_feedback_ptr_);
  (*selectable_objects_)[name] = selectable;

  menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_, selectable.selection_marker_.name);

  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::IKControllerCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::string ID = "";
  PositionType type = StartPosition;

  bool findIKSolution = false;
  if(feedback->marker_name.rfind("_start_control") != std::string::npos)
  {
    ID = feedback->marker_name.substr(0, feedback->marker_name.rfind("_start_control"));
    type = StartPosition;
  }
  else if(feedback->marker_name.rfind("_end_control") != std::string::npos)
  {
    ID = feedback->marker_name.substr(0, feedback->marker_name.rfind("_end_control"));
    type = EndPosition;
  }
  else
  {
    return;
  }

  IKController& controller = (*ik_controllers_)[ID];

  if(feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE)
  {
    btTransform pose = toBulletTransform(feedback->pose);

    if(type == StartPosition)
    {
      (*motion_plan_map_)[controller.motion_plan_ID_].getStartState()->updateKinematicStateWithLinkAt((*motion_plan_map_)[controller.motion_plan_ID_].getEndEffectorLink(), pose);
      findIKSolution = true;
      if(selected_motion_plan_ID_ != controller.motion_plan_ID_)
      {
        selected_motion_plan_ID_ = controller.motion_plan_ID_;
        updateState();
      }
    }
    else
    {
      (*motion_plan_map_)[controller.motion_plan_ID_].getGoalState()->updateKinematicStateWithLinkAt((*motion_plan_map_)[controller.motion_plan_ID_].getEndEffectorLink(), pose);
      findIKSolution = true;
      if(selected_motion_plan_ID_ != controller.motion_plan_ID_)
      {
        selected_motion_plan_ID_ = controller.motion_plan_ID_;
        updateState();
      }
    }

  }
  else if(feedback->event_type == InteractiveMarkerFeedback::MENU_SELECT)
  {
    if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Go To Last Good State"])
    {
      MotionPlanRequestData& data = (*motion_plan_map_)[controller.motion_plan_ID_];

      if(type == StartPosition)
      {
        data.getStartState()->updateKinematicStateWithLinkAt(data.getEndEffectorLink(), (data.getLastGoodStartPose()));
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodStartPose()), feedback->header);
        findIKSolution = true;
      }
      else
      {
        data.getGoalState()->updateKinematicStateWithLinkAt(data.getEndEffectorLink(),(data.getLastGoodGoalPose()));
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodGoalPose()), feedback->header);
        findIKSolution = true;
      }
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Randomly Perturb"])
    {
      MotionPlanRequestData& data = (*motion_plan_map_)[controller.motion_plan_ID_];

      randomlyPerturb(data, type);
      if(type == StartPosition)
      {
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodStartPose()), feedback->header);
      }
      else
      {
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodGoalPose()), feedback->header);
      }
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Plan New Trajectory"])
    {
      MotionPlanRequestData& data = (*motion_plan_map_)[controller.motion_plan_ID_];
      std::string trajectory;
      planToRequest(data, trajectory);
      selected_trajectory_ID_ = trajectory;
      playTrajectory(data, (*trajectory_map_)[selected_trajectory_ID_]);
      updateState();
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Filter Last Trajectory"])
    {
      MotionPlanRequestData& data = (*motion_plan_map_)[controller.motion_plan_ID_];
      std::string trajectory;
      if(selected_trajectory_ID_ != "" && trajectory_map_->find(selected_trajectory_ID_) != trajectory_map_->end())
      {
        filterTrajectory(data,(*trajectory_map_)[selected_trajectory_ID_] ,trajectory);
        selected_trajectory_ID_ = trajectory;
        playTrajectory(data, (*trajectory_map_)[selected_trajectory_ID_]);
        updateState();
      }
    }
  }

  if(findIKSolution)
  {
    if(!solveIKForEndEffectorPose((*motion_plan_map_)[controller.motion_plan_ID_], type, true, false))
       {
         if((*motion_plan_map_)[controller.motion_plan_ID_].hasGoodIKSolution())
         {
           (*motion_plan_map_)[controller.motion_plan_ID_].refreshColors();
         }
         (*motion_plan_map_)[controller.motion_plan_ID_].setHasGoodIKSolution(false);
       }
       else
       {
         if(!(*motion_plan_map_)[controller.motion_plan_ID_].hasGoodIKSolution())
         {
           (*motion_plan_map_)[controller.motion_plan_ID_].refreshColors();
         }
         (*motion_plan_map_)[controller.motion_plan_ID_].setHasGoodIKSolution(true);
       }
  }

  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::createIkControllersFromMotionPlanRequest(MotionPlanRequestData& data)
{
  if(data.isStartEditable())
  {
    createIKController(data, StartPosition);
  }

  if(data.isEndEditable())
  {
    createIKController(data, EndPosition);
  }

  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::createIKController(MotionPlanRequestData& data, PositionType type)
{
  KinematicState* state = NULL;
  std::string nametag = "";
  if(type == StartPosition)
  {
    state = data.getStartState();
    nametag = "_start_control";
  }
  else
  {
    state = data.getGoalState();
    nametag = "_end_control";
  }

  btTransform transform = state->getLinkState(data.getEndEffectorLink())->getGlobalCollisionBodyTransform();

  InteractiveMarker marker;
  marker.header.frame_id = "/" + cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = 0.225f;
  marker.name = data.getID() + nametag;
  marker.description = data.getID() + nametag;


  InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.always_visible = false;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  marker.controls.push_back(control);

  interactive_marker_server_->insert(marker, ik_control_feedback_ptr_);
  control.interaction_mode = InteractiveMarkerControl::MENU;
  //control.markers.push_back(makeMarkerSphere(marker));
  marker.controls.push_back(control);


  (*ik_controllers_)[data.getID()].motion_plan_ID_ = data.getID();
  if(type == StartPosition)
  {
    (*ik_controllers_)[data.getID()].start_controller_ = marker;
    data.setLastGoodStartPose(toBulletTransform((*ik_controllers_)[data.getID()].start_controller_.pose));
  }
  else
  {
    (*ik_controllers_)[data.getID()].end_controller_ = marker;
    data.setLastGoodGoalPose(toBulletTransform((*ik_controllers_)[data.getID()].end_controller_.pose));
  }

  menu_handler_map_["IK Control"].apply(*interactive_marker_server_, marker.name);

}


void PlanningSceneEditor::collisionObjectSelectionCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->marker_name.rfind("collision_object") == string::npos)
  {
    return;
  }
  std::string name = feedback->marker_name.substr(0, feedback->marker_name.rfind("_selection"));
  bool shouldSelect = false;
  switch(feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      shouldSelect = true;
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:
      if(feedback->menu_entry_id == menu_entry_maps_["Collision Object Selection"]["Delete"])
      {
        (*selectable_objects_)[name].collision_object_.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
        interactive_marker_server_->erase((*selectable_objects_)[name].selection_marker_.name);
        sendPlanningScene((*planning_scene_map_)[current_planning_scene_ID_]);
      }
      else if(feedback->menu_entry_id == menu_entry_maps_["Collision Object Selection"]["Select"])
      {
        shouldSelect = true;
      }

      break;
  }

  if(shouldSelect)
  {
    interactive_marker_server_->erase((*selectable_objects_)[name].selection_marker_.name);
    (*selectable_objects_)[name].control_marker_.pose = feedback->pose;
    (*selectable_objects_)[name].control_marker_.header.stamp = ros::Time::now();

    interactive_marker_server_->insert((*selectable_objects_)[name].control_marker_,
                                       collision_object_movement_feedback_ptr_);

    menu_handler_map_["Collision Object"].apply(*interactive_marker_server_, (*selectable_objects_)[name].control_marker_.name);
  }
  interactive_marker_server_->applyChanges();


}

void PlanningSceneEditor::collisionObjectMovementCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->marker_name.rfind("collision_object") == string::npos)
  {
    return;
  }
  std::string name = feedback->marker_name.substr(0, feedback->marker_name.rfind("_control"));

  switch(feedback->event_type)
  {
    case InteractiveMarkerFeedback::MOUSE_UP:
      for(size_t i = 0; i < (*selectable_objects_)[name].collision_object_.poses.size(); i++)
      {
        (*selectable_objects_)[name].collision_object_.poses[i] = feedback->pose;
      }
      sendPlanningScene((*planning_scene_map_)[current_planning_scene_ID_]);
      break;

    case InteractiveMarkerFeedback::MENU_SELECT:
      if(feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Delete"])
     {
       (*selectable_objects_)[name].collision_object_.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
       interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
       sendPlanningScene((*planning_scene_map_)[current_planning_scene_ID_]);
     }
     else if(feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Deselect"])
     {
       interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
       (*selectable_objects_)[name].selection_marker_.pose = feedback->pose;
       (*selectable_objects_)[name].selection_marker_.header.stamp = ros::Time::now();
       interactive_marker_server_->insert((*selectable_objects_)[name].selection_marker_,
                                          collision_object_selection_feedback_ptr_);
       menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_, (*selectable_objects_)[name].selection_marker_.name);

     }
      break;

    case InteractiveMarkerFeedback::POSE_UPDATE:
      break;
  }

  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::createCollisionObject(geometry_msgs::Pose pose, PlanningSceneEditor::GeneratedShape shape, float scaleX, float scaleY, float scaleZ)
{
  lockScene();
  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  collision_object.header.stamp = ros::Time::now();
  collision_object.header.frame_id =  cm_->getWorldFrameId();
  collision_object.id = generateNewCollisionObjectID();
  arm_navigation_msgs::Shape object;

  switch(shape)
  {
    case PlanningSceneEditor::Box:
      object.type = arm_navigation_msgs::Shape::BOX;
      object.dimensions.resize(3);
      object.dimensions[0] = scaleX;
      object.dimensions[1] = scaleY;
      object.dimensions[2] = scaleZ;
      break;
    case PlanningSceneEditor::Cylinder:
      object.type = arm_navigation_msgs::Shape::CYLINDER;
      object.dimensions.resize(2);
      object.dimensions[0] = scaleX*0.5f;
      object.dimensions[1] = scaleZ;
      break;
    case PlanningSceneEditor::Sphere:
      object.type = arm_navigation_msgs::Shape::SPHERE;
      object.dimensions.resize(1);
      object.dimensions[0] = scaleX*0.5f;
      break;
  };


  collision_object.shapes.push_back(object);
  collision_object.poses.push_back(pose);

  btTransform cur = toBulletTransform(pose);

  createSelectableMarkerFromCollisionObject(collision_object, collision_object.id, "");

  ROS_INFO("Created collision object.");
  ROS_INFO("Sending planning scene %s", current_planning_scene_ID_.c_str());

  sendPlanningScene((*planning_scene_map_)[current_planning_scene_ID_]);
  unlockScene();
}

bool PlanningSceneEditor::solveIKForEndEffectorPose(MotionPlanRequestData& mpr, planning_scene_utils::PositionType type, bool coll_aware,
                               bool constrain_pitch_and_roll, double change_redundancy)
{
  kinematics_msgs::PositionIKRequest ik_request;
  ik_request.ik_link_name = mpr.getEndEffectorLink();
  ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
  ik_request.pose_stamped.header.stamp = ros::Time::now();

  KinematicState* state = NULL;

  if(type == StartPosition)
  {
    state = mpr.getStartState();
  }
  else
  {
    state = mpr.getGoalState();
  }

  tf::poseTFToMsg(state->getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform(),
                  ik_request.pose_stamped.pose);

  convertKinematicStateToRobotState(*state, ros::Time::now(), cm_->getWorldFrameId(),
                                    ik_request.robot_state);
  ik_request.ik_seed_state = ik_request.robot_state;

  // if(change_redundancy != 0.0) {
  //   for(unsigned int i = 0; i < ik_request.ik_seed_state.joint_state.name.size(); i++) {
  //     if(ik_request.ik_seed_state.joint_state.name[i] == redundancy_joint_) {
  //       ik_request.ik_seed_state.joint_state.position[i] += change_redundancy;
  //     }
  //   }
  // }
  map<string, double> joint_values;
  vector<string> joint_names;

  if(coll_aware)
  {
    kinematics_msgs::GetConstraintAwarePositionIK::Request ik_req;
    kinematics_msgs::GetConstraintAwarePositionIK::Response ik_res;
    if(constrain_pitch_and_roll)
    {
      arm_navigation_msgs::Constraints goal_constraints;
      goal_constraints.orientation_constraints.resize(1);
      arm_navigation_msgs::Constraints path_constraints;
      path_constraints.orientation_constraints.resize(1);
      std::string name = mpr.getEndEffectorLink();
      determinePitchRollConstraintsGivenState(*state, name,
                                              goal_constraints.orientation_constraints[0],
                                              path_constraints.orientation_constraints[0]);

      arm_navigation_msgs::ArmNavigationErrorCodes err;
      if(!cm_->isKinematicStateValid(*state, std::vector<std::string>(), err,
                                     goal_constraints, path_constraints))
      {
        ROS_INFO_STREAM("Violates rp constraints");
        return false;
      }
      ik_req.constraints = goal_constraints;
    }
    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(0.2);
    if(!(*collision_aware_ik_services_)[mpr.getEndEffectorLink()]->call(ik_req, ik_res))
    {
      ROS_INFO("Problem with ik service call");
      return false;
    }
    if(ik_res.error_code.val != ik_res.error_code.SUCCESS)
    {
      ROS_INFO_STREAM("Call yields bad error code " << ik_res.error_code.val);
      return false;
    }
    joint_names = ik_res.solution.joint_state.name;

    for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++)
    {
      joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
    }

  }
  else
  {
    kinematics_msgs::GetPositionIK::Request ik_req;
    kinematics_msgs::GetPositionIK::Response ik_res;
    ik_req.ik_request = ik_request;
    ik_req.timeout = ros::Duration(0.2);
    if(!(*non_collision_aware_ik_services_)[mpr.getEndEffectorLink()]->call(ik_req, ik_res))
    {
      ROS_INFO("Problem with ik service call");
      return false;
    }
    ROS_INFO_STREAM("Called IK. Code was "<< ik_res.error_code.val);
    if(ik_res.error_code.val != ik_res.error_code.SUCCESS)
    {
      ROS_INFO_STREAM("Call yields bad error code " << ik_res.error_code.val);
      return false;
    }
    for(unsigned int i = 0; i < ik_res.solution.joint_state.name.size(); i++)
    {
      joint_values[ik_res.solution.joint_state.name[i]] = ik_res.solution.joint_state.position[i];
    }

  }

  lockScene();
  state->setKinematicState(joint_values);
  unlockScene();

  //createSelectableJointMarkers(gc);
  if(coll_aware)
  {
    Constraints emp_con;
    ArmNavigationErrorCodes error_code;

    if(!cm_->isKinematicStateValid(*state, joint_names, error_code, emp_con, emp_con,
                                   true))
    {
      ROS_INFO_STREAM("Problem with response");
      return false;
    }
  }

  if(type == StartPosition)
  {
    mpr.setStartStateValues(joint_values);
    convertKinematicStateToRobotState(*state,  ros::Time::now(), cm_->getWorldFrameId(),mpr.getMotionPlanRequest().start_state);
    mpr.setLastGoodStartPose((state->getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform()));
  }
  else
  {
    mpr.setGoalStateValues(joint_values);
    mpr.setLastGoodGoalPose((state->getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform()));
  }

  return true;
}

void PlanningSceneEditor::setIKControlsVisible(std::string ID, PositionType type, bool visible)
{
  if(!visible)
  {
    if(type == StartPosition)
    {
      interactive_marker_server_->erase((*ik_controllers_)[ID].start_controller_.name);
    }
    else
    {
      interactive_marker_server_->erase((*ik_controllers_)[ID].end_controller_.name);
    }
    interactive_marker_server_->applyChanges();
  }
  else
  {
    createIKController((*motion_plan_map_)[ID], type);
    interactive_marker_server_->applyChanges();
  }
}

void PlanningSceneEditor::randomlyPerturb(MotionPlanRequestData& mpr, PositionType type)
{
  btTransform currentPose;

  if(type == StartPosition)
  {
    currentPose = mpr.getStartState()->getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform();
  }
  else
  {
    currentPose = mpr.getGoalState()->getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform();
  }

  int maxTries = 10;
  int numTries = 0;
  bool found = false;
  double posVariance = 0.5;
  double angleVariance = 0.5;

  while(!found && numTries < maxTries)
  {

    double xVar = posVariance * ((double)random() / (double)RAND_MAX) - posVariance / 2.0;
    double yVar = posVariance * ((double)random() / (double)RAND_MAX) - posVariance / 2.0;
    double zVar = posVariance * ((double)random() / (double)RAND_MAX) - posVariance / 2.0;

    double xAngleVar = angleVariance * ((double)random() / (double)RAND_MAX) - angleVariance / 2.0;
    double yAngleVar = angleVariance * ((double)random() / (double)RAND_MAX) - angleVariance / 2.0;
    double zAngleVar = angleVariance * ((double)random() / (double)RAND_MAX) - angleVariance / 2.0;

    double x = currentPose.getOrigin().x() + xVar;
    double y = currentPose.getOrigin().y() + yVar;
    double z = currentPose.getOrigin().z() + zVar;

    double xA = currentPose.getRotation().x() + xAngleVar;
    double yA = currentPose.getRotation().y() + yAngleVar;
    double zA = currentPose.getRotation().z() + zAngleVar;

    btVector3 newPos(x, y, z);
    btQuaternion newOrient(xA, yA, zA, 1.0);
    btTransform newTrans(newOrient, newPos);

    if(type == StartPosition)
    {
      mpr.getStartState()->updateKinematicStateWithLinkAt(mpr.getEndEffectorLink(), newTrans);
    }
    else
    {
      mpr.getGoalState()->updateKinematicStateWithLinkAt(mpr.getEndEffectorLink(), newTrans);
    }

    if(!solveIKForEndEffectorPose(mpr, type, true, false))
    {
      if(mpr.hasGoodIKSolution())
      {
        mpr.refreshColors();
      }
      mpr.setHasGoodIKSolution(false);
    }
    else
    {
      if(!mpr.hasGoodIKSolution())
      {
        mpr.refreshColors();
      }
      mpr.setHasGoodIKSolution(true);
    }
    if(mpr.hasGoodIKSolution())
    {
      found = true;
    }

    numTries++;
    posVariance *= 1.1;
  }
}
