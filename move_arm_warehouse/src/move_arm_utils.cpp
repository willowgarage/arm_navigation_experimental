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
#include <assert.h>

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
using namespace actionlib;
using namespace control_msgs;
using namespace interactive_markers;

#define MARKER_REFRESH_TIME 0.05
#define SAFE_DELETE(x) if(x != NULL) { delete x; x = NULL; }

std_msgs::ColorRGBA makeRandomColor(float brightness, float alpha)
{
  std_msgs::ColorRGBA toReturn;
  toReturn.a = alpha;

  toReturn.r = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;
  toReturn.g = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;
  toReturn.b = ((float)(random()) / (float)RAND_MAX) * (1.0f - brightness) + brightness;

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
  setTimeStamp(ros::Time(ros::WallTime::now().toSec()));
}

PlanningSceneData::PlanningSceneData(string name, ros::Time timestamp, PlanningScene scene)
{
  setName(name);
  setPlanningScene(scene);
  setTimeStamp(timestamp);
}

void PlanningSceneData::getRobotState(KinematicState* state)
{
  // Actually converts a robot state message to a kinematic state message (possibly very slow)
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
  refresh_timer_ = ros::Duration(0.0);
  trajectory_error_code_.val = 0;
  setRenderType(CollisionMesh);
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
  refresh_timer_ = ros::Duration(0.0);
  trajectory_error_code_.val = 0;
  setRenderType(CollisionMesh);
}

void TrajectoryData::moveThroughTrajectory(int step)
{
  unsigned int tsize = getTrajectorySize();

  if(tsize == 0 || getCurrentState() == NULL)
  {
    return;
  }

  // Possibly negative point
  if((int)getCurrentPoint() + step < 0)
  {
    setCurrentPoint(0);
  }
  else
  {
    setCurrentPoint((int)getCurrentPoint() + step);
  }

  // Possibly overstepped the end of the trajectory.
  if(getCurrentPoint() >= tsize - 1)
  {
    setCurrentPoint(tsize - 1);
    stop();
  }

  // Create new kinematic state.
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

void TrajectoryData::updateCollisionMarkers(CollisionModels* cm_, MotionPlanRequestData& motionPlanRequest,
                                            ros::ServiceClient& distance_state_validity_service_client_)
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

    cm_->disableCollisionsForNonUpdatedLinks(getGroupName());
    // Get all collisions as little red spheres.
    cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(MARKER_REFRESH_TIME));

    const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
    Constraints empty_constraints;
    ArmNavigationErrorCodes code;

    // Update validity of the current state.
    cm_->isKinematicStateValid(*state, jsg->getJointNames(), code, empty_constraints,
                               motionPlanRequest.getMotionPlanRequest().path_constraints, true);

    cm_->revertAllowedCollisionToDefault();
    GetStateValidity::Request val_req;
    GetStateValidity::Response val_res;
    convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      val_req.robot_state);

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
  setGoalColor(makeRandomColor(0.3f, 0.6f));
  setStartEditable(true);
  setGoalEditable(true);
  setHasGoodIKSolution(true);
  setID("");
  show();
  showCollisions();
  // Note: these must be registered as StateRegistry entries after this request has been created.
  start_state_ = new KinematicState(*robot_state);
  goal_state_ = new KinematicState(*robot_state);
  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_timer_ = ros::Duration(0.0);
  setRenderType(CollisionMesh);
}

MotionPlanRequestData::MotionPlanRequestData(string ID, string source, MotionPlanRequest request,
                                             KinematicState* robot_state)
{
  // Note: these must be registered as StateRegistry entries after this request has been created.
  start_state_ = new KinematicState(*robot_state);
  goal_state_ = new KinematicState(*robot_state);

  setID(ID);
  setSource(source);
  setMotionPlanRequest(request);

  setStartColor(makeRandomColor(0.3f, 0.6f));
  setGoalColor(makeRandomColor(0.3f, 0.6f));
  setStartEditable(true);
  setGoalEditable(true);
  setHasGoodIKSolution(true);
  show();
  showCollisions();

  should_refresh_colors_ = false;
  has_refreshed_colors_ = true;
  refresh_timer_ = ros::Duration(0.0);
  are_joint_controls_visible_ = false;

  setRenderType(CollisionMesh);
}

// Kinematic states must be converted to joint constraint message
void MotionPlanRequestData::updateGoalState()
{
  vector<JointConstraint>& constraints = getMotionPlanRequest().goal_constraints.joint_constraints;

  map<string, double> jointValues;
  for(size_t i = 0; i < constraints.size(); i++)
  {
    JointConstraint& constraint = constraints[i];
    jointValues[constraint.joint_name] = constraint.position;
  }

  goal_state_->setKinematicState(jointValues);
}

// Kinematic state must be converted to robot state message
void MotionPlanRequestData::updateStartState()
{
  setRobotStateAndComputeTransforms(getMotionPlanRequest().start_state, *start_state_);
}

void MotionPlanRequestData::setJointControlsVisible(bool visible, PlanningSceneEditor* editor)
{
  are_joint_controls_visible_ = visible;

  if(visible)
  {
    if(isStartVisible() && isStartEditable())
    {
      editor->createJointMarkers(*this, StartPosition);
    }
    else
    {
      editor->deleteJointMarkers(*this, StartPosition);
    }
    if(isEndVisible() && isGoalEditable())
    {
      editor->createJointMarkers(*this, GoalPosition);
    }
    else
    {
      editor->deleteJointMarkers(*this, GoalPosition);
    }
  }
  else
  {
    editor->deleteJointMarkers(*this, StartPosition);
    editor->deleteJointMarkers(*this, GoalPosition);
  }
}

void MotionPlanRequestData::setStartStateValues(std::map<std::string, double>& joint_values)
{
  setStateChanged(true);
  start_state_->setKinematicState(joint_values);
}

void MotionPlanRequestData::setGoalStateValues(std::map<std::string, double>& joint_values)
{
  setStateChanged(true);
  goal_state_->setKinematicState(joint_values);

  int constraints = 0;
  for(std::map<std::string, double>::iterator it = joint_values.begin(); it != joint_values.end(); it++)
  {
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].joint_name = it->first;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].position = it->second;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].tolerance_above = 0.001;
    getMotionPlanRequest().goal_constraints.joint_constraints[constraints].tolerance_below = 0.001;
    constraints++;
  }
}

void MotionPlanRequestData::updateCollisionMarkers(CollisionModels* cm_,
                                                   ros::ServiceClient& distance_state_validity_service_client_)
{
  if(areCollisionsVisible())
  {
    const KinematicState* state = getStartState();
    collision_markers_.markers.clear();
    if(state == NULL)
    {
      return;
    }

    ///////
    // Start state block
    ///////
    std_msgs::ColorRGBA bad_color;
    bad_color.a = 1.0f;
    bad_color.r = 1.0f;
    bad_color.g = 0.0f;
    bad_color.b = 0.0f;
    cm_->disableCollisionsForNonUpdatedLinks(getGroupName());
    // Get all the collision points as little red spheres.
    cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(MARKER_REFRESH_TIME));
    const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
    ArmNavigationErrorCodes code;
    Constraints empty_constraints;
    // Ensure that the state is valid.
    cm_->isKinematicStateValid(*state, jsg->getJointNames(), code, empty_constraints,
                               getMotionPlanRequest().path_constraints, true);

    GetStateValidity::Request val_req;
    GetStateValidity::Response val_res;
    convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      val_req.robot_state);

    if(!distance_state_validity_service_client_.call(val_req, val_res))
    {
      ROS_INFO_STREAM("Something wrong with distance server");
    }

    ////////
    // End State block
    ///////
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
      cm_->getAllCollisionPointMarkers(*state, collision_markers_, bad_color, ros::Duration(MARKER_REFRESH_TIME));
      const KinematicState::JointStateGroup* jsg = state->getJointStateGroup(getGroupName());
      ArmNavigationErrorCodes code;
      Constraints empty_constraints;
      cm_->isKinematicStateValid(*state, jsg->getJointNames(), code, empty_constraints,
                                 getMotionPlanRequest().path_constraints, true);

      GetStateValidity::Request val_req;
      GetStateValidity::Response val_res;
      convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                        val_req.robot_state);

      if(!distance_state_validity_service_client_.call(val_req, val_res))
      {
        ROS_INFO_STREAM("Something wrong with distance server");
      }
    }

    cm_->revertAllowedCollisionToDefault();
  }
}

// Warning: lots of copying going on here.
std::vector<std::string> MotionPlanRequestData::getJointNamesInGoal()
{
  std::vector<JointConstraint>& constraints = getMotionPlanRequest().goal_constraints.joint_constraints;
  std::vector<std::string> toReturn;

  for(size_t i = 0; i < constraints.size(); i++)
  {
    JointConstraint& constraint = constraints[i];
    toReturn.push_back(constraint.joint_name);
  }

  return toReturn;
}

// Warning: calls getJointNamesInGoal
bool MotionPlanRequestData::isJointNameInGoal(std::string joint)
{
  vector<string> joints = getJointNamesInGoal();
  for(size_t i = 0; i < joints.size(); i++)
  {
    if(joints[i] == joint)
    {
      return true;
    }
  }

  return false;
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
  ///////
  /// Memory initialization
  /////
  params_ = params;
  monitor_status_ = Idle;

  last_collision_object_color_.r = 0.7;
  last_collision_object_color_.g = 0.7;
  last_collision_object_color_.b = 0.7;
  last_collision_object_color_.a = 1.0;

  planning_scene_map_ = new map<string, PlanningSceneData> ();
  trajectory_map_ = new map<string, TrajectoryData> ();
  motion_plan_map_ = new map<string, MotionPlanRequestData> ();
  collision_aware_ik_services_ = new map<string, ros::ServiceClient*> ();
  non_collision_aware_ik_services_ = new map<string, ros::ServiceClient*> ();
  selectable_objects_ = new map<string, SelectableObject> ();
  ik_controllers_ = new map<string, IKController> ();

  string robot_description_name = nh_.resolveName("robot_description", true);
  cm_ = new CollisionModels(robot_description_name);
  robot_state_ = new KinematicState(cm_->getKinematicModel());
  robot_state_->setKinematicStateToDefault();

  ////
  /// Interactive markers
  ////
  interactive_marker_server_
      = new interactive_markers::InteractiveMarkerServer("planning_scene_warehouse_viewer_controls", "", false);

  collision_object_movement_feedback_ptr_
      = boost::bind(&PlanningSceneEditor::collisionObjectMovementCallback, this, _1);
  collision_object_selection_feedback_ptr_ = boost::bind(&PlanningSceneEditor::collisionObjectSelectionCallback, this,
                                                         _1);
  joint_control_feedback_ptr_ = boost::bind(&PlanningSceneEditor::JointControllerCallback, this, _1);
  ik_control_feedback_ptr_ = boost::bind(&PlanningSceneEditor::IKControllerCallback, this, _1);


  //////
  /// Publishers
  /////
  joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState> ("joint_states", 10);
  vis_marker_publisher_ = nh_.advertise<visualization_msgs::Marker> (params.vis_topic_name_, 128);
  vis_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray> (params.vis_topic_name_ + "_array", 128);
  move_arm_warehouse_logger_reader_ = new MoveArmWarehouseLoggerReader();


  /////
  /// Subscribers
  //////
  if(params.sync_robot_state_with_gazebo_)
  {
    ros::service::waitForService("/gazebo/set_model_configuration");
    ros::service::waitForService(params.list_controllers_service_);
    ros::service::waitForService(params.load_controllers_service_);
    ros::service::waitForService(params.unload_controllers_service_);
    ros::service::waitForService(params.switch_controllers_service_);
    ros::service::waitForService("/gazebo/pause_physics");
    ros::service::waitForService("/gazebo/unpause_physics");
    ros::service::waitForService("/gazebo/set_link_properties");
    ros::service::waitForService("/gazebo/get_link_properties");
  }

  if(params.left_arm_group_ != "none")
  {
    ros::service::waitForService(params.left_ik_name_);
  }

  if(params.right_arm_group_ != "none")
  {
    ros::service::waitForService(params.right_ik_name_);
  }

  if(params.planner_service_name_ != "none")
  {
    ros::service::waitForService(params.planner_service_name_);
  }

  if(params.proximity_space_service_name_ != "none")
  {
    ros::service::waitForService(params.proximity_space_service_name_);
  }

  if(params.proximity_space_validity_name_ != "none")
  {
    ros::service::waitForService(params.proximity_space_validity_name_);
  }

  if(params.sync_robot_state_with_gazebo_)
  {
    gazebo_joint_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration", true);
    list_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::ListControllers>(params.list_controllers_service_, true);
    load_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::LoadController>(params.load_controllers_service_, true);
    unload_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::UnloadController>(params.unload_controllers_service_, true);
    switch_controllers_client_ = nh_.serviceClient<pr2_mechanism_msgs::SwitchController>(params.switch_controllers_service_, true);
    pause_gazebo_client_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics", true);
    unpause_gazebo_client_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics", true);
    set_link_properties_client_ = nh_.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties", true);
    get_link_properties_client_ = nh_.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties", true);
  }

  if(params.left_arm_group_ != "none")
  {
    left_ik_service_client_ = nh_.serviceClient<GetConstraintAwarePositionIK> (params.left_ik_name_, true);
    non_coll_left_ik_service_client_ = nh_.serviceClient<GetPositionIK> (params.non_coll_left_ik_name_, true);
  }
  if(params.right_arm_group_ != "none")
  {
    right_ik_service_client_ = nh_.serviceClient<GetConstraintAwarePositionIK> (params.right_ik_name_, true);
    non_coll_right_ik_service_client_ = nh_.serviceClient<GetPositionIK> (params.non_coll_right_ik_name_, true);
  }

  if(params.planner_service_name_ != "none")
  {
    planning_service_client_ = nh_.serviceClient<GetMotionPlan> (params.planner_service_name_, true);
  }

  if(params.right_interpolate_service_name_ != "none")
  {
    right_interpolate_service_client_ = nh_.serviceClient<GetMotionPlan> (params.right_interpolate_service_name_, true);
  }
  if(params.left_interpolate_service_name_ != "none")
  {
    left_interpolate_service_client_ = nh_.serviceClient<GetMotionPlan> (params.left_interpolate_service_name_, true);
  }

  if(params.trajectory_filter_service_name_ != "none")
  {
    trajectory_filter_service_client_
        = nh_.serviceClient<FilterJointTrajectoryWithConstraints> (params.trajectory_filter_service_name_);
  }

  if(params.proximity_space_service_name_ != "none")
  {
    distance_aware_service_client_ = nh_.serviceClient<GetMotionPlan> (params.proximity_space_service_name_, true);
  }

  if(params.proximity_space_validity_name_ != "none")
  {
    distance_state_validity_service_client_
        = nh_.serviceClient<GetStateValidity> (params.proximity_space_validity_name_, true);
  }

  if(params.proximity_space_planner_name_ != "none")
  {
    collision_proximity_planner_client_ = nh_.serviceClient<GetMotionPlan> (params.proximity_space_planner_name_, true);
  }

  set_planning_scene_diff_client_ = nh_.serviceClient<SetPlanningSceneDiff> (params.set_planning_scene_diff_name_);

  if(params.use_robot_data_)
  {
    if(params_.right_arm_group_ != "none")
    {
      arm_controller_map_[params_.right_arm_group_] = new actionlib::SimpleActionClient<
          control_msgs::FollowJointTrajectoryAction>(params.execute_right_trajectory_, true);

      while(ros::ok() && !arm_controller_map_[params_.right_arm_group_]->waitForServer(ros::Duration(1.0)))
      {
        ROS_INFO("Waiting for the right_joint_trajectory_action server to come up.");
      }
    }

    if(params.left_arm_group_ != "none")
    {
      arm_controller_map_[params.left_arm_group_] = new actionlib::SimpleActionClient<
          control_msgs::FollowJointTrajectoryAction>(params.execute_left_trajectory_, true);

      while(ros::ok() && !arm_controller_map_[params.left_arm_group_]->waitForServer(ros::Duration(1.0)))
      {
        ROS_INFO("Waiting for the left_joint_trajectory_action server to come up.");
      }
    }
  }

  if(params.left_arm_group_ != "none")
  {
    (*collision_aware_ik_services_)[params.left_ik_link_] = &left_ik_service_client_;
  }

  if(params.right_arm_group_ != "none")
  {
    (*collision_aware_ik_services_)[params.right_ik_link_] = &right_ik_service_client_;
  }

  if(params.left_arm_group_ != "none")
  {
    (*non_collision_aware_ik_services_)[params.left_ik_link_] = &non_coll_left_ik_service_client_;
  }

  if(params.left_arm_group_ != "none")
  {
    (*non_collision_aware_ik_services_)[params.right_ik_link_] = &non_coll_right_ik_service_client_;
  }

  /////
  /// Interactive menus
  /////
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

  if(params_.use_robot_data_)
  {
    registerMenuEntry("IK Control", "Execute Last Trajectory", ik_control_feedback_ptr_);
  }

  /////
  /// Connection with sim data
  /////
  if(params.use_robot_data_)
  {
    state_monitor_ = new KinematicModelStateMonitor(cm_, &transform_listenter_);
    state_monitor_->addOnStateUpdateCallback(boost::bind(&PlanningSceneEditor::jointStateCallback, this, _1));
  }
}

void PlanningSceneEditor::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  lockScene();
  if(robot_state_ != NULL)
  {
    state_monitor_->setStateValuesFromCurrentValues(*robot_state_);

    // Records trajectory if currently executing.
    if(monitor_status_ == Executing)
    {
      std::map<std::string, double> joint_state_map;
      std::map<std::string, double> joint_velocity_map;

      //message already been validated in kmsm
      for(unsigned int i = 0; i < joint_state->position.size(); ++i)
      {
        joint_state_map[joint_state->name[i]] = joint_state->position[i];
        joint_velocity_map[joint_state->name[i]] = joint_state->velocity[i];
      }
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions.resize(logged_trajectory_.joint_names.size());
      point.velocities.resize(logged_trajectory_.joint_names.size());
      for(unsigned int i = 0; i < logged_trajectory_.joint_names.size(); i++)
      {
        point.positions[i] = joint_state_map[logged_trajectory_.joint_names[i]];
        point.velocities[i] = joint_velocity_map[logged_trajectory_.joint_names[i]];
      }
      point.time_from_start = ros::Time(ros::WallTime::now().toSec()) - logged_trajectory_start_time_;
      logged_trajectory_.points.push_back(point);
    }
  }
  unlockScene();
}

PlanningSceneEditor::~PlanningSceneEditor()
{
  SAFE_DELETE(robot_state_);
  SAFE_DELETE(interactive_marker_server_);
  SAFE_DELETE(state_monitor_);
  SAFE_DELETE(selectable_objects_);
  SAFE_DELETE(planning_scene_map_);
  SAFE_DELETE(trajectory_map_);
  SAFE_DELETE(motion_plan_map_);
  SAFE_DELETE(ik_controllers_);
}

void PlanningSceneEditor::setCurrentPlanningScene(std::string ID, bool loadRequests, bool loadTrajectories)
{
  lockScene();

  // Need to do this to clear old scene state.
  deleteKinematicStates();

  current_planning_scene_ID_ = ID;
  if(ID == "")
  {
    return;
  }

  /////
  /// Get rid of old interactive markers.
  //////
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


  /////
  /// Make sure all old trajectories and MPRs are gone.
  /////
  for(map<string, MotionPlanRequestData>::iterator it = motion_plan_map_->begin(); it != motion_plan_map_->end(); it ++)
  {
    deleteMotionPlanRequest(it->first);
  }
  motion_plan_map_->clear();

  for(map<string, TrajectoryData>::iterator it = trajectory_map_->begin(); it != trajectory_map_->end(); it++)
  {
    deleteTrajectory(it->first);
  }
  trajectory_map_->clear();


  if((*planning_scene_map_).find(ID) != (*planning_scene_map_).end())
  {

    /////
    /// Load planning scene
    /////
    PlanningSceneData& scene = (*planning_scene_map_)[ID];
    ros::Time time = scene.getTimeStamp();
    error_map_.clear();
    scene.getPipelineStages().clear();
    scene.getErrorCodes().clear();
    getPlanningSceneOutcomes(time, scene.getPipelineStages(), scene.getErrorCodes(), error_map_);

    /////
    /// Create collision object.
    /////
    for(size_t i = 0; i < scene.getPlanningScene().collision_objects.size(); i++)
    {
      std::stringstream ss;
      ss << "collision_object_";
      ss << i;
      std_msgs::ColorRGBA color;
      color.r = 0.5;
      color.g = 0.5;
      color.b = 0.5;
      color.a = 1.0;
      createSelectableMarkerFromCollisionObject(scene.getPlanningScene().collision_objects[i], ss.str(), "", color);
    }

    /////
    /// Load motion plan requests
    /////
    if(loadRequests)
    {
      vector<string> IDs;
      vector<string> stageNames;
      vector<MotionPlanRequest> requests;
      getAllAssociatedMotionPlanRequests(time, IDs, stageNames, requests);
      initMotionPlanRequestData(ID, IDs, stageNames, requests);

      for(size_t j = 0; j < IDs.size(); j++)
      {
        MotionPlanRequest req;
        std::string motionID = IDs[j];

        MotionPlanRequestData& motionData = (*motion_plan_map_)[motionID];
        motionData.setPlanningSceneName(ID);

        std::vector<JointTrajectory> trajs;
        std::vector<string> sources;
        std::vector < string > IDs;
        std::vector<ros::Duration> durations;
        std::vector<int32_t> errors;

        max_request_ID_++;

        /////
        /// Load trajectories
        /////
        if(loadTrajectories)
        {
          move_arm_warehouse_logger_reader_->getAssociatedJointTrajectories("", time, motionData.getID(), trajs, sources,
                                                                            IDs, durations, errors);

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
            trajectoryData.trajectory_error_code_.val = errors[k];

            scene.getTrajectories().push_back(trajectoryData.getID());
            motionData.getTrajectories().push_back(trajectoryData.getID());

            // Make sure we aren't overwriting an existing trajectory
            if(trajectory_map_->find(trajectoryData.getID()) != trajectory_map_->end())
            {
              ROS_INFO("Deleting existing trajectory...");
              deleteTrajectory(trajectoryData.getID());
            }

            (*trajectory_map_)[IDs[k]] = trajectoryData;
            playTrajectory(motionData,(*trajectory_map_)[IDs[k]]);
            max_trajectory_ID_++;
          }
        }

      }

    }

    sendPlanningScene(scene);
  }

  interactive_marker_server_->applyChanges();
  unlockScene();
}

void PlanningSceneEditor::getTrajectoryMarkers(visualization_msgs::MarkerArray& arr)
{
  trajectory_map_->erase("");

  // For each trajectory...
  for(map<string, TrajectoryData>::iterator it = (*trajectory_map_).begin(); it != (*trajectory_map_).end(); it++)
  {

    if(it->second.isPlaying())
    {
      it->second.moveThroughTrajectory(2);
    }

    if(it->second.getCurrentState() == NULL)
    {
      continue;
    }

    it->second.updateCurrentState();

    // When the color of a trajectory has changed, we have to wait for
    // a few milliseconds before the change is registered in rviz.
    if(it->second.shouldRefreshColors())
    {
      it->second.refresh_timer_ += marker_dt_;

      if(it->second.refresh_timer_.toSec() > MARKER_REFRESH_TIME + 0.05)
      {
        it->second.setHasRefreshedColors(true);
        it->second.refresh_timer_ = ros::Duration(0.0);
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

        // Links in group
        switch(it->second.getRenderType())
        {
          case VisualMesh:
            cm_->getRobotMarkersGivenState(*(it->second.getCurrentState()), arr, it->second.getColor(),
                                         it->first + "_trajectory", ros::Duration(MARKER_REFRESH_TIME), 
                                           &lnames, 1.0, false);
            // Bodies held by robot
            cm_->getAttachedCollisionObjectMarkers(*(it->second.getCurrentState()), arr, it->first + "_trajectory",
                                                   it->second.getColor(), ros::Duration(MARKER_REFRESH_TIME), false, &lnames);

            break;
          case CollisionMesh:
            cm_->getRobotMarkersGivenState(*(it->second.getCurrentState()), arr, it->second.getColor(),
                                           it->first + "_trajectory", ros::Duration(MARKER_REFRESH_TIME), 
                                           &lnames, 1.0, true);
            cm_->getAttachedCollisionObjectMarkers(*(it->second.getCurrentState()), arr, it->first + "_trajectory",
                                                   it->second.getColor(), ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
            break;
          case PaddingMesh:
            cm_->getRobotPaddedMarkersGivenState((const KinematicState&)*(it->second.getCurrentState()),
                                                 arr,
                                                 it->second.getColor(),
                                                 it->first + "_trajectory",
                                                 ros::Duration(MARKER_REFRESH_TIME)*2.0,
                                                 (const vector<string>*)&lnames);
            cm_->getAttachedCollisionObjectMarkers(*(it->second.getCurrentState()), arr, it->first + "_trajectory",
                                                   it->second.getColor(), ros::Duration(MARKER_REFRESH_TIME)*2.0, true, &lnames);
            break;
        }

      }
    }


    //////
    /// Get collision markers associated with trajectory.
    /////
    if(it->second.areCollisionsVisible() && it->second.isVisible())
    {

      // Update markers
      if(it->second.hasStateChanged())
      {
        it->second.updateCollisionMarkers(cm_, (*motion_plan_map_)[it->second.getMotionPlanRequestID()],
                                          distance_state_validity_service_client_);
        it->second.setStateChanged(false);
      }

      // Then add them to the global array
      for(size_t i = 0; i < it->second.getCollisionMarkers().markers.size(); i++)
      {
        collision_markers_.markers.push_back(it->second.getCollisionMarkers().markers[i]);
      }
    }
  }
}

void PlanningSceneEditor::getMotionPlanningMarkers(visualization_msgs::MarkerArray& arr)
{
  vector<string> removals;

  // For each motion plan request ...
  for(map<string, MotionPlanRequestData>::iterator it = (*motion_plan_map_).begin(); it != (*motion_plan_map_).end(); it++)
  {
    MotionPlanRequestData& data = it->second;

    // TODO: Find out why this happens.
    if(motion_plan_map_->find(it->first) == motion_plan_map_->end() || data.getID() == "")
    {
      ROS_WARN("Attempting to publish non-existant motion plan request %s Erasing this request!", it->first.c_str());
      removals.push_back(it->first);
      continue;
    }

    // TODO: Find out why this happens.
    if(data.getStartState() == NULL || data.getGoalState() == NULL)
    {
      return;
    }

    // When a motion plan request has its colors changed,
    // we must wait a few milliseconds before rviz registers the change.
    if(data.shouldRefreshColors())
    {
      data.refresh_timer_ += marker_dt_;

      if(data.refresh_timer_.toSec() > MARKER_REFRESH_TIME + 0.05)
      {
        data.setHasRefreshedColors(true);
        data.refresh_timer_ = ros::Duration(0.0);
      }
    }
    else
    {
      std_msgs::ColorRGBA fail_color;
      fail_color.a = 0.9;
      fail_color.r = 1.0;
      fail_color.g = 0.0;
      fail_color.b = 0.0;

      /////
      /// Get markers for the start
      /////
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


        // If we have a good ik solution, publish with the normal color
        // else use bright red.
        std_msgs::ColorRGBA col;
        if(data.hasGoodIKSolution())
        {
          col = data.getStartColor();
        } else {
          col = fail_color;
        }
        
        switch(data.getRenderType())
        {
        case VisualMesh:
          cm_->getRobotMarkersGivenState(*(data.getStartState()), arr, col,
                                         it->first + "_start", ros::Duration(MARKER_REFRESH_TIME), 
                                         &lnames, 1.0, false);
          // Bodies held by robot
          cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
          
          break;
        case CollisionMesh:
          cm_->getRobotMarkersGivenState(*(data.getStartState()), arr, col,
                                         it->first + "_start", ros::Duration(MARKER_REFRESH_TIME), 
                                         &lnames, 1.0, true);
          cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
          break;
        case PaddingMesh:
          cm_->getRobotPaddedMarkersGivenState(*(data.getStartState()),
                                               arr,
                                               col,
                                               it->first + "_start",
                                               ros::Duration(MARKER_REFRESH_TIME),
                                               (const vector<string>*)&lnames);
          cm_->getAttachedCollisionObjectMarkers(*(data.getStartState()), arr, it->first + "_start",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), true, &lnames);
          break;
        }
      }

      /////
      /// Get markers for the end.
      /////
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

        std_msgs::ColorRGBA col;
        if(data.hasGoodIKSolution())
        {
          col = data.getGoalColor();
        } else {
          col = fail_color;
        }
        
        switch(data.getRenderType())
        {
        case VisualMesh:
          cm_->getRobotMarkersGivenState(*(data.getGoalState()), arr, col,
                                         it->first + "_Goal", ros::Duration(MARKER_REFRESH_TIME),
                                         &lnames, 1.0, false);
          // Bodies held by robot
          cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
          
          break;
        case CollisionMesh:
          cm_->getRobotMarkersGivenState(*(data.getGoalState()), arr, col,
                                         it->first + "_Goal", ros::Duration(MARKER_REFRESH_TIME),
                                         &lnames, 1.0, true);
          cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), false, &lnames);
          break;
        case PaddingMesh:
          cm_->getRobotPaddedMarkersGivenState(*(data.getGoalState()),
                                               arr,
                                               col,
                                               it->first + "_Goal",
                                               ros::Duration(MARKER_REFRESH_TIME),
                                               (const vector<string>*)&lnames);
          cm_->getAttachedCollisionObjectMarkers(*(data.getGoalState()), arr, it->first + "_Goal",
                                                 col, ros::Duration(MARKER_REFRESH_TIME), true, &lnames);
          break;
        }
      }
    }

    //////
    /// Get collision markers for the start and end state.
    /////
    if(it->second.areCollisionsVisible() && (it->second.isStartVisible() || it->second.isEndVisible()))
    {
      // Update collision markers
      if(it->second.hasStateChanged())
      {
        it->second.updateCollisionMarkers(cm_, distance_state_validity_service_client_);
        it->second.setStateChanged(false);
      }

      // Add them to the global array.
      for(size_t i = 0; i < it->second.getCollisionMarkers().markers.size(); i++)
      {
        collision_markers_.markers.push_back(it->second.getCollisionMarkers().markers[i]);
      }
    }

  }

  /////
  /// TODO: Figure out why motion plans are occasionally NULL
  ////
  for(size_t i = 0; i < removals.size(); i++)
  {
    motion_plan_map_->erase(removals[i]);
  }
}

void PlanningSceneEditor::createMotionPlanRequest(planning_models::KinematicState& start_state,
                                                  planning_models::KinematicState& end_state, std::string group_name,
                                                  std::string end_effector_name, bool constrain,
                                                  std::string planning_scene_ID, std::string& motionPlan_ID_Out,
                                                  bool fromRobotState)
{
  MotionPlanRequest motion_plan_request;
  motion_plan_request.group_name = group_name;
  motion_plan_request.num_planning_attempts = 1;
  motion_plan_request.allowed_planning_time = ros::Duration(1);
  const KinematicState::JointStateGroup* jsg = end_state.getJointStateGroup(group_name);
  motion_plan_request.goal_constraints.joint_constraints.resize(jsg->getJointNames().size());

  // Must convert kinematic state to robot state message.
  vector<double> joint_values;
  jsg->getKinematicStateValues(joint_values);
  for(unsigned int i = 0; i < jsg->getJointNames().size(); i++)
  {
    motion_plan_request.goal_constraints.joint_constraints[i].joint_name = jsg->getJointNames()[i];
    motion_plan_request.goal_constraints.joint_constraints[i].position = joint_values[i];
    motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.001;
    motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.001;
  }

  // Constraining pitch and roll
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

  // Create start state from kinematic state passed in if robot data is being used
  if(!fromRobotState)
  {
    convertKinematicStateToRobotState(start_state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      motion_plan_request.start_state);
  }
  // Otherwise, use the current robot state.
  else
  {
    convertKinematicStateToRobotState(*robot_state_, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      motion_plan_request.start_state);
  }

  // Turn the motion plan request message into a MotionPlanData
  std::string id = generateNewMotionPlanID();
  MotionPlanRequestData data(id, "Planner", motion_plan_request, robot_state_);
  data.setGroupName(motion_plan_request.group_name);
  data.setEndEffectorLink(end_effector_name);
  data.setGoalEditable(true);
  if(fromRobotState)
  {
    data.setStartEditable(false);
  }

  // Book keeping for kinematic state storage
  StateRegistry start;
  start.state = data.getStartState();
  start.source = "Motion Plan Request Data Start create request";
  StateRegistry end;
  end.state = data.getGoalState();
  end.source = "Motion Plan Request Data End from create request";
  states_.push_back(start);
  states_.push_back(end);


  (*motion_plan_map_)[id] = data;
  data.setPlanningSceneName(planning_scene_ID);

  // Add request to the planning scene
  PlanningSceneData& planningSceneData = (*planning_scene_map_)[planning_scene_ID];
  planningSceneData.getRequests().push_back(data.getID());

  motionPlan_ID_Out = data.getID();
  createIkControllersFromMotionPlanRequest(data, false);
  sendPlanningScene(planningSceneData);
}

bool PlanningSceneEditor::planToKinematicState(KinematicState& state, string group_name, string end_effector_name,
                                               bool constrain, std::string& trajectoryID_Out, string& planning_scene_ID)
{
  std::string motionPlanID;
  createMotionPlanRequest(*robot_state_, state, group_name, end_effector_name, constrain, planning_scene_ID,
                          motionPlanID);
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
  plan_req.motion_plan_request.allowed_planning_time = ros::Duration(10.0);
  GetMotionPlan::Response plan_res;

  if(!planning_service_client_.call(plan_req, plan_res))
  {
    ROS_INFO("Something wrong with planner client");
    return false;
  }

  std::string ID = generateNewTrajectoryID();
  std::string source = "Planner";

  TrajectoryData trajectoryData;
  trajectoryData.setTrajectory(plan_res.trajectory.joint_trajectory);
  trajectoryData.setGroupName(data.getMotionPlanRequest().group_name);
  trajectoryData.setMotionPlanRequestID(data.getID());
  trajectoryID_Out = ID;
  trajectoryData.setPlanningSceneName(data.getPlanningSceneName());
  trajectoryData.setID(trajectoryID_Out);
  trajectoryData.setSource("Planner");
  trajectoryData.setDuration(plan_res.planning_time);
  trajectoryData.setVisible(true);
  trajectoryData.setPlaying(true);

  PlanningSceneData& planningSceneData = (*planning_scene_map_)[data.getPlanningSceneName()];
  planningSceneData.getTrajectories().push_back(trajectoryID_Out);

  if(plan_res.error_code.val != plan_res.error_code.SUCCESS)
  {
    ROS_INFO_STREAM("Bad planning error code " << plan_res.error_code.val);
    trajectoryData.trajectory_error_code_.val = plan_res.error_code.val;
    (*trajectory_map_)[trajectoryID_Out] = trajectoryData;
    data.getTrajectories().push_back(trajectoryData.getID());

    return false;
  }
  trajectoryData.trajectory_error_code_.val = plan_res.error_code.val;
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
  goal_constraint.header.stamp = ros::Time(ros::WallTime::now().toSec());
  goal_constraint.link_name = end_effector_link;
  tf::quaternionTFToMsg(cur.getRotation(), goal_constraint.orientation);
  goal_constraint.absolute_roll_tolerance = 0.04;
  goal_constraint.absolute_pitch_tolerance = 0.04;
  goal_constraint.absolute_yaw_tolerance = M_PI;
  path_constraint.header.frame_id = cm_->getWorldFrameId();
  path_constraint.header.stamp = ros::Time(ros::WallTime::now().toSec());
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

bool PlanningSceneEditor::filterTrajectory(MotionPlanRequestData& requestData, TrajectoryData& trajectory,
                                           std::string& filter_ID)
{
  FilterJointTrajectoryWithConstraints::Request filter_req;
  FilterJointTrajectoryWithConstraints::Response filter_res;

  // Filter request has to have robot state message filled
  convertKinematicStateToRobotState(*robot_state_, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                    filter_req.start_state);

  filter_req.trajectory = trajectory.getTrajectory();
  filter_req.group_name = trajectory.getGroupName();
  filter_req.goal_constraints = requestData.getMotionPlanRequest().goal_constraints;
  filter_req.path_constraints = requestData.getMotionPlanRequest().path_constraints;
  filter_req.allowed_time = ros::Duration(2.0);

  // Time the filtering
  ros::Time startTime = ros::Time(ros::WallTime::now().toSec());
  if(!trajectory_filter_service_client_.call(filter_req, filter_res))
  {
    ROS_INFO("Problem with trajectory filter");
    return false;
  }

  // Convert returned joint trajectory to TrajectoryData
  TrajectoryData data(generateNewTrajectoryID(), "Trajectory Filterer", trajectory.getGroupName(),
                      filter_res.trajectory);
  data.setPlanningSceneName(requestData.getPlanningSceneName());
  data.setMotionPlanRequestID(requestData.getID());
  data.setDuration(ros::Time(ros::WallTime::now().toSec()) - startTime);
  requestData.getTrajectories().push_back(data.getID());
  (*planning_scene_map_)[requestData.getPlanningSceneName()].getTrajectories().push_back(data.getID());


  if(filter_res.error_code.val != filter_res.error_code.SUCCESS)
  {
    ROS_INFO_STREAM("Bad trajectory_filter error code " << filter_res.error_code.val);
    data.trajectory_error_code_.val = filter_res.error_code.val;
    (*trajectory_map_)[data.getID()] = data;
    filter_ID = data.getID();
    data.setVisible(true);
    data.play();
    return false;
  }
  else
  {
    data.trajectory_error_code_.val = filter_res.error_code.val;
    playTrajectory(requestData, (*trajectory_map_)[data.getID()]);
    (*trajectory_map_)[data.getID()] = data;
    filter_ID = data.getID();
    data.setVisible(true);
    data.play();
    return true;
  }
}

void PlanningSceneEditor::updateJointStates()
{

  // If using robot data, the joint states are handled by whatever is
  // running on the robot (presumably)
  if(params_.use_robot_data_)
  {
    return;
  }

  sensor_msgs::JointState msg;
  msg.header.frame_id = cm_->getWorldFrameId();
  msg.header.stamp = ros::Time(ros::WallTime::now().toSec());

  vector<KinematicState::JointState*> jointStates = getRobotState()->getJointStateVector();

  map<string, double> stateMap;
  getRobotState()->getKinematicStateValues(stateMap);
  getRobotState()->setKinematicState(stateMap);

  // Send each joint state out as part of a message to the robot state publisher.
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
  marker_dt_ = (ros::Time::now() - last_marker_start_time_);
  last_marker_start_time_ = ros::Time::now();
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
                                                   vector<ArmNavigationErrorCodes>& error_codes,
                                                   map<std::string, ArmNavigationErrorCodes>& error_map)
{
  if(!move_arm_warehouse_logger_reader_->getAssociatedOutcomes("", time, pipeline_stages, error_codes))
  {
    ROS_WARN_STREAM("No outcome associated with planning scene");
    return false;
  }

  // Fill error map for convenience
  for(size_t i = 0; i < error_codes.size(); i++)
  {
    error_map[pipeline_stages[i]] = error_codes[i];
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
  data.setTimeStamp(ros::Time(ros::WallTime::now().toSec()));

  convertKinematicStateToRobotState(*robot_state_, data.getTimeStamp(), cm_->getWorldFrameId(),
                                    data.getPlanningScene().robot_state);
  //end_effector_state_ = planning_state_;

  data.getPlanningScene().collision_objects.clear();

  sendPlanningScene(data);

  (*planning_scene_map_)[data.getName()] = data;
  lock_scene_.unlock();

  updateJointStates();

  char hostname[256];
  gethostname(hostname, 256);
  data.setHostName(std::string(hostname));

  return data.getName();
}

void PlanningSceneEditor::loadAllWarehouseData()
{
  max_planning_scene_ID_ = 0;
  max_request_ID_ = 0;
  max_trajectory_ID_ = 0;
  max_collision_object_ID_ = 0;

  motion_plan_map_->clear();
  trajectory_map_->clear();
  planning_scene_map_->clear();
  vector<ros::Time> planningSceneTimes;
  getAllPlanningSceneTimes(planningSceneTimes);

  // For each planning scene
  for(size_t i = 0; i < planningSceneTimes.size(); i++)
  {
    ros::Time& time = planningSceneTimes[i];
    std::string ID;
    // Load it
    loadPlanningScene(time, ID);

    ROS_INFO("Got planning scene %s from warehouse.", ID.c_str());
    PlanningSceneData& data = (*planning_scene_map_)[ID];
    data.getPipelineStages().clear();
    data.getErrorCodes().clear();
    getPlanningSceneOutcomes(time, data.getPipelineStages(), data.getErrorCodes(), error_map_);
    onPlanningSceneLoaded((int)i, (int)planningSceneTimes.size());
  }

  error_map_.clear();
}

void PlanningSceneEditor::savePlanningScene(PlanningSceneData& data)
{
  // Have to do this in case robot state was corrupted by sim time.
  data.setTimeStamp(data.getTimeStamp());

  move_arm_warehouse_logger_reader_->pushPlanningSceneToWarehouse(data.getPlanningScene());

  ROS_INFO("Saving Planning Scene %s", data.getName().c_str());

  for(size_t i = 0; i < data.getRequests().size(); i++)
  {
    MotionPlanRequestData& req = (*motion_plan_map_)[data.getRequests()[i]];
    move_arm_warehouse_logger_reader_->pushMotionPlanRequestToWarehouse(data.getPlanningScene(), req.getSource(),
                                                                        req.getMotionPlanRequest(), req.getID());
    ROS_INFO("Saving Request %s", req.getID().c_str());
    for(size_t j = 0; j < req.getTrajectories().size(); j++)
    {
      TrajectoryData& traj = (*trajectory_map_)[req.getTrajectories()[j]];
      move_arm_warehouse_logger_reader_->pushJointTrajectoryToWarehouse(data.getPlanningScene(), traj.getSource(),
                                                                        traj.getDuration(), traj.getTrajectory(),
                                                                        traj.getID(), traj.getMotionPlanRequestID(), traj.trajectory_error_code_);
      move_arm_warehouse_logger_reader_->pushOutcomeToWarehouse(data.getPlanningScene(), traj.getSource(),
                                                                traj.trajectory_error_code_);
      ROS_INFO("Saving Trajectory %s", traj.getID().c_str());
    }
  }
}

bool PlanningSceneEditor::getAllPlanningSceneTimes(vector<ros::Time>& planning_scene_times)
{
  move_arm_warehouse_logger_reader_->getAvailablePlanningSceneList("", last_creation_time_query_);
  planning_scene_times = last_creation_time_query_;
  return true;
}

bool PlanningSceneEditor::loadPlanningScene(const ros::Time& time, std::string& ID)
{
  assert(planning_scene_map_ != NULL);
  deleteKinematicStates();
  PlanningSceneData data;
  data.setTimeStamp(time);
  data.setName(generateNewPlanningSceneID());
  std::string host = "";
  if(!move_arm_warehouse_logger_reader_->getPlanningScene("", time, data.getPlanningScene(), host))
  {
    return false;
  }

  data.setHostName(host);

  std::pair<string, PlanningSceneData> p(data.getName(), data);
  planning_scene_map_->insert(p);
  ID = data.getName();
  return true;
}

bool PlanningSceneEditor::getAllAssociatedMotionPlanRequests(const ros::Time& time, vector<string>& IDs,
                                                             vector<string>& stages,
                                                             vector<MotionPlanRequest>& requests)
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
  convertKinematicStateToRobotState(*robot_state_, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
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

void PlanningSceneEditor::initMotionPlanRequestData(std::string planning_scene_ID, std::vector<std::string>& IDs,
                                                      std::vector<std::string>& stages,
                                                      std::vector<arm_navigation_msgs::MotionPlanRequest>& requests)
{
  for(size_t i = 0; i < requests.size(); i++)
  {
    lockScene();
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
    data.setSource("Planner");

    StateRegistry start;
    start.state = data.getStartState();
    start.source = "Motion Plan Request Data Start from createRequest";
    StateRegistry end;
    end.state = data.getGoalState();
    end.source = "Motion Plan Request Data End from createRequest";
    states_.push_back(start);
    states_.push_back(end);

    const KinematicModel::GroupConfig& config =
        cm_->getKinematicModel()->getJointModelGroupConfigMap().at(mpr.group_name);
    std::string tip = config.tip_link_;
    data.setEndEffectorLink(tip);

    PlanningSceneData& planningSceneData = (*planning_scene_map_)[planning_scene_ID];
    planningSceneData.getRequests().push_back(data.getID());

    if(motion_plan_map_->find(data.getID()) != motion_plan_map_->end())
    {
      ROS_INFO("Deleting existing motion plan request...");
      deleteMotionPlanRequest(data.getID());
    }
    (*motion_plan_map_)[data.getID()] = data;

    lock_scene_.unlock();

    createIkControllersFromMotionPlanRequest(data, false);
    unlockScene();

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
  data.setSource("Planner");
  StateRegistry start;
  start.state = data.getStartState();
  start.source = "Motion Plan Request Data Start from loadRequest";
  StateRegistry end;
  end.state = data.getGoalState();
  end.source = "Motion Plan Request Data End from line loadRequest";
  states_.push_back(start);
  states_.push_back(end);

  const KinematicModel::GroupConfig& config =
      cm_->getKinematicModel()->getJointModelGroupConfigMap().at(mpr.group_name);
  std::string tip = config.tip_link_;
  data.setEndEffectorLink(tip);

  PlanningSceneData& planningSceneData = (*planning_scene_map_)[planning_scene_ID];
  planningSceneData.getRequests().push_back(data.getID());

  (*motion_plan_map_)[data.getID()] = data;
  ID = data.getID();

  lock_scene_.unlock();

  createIkControllersFromMotionPlanRequest(data, false);
  return true;
}

bool PlanningSceneEditor::getAllAssociatedTrajectorySources(const ros::Time& time, vector<string>& trajectory_sources)
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
  ArmNavigationErrorCodes oldValue;
  oldValue.val = data.trajectory_error_code_.val;
  ArmNavigationErrorCodes& errorCode = data.trajectory_error_code_;
  cm_->disableCollisionsForNonUpdatedLinks(data.getGroupName());

  vector<ArmNavigationErrorCodes> trajectory_error_codes;
  cm_->isJointTrajectoryValid(*(data.getCurrentState()), data.getTrajectory(),
                              requestData.getMotionPlanRequest().goal_constraints,
                              requestData.getMotionPlanRequest().path_constraints, errorCode,
                              trajectory_error_codes, false);

  cm_->revertAllowedCollisionToDefault();

  if(errorCode.val != errorCode.SUCCESS)
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
    errorCode.val = oldValue.val;
  }

  data.moveThroughTrajectory(0);
  lock_scene_.unlock();
  return true;
}

void PlanningSceneEditor::createSelectableMarkerFromCollisionObject(CollisionObject& object, string name,
                                                                    string description, std_msgs::ColorRGBA color)
{
  SelectableObject selectable;
  selectable.ID_ = name;
  selectable.collision_object_ = object;
  selectable.control_marker_.pose = object.poses[0];
  selectable.control_marker_.header.frame_id = "/" + cm_->getWorldFrameId();
  selectable.control_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());

  selectable.selection_marker_.pose = object.poses[0];
  selectable.selection_marker_.header.frame_id = "/" + cm_->getWorldFrameId();
  selectable.selection_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());

  selectable.color_ = color;

  InteractiveMarkerControl button;
  button.name = name;
  button.interaction_mode = InteractiveMarkerControl::BUTTON;
  button.description = description;
  float maxScale = 0.0f;

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    arm_navigation_msgs::Shape& shape = object.shapes[i];
    Marker mark;
    mark.color = color;
    //mark.pose = object.poses[i];


    switch (shape.type)
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

void PlanningSceneEditor::JointControllerCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::string ID = "";
  std::string MPR = "";
  PositionType type = StartPosition;

  if(feedback->marker_name.rfind("_start_control") != std::string::npos)
  {
    std::string sub1 = feedback->marker_name.substr(0, feedback->marker_name.rfind("_start_control"));
    ID = sub1.substr(0, sub1.rfind("_mpr_"));
    MPR = sub1.substr(sub1.rfind("_mpr_") + 5, sub1.length());
    type = StartPosition;
  }
  else if(feedback->marker_name.rfind("_end_control") != std::string::npos)
  {
    std::string sub1 = feedback->marker_name.substr(0, feedback->marker_name.rfind("_end_control"));
    ID = sub1.substr(0, sub1.rfind("_mpr_"));
    MPR = sub1.substr(sub1.rfind("_mpr_") + 5, sub1.length());
    type = GoalPosition;
  }

  setJointState((*motion_plan_map_)[MPR], type, ID, toBulletTransform(feedback->pose));
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
    type = GoalPosition;
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
      (*motion_plan_map_)[controller.motion_plan_ID_].getStartState()->updateKinematicStateWithLinkAt((*motion_plan_map_)[controller.motion_plan_ID_].getEndEffectorLink(),
                                                                                                      pose);
      findIKSolution = true;
      if(selected_motion_plan_ID_ != controller.motion_plan_ID_)
      {
        selected_motion_plan_ID_ = controller.motion_plan_ID_;
        updateState();
      }
    }
    else
    {
      (*motion_plan_map_)[controller.motion_plan_ID_].getGoalState()->updateKinematicStateWithLinkAt((*motion_plan_map_)[controller.motion_plan_ID_].getEndEffectorLink(),
                                                                                                     pose);
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
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodStartPose()),
                                            feedback->header);
        findIKSolution = true;
      }
      else
      {
        data.getGoalState()->updateKinematicStateWithLinkAt(data.getEndEffectorLink(), (data.getLastGoodGoalPose()));
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodGoalPose()),
                                            feedback->header);
        findIKSolution = true;
      }
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Randomly Perturb"])
    {
      MotionPlanRequestData& data = (*motion_plan_map_)[controller.motion_plan_ID_];

      randomlyPerturb(data, type);
      if(type == StartPosition)
      {
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodStartPose()),
                                            feedback->header);
      }
      else
      {
        interactive_marker_server_->setPose(feedback->marker_name, toGeometryPose(data.getLastGoodGoalPose()),
                                            feedback->header);
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
        filterTrajectory(data, (*trajectory_map_)[selected_trajectory_ID_], trajectory);
        selected_trajectory_ID_ = trajectory;
        playTrajectory(data, (*trajectory_map_)[selected_trajectory_ID_]);
        updateState();
      }
    }
    else if(feedback->menu_entry_id == menu_entry_maps_["IK Control"]["Execute Last Trajectory"])
    {
      std::string trajectory;
      if(selected_trajectory_ID_ != "" && trajectory_map_->find(selected_trajectory_ID_) != trajectory_map_->end())
      {
        executeTrajectory(selected_trajectory_ID_);
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

  if((*motion_plan_map_)[controller.motion_plan_ID_].areJointControlsVisible())
  {
    createJointMarkers((*motion_plan_map_)[controller.motion_plan_ID_], type);
  }
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::createIkControllersFromMotionPlanRequest(MotionPlanRequestData& data, bool rePose)
{
  if(data.isStartEditable())
  {
    createIKController(data, StartPosition, rePose);
  }

  if(data.isGoalEditable())
  {
    createIKController(data, GoalPosition, rePose);
  }

  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::createIKController(MotionPlanRequestData& data, PositionType type, bool rePose)
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

  btTransform transform = state->getLinkState(data.getEndEffectorLink())->getGlobalLinkTransform();
  InteractiveMarker marker;


  if(interactive_marker_server_->get(data.getID() + nametag, marker) && rePose)
  {
    geometry_msgs::Pose pose =  toGeometryPose(transform);
    interactive_marker_server_->setPose(data.getID() + nametag, pose);
    return;
  }


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

void PlanningSceneEditor::deleteCollisionObject(std::string& name)
{
  (*selectable_objects_)[name].collision_object_.operation.operation
      = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  interactive_marker_server_->erase((*selectable_objects_)[name].selection_marker_.name);
  sendPlanningScene((*planning_scene_map_)[current_planning_scene_ID_]);
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::collisionObjectSelectionCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(feedback->marker_name.rfind("collision_object") == string::npos)
  {
    return;
  }
  std::string name = feedback->marker_name.substr(0, feedback->marker_name.rfind("_selection"));
  bool shouldSelect = false;
  switch (feedback->event_type)
  {
    case InteractiveMarkerFeedback::BUTTON_CLICK:
      shouldSelect = true;
      break;
    case InteractiveMarkerFeedback::MENU_SELECT:
      if(feedback->menu_entry_id == menu_entry_maps_["Collision Object Selection"]["Delete"])
      {
        deleteCollisionObject(name);
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
    (*selectable_objects_)[name].control_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());

    interactive_marker_server_->insert((*selectable_objects_)[name].control_marker_,
                                       collision_object_movement_feedback_ptr_);

    menu_handler_map_["Collision Object"].apply(*interactive_marker_server_,
                                                (*selectable_objects_)[name].control_marker_.name);
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

  switch (feedback->event_type)
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
        (*selectable_objects_)[name].collision_object_.operation.operation
            = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
        interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
        sendPlanningScene((*planning_scene_map_)[current_planning_scene_ID_]);
      }
      else if(feedback->menu_entry_id == menu_entry_maps_["Collision Object"]["Deselect"])
      {
        interactive_marker_server_->erase((*selectable_objects_)[name].control_marker_.name);
        (*selectable_objects_)[name].selection_marker_.pose = feedback->pose;
        (*selectable_objects_)[name].selection_marker_.header.stamp = ros::Time(ros::WallTime::now().toSec());
        interactive_marker_server_->insert((*selectable_objects_)[name].selection_marker_,
                                           collision_object_selection_feedback_ptr_);
        menu_handler_map_["Collision Object Selection"].apply(*interactive_marker_server_,
                                                              (*selectable_objects_)[name].selection_marker_.name);

      }
      break;

    case InteractiveMarkerFeedback::POSE_UPDATE:
      break;
  }

  interactive_marker_server_->applyChanges();
}

std::string PlanningSceneEditor::createCollisionObject(geometry_msgs::Pose pose, PlanningSceneEditor::GeneratedShape shape,
                                                float scaleX, float scaleY, float scaleZ, std_msgs::ColorRGBA color)
{
  lockScene();
  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  collision_object.header.stamp = ros::Time(ros::WallTime::now().toSec());
  collision_object.header.frame_id = cm_->getWorldFrameId();
  collision_object.id = generateNewCollisionObjectID();
  arm_navigation_msgs::Shape object;

  switch (shape)
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
      object.dimensions[0] = scaleX * 0.5f;
      object.dimensions[1] = scaleZ;
      break;
    case PlanningSceneEditor::Sphere:
      object.type = arm_navigation_msgs::Shape::SPHERE;
      object.dimensions.resize(1);
      object.dimensions[0] = scaleX * 0.5f;
      break;
    default:
      object.type = arm_navigation_msgs::Shape::SPHERE;
      object.dimensions.resize(1);
      object.dimensions[0] = scaleX * 0.5f;
      break;
  };

  collision_object.shapes.push_back(object);
  collision_object.poses.push_back(pose);

  btTransform cur = toBulletTransform(pose);

  createSelectableMarkerFromCollisionObject(collision_object, collision_object.id, "", color);

  ROS_INFO("Created collision object.");
  ROS_INFO("Sending planning scene %s", current_planning_scene_ID_.c_str());

  sendPlanningScene((*planning_scene_map_)[current_planning_scene_ID_]);

  unlockScene();
  return collision_object.id;

}

bool PlanningSceneEditor::solveIKForEndEffectorPose(MotionPlanRequestData& mpr,
                                                    planning_scene_utils::PositionType type, bool coll_aware,
                                                    bool constrain_pitch_and_roll, double change_redundancy)
{
  kinematics_msgs::PositionIKRequest ik_request;
  ik_request.ik_link_name = mpr.getEndEffectorLink();
  ik_request.pose_stamped.header.frame_id = cm_->getWorldFrameId();
  ik_request.pose_stamped.header.stamp = ros::Time(ros::WallTime::now().toSec());

  KinematicState* state = NULL;

  if(type == StartPosition)
  {
    state = mpr.getStartState();
  }
  else
  {
    state = mpr.getGoalState();
  }

  tf::poseTFToMsg(state->getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform(), ik_request.pose_stamped.pose);

  convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                    ik_request.robot_state);
  ik_request.ik_seed_state = ik_request.robot_state;

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
      determinePitchRollConstraintsGivenState(*state, name, goal_constraints.orientation_constraints[0],
                                              path_constraints.orientation_constraints[0]);

      arm_navigation_msgs::ArmNavigationErrorCodes err;
      if(!cm_->isKinematicStateValid(*state, std::vector<std::string>(), err, goal_constraints, path_constraints))
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

  if(coll_aware)
  {
    Constraints emp_con;
    ArmNavigationErrorCodes error_code;

    if(!cm_->isKinematicStateValid(*state, joint_names, error_code, emp_con, emp_con, true))
    {
      ROS_INFO_STREAM("Problem with response");
      return false;
    }
  }

  if(type == StartPosition)
  {
    mpr.setStartStateValues(joint_values);
    convertKinematicStateToRobotState(*state, ros::Time(ros::WallTime::now().toSec()), cm_->getWorldFrameId(),
                                      mpr.getMotionPlanRequest().start_state);
    mpr.setLastGoodStartPose((state->getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform()));
  }
  else
  {
    mpr.setGoalStateValues(joint_values);
    mpr.setLastGoodGoalPose((state->getLinkState(mpr.getEndEffectorLink())->getGlobalLinkTransform()));
  }

  return true;
}

void PlanningSceneEditor::setJointState(MotionPlanRequestData& data, PositionType position, std::string& jointName,
                                        btTransform value)
{
  KinematicState* currentState = NULL;

  if(position == StartPosition)
  {
    currentState = data.getStartState();
  }
  else if(position == GoalPosition)
  {
    currentState = data.getGoalState();
  }

  if(currentState == NULL)
  {
    ROS_ERROR("Robot state for request %s is null!", data.getID().c_str());
    return;
  }

  string parentLink = currentState->getKinematicModel()->getJointModel(jointName)->getParentLinkModel()->getName();
  string childLink = currentState->getKinematicModel()->getJointModel(jointName)->getChildLinkModel()->getName();
  KinematicState::JointState* jointState = currentState->getJointState(jointName);
  const KinematicModel::JointModel* jointModel = jointState->getJointModel();

  bool isRotational = (dynamic_cast<const KinematicModel::RevoluteJointModel*> (jointModel) != NULL);
  bool isPrismatic = (dynamic_cast<const KinematicModel::PrismaticJointModel*> (jointModel) != NULL);

  KinematicState::LinkState* linkState = currentState->getLinkState(parentLink);
  btTransform transformedValue;

  if(isPrismatic)
  {
    value.setRotation(jointState->getVariableTransform().getRotation());
    transformedValue = currentState->getLinkState(childLink)->getLinkModel()->getJointOriginTransform().inverse()
        * linkState->getGlobalLinkTransform().inverse() * value;
  }
  else if(isRotational)
  {
    transformedValue = currentState->getLinkState(childLink)->getLinkModel()->getJointOriginTransform().inverse()
        * linkState->getGlobalLinkTransform().inverse() * value;
  }

  btTransform oldState = jointState->getVariableTransform();
  jointState->setJointStateValues(transformedValue);

  map<string, double> stateMap;
  if(currentState->isJointWithinBounds(jointName))
  {
    currentState->getKinematicStateValues(stateMap);
    currentState->setKinematicState(stateMap);


    // Send state to robot model.
    if(position == StartPosition)
    {
      convertKinematicStateToRobotState(*currentState,
                                        data.getMotionPlanRequest().start_state.joint_state.header.stamp,
                                        data.getMotionPlanRequest().start_state.joint_state.header.frame_id,
                                        data.getMotionPlanRequest().start_state);
    }
    else
    {
      std::vector<JointConstraint>& constraints = data.getMotionPlanRequest().goal_constraints.joint_constraints;

      for(size_t i = 0; i < constraints.size(); i++)
      {
        JointConstraint& constraint = constraints[i];
        constraint.position = stateMap[constraint.joint_name];
      }
    }


    createIKController(data, position, true);
    createJointMarkers(data, position);
  }
  else
  {
    jointState->setJointStateValues(oldState);
  }
}

void PlanningSceneEditor::deleteJointMarkers(MotionPlanRequestData& data, PositionType type)
{
  vector<string> jointNames = data.getJointNamesInGoal();
  for(size_t i = 0; i < jointNames.size(); i++)
  {
    if(type == StartPosition)
    {
      std::string markerName = jointNames[i] + "_mpr_" + data.getID() + "_start_control";

      InteractiveMarker dummy;
      if(interactive_marker_server_->get(markerName, dummy))
      {
        interactive_marker_server_->erase(markerName);
      }
    }
    else
    {
      std::string markerName = jointNames[i] + "_mpr_" + data.getID() + "_end_control";
      InteractiveMarker dummy;
      if(interactive_marker_server_->get(markerName, dummy))
      {
        interactive_marker_server_->erase(markerName);
      }
    }
  }
}

void PlanningSceneEditor::createJointMarkers(MotionPlanRequestData& data, PositionType position)
{
  vector<string> jointNames = data.getJointNamesInGoal();

  KinematicState* state = NULL;
  std::string sauce = "";

  if(position == StartPosition)
  {
    state = data.getStartState();
    sauce = "_start_control";
  }
  else if(position == GoalPosition)
  {
    state = data.getGoalState();
    sauce = "_end_control";
  }

  // For each joint model, find the location of its axis and make a control there.
  for(size_t i = 0; i < jointNames.size(); i++)
  {
    const string& jointName = jointNames[i];
    KinematicModel::JointModel* model =
        (KinematicModel::JointModel*)(state->getKinematicModel()->getJointModel(jointName));

    std::string controlName = jointName + "_mpr_" + data.getID() + sauce;
    joint_clicked_map_[controlName] = false;

    if(model->getParentLinkModel() != NULL)
    {
      string parentLinkName = model->getParentLinkModel()->getName();
      string childLinkName = model->getChildLinkModel()->getName();
      btTransform transform = state->getLinkState(parentLinkName)->getGlobalLinkTransform()
          * (state->getKinematicModel()->getLinkModel(childLinkName)->getJointOriginTransform()
              * (state->getJointState(jointName)->getVariableTransform()));

      joint_prev_transform_map_[controlName] = transform;

      InteractiveMarker dummy;
      if(interactive_marker_server_->get(controlName, dummy))
      {
        dummy.header.frame_id = cm_->getWorldFrameId();
        interactive_marker_server_->setPose(controlName, toGeometryPose(transform), dummy.header);
        continue;
      }

      const shapes::Shape* linkShape = model->getChildLinkModel()->getLinkShape();
      const shapes::Mesh* meshShape = dynamic_cast<const shapes::Mesh*> (linkShape);

      KinematicModel::RevoluteJointModel* revoluteJoint = dynamic_cast<KinematicModel::RevoluteJointModel*> (model);
      KinematicModel::PrismaticJointModel* prismaticJoint = dynamic_cast<KinematicModel::PrismaticJointModel*> (model);
      double maxDimension = 0.0f;
      if(meshShape != NULL)
      {

        for(unsigned int i = 0; i < meshShape->vertexCount; i++)
        {
          double x = meshShape->vertices[3 * i];
          double y = meshShape->vertices[3 * i];
          double z = meshShape->vertices[3 * i];

          if(abs(maxDimension) < abs(sqrt(x * x + y * y + z * z)))
          {
            maxDimension = abs(x);
          }

        }

        maxDimension *= 3.0;

        maxDimension = max(0.15, maxDimension);
        maxDimension = min(0.5, maxDimension);
      }
      else
      {
        maxDimension = 0.15;
      }

      if(revoluteJoint != NULL)
      {
        makeInteractive1DOFRotationMarker(transform, revoluteJoint->axis_, controlName, "", (float)maxDimension,
                                          state->getJointState(jointName)->getJointStateValues()[0]);
      }
      else if(prismaticJoint != NULL)
      {
        makeInteractive1DOFTranslationMarker(transform, prismaticJoint->axis_, controlName, "", (float)maxDimension,
                                             state-> getJointState(jointName)->getJointStateValues()[0]);
      }

    }
  }
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::makeInteractive1DOFTranslationMarker(btTransform transform, btVector3 axis, string name,
                                                               string description, float scale, float value)
{
  InteractiveMarker marker;
  marker.header.frame_id = cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = scale;
  marker.name = name;
  marker.description = description;
  InteractiveMarker dummy;
  InteractiveMarkerControl control;
  if(interactive_marker_server_->get(marker.name, dummy))
  {
    interactive_marker_server_->setPose(marker.name, marker.pose, marker.header);
  }
  else
  {
    control.orientation.x = axis.x();
    control.orientation.y = axis.z();
    control.orientation.z = axis.y();
    control.orientation.w = 1;
    control.independent_marker_orientation = false;
    control.always_visible = false;
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
    interactive_marker_server_->insert(marker);
    interactive_marker_server_->setCallback(marker.name, joint_control_feedback_ptr_);
  }

}

void PlanningSceneEditor::makeInteractive1DOFRotationMarker(btTransform transform, btVector3 axis, string name,
                                                            string description, float scale, float angle)
{
  InteractiveMarker marker;
  marker.header.frame_id = cm_->getWorldFrameId();
  marker.pose.position.x = transform.getOrigin().x();
  marker.pose.position.y = transform.getOrigin().y();
  marker.pose.position.z = transform.getOrigin().z();
  marker.pose.orientation.w = transform.getRotation().w();
  marker.pose.orientation.x = transform.getRotation().x();
  marker.pose.orientation.y = transform.getRotation().y();
  marker.pose.orientation.z = transform.getRotation().z();
  marker.scale = scale;
  marker.name = name;
  marker.description = description;

  InteractiveMarker dummy;
  if(interactive_marker_server_->get(marker.name, dummy))
  {
    interactive_marker_server_->setPose(marker.name, marker.pose, marker.header);
  }
  else
  {
    InteractiveMarkerControl control;
    control.orientation.x = axis.x();
    control.orientation.y = axis.z();
    control.orientation.z = axis.y();
    control.orientation.w = 1;
    control.independent_marker_orientation = false;
    control.always_visible = false;
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    interactive_marker_server_->insert(marker);
    interactive_marker_server_->setCallback(marker.name, joint_control_feedback_ptr_);
  }
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
    createIKController((*motion_plan_map_)[ID], type, false);
    interactive_marker_server_->applyChanges();
  }
}

void PlanningSceneEditor::executeTrajectory(TrajectoryData& trajectory)
{
  if(params_.sync_robot_state_with_gazebo_)
  {
    pr2_mechanism_msgs::ListControllers listControllers;

    if(!list_controllers_client_.call(listControllers.request, listControllers.response))
    {
      ROS_ERROR("Failed to get list of controllers!");
      return;
    }

    std::map<std::string, planning_models::KinematicModel::JointModelGroup*> jointModelGroupMap =
        cm_->getKinematicModel()->getJointModelGroupMap();
    planning_models::KinematicModel::JointModelGroup* rightGroup = NULL;
    planning_models::KinematicModel::JointModelGroup* leftGroup = NULL;
    if(params_.right_arm_group_ != "none")
    {
      rightGroup = jointModelGroupMap[params_.right_arm_group_];
    }

    if(params_.left_arm_group_ != "none")
    {
      leftGroup = jointModelGroupMap[params_.left_arm_group_];
    }

    pr2_mechanism_msgs::SwitchController switchControllers;
    switchControllers.request.stop_controllers = listControllers.response.controllers;
    if(!switch_controllers_client_.call(switchControllers.request, switchControllers.response))
    {
      ROS_ERROR("Failed to shut down controllers!");
      return;
    }

    ROS_INFO("Shut down controllers.");

    MotionPlanRequestData& motionPlanData = (*motion_plan_map_)[trajectory.getMotionPlanRequestID()];

    gazebo_msgs::SetModelConfiguration modelConfiguration;
    modelConfiguration.request.model_name = params_.gazebo_model_name_;
    modelConfiguration.request.urdf_param_name = params_.robot_description_param_;

    for(size_t i = 0; i < motionPlanData.getStartState()->getJointStateVector().size(); i++)
    {
      const KinematicState::JointState* jointState = motionPlanData.getStartState()->getJointStateVector()[i];
      if(jointState->getJointStateValues().size() > 0)
      {
        modelConfiguration.request.joint_names.push_back(jointState->getName());
        modelConfiguration.request.joint_positions.push_back(jointState->getJointStateValues()[0]);
      }
    }

    if(!gazebo_joint_state_client_.call(modelConfiguration.request, modelConfiguration.response))
    {
      ROS_ERROR("Failed to call gazebo set joint state client!");
      return;
    }

    ROS_INFO("Set joint state");

    if(!modelConfiguration.response.success)
    {
      ROS_ERROR("Failed to set gazebo model configuration to start state!");
      return;
    }
    ROS_INFO("Gazebo returned: %s", modelConfiguration.response.status_message.c_str());

    pr2_mechanism_msgs::SwitchController restartControllers;
    restartControllers.request.start_controllers = listControllers.response.controllers;
    if(!switch_controllers_client_.call(restartControllers.request, restartControllers.response))
    {
      ROS_ERROR("Failed to restart controllers: service call failed!");
      return;
    }
    else if(!restartControllers.response.ok)
    {
      ROS_ERROR("Failed to restart controllers: Response not ok!");
    }

    ROS_INFO("Restart controllers.");

    ros::Time::sleepUntil(ros::Time::now() + ros::Duration(0.5));
    SimpleActionClient<FollowJointTrajectoryAction>* controller = arm_controller_map_[trajectory.getGroupName()];
    FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = trajectory.getTrajectory().joint_names;
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
    trajectory_msgs::JointTrajectoryPoint endPoint = trajectory.getTrajectory().points[0];
    endPoint.time_from_start = ros::Duration(1.0);
    goal.trajectory.points.push_back(endPoint);
    controller->sendGoalAndWait(goal, ros::Duration(1.0), ros::Duration(1.0));
    ros::Time::sleepUntil(ros::Time::now() + ros::Duration(1.0));

  }

  SimpleActionClient<FollowJointTrajectoryAction>* controller = arm_controller_map_[trajectory.getGroupName()];
  FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory.getTrajectory();
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.2);
  controller->sendGoal(goal, boost::bind(&PlanningSceneEditor::controllerDoneCallback, this, _1, _2));
  logged_group_name_ = trajectory.getGroupName();
  logged_motion_plan_request_ = trajectory.getMotionPlanRequestID();
  logged_trajectory_ = trajectory.getTrajectory();
  logged_trajectory_.points.clear();
  logged_trajectory_start_time_ = ros::Time::now() + ros::Duration(0.2);
  monitor_status_ = Executing;

}

void PlanningSceneEditor::randomlyPerturb(MotionPlanRequestData& mpr, PositionType type)
{
  lockScene();

  //Joint space method

  KinematicState* currentState = NULL;

  if(type == StartPosition)
  {
    currentState = mpr.getStartState();
  }
  else
  {
    currentState = mpr.getGoalState();
  }

  vector<KinematicState::JointState*>& jointStates = currentState->getJointStateVector();
  std::map<string, double> originalState;
  std::map<string, double> stateMap;
  bool goodSolution = false;
  int numIterations = 0;
  int maxIterations = 100;
  while(!goodSolution && numIterations < maxIterations)
  {
    currentState->getKinematicStateValues(stateMap);
    for(size_t i = 0; i < jointStates.size(); i++)
    {
      KinematicState::JointState* jointState = jointStates[i];
      map<string, pair<double, double> > bounds = jointState->getJointModel()->getAllVariableBounds();
      for(map<string, pair<double, double> >::iterator it = bounds.begin(); it != bounds.end(); it++)
      {
        if(!mpr.isJointNameInGoal(it->first))
        {
          continue;
        }
        double range = it->second.second - it->second.first;
        if(range == std::numeric_limits<double>::infinity())
        {
          continue;
        }
        double randVal = ((double)random() / (double)RAND_MAX) * (range * 0.99) + it->second.first;
        stateMap[it->first] = randVal;
      }
    }

    currentState->setKinematicState(stateMap);

    if(!cm_->isKinematicStateInCollision(*currentState))
    {
      goodSolution = true;
      break;
    }
    numIterations++;
  }

  if(!goodSolution)
  {
    currentState->setKinematicState(originalState);
    unlockScene();
    return;
  }
  else
  {
    ROS_INFO("Found a good random solution in %d iterations", numIterations);
  }

  if(type == StartPosition)
  {
    convertKinematicStateToRobotState(*currentState, mpr.getMotionPlanRequest().start_state.joint_state.header.stamp,
                                      mpr.getMotionPlanRequest().start_state.joint_state.header.frame_id,
                                      mpr.getMotionPlanRequest().start_state);
  }
  else
  {
    std::vector<JointConstraint>& constraints = mpr.getMotionPlanRequest().goal_constraints.joint_constraints;
    for(size_t i = 0; i < constraints.size(); i++)
    {
      JointConstraint& constraint = constraints[i];
      constraint.position = stateMap[constraint.joint_name];
    }
  }
  mpr.setHasGoodIKSolution(true);
  createIKController(mpr, type, false);
  mpr.setJointControlsVisible(mpr.areJointControlsVisible(), this);
  interactive_marker_server_->applyChanges();
  mpr.refreshColors();
  unlockScene();

}

void PlanningSceneEditor::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                 const control_msgs::FollowJointTrajectoryResultConstPtr& result)
{
  monitor_status_ = Idle;
  TrajectoryData logged(generateNewTrajectoryID(), "Robot Monitor", logged_group_name_, logged_trajectory_);
  logged.setBadPoint(-1);
  logged.setDuration(ros::Time::now() - logged_trajectory_start_time_);
  logged.setMotionPlanRequestID(logged_motion_plan_request_);
  logged.trajectory_error_code_.val = result->error_code;
  MotionPlanRequestData& mpr = (*motion_plan_map_)[logged_motion_plan_request_];
  mpr.getTrajectories().push_back(logged.getID());
  (*planning_scene_map_)[current_planning_scene_ID_].getTrajectories().push_back(logged.getID());
  (*trajectory_map_)[logged.getID()] = logged;
  logged_trajectory_.points.clear();
  logged_group_name_ = "";
  logged_motion_plan_request_ = "";
  selected_trajectory_ID_ = logged.getID();
  updateState();
  ROS_INFO("CREATING TRAJECTORY %s", logged.getID().c_str());
}

void PlanningSceneEditor::getAllRobotStampedTransforms(const planning_models::KinematicState& state,
                                  vector<geometry_msgs::TransformStamped>& trans_vector, const ros::Time& stamp)
{
  trans_vector.clear();
  const map<string, geometry_msgs::TransformStamped>& transforms = cm_->getSceneTransformMap();
  geometry_msgs::TransformStamped transvec;
  for(map<string, geometry_msgs::TransformStamped>::const_iterator it = transforms.begin(); it
      != transforms.end(); it++)
  {
    if(it->first != cm_->getWorldFrameId())
    {
      trans_vector.push_back(it->second);
    }
  }
  for(unsigned int i = 0; i < state.getLinkStateVector().size(); i++)
  {
    const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[i];

    if(ls->getName() != cm_->getWorldFrameId())
    {
      geometry_msgs::TransformStamped ts;
      ts.header.stamp = stamp;
      ts.header.frame_id = cm_->getWorldFrameId();

      ts.child_frame_id = ls->getName();
      tf::transformTFToMsg(ls->getGlobalLinkTransform(), ts.transform);
      trans_vector.push_back(ts);
    }
  }
}

void PlanningSceneEditor::sendTransformsAndClock()
{
  if(robot_state_ == NULL)
  {
    return;
  }

  if(!params_.use_robot_data_)
  {
    ros::WallTime cur_time = ros::WallTime::now();
    rosgraph_msgs::Clock c;
    c.clock.nsec = cur_time.nsec;
    c.clock.sec = cur_time.sec;
    //clock_publisher_.publish(c);


    getAllRobotStampedTransforms(*robot_state_, robot_transforms_, c.clock);
    transform_broadcaster_.sendTransform(robot_transforms_);
  }
}

MenuHandler::EntryHandle PlanningSceneEditor::registerSubMenuEntry(string menu, string name, string subMenu,
                                                                   MenuHandler::FeedbackCallback& callback)
{

  MenuHandler::EntryHandle toReturn = menu_handler_map_[menu].insert(menu_entry_maps_[menu][subMenu], name, callback);
  menu_entry_maps_[menu][name] = toReturn;
  return toReturn;
}

MenuHandler::EntryHandle PlanningSceneEditor::registerMenuEntry(string menu, string entryName,
                                                                MenuHandler::FeedbackCallback& callback)
{
  MenuHandler::EntryHandle toReturn = menu_handler_map_[menu].insert(entryName, callback);
  menu_entry_maps_[menu][entryName] = toReturn;
  return toReturn;
}

void PlanningSceneEditor::deleteTrajectory(std::string ID)
{
  lockScene();
  if(trajectory_map_->find(ID) != trajectory_map_->end())
  {

    if(current_planning_scene_ID_ != "")
    {
      PlanningSceneData& data = (*planning_scene_map_)[current_planning_scene_ID_];
      MotionPlanRequestData& requestData = (*motion_plan_map_)[(*trajectory_map_)[ID].getMotionPlanRequestID()];

      std::vector<std::string>::iterator erasure = data.getTrajectories().end();
      for(std::vector<std::string>::iterator it = data.getTrajectories().begin(); it != data.getTrajectories().end(); it++)
      {
        if((*it) == ID)
        {
          erasure = it;
          break;
        }
      }

      if(erasure != data.getTrajectories().end())
      {
        data.getTrajectories().erase(erasure);
      }

      std::vector<std::string>::iterator mprerasure = requestData.getTrajectories().end();
      bool found = false;
      int i = 0;
      for(std::vector<std::string>::iterator it = requestData.getTrajectories().begin(); it
          != requestData.getTrajectories().end(); it++)
      {
        if((*it) == ID)
        {
          mprerasure = it;
          found = true;
          break;
        }
        i++;
      }

      if(found)
      {
        requestData.getTrajectories().erase(mprerasure);
      }
    }

    for(size_t i = 0; i < states_.size(); i++)
    {
      if(states_[i].state == (*trajectory_map_)[ID].getCurrentState())
      {
        states_[i].state = NULL;
        states_[i].source = "Delete trajectory";
      }
    }

    (*trajectory_map_)[ID].reset();
    trajectory_map_->erase(ID);

  }
  unlockScene();
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::deleteMotionPlanRequest(std::string ID)
{
  lockScene();
  if(motion_plan_map_->find(ID) != motion_plan_map_->end())
  {
    for(size_t i = 0; i < states_.size(); i++)
    {
      if(states_[i].state == (*motion_plan_map_)[ID].getStartState() || states_[i].state
          == (*motion_plan_map_)[ID].getGoalState())
      {
        states_[i].state = NULL;
        states_[i].source = "Delete motion plan request";
      }
    }
    (*motion_plan_map_)[ID].reset();

    MotionPlanRequestData& motionPlanData = (*motion_plan_map_)[ID];
    for(size_t i = 0; i < motionPlanData.getTrajectories().size(); i++)
    {
      deleteTrajectory(motionPlanData.getTrajectories()[i]);
    }

    motion_plan_map_->erase(ID);
    interactive_marker_server_->erase(ID + "_start_control");
    interactive_marker_server_->erase(ID + "_end_control");

    if(current_planning_scene_ID_ != "")
    {
      PlanningSceneData& data = (*planning_scene_map_)[current_planning_scene_ID_];
      std::vector<std::string>::iterator erasure = data.getRequests().end();

      for(std::vector<std::string>::iterator it = data.getRequests().begin(); it != data.getRequests().end(); it++)
      {
        if((*it) == ID)
        {
          erasure = it;
          break;
        }
      }

      if(erasure != data.getRequests().end())
      {
        data.getRequests().erase(erasure);
      }
    }

    updateState();
  }
  unlockScene();
  interactive_marker_server_->applyChanges();
}

void PlanningSceneEditor::executeTrajectory(std::string trajectory_ID)
{
  if(trajectory_map_->find(trajectory_ID) != trajectory_map_->end())
  {
    executeTrajectory((*trajectory_map_)[trajectory_ID]);
  }
}
