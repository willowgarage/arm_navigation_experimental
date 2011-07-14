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
#ifndef MOVE_ARM_UTILS_H
#define MOVE_ARM_UTILS_H
#include <planning_environment/models/collision_models.h>
#include <arm_navigation_msgs/PlanningScene.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include <planning_environment/models/model_utils.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>
#include <kinematics_msgs/GetConstraintAwarePositionIK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <arm_navigation_msgs/GetMotionPlan.h>
#include <arm_navigation_msgs/GetStateValidity.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <move_arm_warehouse/move_arm_warehouse_logger_reader.h>
#include <arm_navigation_msgs/convert_messages.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/tools.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef map<std::string, interactive_markers::MenuHandler::EntryHandle> MenuEntryMap;
typedef map<std::string, MenuEntryMap> MenuMap;
typedef map<std::string, interactive_markers::MenuHandler> MenuHandlerMap;

namespace planning_scene_utils
{
  enum PositionType
  {
    StartPosition,
    EndPosition
  };

  class PlanningSceneData
  {
    protected:
      std::string name_;
      ros::Time timestamp_;
      arm_navigation_msgs::PlanningScene planning_scene_;
      std::vector<std::string> pipeline_stages_;
      std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> error_codes_;
      std::vector<std::string> trajectories_;
      std::vector<std::string> motion_plan_requests_;

    public:
      PlanningSceneData();
      PlanningSceneData(std::string name, ros::Time timestamp, arm_navigation_msgs::PlanningScene scene);

      inline std::string getName()
      {
        return name_;
      }

      inline ros::Time getTimeStamp()
      {
        return timestamp_;
      }

      inline arm_navigation_msgs::PlanningScene& getPlanningScene()
      {
        return planning_scene_;
      }

      inline void setName(std::string name)
      {
        name_ = name;
      }

      inline void setTimeStamp(ros::Time time)
      {
        timestamp_ = time;
        planning_scene_.robot_state.joint_state.header.stamp = time;
      }

      inline void setPlanningScene(arm_navigation_msgs::PlanningScene& scene)
      {
        planning_scene_ = scene;
        timestamp_ = scene.robot_state.joint_state.header.stamp;
      }

      inline std::vector<std::string>& getPipelineStages()
      {
        return pipeline_stages_;
      }

      inline void setPipelineStages(std::vector<std::string>& stages)
      {
        pipeline_stages_ = stages;
      }

      inline std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& getErrorCodes()
      {
        return error_codes_;
      }

      inline void setErrorCodes(std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& error_codes)
      {
        error_codes_ = error_codes;
      }

      inline std::vector<std::string>& getTrajectories()
      {
        return trajectories_;
      }

      inline std::vector<std::string>& getRequests()
      {
        return motion_plan_requests_;
      }

      void getRobotState(planning_models::KinematicState* state);

  };

  class MotionPlanRequestData
  {
    protected:
      std::string ID_;
      std::string source_;
      std::string planning_scene_name_;
      std::string end_effector_link_;
      std::string group_name_;
      arm_navigation_msgs::MotionPlanRequest motion_plan_request_;
      bool is_start_editable_;
      bool is_end_editable_;
      bool is_start_visible_;
      bool is_end_visible_;
      bool should_refresh_colors_;
      bool has_refreshed_colors_;
      bool has_good_ik_solution_;
      bool collisions_visible_;
      bool state_changed_;
      std_msgs::ColorRGBA start_color_;
      std_msgs::ColorRGBA end_color_;
      std::vector<std::string> trajectories_;
      planning_models::KinematicState* start_state_;
      planning_models:: KinematicState* end_state_;
      btTransform last_good_start_pose_;
      btTransform last_good_end_pose_;
      visualization_msgs::MarkerArray collision_markers_;

    public:
      MotionPlanRequestData(){start_state_ = NULL; end_state_ = NULL;};
      MotionPlanRequestData(planning_models::KinematicState* robot_state);
      MotionPlanRequestData(std::string ID, std::string source, arm_navigation_msgs::MotionPlanRequest request, planning_models::KinematicState* robot_state);
      int refresh_counter_;

      void setStartStateValues(std::map<std::string, double>& joint_values);
      void setGoalStateValues(std::map<std::string, double>& joint_values);

      void updateStartState();
      void updateGoalState();


      inline bool hasStateChanged()
      {
        return state_changed_;
      }

      inline void setStateChanged(bool changed)
      {
        state_changed_ = changed;
      }

      inline visualization_msgs::MarkerArray& getCollisionMarkers()
      {
        return collision_markers_;
      }

      inline bool areCollisionsVisible()
      {
        return collisions_visible_;
      }

      inline void setCollisionsVisible(bool visible)
      {
        collisions_visible_ = visible;
      }

      inline void showCollisions()
      {
        setCollisionsVisible(true);
      }

      inline void hideCollisions()
      {
        setCollisionsVisible(false);
      }

      inline btTransform getLastGoodStartPose()
      {
        return last_good_start_pose_;
      }

      inline btTransform getLastGoodGoalPose()
      {
        return last_good_end_pose_;
      }

      inline void setLastGoodStartPose(btTransform  pose)
      {
        last_good_start_pose_ = pose;
      }

      inline void setLastGoodGoalPose(btTransform  pose)
      {
        last_good_end_pose_ = pose;
      }

      inline void setStartState(planning_models::KinematicState* state)
      {
        start_state_ = state;
        setStateChanged(true);
      }

      inline void setGoalState(planning_models::KinematicState* state)
      {
        end_state_ = state;
        setStateChanged(true);
      }

      inline void reset()
      {
        if(start_state_ != NULL)
        {
          delete start_state_;
          start_state_ = NULL;
        }

        if(end_state_ != NULL)
        {
          delete end_state_;
          end_state_ = NULL;
        }
      }

      inline bool hasGoodIKSolution()
      {
        return has_good_ik_solution_;
      }

      inline void setHasGoodIKSolution(bool solution)
      {
        has_good_ik_solution_ = solution;
      }

      inline planning_models::KinematicState* getStartState()
      {
        return start_state_;
      }

      inline planning_models:: KinematicState* getGoalState()
      {
        return end_state_;
      }

      inline std::string getGroupName()
      {
        return group_name_;
      }

      inline std::string getEndEffectorLink()
      {
        return end_effector_link_;
      }

      inline void setGroupName(std::string name)
      {
        group_name_ = name;
      }

      inline void setEndEffectorLink(std::string name)
      {
        end_effector_link_ = name;
      }

      inline std::string getID()
      {
        return ID_;
      }

      inline void setID(std::string ID)
      {
        ID_ = ID;
      }

      inline bool shouldRefreshColors()
      {
        return should_refresh_colors_;
      }

      inline bool hasRefreshedColors()
      {
        return has_refreshed_colors_;
      }

      inline void setHasRefreshedColors(bool refresh)
      {
        has_refreshed_colors_ = refresh;

        if(refresh)
        {
          should_refresh_colors_ = false;
        }
      }

      inline void refreshColors()
      {
        should_refresh_colors_ = true;
        has_refreshed_colors_ = false;
        refresh_counter_ = 0;
      }

      inline bool isStartVisible()
      {
        return is_start_visible_;
      }

      inline bool isEndVisible()
      {
        return is_end_visible_;
      }

      inline void setStartVisible(bool visible)
      {
        is_start_visible_ = visible;
      }

      inline void setEndVisible(bool visible)
      {
        is_end_visible_ = visible;
      }

      inline void show()
      {
        setStartVisible(true);
        setEndVisible(true);
      }

      inline void hide()
      {
        setStartVisible(false);
        setEndVisible(false);
      }

      inline void showStart()
      {
        setStartVisible(true);
      }

      inline void showEnd()
      {
        setEndVisible(true);
      }

      inline void hideStart()
      {
        setStartVisible(false);
      }

      inline void hideEnd()
      {
        setEndVisible(false);
      }

      inline std_msgs::ColorRGBA getStartColor()
      {
        return start_color_;
      }

      inline std_msgs::ColorRGBA getEndColor()
      {
        return end_color_;
      }

      inline void setStartColor(std_msgs::ColorRGBA color)
      {
        start_color_ = color;
      }

      inline void setEndColor(std_msgs::ColorRGBA color)
      {
        end_color_ = color;
      }

      inline void setStartEditable(bool editable)
      {
        is_start_editable_ = editable;
      }

      inline void setEndEditable(bool editable)
      {
        is_end_editable_ = editable;
      }

      inline bool isStartEditable()
      {
        return is_start_editable_;
      }

      inline bool isEndEditable()
      {
        return is_end_editable_;
      }

      inline void setSource(std::string source)
      {
        source_ = source;
      }

      inline std::string getSource()
      {
        return source_;
      }

      inline arm_navigation_msgs::MotionPlanRequest& getMotionPlanRequest()
      {
        return motion_plan_request_;
      }

      inline void setMotionPlanRequest(arm_navigation_msgs::MotionPlanRequest& request)
      {
        motion_plan_request_ = request;
        updateStartState();
        updateGoalState();

      }

      inline void setPlanningSceneName(std::string name)
      {
        planning_scene_name_ = name;
      }

      inline std::string getPlanningSceneName()
      {
        return planning_scene_name_;
      }

      inline std::vector<std::string>& getTrajectories()
      {
        return trajectories_;
      }

      void updateCollisionMarkers(planning_environment::CollisionModels* cm_,  ros::ServiceClient& distance_state_validity_service_client_);
  };

  class TrajectoryData
  {
    protected:
      std::string ID_;
      std::string source_;
      std::string group_name_;
      std::string planning_scene_name_;
      std::string motion_plan_request_ID_;
      trajectory_msgs::JointTrajectory trajectory_;
      bool is_visible_;
      bool is_playing_;
      bool collisions_visible_;
      bool state_changed_;
      std_msgs::ColorRGBA color_;
      unsigned int current_trajectory_point_;
      unsigned int trajectory_bad_point_;
      planning_models::KinematicState* current_state_;
      ros::Duration duration_;
      bool should_refresh_colors_;
      bool has_refreshed_colors_;
      visualization_msgs::MarkerArray collision_markers_;

    public:
      int refresh_counter_;
      arm_navigation_msgs::ArmNavigationErrorCodes trajectory_error_code_;
      TrajectoryData();
      TrajectoryData(std::string ID, std::string source, std::string group_name,
                     trajectory_msgs::JointTrajectory trajectory);

      void moveThroughTrajectory(int amount);


      void updateCurrentState();

      inline bool hasStateChanged()
      {
        return state_changed_;
      }

      inline void setStateChanged(bool changed)
      {
        state_changed_ = changed;
      }

      inline visualization_msgs::MarkerArray& getCollisionMarkers()
      {
        return collision_markers_;
      }

      inline bool areCollisionsVisible()
      {
        return collisions_visible_;
      }

      inline void setCollisionsVisible(bool shown)
      {
        collisions_visible_ = shown;
      }

      inline void showCollisions()
      {
        setCollisionsVisible(true);
      }

      inline void hideCollisions()
      {
        setCollisionsVisible(false);
      }

      inline size_t getTrajectorySize()
      {
        return trajectory_.points.size();
      }

      inline bool shouldRefreshColors()
      {
        return should_refresh_colors_;
      }

      inline bool hasRefreshedColors()
      {
        return has_refreshed_colors_;
      }

      inline void setHasRefreshedColors(bool refresh)
      {
        has_refreshed_colors_ = refresh;

        if(refresh)
        {
          should_refresh_colors_ = false;
        }
      }

      inline void refreshColors()
      {
        should_refresh_colors_ = true;
        has_refreshed_colors_ = false;
        refresh_counter_ = 0;
      }


      inline void reset()
      {

        if(current_state_ != NULL)
        {
          delete current_state_;
          current_state_ = NULL;
        }

        is_playing_ = false;
        is_visible_ = false;
        current_trajectory_point_ = 0;
        trajectory_bad_point_ = 0;
        trajectory_error_code_.val = 0;
        state_changed_ = false;
      }

      inline planning_models::KinematicState* getCurrentState()
      {
        return current_state_;
      }

      inline void setCurrentState(planning_models::KinematicState* state)
      {
        current_state_ = state;
        state_changed_ = true;
      }

      inline void setMotionPlanRequestID(std::string ID)
      {
        motion_plan_request_ID_ = ID;
      }

      inline std::string getMotionPlanRequestID()
      {
        return motion_plan_request_ID_;
      }

      inline void setCurrentPoint(unsigned int point)
      {
        current_trajectory_point_ = point;
        state_changed_ = true;
      }

      inline unsigned int getCurrentPoint()
      {
        return current_trajectory_point_;
      }

      inline unsigned int getBadPoint()
      {
        return trajectory_bad_point_;
      }

      inline void setGroupname(std::string group_name)
      {
        group_name_ = group_name;
      }

      inline bool isPlaying()
      {
        return is_playing_;
      }

      inline void setPlaying(bool playing)
      {
        is_playing_ = playing;
      }

      inline void play()
      {
        is_playing_ = true;
      }

      inline void stop()
      {
        is_playing_ = false;
      }

      inline bool isVisible()
      {
        return is_visible_;
      }

      inline void setVisible(bool visible)
      {
        is_visible_ = visible;
      }

      inline void show()
      {
        setVisible(true);
      }

      inline void hide()
      {
        setVisible(false);
      }

      inline ros::Duration getDuration()
      {
        return duration_;
      }

      inline void setDuration(ros::Duration duration)
      {
        duration_ = duration;
      }

      inline std_msgs::ColorRGBA getColor()
      {
        return color_;
      }

      inline void setColor(std_msgs::ColorRGBA color)
      {
        color_ = color;
      }

      inline std::string getSource()
      {
        return source_;
      }

      inline trajectory_msgs::JointTrajectory& getTrajectory()
      {
        return trajectory_;
      }

      inline void setSource(std::string source)
      {
        source_ = source;
      }

      inline void setTrajectory(trajectory_msgs::JointTrajectory& trajectory)
      {
        trajectory_ = trajectory;
      }

      inline std::string getID()
      {
        return ID_;
      }

      inline void setID(std::string ID)
      {
        ID_ = ID;
      }

      inline std::string getGroupName()
      {
        return group_name_;
      }

      inline void setBadPoint(unsigned int point)
      {
        trajectory_bad_point_ = point;
      }


      inline void setGroupName(std::string name)
      {
        group_name_ = name;
      }

      inline void setPlanningSceneName(std::string name)
      {
        planning_scene_name_ = name;
      }

      inline std::string getPlanningSceneName()
      {
        return planning_scene_name_;
      }

      void updateCollisionMarkers(planning_environment::CollisionModels* cm_, MotionPlanRequestData& motionPlanRequest, ros::ServiceClient& distance_state_validity_service_client_);
  };

  struct PlanningSceneParameters
  {
      std::string left_ik_name_;
      std::string right_ik_name_;
      std::string non_coll_left_ik_name_;
      std::string non_coll_right_ik_name_;
      std::string left_interpolate_service_name_;
      std::string right_interpolate_service_name_;
      std::string planner_service_name_;
      std::string proximity_space_service_name_;
      std::string proximity_space_validity_name_;
      std::string set_planning_scene_diff_name_;
      std::string trajectory_filter_service_name_;
      std::string proximity_space_planner_name_;
      std::string vis_topic_name_;
      std::string right_ik_link_;
      std::string left_ik_link_;
      std::string left_redundancy_;
      std::string right_redundancy_;
      std::string right_arm_group_;
      std::string left_arm_group_;
      bool use_robot_data_;
  };

  class PlanningSceneEditor
  {
    protected:
      enum GeneratedShape
      {
        Box,
        Cylinder,
        Sphere
      };

      enum MonitorStatus
      {
        Idle,
        Executing,
        Done
      };

      struct StateRegistry
      {
          planning_models::KinematicState* state;
          std::string source;
      };
      struct SelectableObject
      {
          arm_navigation_msgs::CollisionObject collision_object_;
          visualization_msgs::InteractiveMarker selection_marker_;
          visualization_msgs::InteractiveMarker control_marker_;
          std::string ID_;
      };

      struct IKController
      {
          std::string motion_plan_ID_;
          visualization_msgs::InteractiveMarker start_controller_;
          visualization_msgs::InteractiveMarker end_controller_;
      };

      virtual void updateState() {};

      boost::recursive_mutex lock_scene_;
      arm_navigation_msgs::ArmNavigationErrorCodes last_collision_set_error_code_;
      move_arm_warehouse::MoveArmWarehouseLoggerReader* move_arm_warehouse_logger_reader_;
      planning_environment::CollisionModels* cm_;
      planning_models::KinematicState* robot_state_;
      PlanningSceneParameters params_;
      ros::NodeHandle nh_;
      ros::Publisher clock_publisher_;
      ros::Publisher joint_state_publisher_;
      ros::Publisher vis_marker_array_publisher_;
      ros::Publisher vis_marker_publisher_;
      ros::ServiceClient collision_proximity_planner_client_;
      ros::ServiceClient distance_aware_service_client_;
      ros::ServiceClient distance_state_validity_service_client_;
      ros::ServiceClient set_planning_scene_diff_client_;
      ros::ServiceClient left_ik_service_client_;
      ros::ServiceClient left_interpolate_service_client_;
      ros::ServiceClient non_coll_left_ik_service_client_;
      ros::ServiceClient non_coll_right_ik_service_client_;
      ros::ServiceClient planning_service_client_;
      ros::ServiceClient right_ik_service_client_;
      ros::ServiceClient right_interpolate_service_client_;
      ros::ServiceClient trajectory_filter_service_client_;
      std::map<std::string, double> robot_state_joint_values_;
      std::vector<ros::Time> last_creation_time_query_;
      tf::TransformBroadcaster transform_broadcaster_;
      tf::TransformListener transform_listenter_;
      planning_environment::KinematicModelStateMonitor* state_monitor_;
      std::map<std::string, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>*> arm_controller_map_;
      unsigned int max_trajectory_ID_;
      unsigned int max_request_ID_;
      unsigned int max_planning_scene_ID_;
      unsigned int max_collision_object_ID_;

      trajectory_msgs::JointTrajectory logged_trajectory_;
      ros::Time logged_trajectory_start_time_;


      bool send_collision_markers_;
      std::string collision_marker_state_;
      visualization_msgs::MarkerArray collision_markers_;
      planning_models::KinematicState* paused_collision_state_;
      std_msgs::ColorRGBA point_color_;
      std::vector<geometry_msgs::TransformStamped> robot_transforms_;

      interactive_markers::MenuHandler::FeedbackCallback collision_object_selection_feedback_ptr_;
      interactive_markers::MenuHandler::FeedbackCallback collision_object_movement_feedback_ptr_;
      interactive_markers::MenuHandler::FeedbackCallback ik_control_feedback_ptr_;
      interactive_markers::InteractiveMarkerServer* interactive_marker_server_;

      std::map<std::string, SelectableObject>* selectable_objects_;
      std::map<std::string, IKController>* ik_controllers_;

      void createSelectableMarkerFromCollisionObject(arm_navigation_msgs::CollisionObject& object, std::string name, std::string description);
      std::string current_planning_scene_ID_;
      std::string selected_motion_plan_ID_;
      std::string selected_trajectory_ID_;
      std::string logged_group_name_;
      std::string logged_motion_plan_request_;
      std::map<string,MenuEntryMap> menu_entry_maps_;
      MenuHandlerMap menu_handler_map_;

      std::map<string, ros::ServiceClient*>* collision_aware_ik_services_;
      std::map<string, ros::ServiceClient*>* non_collision_aware_ik_services_;
      std::map<string, arm_navigation_msgs::ArmNavigationErrorCodes> error_map_;
      std::vector<StateRegistry> states_;

      MonitorStatus monitor_status_;

    public:
      static geometry_msgs::Pose toGeometryPose(btTransform transform)
      {
        geometry_msgs::Pose toReturn;
        toReturn.position.x = transform.getOrigin().x();
        toReturn.position.y = transform.getOrigin().y();
        toReturn.position.z = transform.getOrigin().z();
        toReturn.orientation.x = transform.getRotation().x();
        toReturn.orientation.y = transform.getRotation().y();
        toReturn.orientation.z = transform.getRotation().z();
        toReturn.orientation.w = transform.getRotation().w();
        return toReturn;
      }

      static btTransform toBulletTransform(geometry_msgs::Pose pose)
      {
        btQuaternion quat = btQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        btVector3 vec = btVector3(pose.position.x, pose.position.y, pose.position.z);
        return btTransform(quat, vec);
      }

      std::map<std::string, PlanningSceneData>* planning_scene_map_;
      std::map<std::string, TrajectoryData>* trajectory_map_;
      std::map<std::string, MotionPlanRequestData>* motion_plan_map_;

      PlanningSceneEditor();
      PlanningSceneEditor(PlanningSceneParameters& params);
      ~PlanningSceneEditor();

      void updateJointStates();
      void sendMarkers();
      void getTrajectoryMarkers(visualization_msgs::MarkerArray& arr);
      void getMotionPlanningMarkers(visualization_msgs::MarkerArray& arr);
      void createMotionPlanRequest(planning_models::KinematicState& start_state,planning_models::KinematicState& end_state,
                                   std::string group_name, std::string end_effector_name, bool constrain,
                                   std::string planning_scene_name, std::string& motionPlan_ID_Out, bool fromRobotState = false);
      bool planToKinematicState(planning_models::KinematicState& state, std::string group_name,
                                std::string end_effector_name, bool constrain, std::string& trajectoryID_Out,
                                std::string& planning_scene_name);
      bool planToRequest(std::string requestID, std::string& trajectoryID_Out);
      bool planToRequest(MotionPlanRequestData& data, std::string& trajectoryID_Out);
      void determinePitchRollConstraintsGivenState(const planning_models::KinematicState& state,
                                                   std::string& end_effector_link,
                                                   arm_navigation_msgs::OrientationConstraint& goal_constraint,
                                                   arm_navigation_msgs::OrientationConstraint& path_constraint);
      void printTrajectoryPoint(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values);

      bool getPlanningSceneOutcomes(const ros::Time& time, std::vector<std::string>& pipeline_stages,
                                    std::vector<arm_navigation_msgs::ArmNavigationErrorCodes>& error_codes,
                                    std::map<std::string,arm_navigation_msgs::ArmNavigationErrorCodes>& error_map);
      bool getAllAvailableWarehousePlanningScenes(std::vector<ros::Time>& planning_scene_times);
      bool loadPlanningScene(const ros::Time& time, std::string& ID);
      bool getAllAssociatedMotionPlanRequests(const ros::Time& time,std::vector<std::string>& IDs,
                                              std::vector<std::string>& stages,
                                              std::vector<arm_navigation_msgs::MotionPlanRequest>& requests);
      void createMotionPlanRequestData(std::string planning_scene_ID,std::vector<std::string>& IDs,
                                       std::vector<std::string>& stages,
                                       std::vector<arm_navigation_msgs::MotionPlanRequest>& requests);
      bool sendPlanningScene(PlanningSceneData& data);
      void deleteKinematicStates();
      std::string createNewPlanningScene();
      bool getMotionPlanRequest(const ros::Time& time, const std::string& stage,
                                arm_navigation_msgs::MotionPlanRequest& mpr, std::string& ID,
                                std::string& planning_scene_ID);

      bool getAllAssociatedTrajectories(const ros::Time& time, std::vector<std::string>& trajectory_sources);
      bool getAllAssociatedPausedStates(const ros::Time& time, std::vector<ros::Time>& paused_times);
      bool getPausedState(const ros::Time& time, const ros::Time& paused_time,
                          head_monitor_msgs::HeadMonitorFeedback& paused_state);

      bool loadAndPlayTrajectory(MotionPlanRequestData& requestData,const ros::Time& time, const std::string& source_name,
                                 unsigned int num, std::string& planning_scene_name);
      bool playTrajectory(MotionPlanRequestData& requestData,TrajectoryData& data);

      void setShowCollisions(bool send_collision_markers, const std::string& csd, std::string& arm_group_name, planning_scene_utils::MotionPlanRequestData& data);
      void updateCurrentCollisionSet(const std::string& csd, std::string& arm_group_name, planning_scene_utils::MotionPlanRequestData& data);
      bool getSendCollisionMarkers() const;
      bool filterTrajectory(MotionPlanRequestData& requestData, TrajectoryData& trajectory, std::string& filter_ID);
      void loadAllWarehouseData();
      void executeTrajectory(TrajectoryData& data);
      inline void executeTrajectory(std::string trajectory_ID)
      {
        if(trajectory_map_->find(trajectory_ID) != trajectory_map_->end())
        {
          executeTrajectory((*trajectory_map_)[trajectory_ID]);
        }
      }
      void getAllRobotStampedTransforms(const planning_models::KinematicState& state,
                                                                 vector<geometry_msgs::TransformStamped>& trans_vector,
                                                                 const ros::Time& stamp)
      {
        trans_vector.clear();
        const map<string, geometry_msgs::TransformStamped>& transforms = cm_->getSceneTransformMap();
        geometry_msgs::TransformStamped transvec;
        for(map<string, geometry_msgs::TransformStamped>::const_iterator it = transforms.begin(); it != transforms.end(); it++)
        {
          trans_vector.push_back(it->second);
        }
        for(unsigned int i = 0; i < state.getLinkStateVector().size(); i++)
        {
          const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[i];
          geometry_msgs::TransformStamped ts;
          ts.header.stamp = stamp;
          ts.header.frame_id = cm_->getWorldFrameId();
          ts.child_frame_id = ls->getName();
          tf::transformTFToMsg(ls->getGlobalLinkTransform(), ts.transform);
          trans_vector.push_back(ts);
        }
      }

      void sendTransformsAndClock()
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
      void savePlanningScene(PlanningSceneData& data);

      void collisionObjectSelectionCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
      void collisionObjectMovementCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
      void IKControllerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

      void createIkControllersFromMotionPlanRequest(MotionPlanRequestData& data);
      void createIKController(MotionPlanRequestData& data, PositionType type);
      void createCollisionObject(geometry_msgs::Pose pose, GeneratedShape shape, float scaleX, float scaleY, float scaleZ);


      bool solveIKForEndEffectorPose(MotionPlanRequestData& mpr, PositionType type, bool coll_aware = true,
                                     bool constrain_pitch_and_roll = false, double change_redundancy = 0.0);


      void randomlyPerturb(MotionPlanRequestData& mpr, PositionType type);

      void jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state);
     interactive_markers::MenuHandler::EntryHandle registerSubMenuEntry(std::string menu, std::string name,
                                                                             std::string subMenu, interactive_markers::MenuHandler::FeedbackCallback& callback)
     {

       interactive_markers::MenuHandler::EntryHandle toReturn = menu_handler_map_[menu].insert(menu_entry_maps_[menu][subMenu], name, callback);
       menu_entry_maps_[menu][name] = toReturn;
       return toReturn;
     }


     interactive_markers::MenuHandler::EntryHandle registerMenuEntry(std::string menu, std::string entryName,
                                                                     interactive_markers::MenuHandler::FeedbackCallback callback)
     {
       interactive_markers::MenuHandler::EntryHandle toReturn = menu_handler_map_[menu].insert(entryName, callback);
       menu_entry_maps_[menu][entryName] = toReturn;
       return toReturn;
     }


      void setIKControlsVisible(std::string ID,PositionType type, bool visible);

      void setCurrentPlanningScene(std::string ID, bool loadRequests = true, bool loadTrajectories = true);

      inline void deleteTrajectory(std::string ID)
      {
        if(trajectory_map_->find(ID) != trajectory_map_->end())
        {

          if(current_planning_scene_ID_ != "")
          {
            PlanningSceneData& data = (*planning_scene_map_)[current_planning_scene_ID_];
            MotionPlanRequestData& requestData = (*motion_plan_map_)[(*trajectory_map_)[ID].getMotionPlanRequestID()];

            std::vector<std::string>::iterator erasure = data.getTrajectories().end();
            for(std::vector<std::string>::iterator it = data.getTrajectories().begin(); it
                != data.getTrajectories().end(); it++)
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
                ROS_INFO("Found %s at position %d", ID.c_str(), i);
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
      }


      inline void deleteMotionPlanRequest(std::string ID)
      {
        if(motion_plan_map_->find(ID) != motion_plan_map_->end())
        {
          for(size_t i = 0; i < states_.size(); i++)
          {
            if(states_[i].state == (*motion_plan_map_)[ID].getStartState() ||states_[i].state == (*motion_plan_map_)[ID].getGoalState())
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
      }

      inline std::string generateNewTrajectoryID()
      {
        std::stringstream stream;
        stream << "Trajectory ";
        max_trajectory_ID_++;
        stream << max_trajectory_ID_;
        return stream.str();
      }

      inline std::string generateNewCollisionObjectID()
      {
        std::stringstream stream;
        stream << "collision_object_";
        max_collision_object_ID_++;
        stream << max_collision_object_ID_;
        return stream.str();
      }

      inline std::string generateNewMotionPlanID()
      {
        std::stringstream stream;
        stream << "MPR ";
        max_request_ID_++;
        stream << max_request_ID_;
        return stream.str();
      }

      inline std::string generateNewPlanningSceneID()
      {
        std::stringstream stream;
        stream << "Planning Scene ";
        max_planning_scene_ID_++;
        stream << max_planning_scene_ID_;
        return stream.str();
      }

      inline void lockScene()
      {
        lock_scene_.lock();
      }

      inline void unlockScene()
      {
        lock_scene_.unlock();
      }

      inline planning_environment::CollisionModels* getCollisionModel()
      {
        return cm_;
      }

      inline void setCollisionModel(planning_environment::CollisionModels* model, bool shouldDelete = false)
      {
        if(shouldDelete && cm_ != NULL)
        {
          delete cm_;
          cm_ = model;
        }
        cm_ = model;
      }

      inline planning_models::KinematicState* getRobotState()
      {
        return robot_state_;
      }

      inline void setRobotState(planning_models::KinematicState* robot_state, bool shouldDelete = true)
      {
        if(shouldDelete && robot_state_ != NULL)
        {
          delete robot_state_;
          robot_state_ = NULL;
        }

        robot_state_ = robot_state;
      }

      inline move_arm_warehouse::MoveArmWarehouseLoggerReader* getLoggerReader()
      {
        return move_arm_warehouse_logger_reader_;
      }

      void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                  const control_msgs::FollowJointTrajectoryResultConstPtr& result);

      inline void setLoggerReader(move_arm_warehouse::MoveArmWarehouseLoggerReader* loggerReader,
                                  bool shouldDelete = true)
      {
        if(move_arm_warehouse_logger_reader_ != NULL)
        {
          delete move_arm_warehouse_logger_reader_;
          move_arm_warehouse_logger_reader_ = NULL;
        }

        move_arm_warehouse_logger_reader_ = loggerReader;
      }
  };

}
#endif
