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
 *     * Neither the name of the <ORGANIZATION> nor the names of its
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

// Author: Matthew Klingensmith, E. Gil Jones

#ifndef PLANNING_SCENE_WAREHOUSE_H
#define PLANNING_SCENE_WAREHOUSE_H

#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <string>
#include <map>

#include <ros/ros.h>

#include <move_arm_warehouse/move_arm_utils.h>
#include <move_arm_warehouse/trajectory_stats.h>

#include <qt4/QtGui/qwidget.h>
#include <qt4/QtGui/qmenubar.h>
#include <qt4/QtGui/qmenu.h>
#include <qt4/QtGui/qprogressbar.h>
#include <qt4/QtGui/qdialog.h>
#include <qt4/QtGui/qtablewidget.h>
#include <qt4/QtGui/qlayout.h>
#include <qt4/QtGui/qpushbutton.h>
#include <qt4/Qt/qthread.h>
#include <qt4/QtGui/qslider.h>
#include <qt4/QtGui/qspinbox.h>
#include <qt4/QtGui/qlabel.h>
#include <qt4/QtGui/qgroupbox.h>
#include <qt4/QtGui/qcombobox.h>
#include <qt4/QtGui/qcheckbox.h>
#include <qt4/QtGui/qcolordialog.h>
#include <qt4/QtGui/qtreewidget.h>
#include <qt4/QtGui/qpalette.h>
#include <qt4/QtGui/qformlayout.h>
#include <qt4/QtGui/qlineedit.h>
#include <qt4/QtGui/qfiledialog.h>
#include <qt4/QtGui/qmessagebox.h>
#include <qt4/QtGui/qmainwindow.h>
#include <qt4/QtGui/qlistwidget.h>
#include <QDateTime>

static const std::string VIS_TOPIC_NAME = "planning_scene_visualizer_markers";

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 5;
static const std::string EXECUTE_RIGHT_TRAJECTORY = "/r_arm_controller/follow_joint_trajectory";
static const std::string EXECUTE_LEFT_TRAJECTORY = "/l_arm_controller/follow_joint_trajectory";
static const std::string LEFT_IK_NAME = "/pr2_left_arm_kinematics/get_constraint_aware_ik";
static const std::string RIGHT_IK_NAME = "/pr2_right_arm_kinematics/get_constraint_aware_ik";
static const std::string NON_COLL_LEFT_IK_NAME = "/pr2_left_arm_kinematics/get_ik";
static const std::string NON_COLL_RIGHT_IK_NAME = "/pr2_right_arm_kinematics/get_ik";
static const std::string RIGHT_ARM_GROUP = "right_arm";
static const std::string LEFT_ARM_GROUP = "left_arm";
static const std::string RIGHT_ARM_REDUNDANCY = "r_upper_arm_roll_joint";
static const std::string LEFT_ARM_REDUNDANCY = "l_upper_arm_roll_joint";
static const std::string LEFT_IK_LINK = "l_wrist_roll_link";
static const std::string RIGHT_IK_LINK = "r_wrist_roll_link";
static const std::string PLANNER_SERVICE_NAME = "/ompl_planning/plan_kinematic_path";
static const std::string LEFT_INTERPOLATE_SERVICE_NAME = "/l_interpolated_ik_motion_plan";
static const std::string RIGHT_INTERPOLATE_SERVICE_NAME = "/r_interpolated_ik_motion_plan";
static const std::string TRAJECTORY_FILTER_SERVICE_NAME = "/trajectory_filter/filter_trajectory_with_constraints";
static const std::string PROXIMITY_SPACE_SERVICE_NAME = "/collision_proximity_server_test/get_distance_aware_plan";
static const std::string PROXIMITY_SPACE_VALIDITY_NAME = "/collision_proximity_server_test/get_state_validity";
static const std::string PROXIMITY_SPACE_PLANNER_NAME = "/collision_proximity_planner/plan";
static const std::string LIST_CONTROLLERS_SERVICE = "/pr2_controller_manager/list_controllers";
static const std::string LOAD_CONTROLLERS_SERVICE = "/pr2_controller_manager/load_controller";
static const std::string UNLOAD_CONTROLLERS_SERVICE = "/pr2_controller_manager/unload_controller";
static const std::string SWITCH_CONTROLLERS_SERVICE = "/pr2_controller_manager/switch_controller";
static const std::string GAZEBO_ROBOT_MODEL = "pr2";
static const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const ros::Duration PLANNING_DURATION = ros::Duration(5.0);
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "environment_server/set_planning_scene_diff";

/////
/// @brief Class for configuring PlanningSceneParameter struct in GUI.
/////
class ParameterDialog: public QDialog
{
public:
  planning_scene_utils::PlanningSceneParameters params_;
  ParameterDialog(planning_scene_utils::PlanningSceneParameters params, QWidget* parent = NULL) :
    QDialog(parent)
  {
    params_ = params;
    setMinimumWidth(640);
    setup();
  }

  /////
  /// @brief Creates all the Qt Widgets
  /////
  void setup()
  {
    QGroupBox* groupBox = new QGroupBox(this);
    groupBox->setTitle("Planning Scene Editor Parameters");
    QVBoxLayout* boxLayout = new QVBoxLayout(this);

    layout = new QFormLayout(groupBox);
    left_ik_name_ = new QLineEdit(groupBox);
    layout->addRow("Left IK Service", left_ik_name_);
    left_ik_name_->setText(QString::fromStdString(params_.left_ik_name_));

    right_ik_name_ = new QLineEdit(groupBox);
    layout->addRow("Right IK Service", right_ik_name_);
    right_ik_name_->setText(QString::fromStdString(params_.right_ik_name_));

    non_coll_left_ik_name_ = new QLineEdit(groupBox);
    layout->addRow("Non Collision-Aware Left IK Service", non_coll_left_ik_name_);
    non_coll_left_ik_name_->setText(QString::fromStdString(params_.non_coll_left_ik_name_));

    non_coll_right_ik_name_ = new QLineEdit(groupBox);
    layout->addRow("Non Collision-Aware Right IK Service", non_coll_right_ik_name_);
    non_coll_right_ik_name_->setText(QString::fromStdString(params_.non_coll_right_ik_name_));

    right_arm_group_ = new QLineEdit(groupBox);
    layout->addRow("Right Arm Group", right_arm_group_);
    right_arm_group_->setText(QString::fromStdString(params_.right_arm_group_));

    left_arm_group_ = new QLineEdit(groupBox);
    layout->addRow("Left Arm Group", left_arm_group_);
    left_arm_group_->setText(QString::fromStdString(params_.left_arm_group_));

    right_arm_redundancy_ = new QLineEdit(groupBox);
    layout->addRow("Right Arm Redundancy DOF", right_arm_redundancy_);
    right_arm_redundancy_->setText(QString::fromStdString(params_.right_redundancy_));

    left_arm_redundancy_ = new QLineEdit(groupBox);
    layout->addRow("Left Arm Redundancy DOF", left_arm_redundancy_);
    left_arm_redundancy_->setText(QString::fromStdString(params_.left_redundancy_));

    left_ik_link_ = new QLineEdit(groupBox);
    layout->addRow("Left IK Link", left_ik_link_);
    left_ik_link_ ->setText(QString::fromStdString(params_.left_ik_link_));

    right_ik_link_ = new QLineEdit(groupBox);
    layout->addRow("Right IK Link", right_ik_link_);
    right_ik_link_->setText(QString::fromStdString(params_.right_ik_link_));

    planner_service_name_ = new QLineEdit(groupBox);
    layout->addRow("Planner Service", planner_service_name_);
    planner_service_name_->setText(QString::fromStdString(params_.planner_service_name_));

    left_interpolate_service_name_ = new QLineEdit(groupBox);
    layout->addRow("Left Interpolation Service", left_interpolate_service_name_);
    left_interpolate_service_name_->setText(QString::fromStdString(params_.left_interpolate_service_name_));

    right_interpolate_service_name_ = new QLineEdit(groupBox);
    layout->addRow("Right Interpolation Service", right_interpolate_service_name_);
    right_interpolate_service_name_->setText(QString::fromStdString(params_.right_interpolate_service_name_));

    trajectory_filter_service_name_ = new QLineEdit(groupBox);
    layout->addRow("Trajectory Filter Service", trajectory_filter_service_name_);
    trajectory_filter_service_name_ ->setText(QString::fromStdString(params_.trajectory_filter_service_name_));

    proximity_space_service_name_ = new QLineEdit(groupBox);
    layout->addRow("Proximity Space Service", proximity_space_service_name_);
    proximity_space_service_name_ ->setText(QString::fromStdString(params_.proximity_space_service_name_));

    proximity_space_validity_name_ = new QLineEdit(groupBox);
    layout->addRow("Proximity Space Validity Service", proximity_space_validity_name_);
    proximity_space_validity_name_ ->setText(QString::fromStdString(params_.proximity_space_validity_name_));

    proximity_space_planner_name_ = new QLineEdit(groupBox);
    layout->addRow("Proximity Space Planner", proximity_space_planner_name_);
    proximity_space_planner_name_ ->setText(QString::fromStdString(params_.proximity_space_planner_name_));

    execute_left_trajectory_ = new QLineEdit(groupBox);
    layout->addRow("Execute Left Trajectory", execute_left_trajectory_);
    execute_left_trajectory_ ->setText(QString::fromStdString(params_.execute_left_trajectory_));

    execute_right_trajectory_ = new QLineEdit(groupBox);
    layout->addRow("Execute Right Trajectory", execute_right_trajectory_);
    execute_right_trajectory_ ->setText(QString::fromStdString(params_.execute_right_trajectory_));

    use_robot_data_ = new QCheckBox(groupBox);
    use_robot_data_->setText("Use Robot Data");
    use_robot_data_->setChecked(params_.use_robot_data_);
    layout->addRow("Use Data From Simulated/Real Robot?", use_robot_data_);
    use_robot_data_->setToolTip("When this is checked, a robot state publisher will not be created. Instead, TF data\n\
                                  will be taken from the robot's actual state.");
    sync_with_gazebo_ = new QCheckBox(groupBox);
    sync_with_gazebo_->setText("Sync With Gazebo");
    sync_with_gazebo_->setChecked(false);
    sync_with_gazebo_->setToolTip("When this is checked, the warehouse viewer will attempt to set the robot state\n\
                                     to the start state of any motion plan requests played.");
    layout->addRow("Synchronize Robot State With Gazebo?", sync_with_gazebo_);

    QPushButton* button = new QPushButton(groupBox);
    button->setText("Accept");
    button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    connect(button, SIGNAL(clicked()), this, SLOT(accept()));

    boxLayout->addWidget(groupBox);
    boxLayout->addWidget(button);
    groupBox->setLayout(layout);
    setLayout(boxLayout);

  }

  /////
  /// @brief Sets the params_ struct to use data from the Qt widgets.
  /////
  void updateParams()
  {
    params_.left_ik_name_ = left_ik_name_->text().toStdString();
    params_.right_ik_name_ = right_ik_name_->text().toStdString();
    params_.non_coll_left_ik_name_ = non_coll_left_ik_name_ ->text().toStdString();
    params_.non_coll_right_ik_name_ = non_coll_right_ik_name_ ->text().toStdString();
    params_.right_arm_group_ = right_arm_group_->text().toStdString();
    params_.left_arm_group_ = left_arm_group_ ->text().toStdString();
    params_.right_redundancy_ = right_arm_redundancy_->text().toStdString();
    params_.left_redundancy_ = left_arm_redundancy_->text().toStdString();
    params_.left_ik_link_ = left_ik_link_->text().toStdString();
    params_.right_ik_link_ = right_ik_link_->text().toStdString();
    params_.planner_service_name_ = planner_service_name_->text().toStdString();
    params_.left_interpolate_service_name_ = left_interpolate_service_name_->text().toStdString();
    params_.right_interpolate_service_name_ = right_interpolate_service_name_->text().toStdString();
    params_.trajectory_filter_service_name_ = trajectory_filter_service_name_->text().toStdString();
    params_.proximity_space_service_name_ = proximity_space_service_name_->text().toStdString();
    params_.proximity_space_validity_name_ = proximity_space_validity_name_->text().toStdString();
    params_.proximity_space_planner_name_ = proximity_space_planner_name_->text().toStdString();
    params_.execute_left_trajectory_ = execute_left_trajectory_->text().toStdString();
    params_.execute_right_trajectory_ = execute_right_trajectory_->text().toStdString();
    params_.use_robot_data_ = use_robot_data_->isChecked();
    params_.sync_robot_state_with_gazebo_ = sync_with_gazebo_->isChecked();
  }

private:
  QFormLayout* layout;
  QLineEdit* left_ik_name_;
  QLineEdit* right_ik_name_;
  QLineEdit* non_coll_left_ik_name_;
  QLineEdit* non_coll_right_ik_name_;
  QLineEdit* right_arm_group_;
  QLineEdit* left_arm_group_;
  QLineEdit* right_arm_redundancy_;
  QLineEdit* left_arm_redundancy_;
  QLineEdit* left_ik_link_;
  QLineEdit* right_ik_link_;
  QLineEdit* planner_service_name_;
  QLineEdit* left_interpolate_service_name_;
  QLineEdit* right_interpolate_service_name_;
  QLineEdit* trajectory_filter_service_name_;
  QLineEdit* proximity_space_service_name_;
  QLineEdit* proximity_space_validity_name_;
  QLineEdit* proximity_space_planner_name_;
  QLineEdit* execute_left_trajectory_;
  QLineEdit* execute_right_trajectory_;
  QCheckBox* use_robot_data_;
  QCheckBox* sync_with_gazebo_;

};

class PlanningSceneNameTableItem: public QObject, public QTableWidgetItem
{
  Q_OBJECT
  public:

  PlanningSceneNameTableItem(const QString& qs) : QTableWidgetItem(qs) {}
  
  bool operator< (const QTableWidgetItem& other) const {
    unsigned int other_id = planning_scene_utils::getPlanningSceneIdFromName(other.text().toStdString());
    unsigned int my_id = planning_scene_utils::getPlanningSceneIdFromName(this->text().toStdString());
    if(my_id < other_id) {
      return true;
    } else {
      return false;
    }
  }
};

class PlanningSceneDateTableItem: public QObject, public QTableWidgetItem
{
  Q_OBJECT
  public:

  PlanningSceneDateTableItem(const QString& qs) : QTableWidgetItem(qs) {}
  
  bool operator< (const QTableWidgetItem& other) const {
    QDateTime other_time = QDateTime::fromString(other.text());
    QDateTime my_time = QDateTime::fromString(this->text());
    if(my_time < other_time) {
      return true;
    } else {
      return false;
    }
  }
};

/////
/// @brief Main Warehouse Viewer application.
/////
class WarehouseViewer: public QMainWindow, public planning_scene_utils::PlanningSceneEditor
{
  Q_OBJECT
  public:
  /// @brief flag for causing the spin thread and marker thread to stop.
  bool quit_threads_;

  /////
  /// @brief Thread that loads all warehouse data and creates a table from it.
  /////
  class TableLoadThread: public QThread
  {
  public:
    WarehouseViewer* visualizer_;
    TableLoadThread(WarehouseViewer* visualizer) :
      QThread(visualizer), visualizer_(visualizer)
    {
    }

    void run()
    {
      visualizer_->createPlanningSceneTable();
    }
  };

  WarehouseViewer(QWidget* parent, planning_scene_utils::PlanningSceneParameters& params);
  ~WarehouseViewer();

  /// @brief Creates all of the Qt objects associated with the viewer
  void initQtWidgets();
  /// @brief Creates the load planning scenes dialog.
  void setupPlanningSceneDialog();
  /// @brief Creates the table in the planning scenes dialog.
  void createPlanningSceneTable();
  /// @brief Creates the trajectory tree
  void createTrajectoryTable();
  /// @brief Creates the motion plan request tree
  void createMotionPlanTable();
  /// @brief Creates the "new primitive collision object" dialog
  void createNewObjectDialog();
  /// @brief Creates the "new mesh collision object" dialog
  void createNewMeshDialog();
  /// @brief Creates the "new motion plan request" dialog
  void createRequestDialog();

  void createSetPathConstraintsDialog(planning_scene_utils::MotionPlanRequestData& data);

  /// @brief resets the trajectory and motion plan tables.
  void updateState();
  /// @brief callback that occurs when a planning scene is loaded from the warehouse
  void onPlanningSceneLoaded(int scene, int numScenes);
  void createOutcomeDialog();
  void createAlterLinkPaddingDialog();
  void createAlterAllowedCollisionDialog();
  void createAttachObjectDialog(const std::string& name);
  bool createNewPlanningSceneConfirm();  
  void createRobotStateEditor();

  /// @brief Creates the "Motion Plan Request" box
  QGroupBox *createMotionPlanBox();
  /// @brief Creates the "Trajectory" box
  QGroupBox *createTrajectoryBox();
  /// @brief Creates the "Controls" box
  QGroupBox *createTrajectoryControlsBox();
  /// @brief Creates the "Trajectory Info" box
  QGroupBox *createTrajectoryInfoBox();

  void saveCurrentPlanningScene(bool copy);

  virtual void planCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode);
  virtual void filterCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode);

  virtual void attachObjectCallback(const std::string& name) {
    emit attachObjectSignal(name);
  }

  /// @brief overridden to update trajectory_slider_
  virtual void selectedTrajectoryCurrentPointChanged( unsigned int new_current_point );

  void setEnabledDisabledDisplay(const QString& qs1, const QString& qs2);
  void getEntryList(const std::string& s1, 
                    std::vector<std::string>& sv1);
  void addSpecialEntries(QListWidget* list_widget);

signals:
  /// @brief Changes the progress bar of the load planning scene dialog.
  void changeProgress(int progress);
  /// @brief Updates the trajectory and motion plan tables.
  void updateTables();
  
  void plannerFailure(int value);
  void filterFailure(int value);
                               
  void attachObjectSignal(const std::string& name);

  void allScenesLoaded();

  void selectedTrajectoryPointChanged( unsigned int new_point );

public slots:

  void refreshPlanningSceneDialog();

  void popupPlannerFailure(int value);
  void popupFilterFailure(int value);

  /// @brief closes the window and deconstructs.
  void quit();
  /// @brief Opens the planning scene load dialog.
  void popupLoadPlanningScenes();
  /// @brief Changes the loading bar value to that given, out of 100
  void progressChanged(int progress)
  {
    load_scene_progress_->setValue(progress);
  }
  void planningSceneTableHeaderClicked(int col);
  void newButtonPressed();
  /// @brief Called when the load planning scene button is pressed
  void loadButtonPressed();
  /// @brief Called when the refresh planning scene action is triggered
  void refreshButtonPressed();
  void removePlanningSceneButtonPressed();
  /// @brief Called when the user clicks on an item in the trajectory tree
  void trajectoryTableSelection();
  /// @brief Called when the user clicks on an item in the motion plan tree
  void motionPlanTableSelection();
  /// @brief Called when the user plays a trajectory.
  void playButtonPressed();
  /// @brief Called when the user presses the "filter trajectory" button.
  void filterButtonPressed();
  /// @brief Called when the trajectory slider is changed.
  void trajectorySliderChanged(int nv);
  /// @brief Called when the user presses the "Plan New Trajectory" button.
  void replanButtonPressed();
  /// @brief Called when the user changes the box displaying the current trajectory point.
  void trajectoryEditChanged();
  /// @brief Called when the "Create New Planning Scene ..." action is triggered.
  void createNewPlanningSceneSlot();
  /// @brief Called when the "Save Current Planning Scene ..." action is triggered.
  void savePlanningSceneSlot();
  /// @brief Called when the "Save Current Planning Scene ..." action is triggered.
  void copyPlanningSceneSlot();
  /// @brief Creates a new motion plan request for the given group and end effector link.
  void createNewMotionPlanRequest(std::string group_name, std::string end_effector_name);
  /// @brief Called when the start position color button is pressed for a particular motion plan request.
  void motionPlanStartColorButtonClicked();
  /// @brief Called when the end position color button is pressed for a particular motion plan request.
  void motionPlanEndColorButtonClicked();
  /// @brief Called when the start position visible check box is pressed.
  void motionPlanStartVisibleButtonClicked(bool checked);
  /// @brief Called when the end position visible check box is pressed.
  void motionPlanEndVisibleButtonClicked(bool checked);
  /// @brief Called when the change color button for a particular trajectory is pressed.
  void trajectoryColorButtonClicked();
  /// @brief Called when the visible check box for a particular trajectory is clicked.
  void trajectoryVisibleButtonClicked(bool checked);
  /// @brief Called when the "Create New Motion Plan Request ..." action is triggered.
  void createNewMotionPlanPressed();
  /// @brief Called when the "Create New Collision Object ..." action is triggered.
  void createNewObjectPressed();
  /// @brief Called when the "Create..." button in the collision object dialog is pressed.
  void createObjectConfirmedPressed();
  /// @brief Called when the "Create..." button in the collision object dialog is pressed.
  void createMeshConfirmedPressed();
  /// @brief Called when the "Create New Mesh Collision Object ..." action is triggered.
  void createNewMeshPressed();
  /// @brief Called when the "Create..." button in the motion plan dialog is pressed.
  void createRequestPressed();
  /// @brief Called when the user checks the "collisions visible" check box for a particular motion plan request.
  void motionPlanCollisionVisibleButtonClicked(bool checked);
  /// @brief Called when the user checks the "collisions visible" check box for a particular trajectory.
  void trajectoryCollisionsVisibleButtonClicked(bool checked);
  /// @brief Called when the user checks the "joint controls active" check box for a particular motion plan request.
  void motionPlanJointControlsActiveButtonClicked(bool checked);
  /// @brief Sets the currently selected motion plan request to the given ID.
  void selectMotionPlan(std::string ID);
  /// @brief Sets the currently selected trajectory to the given ID.
  void selectTrajectory(std::string ID);
  /// @brief Removes the selected motion plan request and its associated trajectories.
  void deleteSelectedMotionPlan();
  /// @brief Removes the selected trajectory and sets the selected trajectory to ""
  void deleteSelectedTrajectory();
  /// @brief Callback when a selected trajectory or motion plan request is changed.
  void updateStateTriggered();
  /// @brief Called when the user presses the "Execute Trajectory" button.
  void executeButtonPressed();
  /// @brief Called when the user triggers the "Refresh planning scene..." action.
  void refreshSceneButtonPressed();
  /// @brief Called when the user triggers the "View planning scene outcomes ..." action.
  void viewOutcomesPressed();
  /// @brief Called when the user triggers the "Alter link padding ..." action.
  void alterLinkPaddingPressed();
  /// @brief Called when the user triggers the "Alter allowed collision ..." action.
  void alterAllowedCollisionPressed();
  /// @brief Called when the user changes the model render type.
  void modelRenderTypeChanged(const QString& type);
  /// @brief Called when the user changes the trajectory render type.
  void trajectoryRenderTypeChanged(const int& type);
  /// @brief Called when the user changes the render type of a motion plan request.
  void motionPlanRenderTypeChanged(const QString& type);
  /// @brief Called when the user presses the change color button of a collision object.
  void objectColorButtonPressed();
  void meshColorButtonPressed();

  /// @brief Called when the user changes a link padding
  void alteredLinkPaddingValueChanged(double d);

  void firstEntityListSelected();
  void secondEntityListSelected();
  void entityListsEdited();
  void disableCollisionClicked();
  void enableCollisionClicked();
  void resetAllowedCollisionClicked();

  void meshFileSelected(const QString& s);

  void attachObject(const std::string& name);
  void addTouchLinkClicked();
  void removeTouchLinkClicked();

  void editRobotStatePressed();
  void editJointBoxChanged(const QString& joint);
  void jointStateSliderChanged(int nv);

  void motionPlanHasPathConstraintsButtonClicked(bool checked);
  void setPathConstraintsButtonClicked();

  void primaryPlannerTriggered();
  void secondaryPlannerTriggered();

  void onSelectedTrajectoryPointChanged( unsigned int new_point );

protected:
  
  bool planning_scene_initialized_;
  QLabel* selected_trajectory_label_;
  QLabel* selected_trajectory_error_label_;
  QLabel* selected_request_label_;
  QLabel* selected_trajectory_duration_title_;
  QLabel* selected_trajectory_duration_label_;
  QLabel* selected_trajectory_stat_1_title_;
  QLabel* selected_trajectory_stat_1_label_;
  QLabel* selected_trajectory_stat_2_title_;
  QLabel* selected_trajectory_stat_2_label_;
  QLabel* selected_trajectory_stat_3_title_;
  QLabel* selected_trajectory_stat_3_label_;

  QMenuBar* menu_bar_;
  QMenu* file_menu_;
  QMenu* planning_scene_menu_;
  QMenu* collision_object_menu_;
  QAction* new_object_action_;
  QAction* new_mesh_action_;
  QAction* refresh_action_;
  QAction* view_outcomes_action_;
  
  QMenu* planner_configuration_menu_;
  QAction* set_primary_planner_action_;
  QAction* set_secondary_planner_action_;
  
  QAction* alter_link_padding_action_;
  QDialog* alter_link_padding_dialog_;
  QTableWidget* alter_link_padding_table_;

  QAction* alter_allowed_collision_action_;
  QDialog* alter_allowed_collision_dialog_;
  QLineEdit* first_allowed_collision_line_edit_;
  QLineEdit* second_allowed_collision_line_edit_;
  QLineEdit* allowed_status_line_edit_;
  QListWidget* first_allowed_collision_list_;
  QListWidget* second_allowed_collision_list_;

  QDialog* attach_object_dialog_;
  QComboBox* attach_link_box_;
  QListWidget* possible_touch_links_;
  QListWidget* added_touch_links_;

  QAction* edit_robot_state_action_;
  QDialog* edit_robot_state_dialog_;
  QComboBox* edit_joint_box_;
  QSlider* joint_state_slider_;
  QLineEdit* lower_bound_edit_window_;
  QLineEdit* upper_bound_edit_window_;
  QLineEdit* current_value_window_;

  QDialog* set_path_constraints_dialog_;
  QCheckBox* constrain_roll_check_box_;
  QCheckBox* constrain_pitch_check_box_;
  QCheckBox* constrain_yaw_check_box_;
  QDoubleSpinBox* constrain_roll_tolerance_;
  QDoubleSpinBox* constrain_pitch_tolerance_;
  QDoubleSpinBox* constrain_yaw_tolerance_;

  QDialog* load_planning_scene_dialog_;
  QDialog* new_object_dialog_;
  QDialog* new_mesh_dialog_;
  QDialog* new_request_dialog_;
  QDialog* outcome_dialog_;
  QTableWidget* stage_outcome_table_;
  QTableWidget* trajectory_outcome_table_;
  QProgressBar* load_scene_progress_;
  QAction* new_planning_scene_action_;
  QAction* new_motion_plan_action_;
  QAction* load_planning_scene_action_;
  QAction* save_planning_scene_action_;
  QAction* copy_planning_scene_action_;
  QAction* quit_action_;
  QTableWidget* planning_scene_table_;
  QTreeWidget* motion_plan_tree_;
  QTreeWidget* trajectory_tree_;
  QSlider* trajectory_slider_;
  QPushButton* play_button_;
  QPushButton* filter_button_;
  QPushButton* replan_button_;
  QPushButton* execute_button_;

  QPushButton* new_planning_scene_button_;
  QPushButton* load_planning_scene_button_;
  QPushButton* refresh_planning_scene_button_;
  QPushButton* remove_planning_scene_button_;
  QPushButton* object_color_button_;
  QComboBox* collision_display_box_;

  QSpinBox* trajectory_point_edit_;

  TableLoadThread* table_load_thread_;

  QComboBox* collision_object_type_box_;
  QLineEdit* collision_object_name_;
  QComboBox* request_group_name_box_;
  QSpinBox* collision_object_scale_x_box_;
  QSpinBox* collision_object_scale_y_box_;
  QSpinBox* collision_object_scale_z_box_;
  QSpinBox* collision_object_pos_x_box_;
  QSpinBox* collision_object_pos_y_box_;
  QSpinBox* collision_object_pos_z_box_;
  QPushButton* make_object_button_;

  QLineEdit* mesh_object_name_;
  QSpinBox* mesh_object_pos_x_box_;
  QSpinBox* mesh_object_pos_y_box_;
  QSpinBox* mesh_object_pos_z_box_;
  QDoubleSpinBox* mesh_object_scale_x_box_;
  QDoubleSpinBox* mesh_object_scale_y_box_;
  QDoubleSpinBox* mesh_object_scale_z_box_;
  QPushButton* mesh_color_button_;
  QPushButton* make_mesh_button_;

  QCheckBox* load_motion_plan_requests_box_;
  QCheckBox* load_trajectories_box_;
  QCheckBox* create_request_from_robot_box_;

  QFileDialog* file_selector_;
  QLineEdit* mesh_filename_field_;

  QFormLayout* trajectory_info_box_layout_;

  /// @brief Find the currently-selected trajectory in the
  /// trajectory_tree_ and set its visibility checkbox to checked.
  void setSelectedTrajectoryCheckboxVisible();

  /// @brief Set the trajectory info box to show no trajectory stats
  void setCommonTrajectoryInfo(const ros::Duration& duration);
  /// @brief Set the trajectory info box to show planned trajectory stats
  void setPlannedTrajectoryInfo(bool success, planning_scene_utils::TrajectoryData& trajectory);
  /// @brief Set the trajectory info box to show filtered trajectory stats
  void setFilteredTrajectoryInfo(bool success, planning_scene_utils::TrajectoryData& trajectory);
  /// @brief Set the trajectory info box to show executed trajectory stats
  void setExecutedTrajectoryInfo(bool success, planning_scene_utils::TrajectoryData& trajectory);

  // convenience functions
  std::string intToString(int val);
  std::string floatToString(double val);

};
#endif
