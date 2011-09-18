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

#include <move_arm_warehouse/planning_scene_warehouse_viewer.h>
#include <qt4/QtGui/qapplication.h>
#include <assert.h>
#include <qt4/QtGui/qsplashscreen.h>
#include <qt4/QtGui/qdialogbuttonbox.h>
#include <qt4/QtGui/qdialog.h>
#include <qt4/QtGui/qheaderview.h>
#include <qt4/QtGui/qimage.h>
#include <ros/package.h>

using namespace collision_space;
using namespace kinematics_msgs;
using namespace arm_navigation_msgs;
using namespace head_monitor_msgs;
using namespace move_arm_warehouse;
using namespace planning_environment;
using namespace planning_models;
using namespace std;
using namespace trajectory_msgs;
using namespace planning_scene_utils;
using namespace ros::param;
WarehouseViewer* psv = NULL;
bool inited = false;

WarehouseViewer::WarehouseViewer(QWidget* parent, planning_scene_utils::PlanningSceneParameters& params) :
  QMainWindow(parent), PlanningSceneEditor(params)
{
  quit_threads_ = false;
  initQtWidgets();
  selected_trajectory_name_ = "";
  warehouse_data_loaded_once_ = false;
  table_load_thread_ = NULL;
  planning_scene_initialized_ = false;

  qRegisterMetaType < std::string > ( "std::string" );

  popupLoadPlanningScenes();

}

WarehouseViewer::~WarehouseViewer()
{

}

void WarehouseViewer::initQtWidgets()
{
  menu_bar_ = new QMenuBar(this);
  setMenuBar(menu_bar_);
  QWidget* centralWidget = new QWidget(this);
  setCentralWidget(centralWidget);

  QGroupBox* motionPlanBox = new QGroupBox(this);
  motionPlanBox->setTitle("Motion Plan Requests");

  selected_request_label_ = new QLabel(motionPlanBox);
  selected_request_label_->setText("Selected Request: None");
  selected_request_label_->setToolTip("Selected motion plan request. New trajectories will be planned for this request.");

  QGroupBox* trajectoryBox = new QGroupBox(this);
  trajectoryBox->setTitle("Trajectories");


  QVBoxLayout* layout = new QVBoxLayout(this);
  layout->addWidget(motionPlanBox);
  layout->addWidget(trajectoryBox);
  QVBoxLayout* motionBoxLayout = new QVBoxLayout(motionPlanBox);
  motion_plan_tree_ = new QTreeWidget(motionPlanBox);
  motion_plan_tree_->setColumnCount(5);
  motion_plan_tree_->setToolTip("Motion plan tree. Open a motion plan request to access its controls. Click to select a request.");

  QVBoxLayout* trajectoryBoxLayout = new QVBoxLayout(trajectoryBox);
  file_menu_ = menu_bar_->addMenu("File");
  planning_scene_menu_ = menu_bar_->addMenu("Planning Scene");

  new_planning_scene_action_ = file_menu_->addAction("New Planning Scene ...");
  load_planning_scene_action_ = file_menu_->addAction("Load Planning Scene ...");
  save_planning_scene_action_ = file_menu_->addAction("Save Planning Scene ...");
  copy_planning_scene_action_ = file_menu_->addAction("Save Copy of Planning Scene ...");
  new_motion_plan_action_ = file_menu_->addAction("New Motion Plan Request ...");
  refresh_action_ = planning_scene_menu_->addAction("Refresh Planning Scene...");
  view_outcomes_action_ = planning_scene_menu_->addAction("View Outcomes ...");
  alter_link_padding_action_ = planning_scene_menu_->addAction("Alter Link Padding ...");
  alter_allowed_collision_action_ = planning_scene_menu_->addAction("Alter Allowed Collision Operations ...");
  edit_robot_state_action_ = planning_scene_menu_->addAction("Edit Robot State ...");
  quit_action_ = file_menu_->addAction("Quit");

  collision_object_menu_ = menu_bar_->addMenu("Collision Objects");
  new_object_action_ = collision_object_menu_->addAction("New Primitive Collision Object ...");
  new_mesh_action_ = collision_object_menu_->addAction("New Mesh Collision Object ...");

  planner_configuration_menu_ = menu_bar_->addMenu("Planner configuration");
  set_primary_planner_action_ = planner_configuration_menu_->addAction("Use primary planner");
  set_primary_planner_action_->setCheckable(true);
  set_primary_planner_action_->setChecked(true);
  set_secondary_planner_action_ = planner_configuration_menu_->addAction("Use secondary planner");
  set_secondary_planner_action_->setCheckable(true);
  set_secondary_planner_action_->setChecked(false);

  trajectory_tree_ = new QTreeWidget(trajectoryBox);
  trajectory_tree_->setColumnCount(8);


  QLabel* trajectory_label = new QLabel(trajectoryBox);
  trajectory_label->setText("Trajectory Position");

  QGroupBox* buttonsBox = new QGroupBox(trajectoryBox);
  QHBoxLayout* buttonLayout = new QHBoxLayout(buttonsBox);

  trajectory_slider_ = new QSlider(Qt::Horizontal,trajectoryBox);
  trajectory_slider_->setMinimum(0);
  trajectory_slider_->setMaximum(0);

  QLabel* modeLabel = new QLabel(trajectoryBox);
  modeLabel->setText("Selected Trajectory: ");
  modeLabel->setToolTip("Selected trajectory.");

  play_button_ = new QPushButton(this);
  play_button_->setText("Play Trajectory");
  play_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  play_button_->setToolTip("Plays the currently selected trajectory in RVIZ. Makes the trajectory visible.");

  filter_button_ = new QPushButton(this);
  filter_button_->setText("Filter Trajectory");
  filter_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  filter_button_->setToolTip("Sends the currently selected trajectory to the trajectory filter, producing a new trajectory.");

  if(params_.trajectory_filter_service_name_ == "none")
  {
    filter_button_->setEnabled(false);
  }

  replan_button_ = new QPushButton(this);
  replan_button_->setText("Plan New Trajectory");
  replan_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  replan_button_->setToolTip("Plans a new trajectory between the start and goal states of the current motion plan request, producing a new trajectory.");

  if(params_.planner_service_name_ == "none")
  {
    replan_button_->setEnabled(false);
  }

  execute_button_ = new QPushButton(this);
  execute_button_->setText("Execute On Robot");
  execute_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  execute_button_->setEnabled(params_.use_robot_data_);
  execute_button_->setToolTip("Sends the currently selected trajectory to the robot's controllers. (Only works if using robot data). Produces a new trajectory.");

  trajectory_point_edit_ = new QSpinBox(this);
  trajectory_point_edit_->setRange(0, 0);
  trajectory_point_edit_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  trajectory_point_edit_->setToolTip("Currently displayed trajectory point. Drag to change.");

  selected_trajectory_label_ = new QLabel(this);
  selected_trajectory_label_->setText("None");
  selected_trajectory_label_->setToolTip("Currently selected trajectory.");

  QPushButton* deleteMPRButton = new QPushButton(motionPlanBox);
  deleteMPRButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  deleteMPRButton->setText("Delete Motion Plan Request");
  deleteMPRButton->setToolTip("Deletes the currently selected motion plan request.");

  QPushButton* deleteTrajectoryButton = new QPushButton(trajectoryBox);
  deleteTrajectoryButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  deleteTrajectoryButton->setText("Delete Trajectory");
  deleteTrajectoryButton->setToolTip("Deletes the currently selected trajectory.");

  motionBoxLayout->addWidget(motion_plan_tree_);
  motionBoxLayout->addWidget(selected_request_label_);
  motionBoxLayout->addWidget(deleteMPRButton);
  trajectoryBoxLayout->addWidget(trajectory_tree_);
  trajectoryBoxLayout->addWidget(modeLabel);
  trajectoryBoxLayout->addWidget(selected_trajectory_label_);
  trajectoryBoxLayout->addWidget(trajectory_label);
  trajectoryBoxLayout->addWidget(trajectory_point_edit_);
  trajectoryBoxLayout->addWidget(trajectory_slider_);
  trajectoryBoxLayout->addWidget(deleteTrajectoryButton);
  trajectoryBoxLayout->addWidget(buttonsBox);
  buttonLayout->addWidget(play_button_);
  buttonLayout->addWidget(replan_button_);
  buttonLayout->addWidget(filter_button_);
  buttonLayout->addWidget(execute_button_);
  buttonsBox->setTitle("Trajectory Controls");

  connect(deleteMPRButton, SIGNAL(clicked()), this, SLOT(deleteSelectedMotionPlan()));
  connect(deleteTrajectoryButton, SIGNAL(clicked()), this, SLOT(deleteSelectedTrajectory()));
  connect(new_planning_scene_action_, SIGNAL(triggered()), this, SLOT(createNewPlanningSceneSlot()));
  connect(new_motion_plan_action_, SIGNAL(triggered()), this, SLOT(createNewMotionPlanPressed()));
  connect(new_object_action_, SIGNAL(triggered()), this, SLOT(createNewObjectPressed()));
  connect(new_mesh_action_, SIGNAL(triggered()), this, SLOT(createNewMeshPressed()));
  connect(save_planning_scene_action_, SIGNAL(triggered()), this, SLOT(savePlanningSceneSlot()));
  connect(copy_planning_scene_action_, SIGNAL(triggered()), this, SLOT(copyPlanningSceneSlot()));
  connect(load_planning_scene_action_, SIGNAL(triggered()), this, SLOT(popupLoadPlanningScenes()));
  connect(quit_action_, SIGNAL(triggered()), this, SLOT(quit()));
  connect(play_button_, SIGNAL(clicked()), this, SLOT(playButtonPressed()));
  connect(filter_button_, SIGNAL(clicked()), this, SLOT(filterButtonPressed()));
  connect(trajectory_slider_, SIGNAL(sliderMoved(int)), this, SLOT(sliderDragged(int)));
  connect(trajectory_tree_, SIGNAL(itemSelectionChanged()), this, SLOT(trajectoryTableSelection()));
  connect(replan_button_, SIGNAL(clicked()), this, SLOT(replanButtonPressed()));
  connect(trajectory_point_edit_, SIGNAL(valueChanged(int)), this, SLOT(trajectoryEditChanged()));
  connect(motion_plan_tree_, SIGNAL(itemSelectionChanged()), this, SLOT(motionPlanTableSelection()));
  connect(this, SIGNAL(updateTables()), this, SLOT(updateStateTriggered()));
  connect(execute_button_, SIGNAL(clicked()), this, SLOT(executeButtonPressed()));
  connect(refresh_action_, SIGNAL(triggered()), this, SLOT(refreshSceneButtonPressed()));
  connect(view_outcomes_action_, SIGNAL(triggered()), this, SLOT(viewOutcomesPressed()));
  connect(alter_link_padding_action_, SIGNAL(triggered()), this, SLOT(alterLinkPaddingPressed()));
  connect(alter_allowed_collision_action_, SIGNAL(triggered()), this, SLOT(alterAllowedCollisionPressed()));
  connect(edit_robot_state_action_, SIGNAL(triggered()), this, SLOT(editRobotStatePressed()));
  connect(this, SIGNAL(plannerFailure(int)),this, SLOT(popupPlannerFailure(int)));
  connect(this, SIGNAL(filterFailure(int)),this, SLOT(popupFilterFailure(int)));
  connect(this, SIGNAL(attachObjectSignal(const std::string&)), this, SLOT(attachObject(const std::string&)));
  connect(this, SIGNAL(allScenesLoaded()), this, SLOT(refreshPlanningSceneDialog()));
  connect(set_primary_planner_action_, SIGNAL(triggered()), this, SLOT(primaryPlannerTriggered()));
  connect(set_secondary_planner_action_, SIGNAL(triggered()), this, SLOT(secondaryPlannerTriggered()));
  load_planning_scene_dialog_ = new QDialog(this);

  setupPlanningSceneDialog();
  connect(this, SIGNAL(changeProgress(int)), load_scene_progress_, SLOT(setValue(int)));
  menu_bar_->setMinimumWidth(500);
  buttonsBox->setLayout(buttonLayout);
  trajectoryBox->setLayout(trajectoryBoxLayout);
  motionPlanBox->setLayout(motionBoxLayout);
  centralWidget->setLayout(layout);

  createNewObjectDialog();
  createNewMeshDialog();
  createRequestDialog();

  //setCurrentPlanningScene(createNewPlanningScene(), false, false);

}

void WarehouseViewer::createOutcomeDialog()
{
  outcome_dialog_ = new QDialog(this);
  outcome_dialog_->setWindowTitle("Planning Scene Outcomes");
  QVBoxLayout* layout = new QVBoxLayout(outcome_dialog_);

  QGroupBox* stagesBox = new QGroupBox(outcome_dialog_);
  stagesBox->setTitle("Planning Stage Outcomes");

  QVBoxLayout* stagesLayout = new QVBoxLayout(stagesBox);
  stagesBox->setLayout(stagesLayout);

  stage_outcome_table_ = new QTableWidget(stagesBox);
  stagesLayout->addWidget(stage_outcome_table_);

  QStringList stageHeaders;
  stageHeaders.append("Pipeline Stage");
  stageHeaders.append("Outcome");

  stage_outcome_table_->setColumnCount(2);
  stage_outcome_table_->setRowCount((int)error_map_.size());

  stage_outcome_table_->setHorizontalHeaderLabels(stageHeaders);
  stage_outcome_table_->setColumnWidth(0, 150);
  stage_outcome_table_->setColumnWidth(1, 200);
  stage_outcome_table_->setMinimumWidth(350);

  int r = 0;
  for(map<string, ArmNavigationErrorCodes>::iterator it = error_map_.begin(); it != error_map_.end(); it++)
  {
    QTableWidgetItem* stageItem = new QTableWidgetItem(QString::fromStdString(it->first));
    stageItem->setFlags(Qt::ItemIsEnabled);
    stage_outcome_table_->setItem(r,0, stageItem);

    QTableWidgetItem* outcomeItem = new QTableWidgetItem(QString::fromStdString(armNavigationErrorCodeToString(it->second)));
    outcomeItem->setFlags(Qt::ItemIsEnabled);
    stage_outcome_table_->setItem(r,1, outcomeItem);

    if(it->second.val != ArmNavigationErrorCodes::SUCCESS)
    {
      outcomeItem->setTextColor(QColor(150, 0, 0));
    }

    r++;
  }


  QGroupBox* trajectoryBox = new QGroupBox(outcome_dialog_);
  trajectoryBox->setTitle("Trajectory Outcomes");

  QVBoxLayout* trajectoryLayout = new QVBoxLayout(trajectoryBox);
  trajectoryBox->setLayout(trajectoryLayout);

  trajectory_outcome_table_ = new QTableWidget(trajectoryBox);
  trajectoryLayout->addWidget(trajectory_outcome_table_);

  QStringList trajectoryHeaders;
  trajectoryHeaders.append("Trajectory");
  trajectoryHeaders.append("Stage");
  trajectoryHeaders.append("Outcome");

  // trajectory_outcome_table_->setColumnCount(3);
  // trajectory_outcome_table_->setRowCount((int)(trajectory_map_.find(selected_motion_plan_name_)->second.size()));
  // trajectory_outcome_table_->setHorizontalHeaderLabels(trajectoryHeaders);
  // trajectory_outcome_table_->setColumnWidth(0, 150);
  // trajectory_outcome_table_->setColumnWidth(1, 150);
  // trajectory_outcome_table_->setColumnWidth(2, 200);
  // trajectory_outcome_table_->setMinimumWidth(550);

  // r = 0;
  // for(map<string, TrajectoryData>::iterator it = trajectory_map_->begin(); it != trajectory_map_->end(); it++)
  // {
  //   QTableWidgetItem* trajectoryItem = new QTableWidgetItem(QString::fromStdString(it->first));
  //   trajectoryItem->setFlags(Qt::ItemIsEnabled);
  //   trajectory_outcome_table_->setItem(r, 0, trajectoryItem);

  //   QTableWidgetItem* stageItem = new QTableWidgetItem(QString::fromStdString(it->second.getSource()));
  //   stageItem->setFlags(Qt::ItemIsEnabled);
  //   trajectory_outcome_table_->setItem(r, 1, stageItem);

  //   QTableWidgetItem* outcomeItem = new QTableWidgetItem(QString::fromStdString(armNavigationErrorCodeToString(it->second.trajectory_error_code_)));
  //   outcomeItem ->setFlags(Qt::ItemIsEnabled);
  //   trajectory_outcome_table_->setItem(r, 2, outcomeItem);

  //   if(it->second.trajectory_error_code_.val != ArmNavigationErrorCodes::SUCCESS)
  //   {
  //     outcomeItem->setTextColor(QColor(150, 0, 0));
  //     std::stringstream ss;
  //     ss << "Bad point: " << it->second.getBadPoint();
  //     outcomeItem->setToolTip(QString::fromStdString(ss.str()));
  //   }

  //   r++;
  // }

  // layout->addWidget(stagesBox);
  // layout->addWidget(trajectoryBox);
  outcome_dialog_->setLayout(layout);

}

void WarehouseViewer::viewOutcomesPressed()
{
  createOutcomeDialog();
  outcome_dialog_->exec();
  delete outcome_dialog_;
}

void WarehouseViewer::createAlterLinkPaddingDialog()
{
  if(current_planning_scene_name_ == "") {
    return;
  }

  arm_navigation_msgs::PlanningScene planning_scene = planning_scene_map_[current_planning_scene_name_].getPlanningScene();

  if(planning_scene.link_padding.size() == 0) {
    planning_environment::convertFromLinkPaddingMapToLinkPaddingVector(cm_->getDefaultLinkPaddingMap(), planning_scene.link_padding);
    planning_scene_map_[current_planning_scene_name_].setPlanningScene(planning_scene);
  }

  alter_link_padding_dialog_ = new QDialog(this);
  alter_link_padding_dialog_->setWindowTitle("Alter Link Padding");
  QVBoxLayout* layout = new QVBoxLayout(alter_link_padding_dialog_);
  
  alter_link_padding_table_ = new QTableWidget(alter_link_padding_dialog_);
  QStringList alter_headers;
  alter_headers.append("Link name");
  alter_headers.append("Padding (m)");
  
  alter_link_padding_table_->setColumnCount(2);
  alter_link_padding_table_->setRowCount((int)planning_scene.link_padding.size());
  
  alter_link_padding_table_->setHorizontalHeaderLabels(alter_headers);
  alter_link_padding_table_->setColumnWidth(0, 300);
  alter_link_padding_table_->setColumnWidth(1, 150);
  alter_link_padding_table_->setMinimumWidth(500);

  for(unsigned int i = 0; i < planning_scene.link_padding.size(); i++) {
    QTableWidgetItem* link_item = new QTableWidgetItem(QString::fromStdString(planning_scene.link_padding[i].link_name));
    link_item->setFlags(Qt::ItemIsEnabled);
    alter_link_padding_table_->setItem(i, 0, link_item);

    QDoubleSpinBox* padding_spin_box = new QDoubleSpinBox(alter_link_padding_table_);
    padding_spin_box->setMinimum(0.0);
    padding_spin_box->setDecimals(3);
    padding_spin_box->setSingleStep(.005);
    padding_spin_box->setValue(planning_scene.link_padding[i].padding);
    connect(padding_spin_box, SIGNAL(valueChanged(double)), this, SLOT(alteredLinkPaddingValueChanged(double)));

    alter_link_padding_table_->setCellWidget(i, 1, padding_spin_box);
  }
  layout->addWidget(alter_link_padding_table_);
  alter_link_padding_dialog_->setLayout(layout);
}

void WarehouseViewer::alterLinkPaddingPressed()
{
  createAlterLinkPaddingDialog();
  alter_link_padding_dialog_->exec();
  delete alter_link_padding_dialog_;
}

void WarehouseViewer::alteredLinkPaddingValueChanged(double d)
{
  arm_navigation_msgs::PlanningScene planning_scene = planning_scene_map_[current_planning_scene_name_].getPlanningScene();
  planning_scene.link_padding.clear();

  for(int i = 0; i < alter_link_padding_table_->rowCount(); i++)
  {
    arm_navigation_msgs::LinkPadding lp;
    lp.link_name = alter_link_padding_table_->item(i,0)->text().toStdString();
    lp.padding = dynamic_cast<QDoubleSpinBox*>(alter_link_padding_table_->cellWidget(i,1))->value();
    planning_scene.link_padding.push_back(lp);
  }
  planning_scene_map_[current_planning_scene_name_].setPlanningScene(planning_scene);
  sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
}

void WarehouseViewer::alterAllowedCollisionPressed()
{
  createAlterAllowedCollisionDialog();
  alter_allowed_collision_dialog_->exec();
  delete alter_allowed_collision_dialog_;
}

void WarehouseViewer::createAlterAllowedCollisionDialog()
{
  if(current_planning_scene_name_ == "") {
    return;
  }

  arm_navigation_msgs::PlanningScene planning_scene = planning_scene_map_[current_planning_scene_name_].getPlanningScene();

  if(planning_scene.allowed_collision_matrix.link_names.size() == 0) {
    planning_environment::convertFromACMToACMMsg(cm_->getDefaultAllowedCollisionMatrix(), planning_scene.allowed_collision_matrix);
    planning_scene_map_[current_planning_scene_name_].setPlanningScene(planning_scene);
  }

  alter_allowed_collision_dialog_ = new QDialog(this);
  alter_allowed_collision_dialog_->setWindowTitle("Alter Allowed Collision");

  QGridLayout* layout = new QGridLayout(alter_allowed_collision_dialog_);
  
  QLabel* col_1_label = new QLabel(alter_allowed_collision_dialog_);
  col_1_label->setText("First entity");

  QLabel* col_2_label = new QLabel(alter_allowed_collision_dialog_);
  col_2_label->setText("Second entity");

  QLabel* col_3_label = new QLabel(alter_allowed_collision_dialog_);
  col_3_label->setText("Status");

  QLabel* col_4_label = new QLabel(alter_allowed_collision_dialog_);
  col_4_label->setText("Operation");

  layout->addWidget(col_1_label, 0, 0);
  layout->addWidget(col_2_label, 0, 1);
  layout->addWidget(col_3_label, 0, 2);
  layout->addWidget(col_4_label, 0, 3);

  first_allowed_collision_line_edit_ = new QLineEdit(alter_allowed_collision_dialog_);
  second_allowed_collision_line_edit_ = new QLineEdit(alter_allowed_collision_dialog_);

  layout->addWidget(first_allowed_collision_line_edit_, 1, 0);
  layout->addWidget(second_allowed_collision_line_edit_, 1, 1);

  connect(first_allowed_collision_line_edit_,SIGNAL(textEdited(const QString&)), this, SLOT(entityListsEdited()));
  connect(second_allowed_collision_line_edit_,SIGNAL(textEdited(const QString&)), this, SLOT(entityListsEdited()));

  first_allowed_collision_list_ = new QListWidget(alter_allowed_collision_dialog_);
  second_allowed_collision_list_ = new QListWidget(alter_allowed_collision_dialog_);
  
  for(unsigned int i = 0; i < planning_scene.allowed_collision_matrix.link_names.size(); i++) {
    if(cm_->getKinematicModel()->hasLinkModel(planning_scene.allowed_collision_matrix.link_names[i])) {
      first_allowed_collision_list_->addItem(planning_scene.allowed_collision_matrix.link_names[i].c_str());
      second_allowed_collision_list_->addItem(planning_scene.allowed_collision_matrix.link_names[i].c_str());
    }
  }
  addSpecialEntries(first_allowed_collision_list_);
  addSpecialEntries(second_allowed_collision_list_);

  connect(first_allowed_collision_list_,SIGNAL(itemSelectionChanged()), this, SLOT(firstEntityListSelected()));
  connect(second_allowed_collision_list_,SIGNAL(itemSelectionChanged()), this, SLOT(secondEntityListSelected()));


  layout->addWidget(first_allowed_collision_list_, 2, 0);
  layout->addWidget(second_allowed_collision_list_, 2, 1);

  allowed_status_line_edit_ = new QLineEdit(alter_allowed_collision_dialog_);
  allowed_status_line_edit_->setMinimumWidth(125);
  allowed_status_line_edit_->setMaximumWidth(125);
  allowed_status_line_edit_->setAlignment(Qt::AlignCenter);
  layout->addWidget(allowed_status_line_edit_, 1, 2);
  
  QPushButton* enable_collision_button = new QPushButton(alter_allowed_collision_dialog_);
  enable_collision_button->setText("Enable");
  enable_collision_button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  QPushButton* disable_collision_button = new QPushButton(alter_allowed_collision_dialog_);
  disable_collision_button->setText("Disable");
  disable_collision_button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  connect(enable_collision_button, SIGNAL(clicked()), this, SLOT(enableCollisionClicked()));
  connect(disable_collision_button, SIGNAL(clicked()), this, SLOT(disableCollisionClicked()));
  setEnabledDisabledDisplay(first_allowed_collision_line_edit_->text(), second_allowed_collision_line_edit_->text());
  
  QPushButton* reset_default_button = new QPushButton(alter_allowed_collision_dialog_);
  reset_default_button->setText("Reset");
  reset_default_button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

  connect(reset_default_button, SIGNAL(clicked()), this, SLOT(resetAllowedCollisionClicked()));

  layout->addWidget(enable_collision_button, 1, 3);
  layout->addWidget(disable_collision_button, 2, 3, Qt::AlignTop);
  layout->addWidget(reset_default_button, 2, 3, Qt::AlignBottom);

  alter_allowed_collision_dialog_->setLayout(layout);

  alter_allowed_collision_dialog_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Minimum);
}

void WarehouseViewer::addSpecialEntries(QListWidget* list_widget) {
  list_widget->addItem("------Groups---------");
  std::vector<std::string> group_names;
  cm_->getKinematicModel()->getModelGroupNames(group_names);
  for(unsigned int i = 0; i < group_names.size(); i++) {
    list_widget->addItem(group_names[i].c_str());
  }
  list_widget->addItem("------Objects---------");
  std::vector<std::string> object_names;
  cm_->getCollisionObjectNames(object_names);
  for(unsigned int i = 0; i < object_names.size(); i++) {
    list_widget->addItem(object_names[i].c_str());
  }
  list_widget->addItem("------Attached Objects---------");
  std::vector<std::string> attached_object_names;
  cm_->getAttachedCollisionObjectNames(attached_object_names);
  for(unsigned int i = 0; i < attached_object_names.size(); i++) {
    list_widget->addItem(attached_object_names[i].c_str());
  }
  list_widget->addItem("------Special-------");
  list_widget->addItem("COLLISION_SET_ALL");
  list_widget->addItem("COLLISION_SET_OBJECTS");
  list_widget->addItem("COLLISION_SET_ATTACHED_OBJECTS");
}

void WarehouseViewer::getEntryList(const std::string& s1, 
                                   std::vector<std::string>& sv1)
{
  bool all = false;
  if(s1 == "COLLISION_SET_ALL") {
    all = true;
  } else if(s1 == "COLLISION_SET_OBJECTS") {
    cm_->getCollisionObjectNames(sv1);
  } else if(s1 == "COLLISION_SET_ATTACHED_OBJECTS") {
    cm_->getAttachedCollisionObjectNames(sv1);
  } else if(cm_->getKinematicModel()->hasModelGroup(s1)) {
    sv1 = cm_->getKinematicModel()->getModelGroup(s1)->getGroupLinkNames();
  } else {
    sv1.push_back(s1);
  }
  if(all) {
    collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm_->getCurrentAllowedCollisionMatrix();
    acm.getAllEntryNames(sv1);
  }
}

void WarehouseViewer::resetAllowedCollisionClicked() 
{
  arm_navigation_msgs::PlanningScene planning_scene = planning_scene_map_[current_planning_scene_name_].getPlanningScene();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = cm_->getDefaultAllowedCollisionMatrix(); 
  
  planning_environment::convertFromACMToACMMsg(acm, planning_scene.allowed_collision_matrix);
  planning_scene_map_[current_planning_scene_name_].setPlanningScene(planning_scene);
  sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
  setEnabledDisabledDisplay(first_allowed_collision_line_edit_->text(), second_allowed_collision_line_edit_->text());
}

void WarehouseViewer::firstEntityListSelected() {
  first_allowed_collision_line_edit_->setText(first_allowed_collision_list_->selectedItems()[0]->text());
  setEnabledDisabledDisplay(first_allowed_collision_line_edit_->text(), second_allowed_collision_line_edit_->text());
}

void WarehouseViewer::secondEntityListSelected() {
  second_allowed_collision_line_edit_->setText(second_allowed_collision_list_->selectedItems()[0]->text());
  setEnabledDisabledDisplay(first_allowed_collision_line_edit_->text(), second_allowed_collision_line_edit_->text());
}

void WarehouseViewer::entityListsEdited() {
  setEnabledDisabledDisplay(first_allowed_collision_line_edit_->text(), second_allowed_collision_line_edit_->text());
}

void WarehouseViewer::enableCollisionClicked() {
  arm_navigation_msgs::PlanningScene planning_scene = planning_scene_map_[current_planning_scene_name_].getPlanningScene();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = planning_environment::convertFromACMMsgToACM(planning_scene.allowed_collision_matrix);
  QString qs1 = first_allowed_collision_line_edit_->text();
  QString qs2 = second_allowed_collision_line_edit_->text();

  std::vector<std::string> first_list;
  std::vector<std::string> second_list;

  getEntryList(qs1.toStdString(), first_list); 
  getEntryList(qs2.toStdString(), second_list);

  bool all_enabled = true;

  for(unsigned int i = 0; i < first_list.size(); i++) {
    for(unsigned int j = 0; j < second_list.size(); j++) {
      bool allowed = false;
      if(acm.getAllowedCollision(first_list[i], second_list[j], allowed)) {
        if(allowed) {
          all_enabled = false;
          break;
        }
      }
    }
  }
  if(all_enabled) return;
  
  acm.changeEntry(first_list, second_list, false);
  planning_environment::convertFromACMToACMMsg(acm, planning_scene.allowed_collision_matrix);
  planning_scene_map_[current_planning_scene_name_].setPlanningScene(planning_scene);
  sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
  setEnabledDisabledDisplay(first_allowed_collision_line_edit_->text(), second_allowed_collision_line_edit_->text());
}

void WarehouseViewer::disableCollisionClicked() {
  arm_navigation_msgs::PlanningScene planning_scene = planning_scene_map_[current_planning_scene_name_].getPlanningScene();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = planning_environment::convertFromACMMsgToACM(planning_scene.allowed_collision_matrix);
  QString qs1 = first_allowed_collision_line_edit_->text();
  QString qs2 = second_allowed_collision_line_edit_->text();

  std::vector<std::string> first_list;
  std::vector<std::string> second_list;

  getEntryList(qs1.toStdString(), first_list); 
  getEntryList(qs2.toStdString(), second_list);

  bool all_disabled = true;

  for(unsigned int i = 0; i < first_list.size(); i++) {
    for(unsigned int j = 0; j < second_list.size(); j++) {
      bool allowed = false;
      if(acm.getAllowedCollision(first_list[i], second_list[j], allowed)) {
        if(!allowed) {
          all_disabled = false;
          break;
        }
      }
    }
  }
  
  if(all_disabled) return;

  acm.changeEntry(first_list, second_list, true);
  planning_environment::convertFromACMToACMMsg(acm, planning_scene.allowed_collision_matrix);
  planning_scene_map_[current_planning_scene_name_].setPlanningScene(planning_scene);
  sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
  setEnabledDisabledDisplay(first_allowed_collision_line_edit_->text(), second_allowed_collision_line_edit_->text());
}

void WarehouseViewer::setEnabledDisabledDisplay(const QString& qs1, const QString& qs2) {
  arm_navigation_msgs::PlanningScene planning_scene = planning_scene_map_[current_planning_scene_name_].getPlanningScene();
  collision_space::EnvironmentModel::AllowedCollisionMatrix acm = planning_environment::convertFromACMMsgToACM(planning_scene.allowed_collision_matrix);
  std::vector<std::string> first_list;
  std::vector<std::string> second_list;
  
  getEntryList(qs1.toStdString(), first_list); 
  getEntryList(qs2.toStdString(), second_list);

  bool all_disabled = true;
  bool all_enabled = true;
  bool some_found = false;

  for(unsigned int i = 0; i < first_list.size(); i++) {
    for(unsigned int j = 0; j < second_list.size(); j++) {
      bool allowed = false;
      if(acm.getAllowedCollision(first_list[i], second_list[j], allowed)) {
        some_found = true;
        if(allowed) {
          all_enabled = false;
        } else {
          all_disabled = false;
        }
      } 
    }
  }
  if(!some_found) {
    allowed_status_line_edit_->setText("NO ENTRY");
  } else if(!all_disabled && !all_enabled) {
    allowed_status_line_edit_->setText("MIXED");
  } else if(all_disabled) {
    allowed_status_line_edit_->setText("DISABLED");
  } else {
    allowed_status_line_edit_->setText("ENABLED");
  }
}

void WarehouseViewer::refreshSceneButtonPressed()
{
  if(current_planning_scene_name_ != "" )
  {
    sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
  }
  else
  {
    QMessageBox msg(QMessageBox::Warning,"Refresh", "No planning scene loaded!");
    msg.addButton(QMessageBox::Ok);
    msg.exec();
  }
}

void WarehouseViewer::executeButtonPressed()
{
  if(selected_trajectory_name_ != "")
  {
    executeTrajectory(selected_motion_plan_name_,
                      selected_trajectory_name_);
  }
  else
  {
    QMessageBox msg(QMessageBox::Warning, "Execute Trajectory", "No Trajectory Selected!");
    msg.addButton("Ok", QMessageBox::AcceptRole);
    msg.exec();
  }
}

void WarehouseViewer::deleteSelectedMotionPlan()
{
  if(selected_motion_plan_name_ != "")
  {
    lockScene();
    std::vector<unsigned int> erased_trajectories;
    deleteMotionPlanRequest(motion_plan_map_[selected_motion_plan_name_].getId(), erased_trajectories);
    unlockScene();
  }
  else
  {
    QMessageBox msg(QMessageBox::Warning, "Delete Motion Plan Request", "No Motion Plan Request Selected!");
    msg.addButton("Ok", QMessageBox::AcceptRole);
    msg.exec();
  }
}

void WarehouseViewer::deleteSelectedTrajectory()
{
  if(selected_trajectory_name_ != "")
  {
    lockScene();
    unsigned int mpr_id = motion_plan_map_[selected_motion_plan_name_].getId();
    unsigned int traj_id = trajectory_map_[selected_motion_plan_name_][selected_trajectory_name_].getId();
    deleteTrajectory(mpr_id, 
                     traj_id);
    emit updateTables();
    unlockScene();
  }
  else
  {
    QMessageBox msg(QMessageBox::Warning, "Delete Trajectory", "No Trajectory Selected!");
    msg.addButton("Ok", QMessageBox::AcceptRole);
    msg.exec();
  }
}

void WarehouseViewer::createNewMotionPlanPressed()
{
  new_request_dialog_->open();
}

void WarehouseViewer::trajectoryEditChanged()
{
  if(selected_trajectory_name_ != "")
  {
    TrajectoryData& trajectory = trajectory_map_[selected_motion_plan_name_][selected_trajectory_name_];
    trajectory.setCurrentPoint(trajectory_point_edit_->value());
    trajectory_slider_->setValue(trajectory_point_edit_->value());
  }
}

void WarehouseViewer::setupPlanningSceneDialog()
{
  planning_scene_table_ = new QTableWidget(load_planning_scene_dialog_);
  QVBoxLayout* layout = new QVBoxLayout(load_planning_scene_dialog_);
  layout->addWidget(planning_scene_table_);
  load_scene_progress_ = new QProgressBar(load_planning_scene_dialog_);
  layout->addWidget(load_scene_progress_);

  new_planning_scene_button_ = new QPushButton(load_planning_scene_dialog_);
  new_planning_scene_button_->setText("New...");
  new_planning_scene_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  load_planning_scene_button_ = new QPushButton(load_planning_scene_dialog_);
  load_planning_scene_button_->setText("Load...");
  load_planning_scene_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  remove_planning_scene_button_ = new QPushButton(load_planning_scene_dialog_);
  remove_planning_scene_button_->setText("Remove...");
  remove_planning_scene_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  refresh_planning_scene_button_ = new QPushButton(load_planning_scene_dialog_);
  refresh_planning_scene_button_->setText("Refresh...");
  refresh_planning_scene_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  connect(new_planning_scene_button_, SIGNAL(clicked()), this, SLOT(newButtonPressed()));
  connect(load_planning_scene_button_, SIGNAL(clicked()), this, SLOT(loadButtonPressed()));
  connect(refresh_planning_scene_button_, SIGNAL(clicked()), this, SLOT(refreshButtonPressed()));
  connect(remove_planning_scene_button_, SIGNAL(clicked()), this, SLOT(removePlanningSceneButtonPressed()));

  load_motion_plan_requests_box_ = new QCheckBox(load_planning_scene_dialog_);
  load_motion_plan_requests_box_->setChecked(true);
  load_motion_plan_requests_box_->setText("Load Motion Plan Requests");
  load_trajectories_box_ = new QCheckBox(load_planning_scene_dialog_);
  load_trajectories_box_->setChecked(true);
  load_trajectories_box_->setText("Load Trajectories");

  layout->addWidget(load_motion_plan_requests_box_);
  layout->addWidget(load_trajectories_box_);
  layout->addWidget(new_planning_scene_button_);
  layout->addWidget(load_planning_scene_button_);
  layout->addWidget(refresh_planning_scene_button_);
  layout->addWidget(remove_planning_scene_button_);

  load_planning_scene_dialog_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  load_planning_scene_dialog_->setLayout(layout);
}

void WarehouseViewer::quit()
{
  QMessageBox msgBox(QMessageBox::Warning, "Quit?", "Are you sure you want to quit? Unsaved changes will be lost.");
  QPushButton *quitButton = msgBox.addButton("Quit", QMessageBox::ActionRole);
  QPushButton *saveQuitButton = msgBox.addButton("Save And Quit", QMessageBox::ActionRole);
  msgBox.addButton(QMessageBox::Cancel);

  msgBox.exec();

  if (msgBox.clickedButton() == quitButton)
  {
    quit_threads_ = true;
    delete this;
    return;
  }
  else if (msgBox.clickedButton() == saveQuitButton)
  {
    saveCurrentPlanningScene(false);
    quit_threads_ = true;
    delete this;
    return;
  }
  else
  {
    return;
  }
}

void WarehouseViewer::motionPlanTableSelection()
{
  QList<QTreeWidgetItem*> selected = motion_plan_tree_->selectedItems();

  if(selected.size() > 0)
  {
    const QString& name = selected[0]->toolTip(0);
    selectMotionPlan(name.toStdString());
  }
}

void WarehouseViewer::selectMotionPlan(std::string ID)
{
  selected_motion_plan_name_ = ID;
  selected_trajectory_name_ = "";
  createTrajectoryTable();
  selected_request_label_->setText(QString::fromStdString("Selected Request: "+selected_motion_plan_name_ ));


  if(motion_plan_map_[selected_motion_plan_name_].getTrajectories().size() > 0)
  {
    unsigned int id = *(motion_plan_map_[selected_motion_plan_name_].getTrajectories().begin()); 
    selectTrajectory(getTrajectoryNameFromId(id));
  }
}

void WarehouseViewer::createMotionPlanTable()
{
  if(motion_plan_map_.find(selected_motion_plan_name_) == motion_plan_map_.end()) {
    selected_request_label_->setText("Selected Request: None");
    selected_motion_plan_name_ = "";
  } else {
    ROS_INFO_STREAM("Have selected motion plan " << selected_motion_plan_name_);
  }

  if(current_planning_scene_name_ != "")
  {
    PlanningSceneData& planningSceneData = planning_scene_map_[current_planning_scene_name_];

    motion_plan_tree_->clear();
    motion_plan_tree_->setColumnCount(4);
    motion_plan_tree_->setColumnWidth(0, 150);

    unsigned int count = 0;
    for(std::set<unsigned int>::iterator it = planningSceneData.getRequests().begin();
        it != planningSceneData.getRequests().end();
        it++, count++) {

      if(motion_plan_map_.find(getMotionPlanRequestNameFromId(*it)) == motion_plan_map_.end()) {
        ROS_INFO_STREAM("Bad matt 1");
      }
      MotionPlanRequestData& requestData = motion_plan_map_[getMotionPlanRequestNameFromId(*it)];


      QTreeWidgetItem* nameItem = new QTreeWidgetItem(QStringList(QString::fromStdString(requestData.getName())));
      nameItem->setToolTip(0,nameItem->text(0));
      nameItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      motion_plan_tree_->insertTopLevelItem(count, nameItem);

      QStringList sourceList;
      sourceList.append("Source");
      sourceList.append(QString::fromStdString(requestData.getSource()));
      QTreeWidgetItem* sourceItem = new QTreeWidgetItem(sourceList);
      sourceItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
      sourceItem->setToolTip(0, nameItem->text(0));
      sourceItem->setToolTip(1, nameItem->text(0));
      nameItem->insertChild(0, sourceItem);

      QStringList collisionList;
      collisionList.append("Collisions");
      QTreeWidgetItem* collisionItem = new QTreeWidgetItem(collisionList);
      collisionItem->setToolTip(0, nameItem->text(0));
      QCheckBox* collisionsVisible = new QCheckBox(motion_plan_tree_);
      collisionsVisible->setText("Show Collisions");
      collisionsVisible->setToolTip(nameItem->text(0));
      nameItem->insertChild(1, collisionItem);
      motion_plan_tree_->setItemWidget(collisionItem, 1, collisionsVisible);
      collisionsVisible->setChecked(requestData.areCollisionsVisible());

      connect(collisionsVisible, SIGNAL(clicked(bool)), this, SLOT(motionPlanCollisionVisibleButtonClicked(bool)));

      QStringList startList;
      startList.append("Start Position");
      startList.append("");
      startList.append("Color");
      startList.append("");
      QTreeWidgetItem* startItem = new QTreeWidgetItem(startList);
      startItem->setToolTip(0, nameItem->text(0));
      QCheckBox* startVisible = new QCheckBox(motion_plan_tree_);
      startVisible->setText("Visible");
      startVisible->setToolTip(nameItem->text(0));
      nameItem->insertChild(2, startItem);
      motion_plan_tree_->setItemWidget(startItem, 1, startVisible);
      startVisible->setChecked(requestData.isStartVisible());

      connect(startVisible, SIGNAL(clicked(bool)), this, SLOT(motionPlanStartVisibleButtonClicked(bool)));

      QPushButton* startColorButton = new QPushButton(motion_plan_tree_);
      std::stringstream startColorStream;
      startColorStream<< "(" << (int)(requestData.getStartColor().r*255) <<" , ";
      startColorStream << (int)(requestData.getStartColor().g*255) << " , ";
      startColorStream << (int)(requestData.getStartColor().b*255) << ")";
      startColorButton->setText(QString::fromStdString(startColorStream.str()));
      startColorButton->setToolTip(nameItem->text(0));
      startColorButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
      connect(startColorButton, SIGNAL(clicked()), this, SLOT(motionPlanStartColorButtonClicked()));
      motion_plan_tree_->setItemWidget(startItem, 3, startColorButton);


      QStringList endList;
      endList.append("End Position");
      endList.append("");
      endList.append("Color");
      endList.append("");
      QTreeWidgetItem* endItem = new QTreeWidgetItem(endList);
      endItem->setToolTip(0, nameItem->text(0));
      QCheckBox* endVisible = new QCheckBox(motion_plan_tree_);
      endVisible->setText("Visible");
      endVisible->setToolTip(nameItem->text(0));
      nameItem->insertChild(3, endItem);
      motion_plan_tree_->setItemWidget(endItem, 1, endVisible);
      endVisible->setChecked(requestData.isEndVisible());

      connect(endVisible, SIGNAL(clicked(bool)), this, SLOT(motionPlanEndVisibleButtonClicked(bool)));

      QPushButton* endColorButton = new QPushButton(motion_plan_tree_);
      std::stringstream endColorStream;
      endColorStream<< "(" << (int)(requestData.getGoalColor().r*255) <<" , ";
      endColorStream << (int)(requestData.getGoalColor().g*255) << " , ";
      endColorStream << (int)(requestData.getGoalColor().b*255) << ")";
      endColorButton->setText(QString::fromStdString(endColorStream.str()));
      endColorButton->setToolTip(nameItem->text(0));
      endColorButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
      connect(endColorButton, SIGNAL(clicked()), this, SLOT(motionPlanEndColorButtonClicked()));
      motion_plan_tree_->setItemWidget(endItem, 3, endColorButton);

      QStringList controlList;
      controlList.append("Joint Control");
      QTreeWidgetItem* controlItem = new QTreeWidgetItem(controlList);
      controlItem->setToolTip(0, nameItem->text(0));
      nameItem->insertChild(4, controlItem);
      QCheckBox* controlsVisible = new QCheckBox(motion_plan_tree_);
      controlsVisible->setText("Joint Control");
      controlsVisible->setToolTip(nameItem->text(0));
      connect(controlsVisible, SIGNAL(clicked(bool)), this, SLOT(motionPlanJointControlsActiveButtonClicked(bool)));
      motion_plan_tree_->setItemWidget(controlItem,1, controlsVisible);

      QStringList renderTypeList;
      renderTypeList.append("Render Mode");
      QTreeWidgetItem* renderTypeItem = new QTreeWidgetItem(renderTypeList);
      renderTypeItem->setToolTip(0, nameItem->text(0));

      QComboBox* renderTypeBox = new QComboBox(motion_plan_tree_);
      QStringList items;
      items.append("Collision Mesh");
      items.append("Visual Mesh");
      items.append("Padding Mesh");
      renderTypeBox->addItems(items);
      connect(renderTypeBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(motionPlanRenderTypeChanged(const QString&)));
      renderTypeBox->setToolTip(nameItem->text(0));
      nameItem->insertChild(5, renderTypeItem);
      motion_plan_tree_->setItemWidget(renderTypeItem, 1, renderTypeBox);

      switch(requestData.getRenderType())
      {
        case CollisionMesh:
          renderTypeBox->setCurrentIndex(0);
          break;
        case VisualMesh:
          renderTypeBox->setCurrentIndex(1);
          break;
        case PaddingMesh:
          renderTypeBox->setCurrentIndex(2);
          break;
      }
    }
  }
}

void WarehouseViewer::motionPlanCollisionVisibleButtonClicked(bool checked)
{
  QObject* sender = QObject::sender();
  QCheckBox* button = qobject_cast<QCheckBox*> (sender);

  if(button != NULL)
  {
    std::string mprID = button->toolTip().toStdString();
    MotionPlanRequestData& data = motion_plan_map_[mprID];
    data.setCollisionsVisible(checked);
  }
}

void WarehouseViewer::motionPlanStartColorButtonClicked()
{
  QObject* sender = QObject::sender();
  QPushButton* button = qobject_cast<QPushButton*>(sender);

  if(button != NULL)
  {
    std::string trajectoryID = button->toolTip().toStdString();
    MotionPlanRequestData& data = motion_plan_map_[trajectoryID];
    QColor col(data.getStartColor().r*255, data.getStartColor().g*255, data.getStartColor().b*255);
    QColor colorSelected = QColorDialog::getColor(col, this);
    if(!colorSelected.isValid())
    {
      return;
    }
    std::stringstream colorStream;
    colorStream<< "(" << colorSelected.red()<<" , ";
    colorStream << colorSelected.green()<< " , ";
    colorStream << colorSelected.blue() << ")";

    button->setText(QString::fromStdString(colorStream.str()));

    std_msgs::ColorRGBA trajColor;
    trajColor.r = colorSelected.redF();
    trajColor.g = colorSelected.greenF();
    trajColor.b = colorSelected.blueF();
    trajColor.a = 0.5f;
    data.setStartColor(trajColor);
    data.refreshColors();
  }
}

void WarehouseViewer::motionPlanEndColorButtonClicked()
{
  QObject* sender = QObject::sender();
  QPushButton* button = qobject_cast<QPushButton*>(sender);

  if(button != NULL)
  {
    std::string trajectoryID = button->toolTip().toStdString();
    MotionPlanRequestData& data = motion_plan_map_[trajectoryID];
    QColor col(data.getGoalColor().r*255, data.getGoalColor().g*255, data.getGoalColor().b*255);
    QColor colorSelected = QColorDialog::getColor(col, this);
    if(!colorSelected.isValid())
    {
      return;
    }
    std::stringstream colorStream;
    colorStream<< "(" << colorSelected.red()<<" , ";
    colorStream << colorSelected.green()<< " , ";
    colorStream << colorSelected.blue() << ")";

    button->setText(QString::fromStdString(colorStream.str()));

    std_msgs::ColorRGBA trajColor;
    trajColor.r = colorSelected.redF();
    trajColor.g = colorSelected.greenF();
    trajColor.b = colorSelected.blueF();
    trajColor.a = 0.5f;
    data.setGoalColor(trajColor);
    data.refreshColors();

  }
}

void WarehouseViewer::motionPlanStartVisibleButtonClicked(bool checked)
{
  QObject* sender = QObject::sender();
  QCheckBox* button = qobject_cast<QCheckBox*> (sender);

  if(button != NULL)
  {
    std::string mprID = button->toolTip().toStdString();
    MotionPlanRequestData& data = motion_plan_map_[mprID];
    data.setStartVisible(checked);
    data.setJointControlsVisible(data.areJointControlsVisible(), this);
    setIKControlsVisible(mprID, StartPosition, button->isChecked());
  }


}

void WarehouseViewer::motionPlanEndVisibleButtonClicked(bool checked)
{
  QObject* sender = QObject::sender();
  QCheckBox* button = qobject_cast<QCheckBox*> (sender);

  if(button != NULL)
  {
    std::string mprID = button->toolTip().toStdString();
    MotionPlanRequestData& data = motion_plan_map_[mprID];
    data.setEndVisible(checked);
    data.setJointControlsVisible(data.areJointControlsVisible(), this);
    setIKControlsVisible(mprID, GoalPosition, button->isChecked());
  }
}

void WarehouseViewer::createNewMotionPlanRequest(std::string group_name, std::string end_effector_name)
{

  if(current_planning_scene_name_ != "")
  {
    unsigned int motion_plan_id;
    createMotionPlanRequest(*robot_state_, *robot_state_, 
                            group_name, end_effector_name, false,
                            getPlanningSceneIdFromName(current_planning_scene_name_), 
                            create_request_from_robot_box_->isChecked(), 
                            motion_plan_id);
    selectMotionPlan(getMotionPlanRequestNameFromId(motion_plan_id));
  } 
  else
  {
    QMessageBox msg(QMessageBox::Warning, "Create Request", "No Planning Scene Loaded!");
    msg.addButton("Ok", QMessageBox::AcceptRole);
    msg.exec();
  }
}

void WarehouseViewer::savePlanningSceneSlot() {
  saveCurrentPlanningScene(false);
}

void WarehouseViewer::copyPlanningSceneSlot() {
  saveCurrentPlanningScene(true);
  trajectory_tree_->clear();
  updateJointStates();
  createMotionPlanTable();
  planning_scene_map_[current_planning_scene_name_].getRobotState(robot_state_);
}

void WarehouseViewer::saveCurrentPlanningScene(bool copy)
{
  ROS_INFO_STREAM("Current planning scene id is " << current_planning_scene_name_);
  ROS_INFO_STREAM("Hostname is " << planning_scene_map_[current_planning_scene_name_].getHostName());
 savePlanningScene(planning_scene_map_[current_planning_scene_name_], copy);
  QMessageBox msgBox(QMessageBox::Information, "Saved", "Saved planning scene successfully.");
  msgBox.addButton(QMessageBox::Ok);
  msgBox.exec();
}

void WarehouseViewer::createNewPlanningSceneSlot() {
  createNewPlanningSceneConfirm();
}

bool WarehouseViewer::createNewPlanningSceneConfirm()
{
  if(planning_scene_initialized_) {
    QMessageBox msgBox(QMessageBox::Warning, "Create New Planning Scene", "Are you sure you want to create a new planning scene? Unsaved changes to the current planning scene will be lost.");
    QPushButton *createButton= msgBox.addButton("Create New Without Saving Changes", QMessageBox::ActionRole);
    QPushButton *saveCreateButton = msgBox.addButton("Save Changes Before Creating New", QMessageBox::ActionRole);
    msgBox.addButton(QMessageBox::Cancel);

    msgBox.exec();
    if (msgBox.clickedButton() == saveCreateButton)
    {
      saveCurrentPlanningScene(false);
      setCurrentPlanningScene(createNewPlanningScene(), true, true);
      motion_plan_tree_->clear();
      trajectory_tree_->clear();
      ROS_INFO("Created a new planning scene: %s", current_planning_scene_name_.c_str());
    }
    else if (msgBox.clickedButton() != createButton)
    {
      return false;
    }
  }
  setCurrentPlanningScene(createNewPlanningScene(), true, true);
  planning_scene_initialized_ = true;
  motion_plan_tree_->clear();
  trajectory_tree_->clear();
  ROS_INFO("Created a new planning scene: %s", current_planning_scene_name_.c_str());
  return true;
}

void WarehouseViewer::updateState()
{
  emit updateTables();
}

void WarehouseViewer::updateStateTriggered()
{
  lockScene();
  createMotionPlanTable();
  createTrajectoryTable();
  unlockScene();
}

void WarehouseViewer::primaryPlannerTriggered()
{
  if(set_primary_planner_action_->isChecked()) {
    set_secondary_planner_action_->setChecked(false);
    use_interpolated_planner_ = false;
  } else if(!set_secondary_planner_action_->isChecked()){
    set_primary_planner_action_->setChecked(true);
  }
}

void WarehouseViewer::secondaryPlannerTriggered()
{
  if(set_secondary_planner_action_->isChecked()) {
    set_primary_planner_action_->setChecked(false);
    use_interpolated_planner_ = true;
  } else if(!set_primary_planner_action_->isChecked()){
    set_secondary_planner_action_->setChecked(true);
  } 
}

void WarehouseViewer::popupLoadPlanningScenes()
{
  if(!warehouse_data_loaded_once_)
  {
    refreshButtonPressed();
  }
  load_planning_scene_dialog_->exec();
}

void WarehouseViewer::refreshButtonPressed()
{
  if(table_load_thread_ != NULL)
  {
    table_load_thread_->wait();
    delete table_load_thread_;
    table_load_thread_ = NULL;
  }
  table_load_thread_= new TableLoadThread(this);
  table_load_thread_->start();
}

void WarehouseViewer::newButtonPressed() 
{
  if(createNewPlanningSceneConfirm()) {
    load_planning_scene_dialog_->close();
  }
}

void WarehouseViewer::loadButtonPressed()
{
  if(planning_scene_initialized_) {
    QMessageBox msgBox(QMessageBox::Information, "Load New Planning Scene", "Are you sure you want to load a new planning scene? Unsaved changes to the current planning scene will be lost.");
    QPushButton *createButton= msgBox.addButton("Load New Without Saving Changes", QMessageBox::ActionRole);
    QPushButton *saveCreateButton = msgBox.addButton("Save Changes Before Loading New", QMessageBox::ActionRole);
    msgBox.addButton(QMessageBox::Cancel);
    
    msgBox.exec();
    
    if (msgBox.clickedButton() == saveCreateButton)
    {
      saveCurrentPlanningScene(false);
    }
    else if (msgBox.clickedButton() != createButton)
    {
      return;
    }
  }

  QList<QTableWidgetItem*> items = planning_scene_table_->selectedItems();

  if(items.size() > 0)
  {
    QTableWidgetItem* item = items[0];
    QTableWidgetItem* nameItem = planning_scene_table_->item(item->row(),1);
    PlanningSceneData& data = planning_scene_map_[nameItem->text().toStdString()];
    loadPlanningScene(data.getTimeStamp(), data.getId());
    //this will blow away the above reference, so redoing
    data = planning_scene_map_[nameItem->text().toStdString()];
    setCurrentPlanningScene(nameItem->text().toStdString(), load_motion_plan_requests_box_->isChecked(), load_trajectories_box_->isChecked());
    trajectory_tree_->clear();
    updateJointStates();
    createMotionPlanTable();
    data.getRobotState(robot_state_);
    planning_scene_initialized_ = true;
    load_planning_scene_dialog_->close();
  }
}

void WarehouseViewer::removePlanningSceneButtonPressed()
{
  QMessageBox msgBox(QMessageBox::Information, "Remove Planning Scene", "This will permanently remove the indicated planning scene(s) and all associated data from the warehouse.");
  msgBox.addButton(QMessageBox::Cancel);

  QPushButton *deleteButton= msgBox.addButton("Remove", QMessageBox::ActionRole);

  msgBox.exec();

  if (msgBox.clickedButton() != deleteButton)
  {
    return;
  }

  QList<QTableWidgetItem*> items = planning_scene_table_->selectedItems();
  
  for(int i = 0; i < items.size(); i++) {
    QTableWidgetItem* item = items[i];
    QTableWidgetItem* nameItem = planning_scene_table_->item(item->row(),1);
    PlanningSceneData& data = planning_scene_map_[nameItem->text().toStdString()];
    move_arm_warehouse_logger_reader_->removePlanningSceneAndAssociatedDataFromWarehouse(data.getHostName(), data.getTimeStamp());
    planning_scene_map_.erase(nameItem->text().toStdString());
    planning_scene_table_->removeRow(item->row());
  }
}

void WarehouseViewer::replanButtonPressed()
{
  if(selected_motion_plan_name_ != "")
  {
    unsigned int new_traj;
    planToRequest(motion_plan_map_[selected_motion_plan_name_], new_traj);
    createTrajectoryTable();
    selectTrajectory(getTrajectoryNameFromId(new_traj));
    playTrajectory(motion_plan_map_[selected_motion_plan_name_], 
                   trajectory_map_[selected_motion_plan_name_][getTrajectoryNameFromId(new_traj)]);
  }
  else
  {
    QMessageBox msg(QMessageBox::Warning, "Plan Trajectory", "No Motion Plan Request Selected!");
    msg.addButton("Ok", QMessageBox::AcceptRole);
    msg.exec();
  }
}

void WarehouseViewer::trajectoryTableSelection()
{
  QList<QTreeWidgetItem*> selected = trajectory_tree_->selectedItems();

  if(selected.size() > 0)
  {
    QString name = selected[0]->toolTip(0);
    selectTrajectory(name.toStdString());
  }
}

void WarehouseViewer::selectTrajectory(std::string ID)
{
  selected_trajectory_name_ = ID;
  TrajectoryData& trajectory = trajectory_map_[selected_motion_plan_name_][selected_trajectory_name_];
  trajectory_slider_->setMaximum((int)trajectory.getTrajectory().points.size() - 1);
  trajectory_slider_->setMinimum(0);
  trajectory_slider_->setValue(trajectory.getCurrentPoint());

  trajectory_point_edit_->setRange(0,(int)trajectory.getTrajectory().points.size()-1);
  trajectory_point_edit_->setValue(trajectory.getCurrentPoint());

  std::stringstream ss;
  ss << trajectory.trajectory_error_code_.val;
  selected_trajectory_label_->setText(QString::fromStdString(ID + " Error Code : " + armNavigationErrorCodeToString(trajectory.trajectory_error_code_) + " (" + ss.str().c_str()+ ")"));
}

void WarehouseViewer::playButtonPressed()
{
  if(selected_trajectory_name_ != "")
  {
    TrajectoryData& trajectory = trajectory_map_[selected_motion_plan_name_][selected_trajectory_name_];
    playTrajectory(motion_plan_map_[getMotionPlanRequestNameFromId(trajectory.getMotionPlanRequestId())], 
                   trajectory);
    std::stringstream ss;
    ss << trajectory.trajectory_error_code_.val;
    selected_trajectory_label_->setText(QString::fromStdString(selected_trajectory_name_ + " Error Code : " + armNavigationErrorCodeToString(trajectory.trajectory_error_code_) + " (" + ss.str().c_str()+ ")"));

    // Set checkbox to visible.
    for(int i = 0; i < trajectory_tree_->topLevelItemCount(); i++)
    {
      QTreeWidgetItem* item = trajectory_tree_->topLevelItem(i);

      if(item->text(0).toStdString() == selected_trajectory_name_)
      {
        for(int j = 0; j < item->childCount(); j++)
        {
          QTreeWidgetItem* child = item->child(j);
          QCheckBox* box = dynamic_cast<QCheckBox*>(trajectory_tree_->itemWidget(child, 0));

          if(box != NULL)
          {
            if(box->text().toStdString() == "Visible")
            {
              box->setChecked(true);
              break;
            }
          }
        }
        break;
      }
    }

  }
  else
  {
    QMessageBox msg(QMessageBox::Warning, "Play Trajectory", "No Trajectory Selected!");
    msg.addButton("Ok", QMessageBox::AcceptRole);
    msg.exec();
  }
}

void WarehouseViewer::filterButtonPressed()
{
  if(selected_trajectory_name_ != "")
  {
    TrajectoryData& trajectory = trajectory_map_[selected_motion_plan_name_][selected_trajectory_name_];
    unsigned int filterID;
    MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(trajectory.getMotionPlanRequestId())];
    filterTrajectory(data,trajectory, filterID);
    playTrajectory(data, trajectory_map_[selected_motion_plan_name_][getTrajectoryNameFromId(filterID)]);
    std::stringstream ss;
    ss << trajectory.trajectory_error_code_.val;
    selected_trajectory_label_->setText(QString::fromStdString(selected_trajectory_name_ + " Error Code : " + armNavigationErrorCodeToString(trajectory.trajectory_error_code_) + " (" + ss.str().c_str()+ ")"));
    createTrajectoryTable();
  }
  else
  {
    QMessageBox msg(QMessageBox::Warning, "Filter Trajectory", "No Trajectory Selected!");
    msg.addButton("Ok", QMessageBox::AcceptRole);
    msg.exec();
  }
}

void WarehouseViewer::sliderDragged(int nv)
{
  if(selected_trajectory_name_ != "")
  {
    trajectory_point_edit_->setValue(nv);
  }
  else
  {
    QMessageBox msg(QMessageBox::Warning, "Control Trajectory", "No Trajectory Selected!");
    msg.addButton("Ok", QMessageBox::AcceptRole);
    msg.exec();
  }
}

void WarehouseViewer::createTrajectoryTable()
{
  if(selected_motion_plan_name_ == "")
  {
    trajectory_tree_->clear();
    trajectory_tree_->setColumnCount(2);
    trajectory_tree_->setColumnWidth(0, 200);
    selected_trajectory_name_ = "";
    selected_trajectory_label_->setText("None");
    trajectory_slider_->setMaximum(0);
    trajectory_slider_->setMinimum(0);
    trajectory_point_edit_->setRange(0,0);
    return;
  }

  if(!hasTrajectory(selected_motion_plan_name_,
                    selected_trajectory_name_)) {
    selected_trajectory_name_ = "";
    selected_trajectory_label_->setText("None");
    trajectory_slider_->setMaximum(0);
    trajectory_slider_->setMinimum(0);
    trajectory_point_edit_->setRange(0,0);
  }

  if(motion_plan_map_.find(selected_motion_plan_name_) == motion_plan_map_.end()) {
    ROS_INFO_STREAM("Going to be generating empty MPR");
  }

  MotionPlanRequestData& data = motion_plan_map_[selected_motion_plan_name_];
  trajectory_tree_->clear();
  trajectory_tree_->setColumnCount(2);
  trajectory_tree_->setColumnWidth(0, 200);

  unsigned int count = 0;
  for(std::set<unsigned int>::iterator it = data.getTrajectories().begin();
      it != data.getTrajectories().end();
      it++, count++) {
    TrajectoryData& trajectory = 
      trajectory_map_[selected_motion_plan_name_][getTrajectoryNameFromId(*it)];

    QTreeWidgetItem* nameItem = new QTreeWidgetItem(QStringList(QString::fromStdString(trajectory.getName())));
    nameItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    nameItem->setToolTip(0, nameItem->text(0));
    trajectory_tree_->insertTopLevelItem(count, nameItem);

    QStringList sourceList;
    sourceList.append("Source");
    sourceList.append(QStringList(QString::fromStdString(trajectory.getSource())));
    QTreeWidgetItem* sourceItem = new QTreeWidgetItem(sourceList);
    sourceItem->setToolTip(0, nameItem->text(0));
    sourceItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    nameItem->insertChild(0, sourceItem);

    std::stringstream durationStream;
    durationStream << (float)trajectory.getDuration().toSec() << " seconds";

    QStringList durationList;
    if(trajectory.getSource() == "Planner" || trajectory.getSource() == "planner")
    {
      durationList.append("Planning Time");
    }
    else if (trajectory.getSource() == "Trajectory Filterer" || trajectory.getSource() == "filter")
    {
      durationList.append("Filter Time");
    }
    else if(trajectory.getSource() == "Robot Monitor" || trajectory.getSource() == "monitor")
    {
      durationList.append("Execution Time");
    }
    else
    {
      durationList.append("Duration");
    }
    durationList.append(QString::fromStdString(durationStream.str()));
    QTreeWidgetItem* durationItem = new QTreeWidgetItem(durationList);
    durationItem->setToolTip(0, nameItem->text(0));
    durationItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    nameItem->insertChild(1, durationItem);

    QStringList collisionList;
    collisionList.append("");
    QTreeWidgetItem* collisionItem = new QTreeWidgetItem(collisionList);
    collisionItem->setToolTip(0, nameItem->text(0));
    QCheckBox* collisionsVisibleBox = new QCheckBox(trajectory_tree_);
    collisionsVisibleBox->setText("Show Collisions");
    collisionsVisibleBox->setChecked(trajectory.areCollisionsVisible());
    collisionsVisibleBox->setToolTip(nameItem->text(0));
    nameItem->insertChild(2, collisionItem);
    trajectory_tree_->setItemWidget(collisionItem, 0, collisionsVisibleBox);
    connect(collisionsVisibleBox, SIGNAL(clicked(bool)), this, SLOT(trajectoryCollisionsVisibleButtonClicked(bool)));

    QStringList visibleList;
    visibleList.append("");
    QTreeWidgetItem* visibleItem = new QTreeWidgetItem(visibleList);
    visibleItem->setToolTip(0, nameItem->text(0));
    QCheckBox* visibleBox = new QCheckBox(trajectory_tree_);
    visibleBox->setText("Visible");
    visibleBox->setChecked(trajectory.isVisible());
    nameItem->insertChild(3, visibleItem);
    trajectory_tree_->setItemWidget(visibleItem, 0, visibleBox);
    visibleBox->setToolTip(nameItem->text(0));
    connect(visibleBox, SIGNAL(clicked(bool)), this, SLOT(trajectoryVisibleButtonClicked(bool)));

    QStringList colorList;
    colorList.append("Color");
    colorList.append("");
    QTreeWidgetItem* colorItem = new QTreeWidgetItem(colorList);
    colorItem->setToolTip(0, nameItem->text(0));
    QPushButton* colorButton = new QPushButton(trajectory_tree_);

    std::stringstream colorStream;
    colorStream<< "(" << (int)(trajectory.getColor().r*255) <<" , ";
    colorStream << (int)(trajectory.getColor().g*255) << " , ";
    colorStream << (int)(trajectory.getColor().b*255) << ")";

    colorButton->setText(QString::fromStdString(colorStream.str()));
    colorButton->setToolTip(nameItem->text(0));
    colorButton->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    connect(colorButton, SIGNAL(clicked()), this, SLOT(trajectoryColorButtonClicked()));

    nameItem->insertChild(4, colorItem);
    trajectory_tree_->setItemWidget(colorItem, 1, colorButton);

    QStringList renderTypeList;
    renderTypeList.append("Render Mode");
    QTreeWidgetItem* renderTypeItem = new QTreeWidgetItem(renderTypeList);
    renderTypeItem->setToolTip(0, nameItem->text(0));

    QComboBox* renderTypeBox = new QComboBox(trajectory_tree_);
    QStringList items;
    items.append("Collision Mesh");
    items.append("Visual Mesh");
    items.append("Padding Mesh");
    renderTypeBox->addItems(items);
    renderTypeBox->setToolTip(nameItem->text(0));
    connect(renderTypeBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(trajectoryRenderTypeChanged(const QString&)));

    nameItem->insertChild(5, renderTypeItem);
    trajectory_tree_->setItemWidget(renderTypeItem, 1, renderTypeBox);

    switch(trajectory.getRenderType())
    {
      case CollisionMesh:
        renderTypeBox->setCurrentIndex(0);
        break;
      case VisualMesh:
        renderTypeBox->setCurrentIndex(1);
        break;
      case PaddingMesh:
        renderTypeBox->setCurrentIndex(2);
        break;
    }
  }

}

void WarehouseViewer::trajectoryCollisionsVisibleButtonClicked(bool checked)
{
  QObject* sender = QObject::sender();
  QCheckBox* button = qobject_cast<QCheckBox*> (sender);

  if(button != NULL)
  {
    std::string trajectoryID = button->toolTip().toStdString();
    TrajectoryData& data = trajectory_map_[selected_motion_plan_name_][trajectoryID];
    data.setCollisionsVisible(checked);
  }
}

void WarehouseViewer::trajectoryVisibleButtonClicked(bool checked)
{
  QObject* sender = QObject::sender();
  QCheckBox* button = qobject_cast<QCheckBox*> (sender);

  if(button != NULL)
  {
    std::string trajectoryID = button->toolTip().toStdString();
    TrajectoryData& data = trajectory_map_[selected_motion_plan_name_][trajectoryID];
    data.setVisible(checked);
  }
}

void WarehouseViewer::trajectoryColorButtonClicked()
{
  QObject* sender = QObject::sender();
  QPushButton* button = qobject_cast<QPushButton*>(sender);

  if(button != NULL)
  {
    std::string trajectoryID = button->toolTip().toStdString();
    TrajectoryData& data = trajectory_map_[selected_motion_plan_name_][trajectoryID];
    QColor col(data.getColor().r*255, data.getColor().g*255, data.getColor().b*255);
    QColor colorSelected = QColorDialog::getColor(col, this);
    if(!colorSelected.isValid())
    {
      return;
    }
    std::stringstream colorStream;
    colorStream<< "(" << colorSelected.red()<<" , ";
    colorStream << colorSelected.green()<< " , ";
    colorStream << colorSelected.blue() << ")";

    button->setText(QString::fromStdString(colorStream.str()));

    std_msgs::ColorRGBA trajColor;
    trajColor.r = colorSelected.redF();
    trajColor.g = colorSelected.greenF();
    trajColor.b = colorSelected.blueF();
    trajColor.a = 0.5f;
    data.setColor(trajColor);
    data.refreshColors();
  }
}

void WarehouseViewer::onPlanningSceneLoaded(int scene, int numScenes)
{
  emit changeProgress((int)((100.0f)*((float)(scene + 1)/ (float)numScenes)));
}

void WarehouseViewer::createPlanningSceneTable()
{
  loadAllWarehouseData();
  warehouse_data_loaded_once_ = true;
  planning_scene_table_->clear();
  ROS_INFO_STREAM("Running clear");
  int count = 0;
  for(map<string, PlanningSceneData>::iterator it = planning_scene_map_.begin(); it != planning_scene_map_.end(); it++)
  {
    count ++;
  }
  planning_scene_table_->setRowCount(count);
  planning_scene_table_->setColumnCount(4);
  QStringList labels;
  labels.append("Host");
  labels.append("Name");
  labels.append("Timestamp");
  labels.append("Notes");

  ROS_INFO("Num Planning Scenes: %d", planning_scene_table_->rowCount());

  int r = 0;
  for(map<string, PlanningSceneData>::iterator it = planning_scene_map_.begin(); it != planning_scene_map_.end(); it++)
  {
    PlanningSceneData& data = it->second;

    QTableWidgetItem* hostItem = new QTableWidgetItem(QString::fromStdString(data.getHostName()));
    hostItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    planning_scene_table_->setItem(r, 0, hostItem);

    QTableWidgetItem* nameItem = new QTableWidgetItem(QString::fromStdString(data.getName()));
    nameItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    planning_scene_table_->setItem(r, 1, nameItem);

    QDateTime time;
    time.setTime_t((unsigned int)(data.getTimeStamp().toSec()));
    stringstream timestampStream;
    timestampStream << time.toString().toStdString();

    QTableWidgetItem* timeItem = new QTableWidgetItem(QString::fromStdString(timestampStream.str()));
    timeItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    planning_scene_table_->setItem(r, 2, timeItem);

    stringstream notesStream;

    vector<ArmNavigationErrorCodes>& errorCodes = data.getErrorCodes();


    if(errorCodes.size() == 0)
    {
      notesStream << "No Outcomes";
    }
    else
    {
      notesStream << "Last Outcome: " << armNavigationErrorCodeToString(errorCodes.back());
    }

    QTableWidgetItem* notesItem = new QTableWidgetItem(QString::fromStdString(notesStream.str()));
    notesItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    planning_scene_table_->setItem(r, 3, notesItem);

    if(errorCodes.size() != 0)
    {
      if(errorCodes.back().val != ArmNavigationErrorCodes::SUCCESS)
      {
        notesItem->setTextColor(QColor(180, 0, 0));
      }
    }

    r++;
  }

  planning_scene_table_->sortByColumn(1, Qt::AscendingOrder);
  connect(planning_scene_table_->horizontalHeader(), SIGNAL(sectionClicked(int)), this, SLOT(planningSceneTableHeaderClicked(int)));
  planning_scene_table_->horizontalHeader()->resizeSections(QHeaderView::Stretch);
  planning_scene_table_->setHorizontalHeaderLabels(labels);
  planning_scene_table_->setMinimumWidth(1000);

  emit allScenesLoaded();
}

void WarehouseViewer::refreshPlanningSceneDialog()
{
  planning_scene_table_->horizontalHeader()->resizeSections(QHeaderView::Stretch);
}

void WarehouseViewer::planningSceneTableHeaderClicked(int col) {
  planning_scene_table_->sortByColumn(col);
}

void WarehouseViewer::motionPlanJointControlsActiveButtonClicked(bool checked)
{
  QObject* sender = QObject::sender();
  QCheckBox* box = dynamic_cast<QCheckBox*>(sender);

  if(box == NULL)
  {
    return;
  }

  std::string ID = box->toolTip().toStdString();

  if(motion_plan_map_.find(ID) == motion_plan_map_.end())
  {
    return;
  }

  MotionPlanRequestData& data = motion_plan_map_[ID];
  data.setJointControlsVisible(checked, this);
  interactive_marker_server_->applyChanges();
}

void WarehouseViewer::createNewObjectPressed()
{
  collision_object_name_->setText(generateNewCollisionObjectId().c_str());
  new_object_dialog_->show();
}

void WarehouseViewer::createNewObjectDialog()
{
  new_object_dialog_ = new QDialog(this);
  QVBoxLayout* layout = new QVBoxLayout(new_object_dialog_);
  QGroupBox* panel = new QGroupBox(new_object_dialog_);
  panel->setTitle("New Collision Object");

  QVBoxLayout* panelLayout = new QVBoxLayout(panel);

  collision_object_name_ = new QLineEdit(new_object_dialog_);
  panelLayout->addWidget(collision_object_name_);

  collision_object_type_box_ = new QComboBox(new_object_dialog_);
  collision_object_type_box_->addItem("Box");
  collision_object_type_box_->addItem("Cylinder");
  collision_object_type_box_->addItem("Sphere");
  collision_object_type_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  panelLayout->addWidget(collision_object_type_box_);

  QGroupBox* scaleBox = new QGroupBox(panel);
  scaleBox->setTitle("Scale (x,y,z) cm");
  QHBoxLayout* scaleLayout = new QHBoxLayout(scaleBox);

  collision_object_scale_x_box_ = new QSpinBox(scaleBox);
  collision_object_scale_y_box_ = new QSpinBox(scaleBox);
  collision_object_scale_z_box_ = new QSpinBox(scaleBox);
  collision_object_scale_x_box_->setRange(1, 10000);
  collision_object_scale_y_box_->setRange(1, 10000);
  collision_object_scale_z_box_->setRange(1, 10000);
  collision_object_scale_x_box_->setValue(10);
  collision_object_scale_y_box_->setValue(10);
  collision_object_scale_z_box_->setValue(10);
  collision_object_scale_x_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_scale_y_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_scale_z_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  scaleLayout->addWidget(collision_object_scale_x_box_);
  scaleLayout->addWidget(collision_object_scale_y_box_);
  scaleLayout->addWidget(collision_object_scale_z_box_);

  scaleBox->setLayout(scaleLayout);
  panelLayout->addWidget(scaleBox);

  QGroupBox* posBox = new QGroupBox(panel);
  posBox->setTitle("Position (x,y,z) cm");
  QHBoxLayout* posLayout = new QHBoxLayout(posBox);


  collision_object_pos_x_box_ = new QSpinBox(posBox);
  collision_object_pos_y_box_ = new QSpinBox(posBox);
  collision_object_pos_z_box_ = new QSpinBox(posBox);
  collision_object_pos_x_box_->setRange(-10000, 10000);
  collision_object_pos_y_box_->setRange(-10000, 10000);
  collision_object_pos_z_box_->setRange(-10000, 10000);
  collision_object_pos_x_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_pos_y_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  collision_object_pos_z_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  posLayout->addWidget(collision_object_pos_x_box_);
  posLayout->addWidget(collision_object_pos_y_box_);
  posLayout->addWidget(collision_object_pos_z_box_);

  posBox->setLayout(posLayout);
  panelLayout->addWidget(posBox);

  panel->setLayout(panelLayout);
  layout->addWidget(panel);

  object_color_button_ = new QPushButton(new_object_dialog_);

  std::stringstream colorStream;
  colorStream<< "Color: (" << (int)(last_collision_object_color_.r*255) <<" , ";
  colorStream << (int)(last_collision_object_color_.g*255) << " , ";
  colorStream << (int)(last_collision_object_color_.b*255) << ")";

  object_color_button_->setText(QString::fromStdString(colorStream.str()));
  object_color_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  connect(object_color_button_, SIGNAL(clicked()), this, SLOT(objectColorButtonPressed()));

  make_object_button_ = new QPushButton(new_object_dialog_);
  make_object_button_->setText("Create...");
  make_object_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  connect(make_object_button_, SIGNAL(clicked()), this, SLOT(createObjectConfirmedPressed()));

  layout->addWidget(object_color_button_);
  layout->addWidget(make_object_button_);
  new_object_dialog_->setLayout(layout);

}

void WarehouseViewer::createNewMeshPressed()
{
  mesh_object_name_->setText(generateNewCollisionObjectId().c_str());
  new_mesh_dialog_->show();
}

void WarehouseViewer::meshFileSelected(const QString& filename)
{
  mesh_filename_field_->setText(filename);
}

void WarehouseViewer::createMeshConfirmedPressed()
{
  geometry_msgs::Pose pose;
  pose.position.x = (float)mesh_object_pos_x_box_->value() / 100.0f;
  pose.position.y = (float)mesh_object_pos_y_box_->value() / 100.0f;
  pose.position.z = (float)mesh_object_pos_z_box_->value() / 100.0f;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  createMeshObject(mesh_object_name_->text().toStdString(), pose, "file://"+mesh_filename_field_->text().toStdString(), last_mesh_object_color_);
  new_mesh_dialog_->hide();
}

void WarehouseViewer::createNewMeshDialog()
{
  new_mesh_dialog_ = new QDialog(this);
  QVBoxLayout* layout = new QVBoxLayout(new_mesh_dialog_);
  QGroupBox* panel = new QGroupBox(new_mesh_dialog_);
  panel->setTitle("New Mesh Collision Object");

  QVBoxLayout* panelLayout = new QVBoxLayout(panel);
  mesh_object_name_ = new QLineEdit(this);
  panelLayout->addWidget(mesh_object_name_);

  mesh_filename_field_ = new QLineEdit(this);
  mesh_filename_field_->setText("<mesh_filename>");
  panelLayout->addWidget(mesh_filename_field_);

  QPushButton* selectFileButton = new QPushButton(this);
  selectFileButton->setText("Select Mesh file ...");

  file_selector_ = new QFileDialog(this);
  file_selector_->setFileMode(QFileDialog::ExistingFile);
  file_selector_->setOption(QFileDialog::ReadOnly, true);
  QStringList filters;
  filters << "Mesh files (*.stl *.stla *.stlb *.dae)";
  file_selector_->setNameFilters(filters);

  connect(file_selector_, SIGNAL(fileSelected(const QString&)), this, SLOT(meshFileSelected(const QString&)));
  connect(selectFileButton, SIGNAL(clicked()), file_selector_, SLOT(open()));
  panelLayout->addWidget(selectFileButton);

  QGroupBox* posBox = new QGroupBox(panel);
  posBox->setTitle("Position (x,y,z) cm");
  QHBoxLayout* posLayout = new QHBoxLayout(posBox);

  mesh_object_pos_x_box_ = new QSpinBox(posBox);
  mesh_object_pos_y_box_ = new QSpinBox(posBox);
  mesh_object_pos_z_box_ = new QSpinBox(posBox);
  mesh_object_pos_x_box_->setRange(-10000, 10000);
  mesh_object_pos_y_box_->setRange(-10000, 10000);
  mesh_object_pos_z_box_->setRange(-10000, 10000);
  mesh_object_pos_x_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  mesh_object_pos_y_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  mesh_object_pos_z_box_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);

  posLayout->addWidget(mesh_object_pos_x_box_);
  posLayout->addWidget(mesh_object_pos_y_box_);
  posLayout->addWidget(mesh_object_pos_z_box_);

  posBox->setLayout(posLayout);
  panelLayout->addWidget(posBox);

  panel->setLayout(panelLayout);
  layout->addWidget(panel);

  mesh_color_button_ = new QPushButton(new_mesh_dialog_);

  std::stringstream colorStream;
  colorStream<< "Color: (" << (int)(last_mesh_object_color_.r*255) <<" , ";
  colorStream << (int)(last_mesh_object_color_.g*255) << " , ";
  colorStream << (int)(last_mesh_object_color_.b*255) << ")";

  mesh_color_button_->setText(QString::fromStdString(colorStream.str()));
  mesh_color_button_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  connect(mesh_color_button_, SIGNAL(clicked()), this, SLOT(meshColorButtonPressed()));

  make_mesh_button_ = new QPushButton(new_mesh_dialog_);
  make_mesh_button_->setText("Create...");
  make_mesh_button_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  connect(make_mesh_button_, SIGNAL(clicked()), this, SLOT(createMeshConfirmedPressed()));

  layout->addWidget(mesh_color_button_);
  layout->addWidget(make_mesh_button_);
  new_mesh_dialog_->setLayout(layout);

}

void WarehouseViewer::meshColorButtonPressed()
{

  QColor selected = QColorDialog::getColor(QColor(last_mesh_object_color_.r*255, last_mesh_object_color_.g*255, last_mesh_object_color_.b*255), new_object_dialog_);

  if(selected.isValid())
  {
    last_mesh_object_color_.r = selected.redF();
    last_mesh_object_color_.g = selected.greenF();
    last_mesh_object_color_.b = selected.blueF();
    last_mesh_object_color_.a = 1.0;

    std::stringstream colorStream;
    colorStream<< "Color: (" << (int)(last_mesh_object_color_.r*255) <<" , ";
    colorStream << (int)(last_mesh_object_color_.g*255) << " , ";
    colorStream << (int)(last_mesh_object_color_.b*255) << ")";

    mesh_color_button_->setText(QString::fromStdString(colorStream.str()));
  }
}

void WarehouseViewer::objectColorButtonPressed()
{

  QColor selected = QColorDialog::getColor(QColor(last_collision_object_color_.r*255, last_collision_object_color_.g*255, last_collision_object_color_.b*255), new_object_dialog_);

  if(selected.isValid())
  {
    last_collision_object_color_.r = selected.redF();
    last_collision_object_color_.g = selected.greenF();
    last_collision_object_color_.b = selected.blueF();
    last_collision_object_color_.a = 1.0;

    std::stringstream colorStream;
    colorStream<< "Color: (" << (int)(last_collision_object_color_.r*255) <<" , ";
    colorStream << (int)(last_collision_object_color_.g*255) << " , ";
    colorStream << (int)(last_collision_object_color_.b*255) << ")";

    object_color_button_->setText(QString::fromStdString(colorStream.str()));
  }
}

void WarehouseViewer::createObjectConfirmedPressed()
{
  geometry_msgs::Pose pose;
  pose.position.x = (float)collision_object_pos_x_box_->value() / 100.0f;
  pose.position.y = (float)collision_object_pos_y_box_->value() / 100.0f;
  pose.position.z = (float)collision_object_pos_z_box_->value() / 100.0f;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  std::string name = collision_object_type_box_->currentText().toStdString();
  PlanningSceneEditor::GeneratedShape shape;
  if(name == "Box")
  {
    shape = PlanningSceneEditor::Box;
  }
  if(name == "Cylinder")
  {
    shape = PlanningSceneEditor::Cylinder;
  }
  if(name == "Sphere")
  {
    shape = PlanningSceneEditor::Sphere;
  }


  createCollisionObject(collision_object_name_->text().toStdString(),
                        pose, shape, (float)collision_object_scale_x_box_->value() / 100.0f,
                        (float)collision_object_scale_y_box_->value() / 100.0f,
                        (float)collision_object_scale_z_box_->value() / 100.0f, last_collision_object_color_);
  new_object_dialog_->hide();
}

void WarehouseViewer::createRequestDialog()
{
  new_request_dialog_ = new QDialog(this);
  QVBoxLayout* layout = new QVBoxLayout(new_request_dialog_);
  QGroupBox* box = new QGroupBox(new_request_dialog_);
  QVBoxLayout* boxLayout = new QVBoxLayout(box);
  box->setTitle("New Motion Plan Request");
  QLabel* boxLabel = new QLabel(box);
  boxLabel->setText("Planning Group:");
  request_group_name_box_ = new QComboBox(box);

  if(params_.right_arm_group_ != "none")
  {
    request_group_name_box_->addItem(QString::fromStdString(params_.right_arm_group_));
  }

  if(params_.left_arm_group_ != "none")
  {
    request_group_name_box_->addItem(QString::fromStdString(params_.left_arm_group_));
  }

  create_request_from_robot_box_ = new QCheckBox(box);
  create_request_from_robot_box_->setChecked(params_.use_robot_data_);
  create_request_from_robot_box_->setText("Start From Current Robot State");

  boxLayout->addWidget(boxLabel);
  boxLayout->addWidget(request_group_name_box_);
  boxLayout->addWidget(create_request_from_robot_box_);

  QPushButton* createRequestButton = new QPushButton(box);
  createRequestButton->setText("Create ...");
  createRequestButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  boxLayout->addWidget(createRequestButton);

  connect(createRequestButton, SIGNAL(clicked()), this, SLOT(createRequestPressed()));
  layout->addWidget(box);
  new_request_dialog_->setLayout(layout);
}

void WarehouseViewer::createRequestPressed()
{
  std::string group_name = request_group_name_box_->currentText().toStdString();
  std::string end_effector_name = "";

  if(group_name == params_.right_arm_group_)
  {
    end_effector_name = params_.right_ik_link_;
  }
  else if(group_name == params_.left_arm_group_)
  {
    end_effector_name = params_.left_ik_link_;
  }
  else
  {
    return;
  }
  createNewMotionPlanRequest(group_name, end_effector_name);
  createMotionPlanTable();
  new_request_dialog_->close();
}

void WarehouseViewer::trajectoryRenderTypeChanged(const QString& type)
{
  QObject* sender = QObject::sender();
  QComboBox* box = dynamic_cast<QComboBox*>(sender);
  std::string ID = box->toolTip().toStdString();

  if(!hasTrajectory(selected_motion_plan_name_,
                   ID)) {
    ROS_WARN_STREAM("Changing render type when we don't have trajectory");
    return;
  }

  TrajectoryData& data = trajectory_map_[selected_motion_plan_name_][ID];

  if(type == "Visual Mesh")
  {
    data.setRenderType(VisualMesh);
  }
  else if(type == "Collision Mesh")
  {
    data.setRenderType(CollisionMesh);
  }
  else
  {
    data.setRenderType(PaddingMesh);
  }
}

void WarehouseViewer::motionPlanRenderTypeChanged(const QString& type)
{
  QObject* sender = QObject::sender();
  QComboBox* box = dynamic_cast<QComboBox*>(sender);
  std::string ID = box->toolTip().toStdString();

  if(motion_plan_map_.find(ID) == motion_plan_map_.end())
  {
    return;
  }

  MotionPlanRequestData& data = motion_plan_map_[ID];

  if(type == "Visual Mesh")
  {
    data.setRenderType(VisualMesh);
  }
  else if(type == "Collision Mesh")
  {
    data.setRenderType(CollisionMesh);
  }
  else
  {
    data.setRenderType(PaddingMesh);
  }
}

void WarehouseViewer::planCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode)
{
  if(errorCode.val != ArmNavigationErrorCodes::SUCCESS)
  {
    emit plannerFailure(errorCode.val);
  } else {
    selectTrajectory(selected_trajectory_name_);
  }
}


void WarehouseViewer::filterCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode)
{
  if(errorCode.val != ArmNavigationErrorCodes::SUCCESS)
  {
    emit filterFailure(errorCode.val);
  } else {
    selectTrajectory(selected_trajectory_name_);
  }
}

void WarehouseViewer::attachObject(const std::string& name)
{
  createAttachObjectDialog(name);
  int res = attach_object_dialog_->exec();
  if(res == QDialog::Accepted) {
    std::vector<std::string> touch_links;
    for(int i = 0; i < added_touch_links_->count(); i++) {
      touch_links.push_back(added_touch_links_->item(i)->text().toStdString());
      
    }
    attachCollisionObject(name, attach_link_box_->currentText().toStdString(), touch_links);
    changeToAttached(name);
  }
  delete attach_object_dialog_;
}

void WarehouseViewer::createAttachObjectDialog(const std::string& name)
{
  if(current_planning_scene_name_ == "") {
    return;
  }
  
  arm_navigation_msgs::PlanningScene planning_scene = planning_scene_map_[current_planning_scene_name_].getPlanningScene();

  std::stringstream ss;
  ss << "Attach object " << name;
  
  attach_object_dialog_ = new QDialog(this);
  attach_object_dialog_->setWindowTitle(ss.str().c_str());
  
  QVBoxLayout* layout = new QVBoxLayout(attach_object_dialog_);

  QGroupBox* panel = new QGroupBox(attach_object_dialog_);
  panel->setTitle("Link for attaching");

  QVBoxLayout* panel_layout = new QVBoxLayout(panel);
  attach_link_box_= new QComboBox(attach_object_dialog_);

  std::vector<std::string> link_names;
  cm_->getKinematicModel()->getLinkModelNames(link_names);
  for(unsigned int i = 0; i < link_names.size(); i++) {
    attach_link_box_->addItem(link_names[i].c_str());
  }
  panel_layout->addWidget(attach_link_box_);
  panel->setLayout(panel_layout);
  layout->addWidget(panel);

  QGroupBox* grid_panel = new QGroupBox(attach_object_dialog_);
  QGridLayout* grid = new QGridLayout(grid_panel);
  
  QLabel* all_label = new QLabel(attach_object_dialog_);
  all_label->setText("Links and groups");

  QLabel* touch_links = new QLabel(attach_object_dialog_);
  touch_links->setText("Touch links");

  grid->addWidget(all_label, 0, 0);
  grid->addWidget(touch_links, 0, 2);

  QPushButton* add_button = new QPushButton(attach_object_dialog_);
  add_button->setText("Add ->");
  
  connect(add_button, SIGNAL(clicked()), this, SLOT(addTouchLinkClicked()));

  QPushButton* remove_button = new QPushButton(attach_object_dialog_);
  remove_button->setText("Remove");

  connect(remove_button, SIGNAL(clicked()), this, SLOT(removeTouchLinkClicked()));

  grid->addWidget(add_button, 1, 1, Qt::AlignTop);  
  grid->addWidget(remove_button, 1, 1, Qt::AlignBottom);

  possible_touch_links_ = new QListWidget(attach_object_dialog_);
  possible_touch_links_->setSelectionMode(QAbstractItemView::ExtendedSelection);
  for(unsigned int i = 0; i < link_names.size(); i++) {
    possible_touch_links_->addItem(link_names[i].c_str());
  }
  possible_touch_links_->addItem("------Groups---------");
  std::vector<std::string> group_names;
  cm_->getKinematicModel()->getModelGroupNames(group_names);
  for(unsigned int i = 0; i < group_names.size(); i++) {
    possible_touch_links_->addItem(group_names[i].c_str());
  }
  added_touch_links_ = new QListWidget(attach_object_dialog_);
  added_touch_links_->setSelectionMode(QAbstractItemView::ExtendedSelection);

  grid->addWidget(possible_touch_links_, 1, 0);
  grid->addWidget(added_touch_links_, 1, 2);

  layout->addWidget(grid_panel);

  QDialogButtonBox* qdb = new QDialogButtonBox();
  QPushButton* cancel_button = new QPushButton("Cancel");
  QPushButton* attach_button = new QPushButton("Attach");
  qdb->addButton(cancel_button, QDialogButtonBox::RejectRole);
  qdb->addButton(attach_button, QDialogButtonBox::AcceptRole);
  connect(qdb, SIGNAL(accepted()), attach_object_dialog_, SLOT(accept()));
  connect(qdb, SIGNAL(rejected()), attach_object_dialog_, SLOT(reject()));

  layout->addWidget(qdb, Qt::AlignRight);

  attach_object_dialog_->setLayout(layout);
}

void WarehouseViewer::addTouchLinkClicked() 
{
  QList<QListWidgetItem *> l = possible_touch_links_->selectedItems();

  for(int i = 0; i < l.size(); i++) {
    bool found = false;
    for(int j = 0; j < added_touch_links_->count(); j++) {
      if(l[i]->text() == added_touch_links_->item(j)->text()) {
        found = true;
        break;
      }
    }
    if(!found && l[i]->text().toStdString() != std::string("------Groups---------")) {
      added_touch_links_->addItem(l[i]->text());
    }
  }
}

void WarehouseViewer::removeTouchLinkClicked()
{
  qDeleteAll(added_touch_links_->selectedItems());
}

void WarehouseViewer::editRobotStatePressed()
{
  createRobotStateEditor();
  edit_robot_state_dialog_->exec();
  delete edit_robot_state_dialog_;
}

void WarehouseViewer::createRobotStateEditor() {

  edit_robot_state_dialog_ = new QDialog(this);
  edit_robot_state_dialog_->setWindowTitle("Edit robot state");
  
  QVBoxLayout* layout = new QVBoxLayout(edit_robot_state_dialog_);

  QGroupBox* panel = new QGroupBox(edit_robot_state_dialog_);
  panel->setTitle("Joint state to edit");

  QVBoxLayout* panel_layout = new QVBoxLayout(panel);
  edit_joint_box_= new QComboBox(edit_robot_state_dialog_);

  const std::vector<planning_models::KinematicModel::JointModel*>& joint_models = cm_->getKinematicModel()->getJointModels();
  for(unsigned int i = 0; i < joint_models.size(); i++) {
    if(joint_models[i]->getComputatationOrderMapIndex().size() == 1) {
      edit_joint_box_->addItem(joint_models[i]->getName().c_str());
    }
  }

  QGridLayout* slider_layout = new QGridLayout(panel);

  QLabel* lower_label = new QLabel(edit_robot_state_dialog_);
  lower_label->setText("Lower bound");

  QLabel* upper_label = new QLabel(edit_robot_state_dialog_);
  upper_label->setText("Upper bound");

  QLabel* current_label = new QLabel(edit_robot_state_dialog_);
  current_label->setText("Current value");

  lower_bound_edit_window_ = new QLineEdit(edit_robot_state_dialog_);
  upper_bound_edit_window_ = new QLineEdit(edit_robot_state_dialog_);
  current_value_window_ = new QLineEdit(edit_robot_state_dialog_);

  lower_bound_edit_window_->setReadOnly(true);
  upper_bound_edit_window_->setReadOnly(true);
  lower_bound_edit_window_->setMinimumWidth(125);
  lower_bound_edit_window_->setMaximumWidth(125);
  upper_bound_edit_window_->setMinimumWidth(125);
  upper_bound_edit_window_->setMaximumWidth(125);

  joint_state_slider_ = new QSlider(Qt::Horizontal, edit_robot_state_dialog_);
  joint_state_slider_->setMinimum(0);
  joint_state_slider_->setMaximum(0);

  slider_layout->addWidget(lower_label, 0, 0);
  slider_layout->addWidget(current_label, 0, 1);
  slider_layout->addWidget(upper_label, 0, 2);

  slider_layout->addWidget(lower_bound_edit_window_, 1, 0);
  slider_layout->addWidget(joint_state_slider_, 1, 1);
  slider_layout->addWidget(upper_bound_edit_window_, 1, 2);
  
  slider_layout->addWidget(current_value_window_, 2, 1);

  slider_layout->setColumnStretch(1, 100);

  panel_layout->addWidget(edit_joint_box_);
  panel_layout->addLayout(slider_layout);

  panel->setLayout(panel_layout);
  layout->addWidget(panel);

  edit_robot_state_dialog_->setLayout(layout);

  //getting reasonable stuff in there
  editJointBoxChanged(edit_joint_box_->currentText());

  connect(edit_joint_box_, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(editJointBoxChanged(const QString&)));
  connect(joint_state_slider_, SIGNAL(valueChanged(int)), this, SLOT(jointStateSliderChanged(int)));
}

void WarehouseViewer::editJointBoxChanged(const QString& joint) 
{

  std::map<std::string, double> current_joint_state_values;
  robot_state_->getKinematicStateValues(current_joint_state_values);

  double current_value = current_joint_state_values[joint.toStdString()];

  const planning_models::KinematicModel::JointModel* jm = cm_->getKinematicModel()->getJointModel(joint.toStdString());

  std::pair<double, double> bounds; 
  jm->getVariableBounds(jm->getName(), bounds);

  std::stringstream lb;
  lb << bounds.first;
  std::stringstream ub;
  ub << bounds.second;

  std::stringstream cur;
  cur << current_value;

  lower_bound_edit_window_->setText(lb.str().c_str());
  upper_bound_edit_window_->setText(ub.str().c_str());
  current_value_window_->setText(cur.str().c_str());

  joint_state_slider_->setMinimum(bounds.first*1000);
  joint_state_slider_->setMaximum(bounds.second*1000);
  joint_state_slider_->setValue(current_value*1000);
}

void WarehouseViewer::jointStateSliderChanged(int nv) 
{
  std::map<std::string, double> current_joint_state_values;
  robot_state_->getKinematicStateValues(current_joint_state_values);

  current_joint_state_values[edit_joint_box_->currentText().toStdString()] = nv/1000.0;

  std::stringstream cur;
  cur << nv/1000.0;
  current_value_window_->setText(cur.str().c_str());

  robot_state_->setKinematicState(current_joint_state_values);

  planning_environment::convertKinematicStateToRobotState(*robot_state_,
                                                          ros::Time::now(),
                                                          cm_->getWorldFrameId(),
                                                          planning_scene_map_[current_planning_scene_name_].getPlanningScene().robot_state);
  sendPlanningScene(planning_scene_map_[current_planning_scene_name_]);
}
 
void WarehouseViewer::popupPlannerFailure(int value)
{
  ArmNavigationErrorCodes errorCode;
  errorCode.val = value;
  std::string failure  = "Planning Failed: " + armNavigationErrorCodeToString(errorCode);
  QMessageBox msg(QMessageBox::Critical, "Planning Failed!", QString::fromStdString(failure));
  msg.addButton("Ok", QMessageBox::AcceptRole);
  msg.exec();
}

void WarehouseViewer::popupFilterFailure(int value)
{
  ArmNavigationErrorCodes errorCode;
  errorCode.val = value;
  std::string failure  = "Filter Failed: " + armNavigationErrorCodeToString(errorCode);
  QMessageBox msg(QMessageBox::Critical, "Filter Failed!", QString::fromStdString(failure));
  msg.addButton("Ok", QMessageBox::AcceptRole);
  msg.exec();
}

void marker_function()
{
  unsigned int counter = 0;
  while(ros::ok())
  {
    if(inited)
    {
      if(counter % CONTROL_SPEED == 0)
      {
        counter = 1;
        psv->sendMarkers();
      }
      else
      {
        counter++;
      }

      if(psv->quit_threads_)
      {
        break;
      }
    }
    usleep(5000);
  }

}

void spin_function()
{
  ros::WallRate r(100.0);
  while(ros::ok() && !inited)
  {
    r.sleep();
    ros::spinOnce();
  }
  while(ros::ok() && !psv->quit_threads_)
  {
    r.sleep();
    ros::spinOnce();
  }
}

void quit(int sig)
{
  if(psv != NULL)
  {
    delete psv;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_warehouse_viewer", ros::init_options::NoSigintHandler);

  boost::thread spin_thread(boost::bind(&spin_function));
  boost::thread marker_thread(boost::bind(&marker_function));

  QApplication* app = new QApplication(argc, argv);
  planning_scene_utils::PlanningSceneParameters params;
  param<string>("set_planning_scene_diff_name", params.set_planning_scene_diff_name_, SET_PLANNING_SCENE_DIFF_NAME);
  param<string>("left_ik_name", params.left_ik_name_, LEFT_IK_NAME);
  param<string>("left_interpolate_service_name", params.left_interpolate_service_name_, LEFT_INTERPOLATE_SERVICE_NAME);
  param<string>("non_coll_left_ik_name", params.non_coll_left_ik_name_, NON_COLL_LEFT_IK_NAME);
  param<string>("non_coll_right_ik_name", params.non_coll_right_ik_name_, NON_COLL_RIGHT_IK_NAME);
  param<string>("planner_service_name", params.planner_service_name_, PLANNER_SERVICE_NAME);
  param<string>("proximity_space_planner_name", params.proximity_space_planner_name_, PROXIMITY_SPACE_PLANNER_NAME);
  param<string>("proximity_space_service_name",  params.proximity_space_service_name_, PROXIMITY_SPACE_SERVICE_NAME);
  param<string>("proximity_space_validity_name",  params.proximity_space_validity_name_,  PROXIMITY_SPACE_VALIDITY_NAME);
  param<string>("right_ik_name", params.right_ik_name_, RIGHT_IK_NAME);
  param<string>("right_interpolate_service_name", params.right_interpolate_service_name_, RIGHT_INTERPOLATE_SERVICE_NAME);
  param<string>("trajectory_filter_service_name", params.trajectory_filter_service_name_, TRAJECTORY_FILTER_SERVICE_NAME);
  param<string>("vis_topic_name", params.vis_topic_name_ , VIS_TOPIC_NAME);
  param<string>("right_ik_link", params.right_ik_link_ , RIGHT_IK_LINK);
  param<string>("left_ik_link", params.left_ik_link_ , LEFT_IK_LINK);
  param<string>("right_arm_group", params.right_arm_group_ , RIGHT_ARM_GROUP);
  param<string>("left_arm_group", params.left_arm_group_ , LEFT_ARM_GROUP);
  param<string>("right_redundancy", params.right_redundancy_ , RIGHT_ARM_REDUNDANCY);
  param<string>("left_redundancy", params.left_redundancy_ , LEFT_ARM_REDUNDANCY);
  param<string>("execute_left_trajectory", params.execute_left_trajectory_ , EXECUTE_LEFT_TRAJECTORY);
  param<string>("execute_right_trajectory", params.execute_right_trajectory_ , EXECUTE_RIGHT_TRAJECTORY);
  param<string>("list_controllers_service", params.list_controllers_service_, LIST_CONTROLLERS_SERVICE);
  param<string>("load_controllers_service", params.load_controllers_service_, LOAD_CONTROLLERS_SERVICE);
  param<string>("unload_controllers_service", params.unload_controllers_service_, UNLOAD_CONTROLLERS_SERVICE);
  param<string>("switch_controllers_service", params.switch_controllers_service_, SWITCH_CONTROLLERS_SERVICE);
  param<string>("gazebo_robot_model", params.gazebo_model_name_, GAZEBO_ROBOT_MODEL);
  param<string>("robot_description_param", params.robot_description_param_, ROBOT_DESCRIPTION_PARAM);
  param<bool>("use_robot_data", params.use_robot_data_, false);
  params.sync_robot_state_with_gazebo_ = false;

  ParameterDialog* dialog = new ParameterDialog(params);
  dialog->exec();
  dialog->updateParams();

  QImage image;
  if(chdir(ros::package::getPath("move_arm_warehouse").c_str()) != 0)
  {
    ROS_ERROR("FAILED TO CHANGE PACKAGE TO %s", ros::package::getPath("move_arm_warehouse").c_str());
  }
  if(!image.load("./res/splash.png"))
  {
    ROS_ERROR("FAILED TO LOAD ./res/splash.png");
  }

  QSplashScreen screen(QPixmap::fromImage(image));
  screen.show();
  app->processEvents();
  psv = new WarehouseViewer(NULL, dialog->params_);
  app->setActiveWindow(psv);
  psv->show();
  screen.close();
  inited = true;

  int ret = app->exec();
  psv->quit_threads_ = true;
  spin_thread.join();
  marker_thread.join();
  return ret;

}
