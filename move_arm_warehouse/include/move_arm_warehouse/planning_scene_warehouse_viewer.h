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

// Author: E. Gil Jones
#ifndef PLANNING_SCENE_WAREHOUSE_H
#define PLANNING_SCENE_WAREHOUSE_H

#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <ros/ros.h>


#include <move_arm_warehouse/move_arm_utils.h>

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
#include <qt4/QtGui/qmessagebox.h>
#include <qt4/QtGui/qmainwindow.h>
#include <QDateTime>


static const std::string VIS_TOPIC_NAME = "planning_scene_visualizer_markers";

//in 100 hz ticks
static const unsigned int CONTROL_SPEED = 5;

static const double BASE_TRANS_SPEED = .3;
static const double BASE_ROT_SPEED = .15;

static const double HAND_TRANS_SPEED = .05;
static const double HAND_ROT_SPEED = .15;

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

static const ros::Duration PLANNING_DURATION = ros::Duration(5.0);

static const std::string SET_PLANNING_SCENE_DIFF_NAME = "environment_server/set_planning_scene_diff";

class ParameterDialog : public QDialog
{
  public:
    planning_scene_utils::PlanningSceneParameters params_;
    ParameterDialog(planning_scene_utils::PlanningSceneParameters params, QWidget* parent = NULL) : QDialog(parent)
    {
      params_  = params;
      setMinimumWidth(640);
      setup();
    }

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
      trajectory_filter_service_name_ ->setText(QString::fromStdString(params_.trajectory_filter_service_name_ ));

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
      use_robot_data_->setChecked(false);
      layout->addRow("Use Data From Simulated/Real Robot?", use_robot_data_);

      QPushButton* button = new QPushButton(groupBox);
      button->setText("Accept");
      button->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
      connect(button, SIGNAL(clicked()), this, SLOT(accept()));

      boxLayout->addWidget(groupBox);
      boxLayout->addWidget(button);
      groupBox->setLayout(layout);
      setLayout(boxLayout);

    }

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
      params_.left_ik_link_= left_ik_link_->text().toStdString();
      params_.right_ik_link_= right_ik_link_->text().toStdString();
      params_.planner_service_name_= planner_service_name_->text().toStdString();
      params_.left_interpolate_service_name_= left_interpolate_service_name_->text().toStdString();
      params_.right_interpolate_service_name_= right_interpolate_service_name_->text().toStdString();
      params_.trajectory_filter_service_name_= trajectory_filter_service_name_->text().toStdString();
      params_.proximity_space_service_name_= proximity_space_service_name_->text().toStdString();
      params_.proximity_space_validity_name_= proximity_space_validity_name_->text().toStdString();
      params_.proximity_space_planner_name_= proximity_space_planner_name_->text().toStdString();
      params_.execute_left_trajectory_ = execute_left_trajectory_->text().toStdString();
      params_.execute_right_trajectory_ = execute_right_trajectory_->text().toStdString();
      params_.use_robot_data_ = use_robot_data_->isChecked();
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


};

class PlanningSceneVisualizer : public QMainWindow, public planning_scene_utils::PlanningSceneEditor
{
    Q_OBJECT
  public:

    bool quit_threads_;

    class TableLoadThread : public QThread
    {
      public:
        PlanningSceneVisualizer* visualizer_;
        TableLoadThread(PlanningSceneVisualizer* visualizer) : QThread(visualizer), visualizer_(visualizer)
        {
        }

        void run()
        {
          visualizer_->createPlanningSceneTable();
        }
    };

    PlanningSceneVisualizer(QWidget* parent, planning_scene_utils::PlanningSceneParameters& params);

    ~PlanningSceneVisualizer();

    void initQtWidgets();
    void setupPlanningSceneDialog();
    void createPlanningSceneTable();
    void createTrajectoryTable();
    void createMotionPlanTable();
    void createNewObjectDialog();
    void createRequestDialog();
    void updateState();
    void onPlanningSceneLoaded(int scene, int numScenes);
    signals:
      void changeProgress(int progress);
      void updateTables();

  public slots:
    void quit();
    void popupLoadPlanningScenes();
    void progressChanged(int progress) { load_scene_progress_->setValue(progress);}
    void loadButtonPressed();
    void refreshButtonPressed();
    void trajectoryTableSelection();
    void motionPlanTableSelection();
    void playButtonPressed();
    void filterButtonPressed();
    void sliderDragged();
    void replanButtonPressed();
    void trajectoryEditChanged();
    void collisionDisplayChanged(const QString& mode);
    void createNewPlanningScenePressed();
    void saveCurrentPlanningScene();
    void createNewMotionPlanRequest(std::string group_name, std::string end_effector_name);
    void motionPlanStartColorButtonClicked();
    void motionPlanEndColorButtonClicked();
    void motionPlanStartVisibleButtonClicked(bool checked);
    void motionPlanEndVisibleButtonClicked(bool checked);
    void trajectoryColorButtonClicked();
    void trajectoryVisibleButtonClicked(bool checked);
    void createNewMotionPlanPressed();
    void createNewObjectPressed();
    void createObjectConfirmedPressed();
    void createRequestPressed();
    void motionPlanCollisionVisibleButtonClicked(bool checked);
    void trajectoryCollisionsVisibleButtonClicked(bool checked);
    void motionPlanJointControlsActiveButtonClicked(bool checked);
    void selectMotionPlan(std::string ID);
    void selectTrajectory(std::string ID);
    void deleteSelectedMotionPlan();
    void deleteSelectedTrajectory();
    void updateStateTriggered();
    void executeButtonPressed();
    void refreshSceneButtonPressed();


  protected:
    QLabel* selected_trajectory_label_;
    QLabel* selected_request_label_;
    QMenuBar* menu_bar_;
    QMenu* file_menu_;
    QMenu* collision_object_menu_;
    QAction* new_object_action_;
    QAction* refresh_action_;
    QDialog* load_planning_scene_dialog_;
    QDialog* new_object_dialog_;
    QDialog* new_request_dialog_;
    QProgressBar* load_scene_progress_;
    QAction* new_planning_scene_action_;
    QAction* new_motion_plan_action_;
    QAction* load_planning_scene_action_;
    QAction* save_planning_scene_action_;
    QAction* quit_action_;
    QTableWidget* planning_scene_table_;
    QTreeWidget* motion_plan_tree_;
    QTreeWidget* trajectory_tree_;
    QSlider* trajectory_slider_;
    QPushButton* play_button_;
    QPushButton* filter_button_;
    QPushButton* replan_button_;
    QPushButton* execute_button_;

    QPushButton* load_planning_scene_button_;
    QPushButton* refresh_planning_scene_button_;
    QComboBox* collision_display_box_;

    QSpinBox* trajectory_point_edit_;

    TableLoadThread* table_load_thread_;

    QComboBox* collision_object_type_box_;
    QComboBox* request_group_name_box_;
    QSpinBox* collision_object_scale_x_box_;
    QSpinBox* collision_object_scale_y_box_;
    QSpinBox* collision_object_scale_z_box_;
    QSpinBox* collision_object_pos_x_box_;
    QSpinBox* collision_object_pos_y_box_;
    QSpinBox* collision_object_pos_z_box_;
    QPushButton* make_object_button_;

    QCheckBox* load_motion_plan_requests_box_;
    QCheckBox* load_trajectories_box_;
    QCheckBox* create_request_from_robot_box_;

};
#endif
