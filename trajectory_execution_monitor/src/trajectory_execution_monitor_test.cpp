#include <trajectory_execution_monitor/trajectory_execution_monitor.h>
#include <trajectory_execution_monitor/joint_state_recorder.h>
#include <trajectory_execution_monitor/follow_joint_trajectory_controller_handler.h>
#include <trajectory_execution_monitor/pr2_gripper_trajectory_controller_handler.h>

using namespace trajectory_execution_monitor;

bool trajectoryFinishedCallbackFunction(TrajectoryExecutionDataVector tedv) {
  ROS_INFO_STREAM("Trajectories " << tedv.size() << " ok " << (tedv.back().result_ == SUCCEEDED));
  for(unsigned int i = 0; i < tedv.size(); i++) {
    ROS_INFO_STREAM("Recorded  trajectory " << i << " has " << tedv[i].recorded_trajectory_.points.size() << " points" );
    ROS_INFO_STREAM("Recorded  trajectory " << i << " has time " << tedv[i].time_ << " seconds" );
    ROS_INFO_STREAM("Recorded  trajectory " << i << " has angular sum " << tedv[i].angular_distance_ << " radians" );
    if(tedv[i].overshoot_trajectory_.points.size() > 0 )
    {
      ROS_INFO_STREAM("Overshoot trajectory " << i << " has " << tedv[i].overshoot_trajectory_.points.size() << " points" );
      ROS_INFO_STREAM("Overshoot trajectory " << i << " has time " << tedv[i].overshoot_time_ << " seconds" );
    }
  }
  return true;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "trajectory_execution_monitor_test");
  //figuring out whether robot_description has been remapped

  ros::AsyncSpinner spinner(4); 
  spinner.start();

  boost::shared_ptr<TrajectoryRecorder> tr(new JointStateTrajectoryRecorder("/joint_states"));
  boost::shared_ptr<TrajectoryControllerHandler> fjt(
      new FollowJointTrajectoryControllerHandler("right_arm",
                                                 "/r_arm_controller/follow_joint_trajectory"));

  boost::shared_ptr<TrajectoryControllerHandler> gripper(
      new Pr2GripperTrajectoryControllerHandler("r_end_effector",
                                                "/r_gripper_controller/gripper_action"));

  TrajectoryExecutionMonitor tem;
  tem.addTrajectoryRecorder(tr);
  tem.addTrajectoryControllerHandler(fjt);
  tem.addTrajectoryControllerHandler(gripper);

  std::vector<TrajectoryExecutionRequest> traj_reqs;

  trajectory_msgs::JointTrajectory jt;
  jt.joint_names.push_back("r_shoulder_pan_joint");
  jt.joint_names.push_back("r_shoulder_lift_joint");
  jt.joint_names.push_back("r_upper_arm_roll_joint");
  jt.joint_names.push_back("r_elbow_flex_joint");
  jt.joint_names.push_back("r_forearm_roll_joint");
  jt.joint_names.push_back("r_wrist_flex_joint");
  jt.joint_names.push_back("r_wrist_roll_joint");

  jt.points.resize(100);

  double start_angle = 0.0;
  double goal_angle = -.5;

  if(argc > 2) {
    std::stringstream s(argv[1]);
    s >> start_angle;
    std::stringstream g(argv[2]);
    g >> goal_angle;
  }

  ros::Duration r(3.0);

  jt.header.stamp = ros::Time::now();

  for(unsigned int i=0; i < 100; i++)
  {    
    jt.points[i].positions.push_back(start_angle+(i*1.0)*(goal_angle-start_angle)/100.0);
    jt.points[i].velocities.push_back(0.0);
    for(unsigned int j = 0; j < 6; j++) {
      jt.points[i].positions.push_back(0.0);
      jt.points[i].velocities.push_back(0.0);
    }
    jt.points[i].time_from_start = ros::Duration((i*1.0)*r.toSec()/(100.0));
  }

  TrajectoryExecutionRequest ter;
  ter.group_name_="right_arm";
  ter.controller_name_ = "/r_arm_controller/follow_joint_trajectory";
  ter.recorder_name_ = "/joint_states";
  ter.trajectory_ = jt;
  ter.test_for_close_enough_ = true;
  ter.monitor_overshoot_ = true;
  ter.max_overshoot_time_ = ros::Duration(20);
  ter.min_overshoot_time_ = ros::Duration(0.5);
  ter.max_overshoot_velocity_epsilon_ = 0.01;
  ter.max_joint_distance_ = 0.001;
  ter.failure_time_factor_ = 1.5;
  traj_reqs.push_back(ter);

  trajectory_msgs::JointTrajectory gt;
  gt.joint_names.push_back("r_gripper_l_finger_joint");
  gt.joint_names.push_back("r_gripper_r_finger_joint");
  gt.joint_names.push_back("r_gripper_r_finger_tip_joint");
  gt.joint_names.push_back("r_gripper_l_finger_tip_joint");

  gt.points.resize(1);
  gt.points[0].positions.resize(4, .1);

  TrajectoryExecutionRequest gter;
  gter.group_name_="r_end_effector";
  gter.controller_name_ = "/r_gripper_controller/gripper_action";
  gter.recorder_name_ = "/joint_states";
  gter.trajectory_ = gt;
  traj_reqs.push_back(gter);
  
  jt.points.clear();
  jt.points.resize(100);
  for(unsigned int i=0; i < 100; i++)
  {    
    jt.points[i].positions.push_back(goal_angle+(i*1.0)*(start_angle-goal_angle)/100.0);
    jt.points[i].velocities.push_back(0.0);
    for(unsigned int j = 0; j < 6; j++) {
      jt.points[i].positions.push_back(0.0);
      jt.points[i].velocities.push_back(0.0);
    }
    jt.points[i].time_from_start = ros::Duration((i*1.0)*r.toSec()/(100.0));
  }

  ter.trajectory_ = jt;
  traj_reqs.push_back(ter);

  gt.points[0].positions.clear();
  gt.points[0].positions.resize(4, 0.0);
  gter.trajectory_ = gt;
  traj_reqs.push_back(gter);

  boost::function<bool(TrajectoryExecutionDataVector)> f;
  f = trajectoryFinishedCallbackFunction;

  tem.executeTrajectories(traj_reqs, f);
  
  ros::waitForShutdown();

  return 0;
}
