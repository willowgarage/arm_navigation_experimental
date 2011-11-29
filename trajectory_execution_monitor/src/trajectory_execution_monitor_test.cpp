#include <trajectory_execution_monitor/trajectory_execution_monitor.h>
#include <trajectory_execution_monitor/joint_state_recorder.h>
#include <trajectory_execution_monitor/follow_joint_trajectory_controller_handler.h>

using namespace trajectory_execution_monitor;

bool trajectoryFinishedCallbackFunction(TrajectoryExecutionDataVector tedv) {
  ROS_INFO_STREAM("Trajectories " << tedv.size() << " ok " << (tedv.back().result_ == SUCCEEDED));
  for(unsigned int i = 0; i < tedv.size(); i++) {
    ROS_INFO_STREAM("Recorded trajectory " << i << " has " << tedv[i].recorded_trajectory_.points.size());
  }
  return true;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "trajectory_execution_monitor_test");
  //figuring out whether robot_description has been remapped

  ros::AsyncSpinner spinner(4); 
  spinner.start();

  boost::shared_ptr<TrajectoryRecorder> tr(new JointStateTrajectoryRecorder("/joint_states"));
  boost::shared_ptr<TrajectoryControllerHandler> fjt(new FollowJointTrajectoryControllerHandler("right_arm",
                                                                                                "/r_arm_controller/follow_joint_trajectory"));

  TrajectoryExecutionMonitor tem;
  tem.addTrajectoryRecorder(tr);
  tem.addTrajectoryControllerHandler(fjt);

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
  double goal_angle = -.9;

  if(argc > 2) {
    std::stringstream s(argv[1]);
    s >> start_angle;
    std::stringstream g(argv[2]);
    g >> goal_angle;
  }

  ros::Duration r(10.0);

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
  traj_reqs.push_back(ter);

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

  boost::function<bool(TrajectoryExecutionDataVector)> f;
  f = trajectoryFinishedCallbackFunction;

  tem.executeTrajectories(traj_reqs, f);
  
  ros::waitForShutdown();

  return 0;
}
