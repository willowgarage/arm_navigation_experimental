#! /usr/bin/env python

import sys
import yaml
import roslib
import re

# Expect robot name as argument
if(len(sys.argv) != 2):
  print("Usage: create_launch_files <your_robot_name>")
  sys.exit(-1)

robotName = sys.argv[1]
directoryName = roslib.packages.get_pkg_dir(robotName + "_arm_navigation")

# Plnning description yaml file
yamlFileName = directoryName + '/config/' + robotName + '_planning_description.yaml'

# Get dictionary of yaml defs
data = yaml.safe_load(open(yamlFileName, 'r'))

#getting world frame for rviz
multi_dof = data['multi_dof_joints']
world_joint = multi_dof[0]
world_frame = world_joint['parent_frame_id']

# List of groups
groups = data['groups']

if(len(groups) < 1) :
  print("Need at least 1 planning group in the robot to continue!")
  sys.exit(-1)

# Substitution strings
left_arm = None
right_arm = None
left_arm_name = 'none'
right_arm_name = 'none'
left_arm_ik = 'none'
right_arm_ik = 'none'

done_left = False
done_right = False

for i in groups:
  if 'tip_link' in i:
    if not done_left:
      left_arm = i
      done_left = True
    else:
      done_right = True
      right_arm = i
      break

left_arm_name = left_arm['name']
left_arm_ik = left_arm['tip_link']

if done_right:
  right_arm_name = right_arm['name']
  right_arm_ik = right_arm['tip_link']

# Switch left and right arms if we find any of the following strings in the left arm
# this is a terrible hack, but by convention we assume one of these strings is in the right arm
if(re.search("right | Right | RIGHT | ^r_ | _r$ | ^R_ | _R$", left_arm_name)) :
  temp = left_arm_name
  left_arm_name = right_arm_name
  right_arm_name = temp
  temp = left_arm_ik
  left_arm_ik = right_arm_ik
  right_arm_ik = temp

template = open(roslib.packages.get_pkg_dir('move_arm_warehouse')+'/scripts/planning_scene_warehouse_viewer_template.launch', 'r')
text = template.read()

text = re.sub('__ROBOT_NAME__', sys.argv[1], text)
text = re.sub('__LEFT_GROUP__', left_arm_name, text)
text = re.sub('__RIGHT_GROUP__', right_arm_name, text)
text = re.sub('__LEFT_IK__', left_arm_ik, text)
text = re.sub('__RIGHT_IK__', right_arm_ik, text)

#interpolated ik will only work with two arms for now
if right_arm_name == 'none':
  text = re.sub('/l_interpolated_ik_motion_plan', 'none', text)
  text = re.sub('/r_interpolated_ik_motion_plan', 'none', text)

launch = open(directoryName+'/launch/planning_scene_warehouse_viewer_'+sys.argv[1]+'.launch', 'w')
launch.write(text)

launch.close()
template.close()

#Now we do the rviz template 

rviz_template = open(roslib.packages.get_pkg_dir('move_arm_warehouse')+'/scripts/planning_scene_warehouse_viewer_template.vcg', 'r')
rviz_text = rviz_template.read()
rviz_text = re.sub('__WORLD_FRAME__', world_frame, rviz_text)
out_vcg = open(directoryName+'/config/planning_scene_warehouse_viewer.vcg', 'w')
out_vcg.write(rviz_text)
out_vcg.close()
rviz_template.close()

exit(0)
