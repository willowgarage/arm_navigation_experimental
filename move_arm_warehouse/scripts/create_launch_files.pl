#!/usr/bin/env perl
use strict;
use warnings;

# Constants for substitution
my $ROBOT_STRING = "__ROBOT_NAME__";
my $LEFT_GROUP_STRING = "__LEFT_GROUP__";
my $RIGHT_GROUP_STRING = "__RIGHT_GROUP__";
my $LEFT_IK_STRING = "__LEFT_IK__";
my $RIGHT_IK_STRING = "__RIGHT_IK__";

# Check for arguments
if(@ARGV != 5)
{
  print("Usage: create_launch_files <your_robot_name> <left_arm_group_name> <right_arm_group_name> <left_arm_ik_link> <right_arm_ik_link>. If you don't have one of these, enter \"none\" for that field.");

  exit(0);
}

# Save arguments.
my $newRobotName = $ARGV[0];
my $newLeftGroupName = $ARGV[1];
my $newRightGroupName = $ARGV[2];
my $newLeftIKName = $ARGV[3];
my $newRightIKName = $ARGV[4];

# Get template files.
open(my $inFile, "<", "planning_scene_warehouse_viewer_template.launch") or die ("Failed to open planning_scene_warehouse_viewer_template.launch : $!");

my @lines = <$inFile>;
my $outLines = "";

# Global replace each string.
foreach(@lines)
{
  $_ =~ s/$ROBOT_STRING/$newRobotName/g;
  $_ =~ s/$LEFT_GROUP_STRING/$newLeftGroupName/g;
  $_ =~ s/$RIGHT_GROUP_STRING/$newRightGroupName/g;
  $_ =~ s/$LEFT_IK_STRING/$newLeftIKName/g;
  $_ =~ s/$RIGHT_IK_STRING/$newRightIKName/g;
  $outLines .= $_;
  $outLines .= "\n";
}

close $inFile or die "$inFile : $!";

# Write to new file.
open(my $outFile, ">>", "planning_scene_warehouse_viewer_$newRobotName.launch") or die("Failed to create new launch file : $!");
print $outFile $outLines;

close $outFile or die "$outFile : $!";

exit(0);
