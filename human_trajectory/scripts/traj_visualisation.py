#!/usr/bin/env python

import argparse
import rospy
from human_trajectory.visualisation import TrajectoryVisualisation


if __name__ == "__main__":
    mode = "all"
    parser = argparse.ArgumentParser(prog='trajectory')
    parser.add_argument("mode", help="[all | average | shortest | longest]")
    args = parser.parse_args()
    if args.mode != "":
        mode = args.mode

    rospy.init_node("human_trajectory_visualization")
    rospy.loginfo("Running Trajectory Visualization...")

    ta = TrajectoryVisualisation('people_trajectories')
    ta.visualise(mode)

    raw_input("Press 'Enter' to exit.")
