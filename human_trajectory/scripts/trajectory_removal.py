#!/usr/bin/env python

import rospy
import pymongo
import argparse
import scipy.spatial.distance as dist_calc


if __name__ == "__main__":
    rospy.init_node("trajectory_removal")
    parser = argparse.ArgumentParser(prog=rospy.get_name())
    parser.add_argument(
        "-s", dest="size", default="10000",
        help="Total maximum number of trajectories (default: 10000)"
    )
    parser.add_argument(
        "-m", dest="movement_steps", default="1.0",
        help="Maximum distance between each step for normal humans (default: 1 meter)"
    )
    args = parser.parse_args()
    step = float(args.movement_steps)
    # a good range for human motion
    db = pymongo.MongoClient(
        rospy.get_param("mongodb_host", "localhost"),
        rospy.get_param("mongodb_port", 62345)
    ).message_store.people_trajectory
    db.create_index([("uuid", pymongo.ASCENDING)])
    logs = db.find({}, {"uuid":1}).sort("_id", pymongo.ASCENDING)
    rospy.loginfo("Removing trajectories with duplicated uuids...")
    for log in logs:
        db.remove({"_id": {"$gt": log["_id"]}, "uuid": log["uuid"]})
    rospy.loginfo("Removing trajectories with movement steps greater than %.2f meters" % step)
    logs = db.find().limit(int(args.size))
    for log in logs:
        # if log["displacement_pose_ratio"] < 0.01 or log["displacement_pose_ratio"] > 0.21:
        #     db.remove({"uuid": log["uuid"]})
        for ind, val in enumerate(log["trajectory"]):
            if ind == 0:
                continue
            distance = dist_calc.euclidean(
                [
                    val["pose"]["position"]["x"], val["pose"]["position"]["y"]
                ],
                [
                    log["trajectory"][ind-1]["pose"]["position"]["x"],
                    log["trajectory"][ind-1]["pose"]["position"]["y"]
                ]
            )
            if distance > step:
                db.remove({"uuid": log["uuid"]})
