#!/usr/bin/env python

import copy
import rospy
import pymongo
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from scipy.spatial.distance import euclidean
from vision_people_logging.msg import LoggingUBD
from vision_people_logging.save_ubd import SaveUBD
from mongodb_store.message_store import MessageStoreProxy
from vision_people_logging.srv import FindUBD, FindUBDResponse
from vision_people_logging.srv import DeleteUBD, DeleteUBDResponse
from vision_people_logging.srv import CaptureUBD, CaptureUBDResponse


class VisionLoggingService(object):

    def __init__(self, name, wait_time=30):
        self._max_dist = 0.1
        self._wait_time = wait_time
        # ptu
        rospy.loginfo("Subcribe to /ptu/state...")
        self._ptu = JointState()
        self._ptu.position = [0, 0]
        self._ptu_counter = 0
        self._is_ptu_changing = [True for i in range(wait_time)]
        rospy.Subscriber("/ptu/state", JointState, self._ptu_cb, None, 1)
        # robot pose
        rospy.loginfo("Subcribe to /robot_pose...")
        self._robot_pose = Pose()
        self._robot_pose_counter = 0
        self._is_robot_moving = [True for i in range(wait_time)]
        rospy.Subscriber("/robot_pose", Pose, self._robot_cb, None, 1)
        # ubd services
        rospy.loginfo("Creating a delete service under %s/delete" % name)
        self.del_srv = rospy.Service(name+'/delete', DeleteUBD, self.del_srv_cb)
        rospy.loginfo("Creating a find service under %s/find" % name)
        self.find_srv = rospy.Service(name+'/find', FindUBD, self.find_srv_cb)
        rospy.loginfo("Creating a capture service under %s/capture" % name)
        self.cptr_srv = rospy.Service(name+'/capture', CaptureUBD, self.capture_srv_cb)
        rospy.loginfo("Connecting to mongodb message_store - upper_bodies collection...")
        # db
        self._ubd_db = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        ).message_store.upper_bodies
        self._msg_store = MessageStoreProxy(collection="upper_bodies")
        self.save_ubd = SaveUBD(is_stored=False)
        # publisher
        self._pub = rospy.Publisher(rospy.get_name()+'/log', LoggingUBD, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self._publish_logging)

    def _ptu_cb(self, ptu):
        dist = euclidean(ptu.position, self._ptu.position)
        self._is_ptu_changing[self._ptu_counter] = dist >= self._max_dist
        # print "is ptu moving: %s" % str(self._is_ptu_changing)
        self._ptu_counter = (self._ptu_counter+1) % self._wait_time
        self._ptu = ptu
        rospy.sleep(1)

    def _robot_cb(self, pose):
        dist = euclidean(
            [
                pose.position.x, pose.position.y,
                pose.orientation.z, pose.orientation.w
            ],
            [
                self._robot_pose.position.x, self._robot_pose.position.y,
                self._robot_pose.orientation.z, self._robot_pose.orientation.w
            ]
        )
        self._is_robot_moving[self._robot_pose_counter] = dist >= self._max_dist
        self._robot_pose_counter = (
            self._robot_pose_counter+1
        ) % self._wait_time
        self._robot_pose = pose
        rospy.sleep(1)

    def _publish_logging(self, event):
        not_moving = True not in (self._is_robot_moving+self._is_ptu_changing)
        if not_moving:
            log = copy.deepcopy(self.save_ubd.log)
            self._pub.publish(log)
            if not self.save_ubd.is_stored:
                self._msg_store.insert(log, meta={"stored_by": "ubd_service.py"})

    def capture_srv_cb(self, srv):
        rospy.loginfo("Got a request to capture a snapshot of UBD")
        count = len(self.save_ubd.obj_ids)
        self.save_ubd.is_stored = True
        st = rospy.Time.now()
        et = rospy.Time.now()
        while len(self.save_ubd.obj_ids) <= count and (et - st).secs < 5:
            rospy.loginfo("Trying to capture a snapshot of UBD...")
            rospy.sleep(0.05)
            et = rospy.Time.now()
        self.save_ubd.is_stored = False
        if len(self.save_ubd.obj_ids) - count > 1:
            rospy.logwarn(
                "%d snapshots have been captured" % (len(self.save_ubd.obj_ids) - count)
            )
        elif len(self.save_ubd.obj_ids) - count == 0:
            rospy.loginfo("No snapshot has been captured")
        rospy.sleep(0.1)
        return CaptureUBDResponse(
            self.save_ubd.obj_ids[count:len(self.save_ubd.obj_ids)]
        )

    def find_srv_cb(self, srv):
        rospy.loginfo("Got a request to query UBD from db...")
        logs = list()
        if len(srv.obj_ids) > 0:
            for _id in srv.obj_ids:
                log = self._msg_store.query_id(_id, LoggingUBD._type)
                if log is not None:
                    logs.append(log[0])
        elif srv.start_time is not None or srv.stop_time is not None:
            query = {
                "header.stamp.secs": {
                    "$gte": srv.start_time.secs,
                    "$lt": srv.stop_time.secs
                }
            }
            logs = self._msg_store.query(LoggingUBD._type, query)
            rospy.loginfo("Found %d entry(ies)..." % len(logs))
            logs = [log[0] for log in logs]
        rospy.sleep(0.1)
        return FindUBDResponse(logs)

    def del_srv_cb(self, srv):
        rospy.loginfo("Got a request to delete UBD entries from db...")
        if len(srv.obj_ids) > 0:
            count = list()
            for _id in srv.obj_ids:
                count.append(self._msg_store.delete(_id))
            count = [i for i in count if i]
            response = True
            if len(count) > 0:
                rospy.logwarn(
                    "%d entry(ies) have been deleted based on %s" % (len(count), str(srv.obj_ids))
                )
            else:
                rospy.loginfo(
                    "No entry has been found, nothing to delete"
                )
                response = False
        elif srv.start_time is not None or srv.stop_time is not None:
            query = {
                "header.stamp.secs": {
                    "$gte": srv.start_time.secs,
                    "$lt": srv.stop_time.secs
                }
            }
            count = self._ubd_db.find(query).count()
            response = True
            if count > 0:
                rospy.logwarn(
                    "%d entry(ies) from upper_bodies collection will be deleted" % count
                )
                self._ubd_db.remove(query)
            else:
                rospy.loginfo(
                    "No entry has been found, nothing to delete"
                )
                response = False
        rospy.sleep(0.1)
        return DeleteUBDResponse(response)


if __name__ == '__main__':
    rospy.init_node('vision_logging_service')
    wait_time = rospy.get_param("~wait_time", 30)
    vls = VisionLoggingService(rospy.get_name(), wait_time)
    rospy.spin()
