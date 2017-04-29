#! /usr/bin/python

import rospy

from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers
from tf import transformations
# from math import pi


class ObjectMarkerTracking:
    def __init__(self):
        self.marker_id_2_object_id = dict()  # {7: 1})  #, 1: 2})
        self.sub_markers = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_callback)
        self._object_poses = dict()
        self.alpha = 1
        rospy.sleep(0.5)

    def get_object_pose(self, object_id):
        assert object_id in self.marker_id_2_object_id.values()
        if object_id not in self._object_poses:
            rospy.logwarn("No pose for player %i received yet", object_id)
            rospy.sleep(0.1)
            return None
        return self._object_poses[object_id]

    def marker_callback(self, data):
        assert isinstance(data, AlvarMarkers)
        for marker in data.markers:
            assert isinstance(marker, AlvarMarker)
            p = marker.pose.pose.position
            o = marker.pose.pose.orientation
            yaw = transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
            if marker.id in self.marker_id_2_object_id:
                object_id = self.marker_id_2_object_id[marker.id]
                # tracking, filtering could be done here
                # todo: project into ground plane
                if object_id not in self._object_poses:
                    self._object_poses[object_id] = [p.x, p.y, yaw]
                else:
                    x, y, phi = self._object_poses[object_id]
                    x += self.alpha*(p.x-x)
                    y += self.alpha*(p.y-y)
                    self._object_poses[object_id] = [x, y, yaw]

if __name__ == "__main__":
    rospy.init_node("tracker_test")
    rt = ObjectMarkerTracking()
    rospy.spin()
