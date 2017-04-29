#! /usr/bin/python

from math import sin, cos, atan2
import numpy as np
import cv2
from battle_arena_msgs.msg import ArenaObjectState
from copy import deepcopy

class ArenaObject(object):
    next_object_id = 100
    r = 0.1
    px_per_m = -1

    def __init__(self, object_id=-1, name="no_name"):
        self.parent_frame = "arena"
        self.position = np.array([0.0, 0.0])
        self.velocity = 0.0
        self.yaw = 0.0
        self.type = -1
        self.to_be_deleted = False
        self.x_dimension = -1
        self.y_dimension = -1
        self.visualization_state = 0  # something to run animations
        self.enabled = True
        self.red = (0, 0, 255)
        self.green = (0, 255, 0)
        self.blue = (255, 0, 0)

        if object_id < 0:
            self.id = ArenaObject.next_object_id
            ArenaObject.next_object_id += 1
        else:
            self.id = object_id
        self.name = name
        self.team_id = -1

    def _get_state_msg_base(self):
        msg = ArenaObjectState()
        msg.object_id = self.id
        msg.team_id = self.team_id
        msg.pose.x_pos = self.position[0]
        msg.pose.y_pos = self.position[1]
        msg.pose.orientation = self.yaw
        msg.x_dimension = self.x_dimension
        msg.y_dimension = self.y_dimension
        msg.type = self.type
        msg.name = self.name
        return msg

    def get_state_msg(self):
        return self._get_state_msg_base()

    def get_angle_towards(self, other):
        assert isinstance(other, ArenaObject)
        d = other.position - self.position
        return atan2(float(d[1]), float(d[0]))

    def copy_pose_from(self, other):
        assert isinstance(other, ArenaObject)
        self.position = deepcopy(other.position)
        self.yaw = other.yaw

    def set_pose(self, x, y, yaw):
        assert abs(yaw) < 10
        self.set_position(x, y)
        self.yaw = yaw

    def set_position(self, x, y):
        self.position[0] = float(x)
        self.position[1] = float(y)

    def circle(self, img, radius_m, color, thickness=5):
        if 0 < thickness < 1:
            thickness = int(thickness * self.px_per_m)
        cv2.circle(img, self.get_pixel_pos(), int(radius_m * self.px_per_m), color, thickness)

    def ellipse(self, img, radius_m, color, ratio=1, thickness=5):
        if 0 < thickness < 1:
            thickness = int(thickness*self.px_per_m)
        cv2.ellipse(img, self.get_pixel_pos(), (int(radius_m * self.px_per_m),
                                                int(radius_m*self.px_per_m)), 0, 0, int(360*ratio), color, thickness)

    def get_pixel_pos(self):
        # TODO: pixel offset for origin?
        return int(self.position[0]*self.px_per_m), int(self.position[1]*self.px_per_m)

    def move(self, dt):
        self.position[0] += dt * self.velocity * cos(self.yaw)
        self.position[1] += dt * self.velocity * sin(self.yaw)

    def visualize(self, img):
        if not self.enabled:
            return
        self.circle(img, self.r, (255, 255, 255), -1)

    def interact(self, other):
        assert isinstance(other, ArenaObject)
        pass

    def distance_to(self, other):
        assert isinstance(other, ArenaObject)
        return np.linalg.norm(self.position-other.position)
