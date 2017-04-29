#! /usr/bin/python

import rospy
from battle_arena.ArenaObject import ArenaObject
from battle_arena_msgs.msg import ArenaObjectState
from copy import deepcopy

class ArenaAnimation(ArenaObject):
    # could be linked to another ArenaObject e.g. to use its position
    def __init__(self, animation_type=ArenaObjectState.ANIMATION_EXPLOSION, parent_object=None):
        ArenaObject.__init__(self)
        self.animation_type = animation_type
        self.remaining_time = 5.0  # depending on type?
        self.type = animation_type
        self.parent_object_id = -1
        if parent_object:
            print type(parent_object)
            assert isinstance(parent_object, ArenaObject)
            self.parent_object_id = parent_object.id

    def move(self, dt):
        self.remaining_time -= dt
        # continuous animations?
        if self.remaining_time < 0:
            self.to_be_deleted = True
            self.enabled = False
            rospy.loginfo("Animation with type %i finished", self.animation_type)

    def visualize(self, img):
        if not self.enabled:
            return
        # rospy.loginfo("Animation visualization not yet implemented")

    def interact(self, other):
        return []
