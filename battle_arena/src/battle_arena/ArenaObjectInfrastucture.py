#! /usr/bin/python

import rospy
from battle_arena.ArenaObject import ArenaObject, ArenaObjectState
from battle_arena.ArenaObjectPlayer import PlayerRobot


class ScoreEvent(object):
    def __init__(self, team_id, score):
        self.team_id = team_id
        self.score = score

class TeamBase(ArenaObject):
    activation_radius = 0.2
    strength = 20
    recharge_duration = 2

    def __init__(self, team_id):
        ArenaObject.__init__(self)
        self.team_id = team_id
        self.remaining_recharge_time = -1

    def move(self, dt):
        self.remaining_recharge_time -= dt

    def interact(self, other):
        if self.distance_to(other) >= self.activation_radius:
            return

        self._react_to_robot(other)
        return self._react_to_treasure(other)

    def _react_to_robot(self, other):
        if not isinstance(other, PlayerRobot):
            return

        if other.team_id == self.team_id:
            rospy.loginfo("Robot in its homebase")
            other.increase_shield(TeamBase.strength)
            other.increase_hp(TeamBase.strength)
        else:
            rospy.loginfo("Robot %i close to base of team %i", other.id, self.team_id)
            other.apply_damage(TeamBase.strength)

    def _react_to_treasure(self, other):
        if not isinstance(other, Treasure):
            return
        rospy.logwarn("Team %i scored %i", self.team_id, other.value)
        other.enabled = False
        other.to_be_deleted = True
        return ScoreEvent(self.team_id, other.value)


class Treasure(ArenaObject):

    def __init__(self, value):
        ArenaObject.__init__(self)
        self.value = value


class CollectibleTreasure(ArenaObject):
    trigger_distance = 0.1

    def __init__(self, value):
        ArenaObject.__init__(self)
        self.value = value
        self.type = ArenaObjectState.TOKEN_COLLECTIBLE_TREASURE

    def interact(self, other):
        if not self.enabled:
            return

        if not isinstance(other, PlayerRobot):
            return

        if self.distance_to(other) > CollectibleTreasure.trigger_distance:
            return

        rospy.loginfo("Collectible Trasure was collected")
        self.enabled = True
        self.to_be_deleted = True

        se = ScoreEvent(other.team_id, self.value)
        return se

