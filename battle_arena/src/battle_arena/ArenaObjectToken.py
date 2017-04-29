#! /usr/bin/python

import rospy
from battle_arena_msgs.msg import ArenaObjectState
from battle_arena.ArenaObject import ArenaObject
from battle_arena.ArenaObjectPlayer import PlayerRobot


class Token(ArenaObject):
    def __init__(self):
        ArenaObject.__init__(self)
        self.recharge_time = 20
        self.activation_distance = 0.1
        self.remaining_recharge_time = -1
        self.type = -1

    def move(self, dt):
        self.remaining_recharge_time = max(0, self.remaining_recharge_time - dt)

    def visualize(self, img):
        # cv2.circle(img, self.get_int_pos(), self.r*self.px_per_m, (0, 255, 0), -1)
        self.circle(img, self.r, self.green, -1)

        # overpaint with growing partial circle during recharge
        if self.remaining_recharge_time > 0:
            self.circle(img, self.r, (0, 100, 0), -1)
            self.ellipse(img, self.r, (0, 255, 0), 1 - self.remaining_recharge_time * 1.0 / self.recharge_time,
                         thickness=-1)

    def _check_trigger(self, other):
        if not isinstance(other, PlayerRobot):
            return False

        if self.remaining_recharge_time > 0 or not self.enabled:
            return False

        if self.distance_to(other) > self.activation_distance:
            return False

        if not other.is_alive():
            return False

        self.remaining_recharge_time = self.recharge_time

        if self.recharge_time < 0:
            rospy.loginfo("Deleting one time use token")
            self.to_be_deleted = True
            self.enabled = False

        # TODO: or delete object? (e.g. if recharge_time was set to -1)
        return True

    def interact(self, other):
        rospy.logerr("A Base token should not interact")


class ShieldToken(Token):
    def __init__(self, strength=50):
        Token.__init__(self)
        self.strength = strength
        self.type = ArenaObjectState.TOKEN_SHIELD

    def get_state_msg(self):
        msg = self._get_state_msg_base()
        msg.shield_upgrade = self.strength
        msg.shield_radius = self.activation_distance
        msg.shield_remaining_recharge_time = self.remaining_recharge_time
        return msg

    def interact(self, other):
        if not self._check_trigger(other):
            return
        assert isinstance(other, PlayerRobot)
        rospy.loginfo("Shield token for %i shield activated by player %i", self.strength, other.id)
        other.increase_shield(self.strength)


class HPToken(Token):
    def __init__(self, strength=50):
        Token.__init__(self)
        self.strength = strength
        self.type = ArenaObjectState.TOKEN_SHIELD

    def interact(self, other):
        if not self._check_trigger(other):
            return
        assert isinstance(other, PlayerRobot)
        rospy.loginfo("HP token for %i HP activated by player %i", self.strength, other.id)
        other.increase_hp(self.strength)


class WeaponToken(Token):
    type2string = {ArenaObjectState.BANANA: "Banana",
                   ArenaObjectState.ROCKET: "Rocket",
                   ArenaObjectState.MINE: "Mine"}

    @staticmethod
    def is_weapon_type(type_id):
        return type_id in WeaponToken.type2string.keys()

    def __init__(self, weapon_type, count=2):
        Token.__init__(self)
        self.count = count
        self.type = weapon_type
        assert self.type in WeaponToken.type2string.keys()

    def interact(self, other):
        if not self._check_trigger(other):
            return

        assert isinstance(other, PlayerRobot)
        rospy.loginfo("Player %i collected %i weapons of type %s",
                      other.player_id,  self.count, WeaponToken.type2string[self.type])
        other.weapon_count[self.type] += self.count
