#! /usr/bin/python

import rospy
from battle_arena_msgs.msg import ArenaObjectState, PlayerState
from battle_arena.ArenaObject import ArenaObject
from battle_arena.RoboterInterface import RoboterInterface
from thread import start_new_thread


class PlayerRobot(ArenaObject):
    max_shield = 40
    max_hp = 100

    def __init__(self, player_id, team_id, name):
        ArenaObject.__init__(self, name=name)
        assert isinstance(team_id, int)
        self.robot_interface = RoboterInterface(name=name)

        self.player_id = player_id
        self.team_id = team_id

        self.hp = PlayerRobot.max_hp
        self.shield = PlayerRobot.max_shield
        self.score = 0
        self.max_velocity = 1
        self.player_color = (255, 0, 0) if player_id == 1 else (0, 255, 0)
        self.type = ArenaObjectState.PLAYER
        start_new_thread(self._send_state_thread, ())

        self.weapon_count = dict()
        self.weapon_count[ArenaObjectState.ROCKET] = 0
        self.weapon_count[ArenaObjectState.BANANA] = 0
        self.weapon_count[ArenaObjectState.MINE] = 0

    # def move(self, dt):
        # updated by markers (and maybe extrapolation)
        # pass

    def no_move(self, dt):
        pass

    def _get_player_state_msg(self):
        msg = PlayerState()

        msg.hit_points = self.hp
        msg.shield_strength = self.shield

        msg.pose.x_pos = self.position[0]
        msg.pose.y_pos = self.position[1]
        msg.pose.orientation = self.yaw

        msg.rocket_ammo = self.weapon_count[ArenaObjectState.ROCKET]
        msg.banana_ammo = self.weapon_count[ArenaObjectState.BANANA]
        msg.mine_ammo = self.weapon_count[ArenaObjectState.MINE]

        return msg

    def _send_state_thread(self):
        # player gets updates on shield, hp, ...
        while not rospy.is_shutdown():
            msg = self._get_player_state_msg()
            self.robot_interface.send_state(msg)
            rospy.sleep(0.1)

    def get_state_msg(self):
        msg = self._get_state_msg_base()
        msg.player_hp = self.hp
        msg.player_shield = self.shield
        return msg

    def increase_hp(self, amount):
        self.hp = min(self.hp+amount, self.max_hp)

    def increase_shield(self, amount):
        self.shield = min(self.shield+amount, self.max_shield)

    def apply_damage(self, damage):
        assert damage > 0
        if self.shield > damage:
            self.shield -= damage
            rospy.loginfo("player %i: shield protected hp, now at hp:%i shield:%i", self.player_id, self.shield, self.hp)
            return

        damage -= self.shield
        self.shield = 0

        self.hp -= damage
        rospy.loginfo("player %i got %i damage, no shield, remaining hp: %i", self.player_id, damage, self.hp)

        if not self.is_alive():
            rospy.logwarn("Player %i died", self.player_id)

        # we could send a special player_state

    def is_alive(self):
        return self.hp > 0

    def visualize(self, img):
        # player
        self.circle(img, self.r, self.player_color, -1)

        if not self.is_alive():
            self.circle(img, 0.05, self.red, -1)
            self.move = self.no_move
            return

        # shield
        if self.shield > 0:
            self.ellipse(img, self.r + 0.05, (0, 255, 255), self.shield * 1.0 / PlayerRobot.max_shield, thickness=0.01)
        # hp
        self.ellipse(img, self.r + 0.03, (0, 255, 0), self.hp * 1.0 / PlayerRobot.max_hp, thickness=2)

