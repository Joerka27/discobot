#! /usr/bin/python

import rospy
from math import pi

from battle_arena_msgs.msg import ArenaObjectState
from battle_arena.ArenaObject import ArenaObject

from battle_arena.ArenaObjectPlayer import PlayerRobot
from battle_arena.ArenaObjectAnimation import ArenaAnimation
from battle_arena.ArenaObjectInfrastucture import ScoreEvent


class Rocket(ArenaObject):
    max_lifetime = 50

    # def __init__(self, shooter_id, team_id):
    #     ArenaObject.__init__(self)
    #     self._create(shooter_id, team_id)

    def __init__(self, shooter=None):
        ArenaObject.__init__(self)
        # assert isinstance(shooter, PlayerRobot)

        sender_id = -1
        team_id = -1
        if isinstance(shooter, PlayerRobot):
            sender_id = shooter.player_id
            team_id = shooter.team_id
            self.copy_pose_from(shooter)

        self._create(shooter_id=sender_id, team_id=team_id)
        self.velocity = 0.3

    def _create(self, shooter_id, team_id):
        self.damage = 30
        self.shooter_id = shooter_id
        self.team_id = team_id
        self.trigger_distance = 0.1
        self.type = ArenaObjectState.ROCKET
        self.lifetime = Rocket.max_lifetime

    def move(self, dt):
        super(Rocket, self).move(dt)
        self.lifetime -= dt
        if self.lifetime < 0:
            self.to_be_deleted = True

    def visualize(self, img):
        if not self.enabled or self.to_be_deleted:
            return
        self.circle(img, self.r, (0, 255, 255), -1)
        self.circle(img, self.trigger_distance, (0, 0, 255), 2)

    def _is_triggered(self, other):
        if not self.enabled or self.to_be_deleted:
            return False

        if not isinstance(other, PlayerRobot):
            return False

        if not other.is_alive():
            return False

        distance = self.distance_to(other)
        if distance > self.trigger_distance:
            return False

        if other.player_id == self.shooter_id:
            # rospy.loginfo("Rocket does not kill its shooter")
            return False

        return True

    def interact(self, other):
        if not self._is_triggered(other):
            return

        rospy.loginfo("Player %i was hit by a rocket", other.player_id)
        other.apply_damage(self.damage)
        self.enabled = False  # or after animation
        self.to_be_deleted = True

        # create animation (fixed to this position)
        rospy.loginfo("Creating explosion animation for rocket")
        animation = ArenaAnimation(ArenaObjectState.ANIMATION_EXPLOSION)
        animation.copy_pose_from(other=self)

        score = ScoreEvent(team_id=self.team_id, score=10)
        return [animation, score]


class Banana(Rocket):
    _spin_duration_s = 2.0

    # def __init__(self, shooter_id, team_id):
    #     Rocket.__init__(self, shooter_id, team_id)
    #     self.type = ArenaObjectState.BANANA

    def __init__(self, shooter=None):
        Rocket.__init__(self, shooter)
        self.type = ArenaObjectState.BANANA

    def interact(self, other):
        if not self._is_triggered(other):
            return
        assert isinstance(other, PlayerRobot)
        rospy.loginfo("Player %i touched a banana", other.player_id)

        # let player spin (not blocking)
        other.robot_interface.spin(self._spin_duration_s)

        self.enabled = False
        self.to_be_deleted = True

        # rospy.loginfo("Creating banana animation")
        # animation = ArenaAnimation(ArenaObjectState.ANIMATION_BANANA, parent_object=-1)
        # animation.copy_pose_from(other=other)
        # return animation

        score = ScoreEvent(team_id=self.team_id, score=2)
        return score

    def visualize(self, img):
        if not self.enabled or self.to_be_deleted:
            return
        self.circle(img, self.r, (255, 0, 255), -1)
        self.circle(img, self.trigger_distance, (0, 0, 255), 2)


class Sentry(ArenaObject):
    reload_time = 3
    max_fire_distance = 0.5

    def __init__(self, team_id, ammo_type=Rocket):
        ArenaObject.__init__(self)
        self.team_id = team_id
        self.remaining_reload_time = -1  # can fire at start of round
        self.type = ArenaObjectState.SENTRY
        self.ammo_type = ammo_type
        self.velocity = 0
        # print self.ammo_type
        # assert type(self.ammo_type) in [Rocket, Banana]

    def move(self, dt):
        self.remaining_reload_time -= dt

    def visualize(self, img):
        self.circle(img, 0.05, (255, 0, 255), thickness=-1)
        self.ellipse(img, Sentry.max_fire_distance, (255, 0, 255), thickness=2)

    def get_state_msg(self):
        msg = self._get_state_msg_base()
        msg.sentry_vision_radius = self.max_fire_distance
        return msg

    def interact(self, other):
        if not isinstance(other, PlayerRobot):
            return

        if other.team_id == self.team_id:
            return

        if self.remaining_reload_time > 0:
            return

        if not other.is_alive():
            return

        distance = self.distance_to(other)
        if distance > self.max_fire_distance:
            return

        # rospy.loginfo("Firing on player %i", other.player_id)
        self.remaining_reload_time = Sentry.reload_time

        print "shooting towards", other.get_pixel_pos()
        rospy.sleep(0.2)

        r = Rocket(self)
        # r = self.ammo_type(shooter_id=self.id, team_id=self.team_id)
        # r.velocity = 0.1
        # r.position = np.copy(self.position)
        r.yaw = self.get_angle_towards(other)

        return r

if __name__ == "__main__":

    p1 = Rocket(1, 2)
    p1.velocity = 0.1
    p1.yaw = 45/180.0*pi

    l = [p1]

    for i in range(5):
        p1.move(1)
        print p1.position
