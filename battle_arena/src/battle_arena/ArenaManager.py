#! /usr/bin/python

import rospy

from battle_arena.ArenaObject import ArenaObject
from battle_arena.ArenaObjectPlayer import PlayerRobot
from battle_arena.ArenaObjectNPC import Rocket, Sentry, Banana
from battle_arena.ArenaObjectToken import ShieldToken

import cv2
import numpy as np
from itertools import combinations
from math import pi
from copy import deepcopy
from battle_arena_msgs.msg import ArenaObjectState, ArenaObjectStateList
from battle_arena.ObjectMarkerTracking import ObjectMarkerTracking
from battle_arena.ArenaObjectAnimation import ArenaAnimation
from battle_arena.ArenaObjectInfrastucture import Treasure, TeamBase, ScoreEvent, CollectibleTreasure
from battle_arena_msgs.srv import PlayerCommand, PlayerCommandRequest, PlayerCommandResponse
from battle_arena.ArenaObjectToken import WeaponToken


class ArenaManager:
    px_per_m = 600

    def __init__(self):
        ArenaObject.px_per_m = ArenaManager.px_per_m

        self._pub_object_states = rospy.Publisher("~object_states", ArenaObjectStateList, queue_size=10)
        self._pub_object_states_public = rospy.Publisher("~tokens", ArenaObjectStateList, queue_size=10)

        self._token_types = [ArenaObjectState.TOKEN_ROCKET, ArenaObjectState.TOKEN_SHIELD,
                             ArenaObjectState.TOKEN_BANANA, ArenaObjectState.TOKEN_HP,
                             ArenaObjectState.TOKEN_COLLECTIBLE_TREASURE]

        self.arena_width = 2
        self.arena_height = 1.5
        self.battle_time = 0

        #TODO: make separate visualization
        self.width_px = self.arena_width*self.px_per_m
        self.height_px = self.arena_height*self.px_per_m
        self.arena_image = None  # np.zeros((self.height_px, self.width_px, 3), np.uint8)
        self.arena_background = np.zeros((self.height_px, self.width_px, 3), np.uint8)

        self.command_server = rospy.Service('~player_commands', PlayerCommand, self.handle_player_command)
        self.player_id_to_object_id = dict()

        self.arena_objects = dict()
        self._marker_object_tracker = ObjectMarkerTracking()
        self.setup()

        self.publish_object_states()

    def setup(self):
        p1 = PlayerRobot(player_id=1, team_id=1, name="team_red")
        # p1.set_position(0.2, 0.2)
        # p1.set_pose(0.2, 0.2, yaw=0.3)
        # p1.velocity = 0.05
        self.add_object(p1)

        self._marker_object_tracker.marker_id_2_object_id[5] = p1.id

        ct = CollectibleTreasure(20)
        ct.set_position(0.0, 0.5)
        self.add_object(ct)

        b = Banana()
        b.set_position(0.25, 0.25)
        b.velocity = 0.05
        self.add_object(b)

        # w1 = WeaponToken(ArenaObjectState.ROCKET, 1)
        # w1.set_position(0.3, 0.2)
        # self.add_object(w1)

        # w2 = WeaponToken(ArenaObjectState.BANANA, 1)
        # w2.set_position(0.3, 0.2)
        # self.add_object(w2)
        #
        # p2 = PlayerRobot(player_id=2, team_id=2, name="team_red")
        # p2.set_pose(1.0, 0.2, yaw=-pi/2)
        # p2.velocity = 0.1
        # self.add_object(p2)

        # b1 = Rocket(shooter_id=-1, team_id=-1)
        # b1.position[0] = 100
        # b1.velocity = 10
        # b1.yaw = 90 / 180.0 * pi
        # self.arena_objects.append(b1)

        # s1 = ShieldToken()
        # s1.position = np.array([500, 500])
        # self.arena_objects.append(s1)

        # sen1 = Sentry(42)
        # sen1.set_position(0, 0)
        # self.arena_objects.append(sen1)

        # sen2 = Sentry(4711)
        # sen2.set_position(0.0, 0.0)
        # sen2.reload_time = 0.2
        # self.add_object(sen2)

        self.arena_background[:] = (60, 0, 0)

    def handle_player_command(self, req):
        # print "got request", req.te
        assert isinstance(req, PlayerCommandRequest)
        rospy.loginfo("User %i requested activation of %s", req.player_id, WeaponToken.type2string[req.weapon])
        if not req.player_id in self.player_id_to_object_id.keys():
            rospy.logwarn("Invalid player id %i", req.player_id)
            return PlayerCommandResponse(message="Invalid player id")

        obj_id = self.player_id_to_object_id[req.player_id]

        player = self.arena_objects[obj_id]
        if not isinstance(player, PlayerRobot):
            rospy.logerr("Invalid player id %i (is not a player)", obj_id)
            return PlayerCommandResponse(message="Invalid player id")

        if not WeaponToken.is_weapon_type(req.weapon):
            msg = "Invalid weapong type " + str(req.weapon)
            rospy.logerr(msg)
            return PlayerCommandResponse(message=msg)

        ammo = player.weapon_count[req.weapon]
        if ammo == 0:
            msg = "No ammo, player received penalty"
            rospy.logerr(msg)
            player.apply_damage(10)
            return PlayerCommandResponse(message=msg)

        # TODO: create new weapon instance
        player.weapon_count[req.weapon] -= 1

        if req.weapon == ArenaObjectState.ROCKET:
            rospy.loginfo("Activating rocket")
            r = Rocket(player)  # copies id, team, direction
            msg = "Fired Rocket"
            rospy.logwarn(msg)
            self.add_object(r)
            return PlayerCommandResponse(success=True, message=msg)

        if req.weapon == ArenaObjectState.BANANA:
            rospy.loginfo("Activating banana")
            b = Banana(player)  # copies id, team, direction
            # b.velocity = 0
            msg = "Dropped banana"
            rospy.logwarn(msg)
            self.add_object(b)
            return PlayerCommandResponse(success=True, message=msg)

        rospy.logerr("can't produce mines yet")
        return PlayerCommandResponse(success=True, message="No mines yet")

    def update_object_poses(self):
        for o in self.arena_objects.values():
            if not (isinstance(o, PlayerRobot) or isinstance(o, Treasure)):
                continue
            pose = self._marker_object_tracker.get_object_pose(o.id)
            if not pose:
                rospy.logwarn("No pose for player %i", o.id)
                continue
            # else:
            #     print "got pose", pose
            # print pose
            o.set_pose(*pose)

    def publish_object_states(self):
        msg = ArenaObjectStateList()
        msg.states = [o.get_state_msg() for o in self.arena_objects.values()]
        n = self._pub_object_states.get_num_connections()
        # check if someone is trying to eavesdrop
        # if n > 1:
            # rospy.logwarn("%i listeners on object state topic '%s'", n, self._pub_object_states.name)
            # return
        msg.battle_time = self.battle_time
        self._pub_object_states.publish(msg)

        # get public objects (tokens)
        msg.states = [o for o in msg.states if o.type in self._token_types]

        if len(msg.states):
            self._pub_object_states_public.publish(msg)

    def visualize(self):
        self.arena_image = deepcopy(self.arena_background)
        for o in self.arena_objects.values():
            o.visualize(self.arena_image)

    def cleanup_objects(self):
        for obj_id, obj in self.arena_objects.items():
            assert isinstance(obj, ArenaObject)
            if obj.to_be_deleted:
                del self.arena_objects[obj_id]

    def add_object(self, arena_object):
        assert isinstance(arena_object, ArenaObject)
        assert arena_object.id not in self.arena_objects
        self.arena_objects[arena_object.id] = arena_object
        if isinstance(arena_object, PlayerRobot):
            self.player_id_to_object_id[arena_object.player_id] = arena_object.id

    def process_interaction(self, o):
        if not o:
            return

        if not isinstance(o, list):
            o = [o]

        if isinstance(o, list):
            for obj in o:
                if isinstance(obj, ScoreEvent):
                    rospy.loginfo("Team %i scored %i", obj.team_id, obj.score)
                    continue
                else:
                    if isinstance(obj, ArenaObject):
                        self.add_object(obj)
                    else:
                        rospy.logerr("Unhandled type in process_interaction")
                        print obj

    def update_animation_poses(self):
        for o in self.arena_objects.values():
            if not isinstance(o, ArenaAnimation):
                continue
            parent_id = o.parent_object_id
            if parent_id < 0:  # fixed position animation
                return
            # parent could have been deleted -> animations stays fixed
            if not parent_id in self.arena_objects:
                rospy.loginfo("Parent for Animation %i (%i) was already deleted", o.id, parent_id)
                return
            o.copy_pose_from(self.arena_objects[parent_id])

    def iterate(self, dt):
        am.battle_time += dt
        self.update_object_poses()
        self.update_animation_poses()

        for o in self.arena_objects.values():
            o.move(dt)

        for a, b in combinations(self.arena_objects.values(), 2):
            self.process_interaction(a.interact(b))
            self.process_interaction(b.interact(a))

if __name__ == "__main__":
    rospy.init_node("arena_manager")
    am = ArenaManager()

    # cv2.namedWindow("arena")

    # game_duration = 10
    step = 0.01

    # for i in range(1000):

    i = 0
    r = rospy.Rate(200)
    while True:
        i += 1

        r.sleep()

        am.iterate(step)
        # if am.battle_time > 20:
        #     break

        am.publish_object_states()
        # am.visualize()
        # cv2.imshow("arena", am.arena_image)
        am.cleanup_objects()
        cv2.waitKey(10)
        if rospy.is_shutdown():
            break

        # if i == 100:
        #     print "ASD"
        #     req = PlayerCommandRequest()
        #     req.player_id = 1
        #     req.weapon = ArenaObjectState.BANANA
        #     print am.handle_player_command(req)


        # rospy.sleep(0.2)