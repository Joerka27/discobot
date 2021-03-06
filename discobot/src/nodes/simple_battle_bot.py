#! /usr/bin/python

import rospy
from battle_arena_msgs.msg import MoveCommand, PlayerState, Pose, ArenaObjectStateList, ArenaObjectState
from battle_arena_msgs.srv import PlayerCommand
from std_msgs.msg import UInt64
from ar_track_alvar_msgs.msg import AlvarMarkers
from thread import start_new_thread
from math import atan2, pi, radians, sin, cos

TEAM_ID = 5

class SimpleBattleBot:

    def __init__(self, robot_name, team_id):
        self.robot_name = robot_name
        self.team_id = team_id

        self.current_pose = Pose()
        self.goal_acquired = False
        self.goal_token_type = None
        self.target_pose = Pose()
        self.target_angle = 0
        self.tf_angle = None
        self.ammos = {}

        self.pub_cmd = rospy.Publisher(self.robot_name+"/cmd", MoveCommand, queue_size=1)
        self.pub_blink = rospy.Publisher(self.robot_name + "/blink", UInt64, queue_size=1)

        self.sub_player_state = rospy.Subscriber(self.robot_name+"/state",
                                                 PlayerState, self.state_update_cb)

        self.sub_tokens = rospy.Subscriber("/arena_manager/tokens",
                                                ArenaObjectStateList, self.arena_object_callback)

        self.weapon_client = rospy.ServiceProxy('/arena_manager/player_commands', PlayerCommand)

        SimpleBattleBot.wait_for(self.pub_cmd)
        # SimpleBattleBot.wait_for(self.pub_blink)
        SimpleBattleBot.wait_for(self.sub_player_state)
        SimpleBattleBot.wait_for(self.sub_tokens) #  information on all tokens in the arena

        rospy.loginfo("Connections established")
        start_new_thread(self.run_loop, ())

    @staticmethod
    def wait_for(pub_sub):
        assert isinstance(pub_sub, rospy.Subscriber) or isinstance(pub_sub, rospy.Publisher)
        while not pub_sub.get_num_connections() and not rospy.is_shutdown():
            rospy.loginfo("Waiting for connection on %s", pub_sub.name)
            rospy.sleep(0.2)

    def state_update_cb(self, player_state):
        assert isinstance(player_state, PlayerState)
        if player_state.hit_points == 0:
            rospy.logwarn("So long and thanks for all the fish")
            rospy.sleep(1)
            return

        # update our ammo stash
        for weapon in ["rocket", "banana", "mine"]:
            ammo = getattr(player_state, weapon + "_ammo")
            self.ammos[weapon] = ammo

        self.current_pose = player_state.pose

    def arena_object_callback(self, state_list):
        assert isinstance(state_list, ArenaObjectStateList)
	
	# check if we should recharge at the base
	for a in state_list.states:
	    if a.type == ArenaObjectState.PLAYER and a.team_id == TEAM_ID and a.player_hp < 60:
		base_list = [a for a in state_list.states if a.type == ArenaObjectStateList.BASE]
		if not base_list:
		    rospy.logerr("base not found")
		    break
		
		rospy.logwarn("looking for base to recharge")
		self.goal_acquired = True
		self.target_pose = base_list[0].pose
		return

        # look for weapon tokens first
        rocket_tokens = [a for a in state_list.states if a.type ==
                ArenaObjectState.TOKEN_ROCKET]
        if rocket_tokens:
            rospy.loginfo("Rocket token found")
            self.goal_acquired = True
            self.target_pose = rocket_tokens[0].pose
            return

        # aim for opponent next
        if self._has_ammo():
            player_tokens = [a for a in state_list.states if a.type ==
                    ArenaObjectState.PLAYER and a.team_id != TEAM_ID and a.player_hp]
            if player_tokens:
                opponent = player_tokens[0]
                rospy.logwarn("Found opponent: {}".format(opponent.team_id))
                self.goal_acquired = True
                self.target_pose = opponent.pose
                return

        try:
	    rospy.logwarn("waiting for alvar message")
            msg = rospy.wait_for_message("ar_pose_marker", AlvarMarkers, timeout=0.2)
	    if not len(msg.markers):
		rospy.logwarn("no markers found")
	    else:
		rospy.logerr("found marker!!!")
	        translation = msg.markers[0].pose.pose.position
	        self.tf_angle = atan2(translation.x, translation.z)
		return
        except rospy.ROSException:
            # timeout
            pass

        # otherwise hunt dem treasures
        treasure_tokens = [a for a in state_list.states if a.type ==
                ArenaObjectState.TOKEN_COLLECTIBLE_TREASURE]
        if treasure_tokens:
            rospy.loginfo("Found new treasure to collect")
            self.goal_acquired = True
            # self.goal_token_type = o.type
            self.target_pose = treasure_tokens[0].pose
            return


    def get_angle_towards_goal(self):
        if not self.goal_acquired:
            return None
        dx = self.target_pose.x_pos - self.current_pose.x_pos
        dy = self.target_pose.y_pos - self.current_pose.y_pos

        # print "target", self.target_pose
        # print "current", self.current_pose
        # print dx, dy
        return atan2(dy, dx)

    def run_loop(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()
            if not self.goal_acquired:
                rospy.loginfo("Waiting for goal")
                continue

            goal_angle = self.get_angle_towards_goal()

            diff_angle = goal_angle - self.current_pose.orientation

            # print "diff", diff_angle/pi*180.0
            diff_angle = diff_angle/pi*180
            diff_angle -= 90
            diff_angle += 180
	    diff_angle %= 360

            print "diff", diff_angle

            vmax = 250  # robot motors are reversed
	    vturn = 150

            if self._has_ammo():
                # we have ammo! rotate towards enemy
                shoot_angle = 10
                if diff_angle > 170 or diff_angle < 190:
                    for weapon in self.ammos:
                        if self.ammos[weapon]:
                            try:
				weapon_token = getattr(ArenaObjectState, weapon.upper())
				rospy.logwarn("weapon id " + str(weapon_token))
                                response = self.weapon_client(TEAM_ID, weapon_token)
				if not response.success:
				    rospy.logerr(weapon + " service failed")
				    rospy.logerr(response.message)
				else:
				    break
                            except Exception as e:
                                print(e)
                                continue

                if 180 > diff_angle:
                    cmd = MoveCommand(left_speed=-vturn, right_speed=vturn)
                else:
                    cmd = MoveCommand(left_speed=vturn, right_speed=-vturn)
            else:
                if self.tf_angle is not None:
                    # update if TF angle available
                    diff_angle = self.tf_angle
                    # reset if marker not detected in next step
                    self.tf_angle = None

                if 0 <= diff_angle <= 90:
                    cmd = MoveCommand(right_speed=vmax, left_speed=cos(radians(diff_angle))*vmax)
                elif 270 <= diff_angle < 360:
                    cmd = MoveCommand(right_speed=cos(radians(diff_angle))*vmax, left_speed=vmax)
                else:
                    if 180 > diff_angle:
                        rospy.loginfo("left")
                        cmd = MoveCommand(left_speed=-vturn, right_speed=vturn)
                    else:
                        rospy.loginfo("right")
                        cmd = MoveCommand(left_speed=vturn, right_speed=-vturn)

            self.pub_cmd.publish(cmd)

            #if self.goal_token_type is not None:
            #self.pub_blink.publish(0x0000663cff3c6600)
            #rospy.sleep(0.5)

        print "Stopping"
        self.pub_cmd.publish(MoveCommand())

    def _has_ammo(self):
        return any([a for a in self.ammos.values()])


if __name__ == "__main__":
    rospy.init_node("SimpleBattleBot", anonymous=True)
    sbb = SimpleBattleBot("/team_black", TEAM_ID)
    rospy.spin()
