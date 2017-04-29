#! /usr/bin/python

import rospy
from battle_arena_msgs.msg import MoveCommand, PlayerState, Pose, ArenaObjectStateList, ArenaObjectState
from thread import start_new_thread
from math import atan2, pi, radians, sin, cos

class SimpleBattleBot:

    def __init__(self, robot_name, team_id):
        self.robot_name = robot_name
        self.team_id = team_id

        self.current_pose = Pose()
        self.goal_acquired = False
        self.target_pose = Pose()
        self.target_angle = 0

        self.pub_cmd = rospy.Publisher(self.robot_name+"/cmd", MoveCommand, queue_size=1)

        self.sub_player_state = rospy.Subscriber(self.robot_name+"/state",
                                                 PlayerState, self.state_update_cb)

        self.sub_tokens = rospy.Subscriber("/arena_manager/tokens",
                                                ArenaObjectStateList, self.arena_object_callback)

        # self.cmd_client = rospy.ServiceProxy('add_two_ints', PlayerCommand)

        SimpleBattleBot.wait_for(self.pub_cmd)
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

        self.current_pose = player_state.pose

    def arena_object_callback(self, state_list):
        assert isinstance(state_list, ArenaObjectStateList)
        for o in state_list.states:
            assert isinstance(o, ArenaObjectState)
            if o.type == ArenaObjectState.TOKEN_COLLECTIBLE_TREASURE:
                rospy.loginfo("Found new treasure to collect")
                self.goal_acquired = True
                self.target_pose = o.pose

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

            ahead = False
            vmax = 150  # robot motors are reversed
	    vturn = 80
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
            if ahead:
                rospy.sleep(1)
        print "Stopping"
        self.pub_cmd.publish(MoveCommand())


if __name__ == "__main__":
    rospy.init_node("SimpleBattleBot", anonymous=True)
    sbb = SimpleBattleBot("/team_black", 5)
    rospy.spin()
