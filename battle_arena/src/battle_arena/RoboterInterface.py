#! /usr/bin/python

import rospy
from battle_arena_msgs.msg import MoveCommand, PlayerState
from thread import start_new_thread

# TODO: use odom to check if robot is executing the master commands


# this class implements an interface to the physical robot
class RoboterInterface:

    _master_command_topic = '/master_cmd'
    # _tone_topic = '/tone'
    _state_topic = '/state'

    def __init__(self, name):
        self._name = name  # also used as namespace
        self._pub_master_cmd = rospy.Publisher(self._name+self._master_command_topic, MoveCommand, queue_size=1)
        self._pub_state = rospy.Publisher(self._name + self._state_topic, PlayerState, queue_size=1)

        rospy.sleep(0.2)

        while self._pub_master_cmd.get_num_connections() == 0:
            rospy.loginfo("Waiting for connection to %s", self._name)
            rospy.sleep(0.1)
            break  # HACK

        rospy.loginfo("Connected to %s", self._name)

    def send_state(self, player_state):
        assert isinstance(player_state, PlayerState)
        self._pub_state.publish(player_state)
        if self._pub_state.get_num_connections() > 1:
            rospy.logwarn("More than one listener on '%s'", self._pub_state.name)

    def spin(self, duration_sec=2):
        # let's robot spin for given duration
        start_new_thread(self._spin_thread, (duration_sec, ))

    def _spin_thread(self, duration, velocity=200, stop_robot=True):
        cmd = MoveCommand(left_speed=velocity, right_speed=-velocity)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rospy.loginfo("Starting to spin %s", self._name)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self._pub_master_cmd.publish(cmd)
            rospy.sleep(0.2)
        rospy.loginfo("Ending to spin %s", self._name)
        if stop_robot:
            rospy.loginfo("stopping")
            self._pub_master_cmd.publish(MoveCommand())

if __name__ == "__main__":
    rospy.init_node("RobotInterfaceTest")

    r = RoboterInterface("team_red")
    r.spin(1)

    rospy.spin()
