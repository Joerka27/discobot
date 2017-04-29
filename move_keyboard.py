#! /usr/bin/python

import rospy
from battle_arena_msgs.msg import MoveCommand
from sys import argv
# import curses, time



class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


if __name__ == "__main__":
    rospy.init_node("ASD")

    pub = rospy.Publisher("/team_black/cmd", MoveCommand, queue_size=1)
    while not pub.get_num_connections() and not rospy.is_shutdown():
        print "Waiting, call 'rostopic echo /team_black/cmd'"
        rospy.sleep(1)

    asd = _GetchUnix()

    l = 0
    r = 0
    step = 20
    stop = False

    while not rospy.is_shutdown() and not stop:
        k = asd()
        if k in ['q', ' ']:
            l = 0
            r = 0
            stop = k == 'q'
            # quit()

        k = k.upper()
        # print k, len(k)
        if k == 'A':
            r += step
            l -= step
        if k == 'D':
            r -= step
            l += step

        if k == 'W':
            l += step
            r += step
            # print "right"

        if k == 'S':
            r -= step
            l -= step
            # print "left"

        if k == '0':
            l = r = 0

        pub.publish(MoveCommand(left_speed=l, right_speed=r))
        rospy.sleep(0.02)
        print l, r

    rospy.sleep(0.5)
