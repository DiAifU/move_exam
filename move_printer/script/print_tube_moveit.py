#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
from math import pi, cos, sin
from std_msgs.msg import String
import geometry_msgs.msg


class Cercle:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_printer_moveit',
                anonymous=True)
        self.group = moveit_commander.MoveGroupCommander("printer_group")

    def deg_to_rad(self,deg):
        return deg*pi/180

    def plan(self, radius):
        waypoints = []
        wpose = self.group.get_current_pose().pose
        for angle in range(0,360):
            wpose.position.z = 0.2
            wpose.position.x = radius * cos(self.deg_to_rad(angle))
            wpose.position.y = radius * sin(self.deg_to_rad(angle))
            waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        self.group.plan()
        rospy.sleep(1)
        self.group.execute(plan,wait=True)
        self.group.set_named_target('init_pose')
        self.group.plan()
        return plan, fraction



def main():
    cercle = Cercle()
    cercle.plan(0.05)


if __name__ == "__main__":
    main()
