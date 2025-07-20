#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
simple_waypoint_runner.py - Way-point sequencer **with built-in recovery** for AgileX Limo.
"""

import sys, math, rospy, actionlib
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty
import tf.transformations as tfm

# ---------------------------------------------------------------------------
# Helper: make a 2-D pose in "map" with (x,y,yaw); z defaults to 0.
# ---------------------------------------------------------------------------

def make_pose(x, y, yaw_deg, frame_id="map", z=0.0):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
    q = tfm.quaternion_from_euler(0, 0, math.radians(yaw_deg))
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
    return pose

# ---------------------------------------------------------------------------
# EDIT BELOW – fill in the eight centre-point poses of your map
# ---------------------------------------------------------------------------
GOALS = {
    1: make_pose(0.00, 0.00,   0),
    2: make_pose(1.00, 0.00,  90),
    3: make_pose(2.00, 0.00, 180),
    4: make_pose(3.00, 0.00, -90),
    5: make_pose(0.00, 1.00,   0),
    6: make_pose(1.00, 1.00,  90),
    7: make_pose(2.00, 1.00, 180),
    8: make_pose(3.00, 1.00, -90),
}

class WaypointRunner(object):
    """Single-node waypoint sequencer *with* embedded recovery logic."""

    def __init__(self):
        # === Parameters ===
        self.backup_dist  = rospy.get_param("~backup_dist", 0.02)  # m
        self.backup_speed = rospy.get_param("~backup_speed", 0.05) # m/s
        self.spin_angle   = rospy.get_param("~spin_angle", 2*math.pi)  # rad
        self.spin_speed   = rospy.get_param("~spin_speed", 0.7)    # rad/s
        self.goal_timeout = rospy.get_param("~goal_timeout", 180)  # s
        self.max_retries  = rospy.get_param("~max_retries", 3)

        self.costmap_clear_service = rospy.get_param("~costmap_clear_service",
                                                     "/move_base/clear_costmaps")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # === ROS handles ===
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.clear_costmaps = rospy.ServiceProxy(self.costmap_clear_service, Empty)

        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server…")
        self.ac.wait_for_server()
        rospy.loginfo("Connected to move_base")
        self.rate = rospy.Rate(10)

    # ---------------------------------------------------------------------
    # Recovery: reverse → spin → clear costmaps → resend goal
    # ---------------------------------------------------------------------
    def perform_recovery(self, goal_pose, attempt):
        rospy.logwarn("[Recovery %d] backing up %.0f cm",
                      attempt, self.backup_dist*100)
        # 1 – Reverse
        duration = abs(self.backup_dist / max(self.backup_speed, 1e-4))
        tw = Twist(); tw.linear.x = -abs(self.backup_speed)
        self._publish_for(tw, duration)

        # 2 – Spin in place
        rospy.logwarn("[Recovery %d] spinning 360 °", attempt)
        duration = abs(self.spin_angle / max(self.spin_speed, 1e-4))
        tw = Twist(); tw.angular.z = math.copysign(abs(self.spin_speed), 1.0)
        self._publish_for(tw, duration)

        # 3 – Costmap hygiene
        try:
            self.clear_costmaps()
            rospy.loginfo("Costmaps cleared")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to clear costmaps: %s", e)

        # 4 – Re-issue goal
        rospy.loginfo("Re-sending goal after recovery")
        self._send_goal(goal_pose)

    def _publish_for(self, twist, duration):
        stop = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < stop and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()
        self.cmd_pub.publish(Twist())  # hard stop
        rospy.sleep(0.3)

    # ---------------------------------------------------------------------
    # Goal helpers
    # ---------------------------------------------------------------------
    def _send_goal(self, pose):
        goal = MoveBaseGoal(); goal.target_pose = pose
        self.ac.send_goal(goal)

    def _wait_for_result(self):
        finished = self.ac.wait_for_result(rospy.Duration(self.goal_timeout))
        if not finished:
            rospy.logwarn("Goal timed out – cancelling")
            self.ac.cancel_goal()
            return GoalStatus.ABORTED
        return self.ac.get_state()

    # ---------------------------------------------------------------------
    # Interactive prompt and main loop
    # ---------------------------------------------------------------------
    def prompt_sequence(self):
        input = raw_input  # make Py 2 + Py 3 compatible
        seq_str = input("Sequence of way-points (e.g. 1 5 6 2): ")
        seq = [int(tok) for tok in seq_str.split()] if seq_str.strip() else []
        if not seq:
            rospy.logfatal("No indices given – exiting")
            sys.exit(1)
        passes_in = input("How many passes through that sequence? [1]: ")
        passes = int(passes_in) if passes_in.strip() else 1
        return seq * passes

    def run(self):
        full_sequence = self.prompt_sequence()
        for idx in full_sequence:
            if rospy.is_shutdown():
                break
            pose = GOALS.get(idx)
            if pose is None:
                rospy.logerr("Index %d is undefined – skipping", idx)
                continue
            rospy.loginfo("=== Way-point %d ===", idx)
            self._send_goal(pose)
            attempt = 0
            while not rospy.is_shutdown():
                state = self._wait_for_result()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached %d", idx)
                    break  # next way-point
                attempt += 1
                if attempt > self.max_retries:
                    rospy.logerr("Gave up on %d after %d recoveries",
                                 idx, self.max_retries)
                    return
                rospy.logwarn("move_base aborted – launching recovery #%d",
                              attempt)
                self.perform_recovery(pose, attempt)
        rospy.loginfo("Sequence complete – exiting")

# ---------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.init_node("simple_waypoint_runner")
    try:
        WaypointRunner().run()
    except KeyboardInterrupt:
        pass
