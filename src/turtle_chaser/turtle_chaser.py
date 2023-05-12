#!/usr/bin/python3

import math
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

SPAWN_SERVICE_NAME = "/spawn"


class TurtleChaserNode:
    ROS_NODE_NAME = "turtle_chaser_node"
    LEADER_TURTLE_NAME = "turtle1"
    CHASER_TURTLE_NAME = "turtle2"
    LEADER_TURTLE_POSE_TOPIC = f"/{LEADER_TURTLE_NAME}/pose"
    CHASER_TURTLE_POSE_TOPIC = f"/{CHASER_TURTLE_NAME}/pose"
    CHASER_TURTLE_CMD_TOPIC = f"/{CHASER_TURTLE_NAME}/cmd_vel"
    SPAWN_TURTLE_DEFAULT_ARGS = (5.0, 5.0, 0.0, CHASER_TURTLE_NAME)
    CHASE_SPEED_PARAM_NAME = "~chase_speed"
    CHASER_RATE = 10  # Hz
    CHASE_STOP_DISTANCE = 0.25
    CHASER_TURTLE_DEFAULT_SPEED = 1.0

    def __init__(self):
        rospy.init_node(self.ROS_NODE_NAME)
        rospy.wait_for_service(SPAWN_SERVICE_NAME)

        try:
            spawn_turtle = rospy.ServiceProxy(SPAWN_SERVICE_NAME, Spawn)
            spawn_turtle(*self.SPAWN_TURTLE_DEFAULT_ARGS)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        self.rate = rospy.Rate(self.CHASER_RATE)
        self.leader_pose: Optional[Pose] = None
        self.chaser_pose: Optional[Pose] = None

        rospy.Subscriber(self.LEADER_TURTLE_POSE_TOPIC, Pose, self.leader_pose_callback)
        rospy.Subscriber(self.CHASER_TURTLE_POSE_TOPIC, Pose, self.chaser_pose_callback)

        self.cmd_vel_pub = rospy.Publisher(
            self.CHASER_TURTLE_CMD_TOPIC, Twist, queue_size=10
        )

        self.chase_speed = rospy.get_param(
            self.CHASE_SPEED_PARAM_NAME, self.CHASER_TURTLE_DEFAULT_SPEED
        )

    def leader_pose_callback(self, data: Pose):
        self.leader_pose = data

    def chaser_pose_callback(self, data: Pose):
        self.chaser_pose = data

    def compute_velosity(self) -> Twist:
        if self.leader_pose is None or self.chaser_pose is None:
            return Twist()

        dx = self.leader_pose.x - self.chaser_pose.x
        dy = self.leader_pose.y - self.chaser_pose.y
        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)

        angle_diff = target_angle - self.chaser_pose.theta

        # [-pi, pi] range restriction
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        chaser_cmd = Twist()

        if distance < self.CHASE_STOP_DISTANCE:
            chaser_cmd.linear.x = 0
        else:
            chaser_cmd.linear.x = self.chase_speed

        chaser_cmd.angular.z = 2.0 * angle_diff

        return chaser_cmd

    def run(self):
        while not rospy.is_shutdown():
            chaser_cmd = self.compute_velosity()
            self.cmd_vel_pub.publish(chaser_cmd)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        chaser_node = TurtleChaserNode()
        chaser_node.run()
    except rospy.ROSInterruptException:
        pass
