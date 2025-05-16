#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

INITIAL_POSE_POSITION = [-0.08, -0.08]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.EAST

GOAL_POSES = [
    ([-1.95, -0.068], TurtleBot4Directions.SOUTH),
    ([-2.0, -2.0], TurtleBot4Directions.EAST),
    ([-0.8, -2.0], TurtleBot4Directions.EAST),
    ([-2.0, -2.0], TurtleBot4Directions.NORTH),
    ([-1.95, -0.068], TurtleBot4Directions.EAST),
]

class NavigationWithRotationWait(Node):
    def __init__(self):
        super().__init__('navigation_with_rotation_wait')
        self.navigator = TurtleBot4Navigator()
        self.rotation_done = False

        # 토픽 구독: rotation_controller가 퍼블리시하는 Bool
        self.subscription = self.create_subscription(
            Bool,
            'action_finish',
            self.rotation_done_callback,
            10
        )

        self.start_navigation_if_ready()

    def rotation_done_callback(self, msg):
        if msg.data:
            self.get_logger().info('✅ Rotation finished. Starting navigation...')
            self.rotation_done = True
            self.start_navigation_if_ready()

    def start_navigation_if_ready(self):
        if not self.rotation_done:
            return  # 회전 완료 신호가 없으면 대기

        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped(INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION)
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        self.navigator.undock()

        goal_pose_msgs = [
            self.navigator.getPoseStamped(position, direction)
            for position, direction in GOAL_POSES
        ]

        self.navigator.startFollowWaypoints(goal_pose_msgs)

        # self.navigator.dock()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationWithRotationWait()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
