
import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

INITIAL_POSE_POSITION = [-0.08,-0.08]
INITIAL_POSE_DIRECTION = TurtleBot4Directions.EAST

GOAL_POSES = [
    ([-1.95, -0.068], TurtleBot4Directions.SOUTH),
    ([-2.0, -2.0], TurtleBot4Directions.EAST),
    ([-0.8, -2.0], TurtleBot4Directions.EAST),

]

def main():
    rclpy.init()
    navigator = TurtleBot4Navigator()

    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped(INITIAL_POSE_POSITION, INITIAL_POSE_DIRECTION)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    navigator.undock()

    goal_pose_msgs = [navigator.getPoseStamped(position, direction) for position, direction in GOAL_POSES]
    navigator.startFollowWaypoints(goal_pose_msgs)

    rclpy.shutdown()

if __name__ == '__main__':
    main()