#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float32MultiArray

class GlobalPathPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')
        self.publisher = self.create_publisher(Path, 'global_path', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'robot_goal_position',
            self.robot_goal_callback,
            10
        )
        self.robot_position = None
        self.goal_position = None

    def robot_goal_callback(self, msg):
        # Extract robot and goal positions from the message
        self.robot_position = Point(x=msg.data[0], y=msg.data[1], z=0.0)
        self.goal_position = Point(x=msg.data[2], y=msg.data[3], z=0.0)
        self.publish_path()

    def publish_path(self):
        if self.robot_position is None or self.goal_position is None:
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'

        # Generate a straight-line path from robot to goal
        num_points = 10
        for i in range(num_points):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = self.robot_position.x + (self.goal_position.x - self.robot_position.x) * (i / num_points)
            pose.pose.position.y = self.robot_position.y + (self.goal_position.y - self.robot_position.y) * (i / num_points)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.publisher.publish(path)

def main(args=None):
    rclpy.init(args=args)
    global_path_planner = GlobalPathPlanner()
    rclpy.spin(global_path_planner)
    global_path_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
