#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
import yaml
import math

class Nav2GoalClient(Node):
    def __init__(self):
        super().__init__('nav2_goal_client')
        # ゴールを送信する際のROSの設定
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for action server...")

        # 自己位置を取得する際のROSの設定
        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer, self)

        # YAMLファイルから位置リストを読み込み
        file_path = "/home/user24/3d_navigation_ws/src/sirius_navigation/config/map.yaml"
        with open(file_path, 'r', encoding='utf-8') as f:
            self.positions_list = yaml.safe_load(f)['points']  # pointsを直接取得

        self.count = 0
        self.loop_count = 0
        self.distance = float('inf')  # 初期距離を無限大に設定

        # タイマーをセット
        self.timer = self.create_timer(1.0, self.get_position)

    def send_goal(self):
        if self.count >= len(self.positions_list):
            self.get_logger().info("All goals have been sent.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"  # フレームを設定
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.positions_list[self.count][0]
        goal_msg.pose.pose.position.y = self.positions_list[self.count][1]
        goal_msg.pose.pose.position.z = self.positions_list[self.count][2]
        goal_msg.pose.pose.orientation.x = self.positions_list[self.count][3]
        goal_msg.pose.pose.orientation.y = self.positions_list[self.count][4]
        goal_msg.pose.pose.orientation.z = self.positions_list[self.count][5]
        goal_msg.pose.pose.orientation.w = self.positions_list[self.count][6]

        self.get_logger().info("Sending goal point...")
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        self.get_logger().info("way point number: {}".format(self.count))

    def goal_response_callback(self, future):
        if future.result().accepted:
            self.get_logger().info("Goal accepted...")
            self.distance = float('inf')  # 新しいゴールが受理されたので距離をリセット
        else:
            self.get_logger().info("Goal rejected...")

    def get_position(self):
        try:
            transform = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            translation = transform.transform.translation
            self.position = [translation.x, translation.y]

            if self.count < len(self.positions_list):
                x_goal = self.positions_list[self.count][0]
                y_goal = self.positions_list[self.count][1]
                x_distance = x_goal - self.position[0]
                y_distance = y_goal - self.position[1]
                self.distance = math.sqrt(x_distance**2 + y_distance**2)
                self.get_logger().info(f"Current distance to goal: {self.distance}")

                # ゴールまでの距離が閾値以下であれば次のゴールを送信
                if self.distance < 0.7:
                    self.get_logger().info("Goal reached! Sending next goal...")
                    self.count += 1
                    self.send_goal()
                
                elif self.loop_count % 5 == 0:
                    self.send_goal()
                
                self.loop_count += 1 

        except Exception as e:
            self.get_logger().warn(f"Transform error: {e}")

def main(args=None):
    rclpy.init(args=args)
    nav2_goal_client = Nav2GoalClient()
    nav2_goal_client.send_goal()  # 最初のゴールを送信
    rclpy.spin(nav2_goal_client)
    nav2_goal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
