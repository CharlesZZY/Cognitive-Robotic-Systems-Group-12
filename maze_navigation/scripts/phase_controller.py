#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from rclpy.action import ActionClient
import time


class PhaseController(Node):

    def __init__(self):
        super().__init__('phase_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')

        self.get_logger().info("Phase Controller started")

    def rotate_in_place(self, duration=6.0):
        twist = Twist()
        twist.angular.z = 0.5
        start = time.time()

        while time.time() - start < duration:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def navigate(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0  # ⚠️ 如需 yaw，后续改

        self.nav_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        send_goal = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal)

        result = send_goal.result().get_result_async()
        rclpy.spin_until_future_complete(self, result)

    def switch_map(self, map_yaml):
        self.map_client.wait_for_service()
        req = LoadMap.Request()
        req.map_url = map_yaml
        self.map_client.call_async(req)
        time.sleep(2.0)


def main():
    rclpy.init()
    node = PhaseController()

    # Phase 0: AMCL 收敛
    node.rotate_in_place()

    # ========= Phase 1 =========
    # TODO: 填入 START 坐标
    START_X = 0.0   # <-- TODO
    START_Y = 0.0   # <-- TODO
    START_YAW = 0.0

    node.get_logger().info("Navigating to START")
    node.navigate(START_X, START_Y, START_YAW)

    # ========= 切地图 =========
    node.get_logger().info("Switching to full map")
    node.switch_map('config/maze_full_map.yaml')

    node.rotate_in_place(duration=3.0)

    # ========= Phase 2 =========
    # TODO: 填入 END 坐标
    END_X = 0.0     # <-- TODO
    END_Y = 0.0     # <-- TODO
    END_YAW = 0.0

    node.get_logger().info("Navigating to END")
    node.navigate(END_X, END_Y, END_YAW)

    node.get_logger().info("Mission completed")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
