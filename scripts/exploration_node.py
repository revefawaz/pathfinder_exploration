#!/usr/bin/env python3
# File: pathfinder/scripts/exploration_node.py

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf2_ros
from tf2_ros import Buffer, TransformListener, LookupException
from collections import deque

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # Params
        self.declare_parameter('goal_timeout', 30.0)
        self.declare_parameter('min_frontier_distance', 0.5)

        # State
        self.current_map = None
        self.exploring   = False
        self.visited     = deque(maxlen=50)

        # Subscribers
        self.create_subscription(OccupancyGrid, 'map', self.map_cb, 1)

        # TF
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for NavigateToPose server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available, shutting down')
            rclpy.shutdown()
            return

        # Exploration loop
        self.create_timer(1.0, self.explore)

    def map_cb(self, msg):
        self.current_map = msg

    def get_robot_cell(self):
        if not self.current_map:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.current_map.header.frame_id,
                'base_link',
                Time())
        except LookupException:
            return None
        info = self.current_map.info
        rx = int((tf.transform.translation.x - info.origin.position.x) / info.resolution)
        ry = int((tf.transform.translation.y - info.origin.position.y) / info.resolution)
        if 0 <= rx < info.width and 0 <= ry < info.height:
            return rx, ry
        return None

    def find_frontiers(self):
        info = self.current_map.info
        w, h = info.width, info.height
        grid = np.array(self.current_map.data).reshape((h, w))
        free    = (grid == 0)
        unknown = (grid == -1)

        rc = self.get_robot_cell()
        if rc is None:
            return None
        rx, ry = rc

        # BFS to build reachable mask
        reachable = np.zeros((h, w), bool)
        dq = deque([(rx, ry)])
        reachable[ry, rx] = True
        while dq:
            cx, cy = dq.popleft()
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = cx+dx, cy+dy
                if (0 <= nx < w and 0 <= ny < h and
                    free[ny, nx] and not reachable[ny, nx]):
                    reachable[ny, nx] = True
                    dq.append((nx, ny))

        # Collect frontier pixels
        frontiers = []
        for y in range(1, h-1):
            for x in range(1, w-1):
                if free[y,x] and reachable[y,x] and unknown[y-1:y+2, x-1:x+2].any():
                    frontiers.append((x, y))
        return frontiers

    def grid_to_world(self, x, y):
        info = self.current_map.info
        wx = x * info.resolution + info.origin.position.x
        wy = y * info.resolution + info.origin.position.y
        ps = PoseStamped()
        ps.header.frame_id = self.current_map.header.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = wx
        ps.pose.position.y = wy
        ps.pose.orientation.w = 1.0
        return ps

    def explore(self):
        if self.exploring or not self.current_map:
            return

        frontiers = self.find_frontiers()
        if frontiers is None:
            return  # TF or map not ready
        if not frontiers:
            self.get_logger().info('Exploration complete')
            rclpy.shutdown()
            return

        # Filter visited
        candidates = [f for f in frontiers if f not in self.visited]
        if not candidates:
            self.visited.clear()
            candidates = frontiers

        # Pick the farthest frontier
        rc = self.get_robot_cell()
        if rc is None:
            return
        rx, ry = rc
        fx, fy = max(candidates, key=lambda p: (p[0]-rx)**2 + (p[1]-ry)**2)

        # Convert and check distance
        goal = self.grid_to_world(fx, fy)
        dx = goal.pose.position.x - (rx * self.current_map.info.resolution + self.current_map.info.origin.position.x)
        dy = goal.pose.position.y - (ry * self.current_map.info.resolution + self.current_map.info.origin.position.y)
        dist = (dx*dx + dy*dy)**0.5

        if dist < self.get_parameter('min_frontier_distance').value:
            # too close → mark visited and skip
            self.visited.append((fx, fy))
            return

        # Send goal
        self.visited.append((fx, fy))
        self.get_logger().info(
            f'Sending goal to grid ({fx},{fy}) → world '
            f'({goal.pose.position.x:.2f},{goal.pose.position.y:.2f})'
        )
        self.exploring = True
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        self.nav_client.send_goal_async(nav_goal).add_done_callback(self.goal_response)

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted: 
            self.get_logger().error('Goal rejected')
            self.exploring = False
            return
        handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f'Navigation result: {status}')
        self.exploring = False
        timeout = self.get_parameter('goal_timeout').value
        self.create_timer(timeout, self.explore)

def main():
    rclpy.init()
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
