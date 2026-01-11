#!/usr/bin/env python3
"""
show_drone_explore.py
---------------------
Live 2D map viewer for EXPLORE mode (no goal).
Subscribes to /simple_drone/gt_pose and draws the trajectory over an occupancy map.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import yaml
import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
from collections import deque


class MapExploreViewer(Node):
    def __init__(self):
        super().__init__('map_explore_viewer')

        # === Load map YAML (must contain image, resolution, origin) ===
        self.declare_parameter("map_yaml", "/ros2_ws/maps/playground_map.yaml")
        map_yaml_path = str(self.get_parameter("map_yaml").value)

        with open(map_yaml_path, 'r') as f:
            info = yaml.safe_load(f)

        self.resolution = float(info['resolution'])
        self.origin = info['origin']  # [ox, oy, yaw]

        map_image_path = info['image']
        if not map_image_path.startswith('/'):
            map_image_path = os.path.join(os.path.dirname(map_yaml_path), map_image_path)

        img = cv2.imread(map_image_path, cv2.IMREAD_UNCHANGED)
        if img is None:
            raise FileNotFoundError(f"Failed to load map image: {map_image_path}")

        # Build occupancy-like binary map for display (works for PGM)
        img_u8 = img.astype(np.uint8)
        self.map_data = np.zeros_like(img_u8, dtype=np.uint8)
        self.map_data[img_u8 < 50] = 1  # obstacles -> 1, free -> 0 (viz only)
        self.map_data = np.flipud(self.map_data)  # align with origin='lower'

        h, w = self.map_data.shape[:2]
        self.get_logger().info(
            f"Map loaded: {map_image_path}, size=({w},{h}), res={self.resolution}, origin={self.origin}"
        )

        # === Drone pose + trajectory ===
        self.drone_pose_world = None
        self.last_added_map = None

        self.traj = deque(maxlen=20000)
        self.min_step_px = 1

        self.pose_sub = self.create_subscription(
            Pose, '/simple_drone/gt_pose', self.pose_callback, 10
        )

        # === Matplotlib interactive window ===
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.imshow(self.map_data, cmap='gray', origin='lower')
        self.ax.set_title("Drone explore trajectory on occupancy map")

        # trajectory line + current position marker
        (self.traj_line,) = self.ax.plot([], [], '-', linewidth=2)
        (self.curr_point,) = self.ax.plot([], [], 'o', markersize=5)

        plt.show(block=False)

        # UI update timer
        self.create_timer(0.05, self.update_display)

        self.get_logger().info("EXPLORE viewer ready. Waiting for /simple_drone/gt_pose messages...")

    def pose_callback(self, msg: Pose):
        xw, yw = msg.position.x, msg.position.y
        xm, ym = self.world_to_map(xw, yw)

        h, w = self.map_data.shape[:2]
        self.get_logger().debug(
            f"[DBG] world=({xw:.3f},{yw:.3f}) -> map_px=({xm},{ym}) | res={self.resolution} origin={self.origin} size=({w},{h})"
        )

        self.drone_pose_world = (xw, yw)

        # Add to trajectory only if moved enough (in pixels)
        if self.last_added_map is None:
            self.traj.append((xm, ym))
            self.last_added_map = (xm, ym)
            return

        dx = abs(xm - self.last_added_map[0])
        dy = abs(ym - self.last_added_map[1])
        if dx >= self.min_step_px or dy >= self.min_step_px:
            self.traj.append((xm, ym))
            self.last_added_map = (xm, ym)

    # === Conversions ===
    def world_to_map(self, x_world, y_world):
        ox, oy, _ = self.origin
        x = int(round((x_world - ox) / self.resolution))
        y = int(round((y_world - oy) / self.resolution))
        return x, y

    def map_to_world(self, x_map, y_map):
        ox, oy, _ = self.origin
        x_world = x_map * self.resolution + ox
        y_world = y_map * self.resolution + oy
        return x_world, y_world

    def update_display(self):
        if self.drone_pose_world is None:
            self.fig.canvas.flush_events()
            return

        xw, yw = self.drone_pose_world
        x_map, y_map = self.world_to_map(xw, yw)

        # current point
        self.curr_point.set_data([x_map], [y_map])

        # trajectory polyline
        if len(self.traj) >= 2:
            xs = [p[0] for p in self.traj]
            ys = [p[1] for p in self.traj]
            self.traj_line.set_data(xs, ys)

        self.ax.set_title(
            f"EXPLORE | Drone=({xw:.2f},{yw:.2f}) | points={len(self.traj)}"
        )
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


def main():
    rclpy.init()
    node = MapExploreViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
