#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate

class MapNode(Node):
    def __init__(self):
        super().__init__("drone_map")
        self.costmap_sub = self.create_subscription(OccupancyGridUpdate, "costmap_updates", self.callback_costmap_sub, 10)

    def callback_costmap_sub(self, msg:OccupancyGridUpdate):
        map_grid = msg
        data = map_grid.data
        print(data)

def main(args=None):
    rclpy.init(args=args)
    node = MapNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()