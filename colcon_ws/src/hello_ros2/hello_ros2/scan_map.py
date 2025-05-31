# ros2 topic echo /map --once
# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom - tf 임이 발행
# ros2 run hello_ros2 publish_map

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class PublishMap(Node):
    def __init__(self):
        super().__init__("publish_map")  # node name
        self.create_timer(0.001, self.pub_cb)
        self.create_timer(1 / 60, self.update)
        self.pub = self.create_publisher(OccupancyGrid, "/map", 100)
        self.msg = OccupancyGrid()
        self.msg.header.frame_id = "odom"
        self.msg.info.resolution = 0.1
        self.msg.info.width = 200
        self.msg.info.height = 100
        self.msg.info.origin.position.x = 0.0
        self.msg.info.origin.position.y = 0.0
        self.msg.info.origin.position.z = 0.0
        self.msg.info.origin.orientation.x = 0.0
        self.msg.info.origin.orientation.y = 0.0
        self.msg.info.origin.orientation.z = 0.0
        self.msg.info.origin.orientation.w = 1.0

        # subscribe Scan topic
        self.create_subscription(LaserScan, "scan", self.laser_cb, 10)

        self.msg.data = [100 for _ in range(10_000)]
        self.msg.data.extend([0 for _ in range(10_000)])

        self.count = 0
        self.row = 0
        self.laser_s = LaserScan()

    def update(self):
        # 알고리즘 추가
        # self.laser_s .... .... .....
        # .. 처리하는 코드..
        # self.msg
        pass

    def laser_cb(self, msg):
        self.laser_s = msg

    def pub_cb(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()

        index = self.count + (self.msg.info.width * self.row)
        if self.msg.data[index] == -1:
            self.msg.data[index] = 100
        else:
            self.msg.data[index] = -1

        self.count += 1
        if self.count >= self.msg.info.width:
            self.count = 0
            self.row += 1
        if self.row >= self.msg.info.height:
            self.row = 0

        self.pub.publish(self.msg)

def main():
    rclpy.init()
    node = PublishMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()