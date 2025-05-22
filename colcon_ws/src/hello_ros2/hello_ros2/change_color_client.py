# service client 을 만드세요./turtle1/set_pen [turtlesim/srv/SetPen]
# service 이름 타입(turtlesim_node color 변경) 실시간으로 다양한 색상
# launch  파일에 노드 추가 moveTurtle.launch.py

# move_turtle.py -> 파라미터를 설정 각속도 움직임을 외부에서 변화!
# 위 파라미터도 yaml 에 넣어서 작동!

import random
from asyncio import Future

import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen


class Color_client(Node):
    def __init__(self):
        super().__init__("color_client")  # 노드 이름
        self.client = self.create_client(SetPen, "turtle1/set_pen")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")
        self.create_timer(0.3, self.update)
        self.request = SetPen.Request()
        self.future = Future()

    def update(self):
        self.request.b = random.randint(0, 255)
        self.request.g = random.randint(0, 255)
        self.request.r = random.randint(0, 255)
        self.send_request()

    def send_request(self):
        self.get_logger().info(
            f"{self.request.r} {self.request.g} {self.request.b} :RGB 요청"
        )
        self.future = self.client.call_async(self.request)
        self.future.add_done_callback(self.done_callback)

    def done_callback(self, future):
        _ = future.result()
        self.get_logger().info("처리 완료!")


def main():
    rclpy.init()
    node = Color_client()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
