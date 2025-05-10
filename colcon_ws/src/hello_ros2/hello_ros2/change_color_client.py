# service client를 만드세요.
# service 이름 타입(turtlesim_node color 변경) 실시간으로 다양한 색상 그릴 수 있게...

# launch 파일에 노드 추가 moveTurtle.launch.py
# move_turtle.py -> 파라미터를 설정 각속도 움직임을 외부에서 변화! (param set)
# 위 파라미터도 yaml에 넣어서 작동할 수 있게..

import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
import random

class ColorClient(Node):
    def __init__(self):
        super().__init__('service_client')

        # set_pen 서비스 클라이언트 생성
        self.client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen 서비스 대기 중...')

        # 펜 색상 변경 타이머 (0.5초마다 색상 변경)
        self.create_timer(0.1, self.change_color)

    def change_color(self):
        req = SetPen.Request()
        req.r = random.randint(0, 255)
        req.g = random.randint(0, 255)
        req.b = random.randint(0, 255)
        req.width = 3
        req.off = False

        self.get_logger().info(f'색상 변경: RGB({req.r}, {req.g}, {req.b})')
        future = self.client.call_async(req)
        future.add_done_callback(self.color_callback)

    def color_callback(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'색상 변경 실패: {e}')

def main():
    rclpy.init()
    node = ColorClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()