# ROS 2의 tf 변환을 사용하기 위해 필요한 패키지 설치 명령어입니다.
# sudo apt install ros-humble-tf-transformations

import rclpy  # ROS 2 Python 클라이언트 라이브러리
from geometry_msgs.msg import TransformStamped  # 좌표 변환 메시지 타입
from rclpy import time  # 시간 관련 기능 제공
from rclpy.node import Node  # ROS 2 노드의 기본 클래스
from tf2_ros.buffer import Buffer  # 좌표 변환 정보를 저장하는 버퍼
from tf2_ros.transform_listener import TransformListener  # 변환 정보를 수신하는 리스너

# FrameListener 클래스는 ROS 2 노드를 상속받아 좌표 변환 정보를 주기적으로 받아오는 역할을 합니다.
class FrameListener(Node):
    def __init__(self):
        # 부모 클래스(Node)의 생성자를 호출하며, 노드 이름을 "tf2_listener"로 지정합니다.
        super().__init__("tf2_listener")  # 노드 이름 설정
        # 좌표 변환 정보를 저장할 버퍼 객체 생성
        self.tf_buffer = Buffer()
        # 버퍼와 현재 노드를 이용해 TransformListener 객체 생성 (좌표 변환 정보를 구독)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 0.1초(100ms)마다 on_timer 메소드를 호출하는 타이머 생성
        self.timer = self.create_timer(0.1, self.on_timer)

    # 타이머에 의해 주기적으로 호출되는 콜백 함수입니다.
    def on_timer(self):
        try:
            # "world" 프레임에서 "joint" 프레임으로의 변환 정보를 요청합니다.
            # time.Time()은 최신(가장 최근)의 변환 정보를 요청함을 의미합니다.
            t = self.tf_buffer.lookup_transform("joint", "world", time.Time())
        except Exception:
            # 변환 정보를 찾지 못하면 예외가 발생하며, 로그로 실패 메시지를 출력합니다.
            self.get_logger().info("lookup 실패!!")
            return
        # 변환 정보가 있다면, 변환의 x, y, z 위치 정보를 로그로 출력합니다.
        self.get_logger().info(f"{t.transform.translation.x}")
        self.get_logger().info(f"{t.transform.translation.y}")
        self.get_logger().info(f"{t.transform.translation.z}")

# main 함수는 프로그램의 진입점입니다.
def main():
    # ROS 2 Python 클라이언트 라이브러리 초기화
    rclpy.init()
    # FrameListener 노드 객체 생성
    node = FrameListener()
    try:
        # 노드가 종료될 때까지(예: Ctrl+C) 계속 실행
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 키보드 인터럽트(Ctrl+C) 발생 시 노드 종료
        node.destroy_node()

# 이 파일이 메인 프로그램으로 실행될 때만 main() 함수를 호출합니다.
if __name__ == "__main__":
    main()