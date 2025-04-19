# # /turtle1/cmd_vel [geometry_msgs/msg/Twist]
# import rclpy  # ROS 2 파이썬 클라이언트 라이브러리
# from geometry_msgs.msg import Twist  # 거북이의 속도 명령을 나타내는 메시지 타입
# from rclpy.node import Node  # ROS 2의 노드 기본 클래스

# class Move_turtle(Node):  # 'Move_turtle'이라는 이름의 노드 정의
#     def __init__(self):
#         super().__init__('move_turtle')  # 부모 클래스 초기화 및 노드 이름 설정
#         self.create_timer(0.1, self.pub_turtle)  # 0.1초(10Hz)마다 pub_turtle 함수 실행
#         self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)  # /turtle1/cmd_vel 토픽으로 Twist 메시지 발행
#         self.vel = 0.0  # 선속도 초기값 설정

#     def pub_turtle(self):
#         msg = Twist()  # Twist 메시지 객체 생성 (선속도, 각속도 포함)
#         msg.angular.z = 0.5  # z축 회전속도 설정 → 원형 궤적을 위한 회전        # python은 type casting이 자유롭다.
#         msg.linear.x = self.vel  # 현재 선속도를 메시지에 적용                # 하지만 DDS로 넘길 때는 type check가 되어야 한다.
#         self.pub.publish(msg)  # 메시지를 퍼블리시하여 거북이에게 전달
#         self.vel += 0.01  # 선속도를 점차 증가시켜 원을 점점 키움 → 소용돌이 형태

# def main():
#     rclpy.init()  # rclpy 초기화 (ROS 2 사용을 위한 준비)
#     node = Move_turtle()  # Move_turtle 클래스의 인스턴스 생성 (노드 생성)
#     try:
#         rclpy.spin(node)  # 노드를 실행 상태로 유지하며 콜백 함수 처리
#     except KeyboardInterrupt:
#         node.destroy_node()  # Ctrl+C 입력 시 노드 종료

# if __name__== '__main__':  # main 함수가 프로그램의 시작점임을 지정
#     main()

##############################################################################

import rclpy  # ROS 2 파이썬 클라이언트 라이브러리
from rclpy.node import Node  # 모든 노드는 이 클래스를 상속받아 정의됨
from geometry_msgs.msg import Twist  # 거북이에게 줄 선속도 및 각속도 명령 메시지 타입
from turtlesim.msg import Pose  # 거북이의 현재 위치(x, y) 및 방향(theta) 정보를 담는 메시지 타입
import math  # 수학 함수 사용을 위해 import (예: 제곱근, 삼각함수 등)


class DrawSquare(Node):  # 사각형을 그리는 노드 클래스 정의
    def __init__(self):
        super().__init__('draw_square')  # ROS 2 노드 이름을 'draw_square'로 설정
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Twist 메시지를 퍼블리시할 퍼블리셔 생성
        self.subscriber_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)  # Pose를 구독하는 서브스크립션 생성

        self.pose = None  # 현재 거북이의 pose 정보를 저장할 변수
        self.state = 'forward'  # 현재 동작 상태 ('forward' 또는 'turn')
        self.start_x = 0.0  # 직진 시작 지점 x 좌표
        self.start_y = 0.0  # 직진 시작 지점 y 좌표
        self.start_theta = 0.0  # 회전 시작 방향 (라디안)
        self.distance = 2.0  # 각 직선 구간에서 이동할 거리
        self.angle = math.pi / 2  # 회전할 각도 (90도 = π/2 라디안)
        self.turn_count = 0  # 현재까지 몇 번 꺾었는지 (4번이면 사각형 완성)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz 주기로 콜백 실행

    def pose_callback(self, msg):
        self.pose = msg  # 거북이의 현재 pose를 계속 업데이트해 저장

    def timer_callback(self):
        # if self.pose is None or self.turn_count >= 4:
        if self.pose is None:

            return  # pose가 아직 수신되지 않았거나, 사각형을 다 그렸다면 아무 동작 안 함

        msg = Twist()  # 퍼블리시할 Twist 메시지 객체 생성

        if self.state == 'forward':
            # 첫 시작일 경우, 현재 위치를 시작점으로 설정 (정확한 거리 계산용)
            if abs(self.pose.linear_velocity) < 1e-3:  # 움직임이 시작되기 전 속도가 거의 0일 때
                self.start_x = self.pose.x
                self.start_y = self.pose.y

            # 현재 위치와 시작점 간의 거리 계산
            dx = self.pose.x - self.start_x
            dy = self.pose.y - self.start_y
            dist_moved = math.sqrt(dx ** 2 + dy ** 2)

            if dist_moved < self.distance:
                msg.linear.x = 1.0  # 아직 목표 거리 못 갔으면 직진 유지
            else:
                msg.linear.x = 0.0  # 거리 도달 시 멈춤
                self.state = 'turn'  # 다음 상태를 회전으로 전환
                self.start_theta = self.pose.theta  # 회전 시작 시점 방향 저장

        elif self.state == 'turn':
            # 회전한 각도 계산 (현재 방향 - 시작 방향)
            dtheta = self.normalize_angle(self.pose.theta - self.start_theta)

            if abs(dtheta) < self.angle:
                msg.angular.z = 1.0  # 아직 90도 못 돌았으면 회전 계속
            else:
                msg.angular.z = 0.0  # 회전 완료 시 정지
                # self.state = 'forward'  # 다음 상태를 직진으로 전환
                self.turn_count += 1  # 꼭짓점 하나 완료
                if self.turn_count >= 4:
                    self.turn_count = 0       # ⭐️ 꼭짓점 카운터 초기화
                # (선택) 약간의 쉬는 시간 추가 가능
                self.state = 'forward'        # ⭐️ 다시 직선으로 돌아감 → 반복 시작

        self.publisher_.publish(msg)  # 계산된 속도 명령을 퍼블리시

    def normalize_angle(self, angle):
        """
        θ를 -π ~ π 범위로 정규화 (회전이 ±360도 이상인 경우 잘못된 각도 계산 방지)
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    rclpy.init()  # rclpy 초기화
    node = DrawSquare()  # 노드 인스턴스 생성
    try:
        rclpy.spin(node)  # 노드를 실행하고 콜백 루프에 진입
    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료 시 예외 무시
    finally:
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS 클라이언트 종료


if __name__ == '__main__':  # 스크립트가 직접 실행될 경우 main() 호출
    main()