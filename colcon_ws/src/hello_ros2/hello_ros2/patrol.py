# ROS2에서 TurtleBot3 시뮬레이션 및 네비게이션을 실행하는 명령어 예시
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/$(USER)/kuLimo/map.yaml
# initial pose(초기 위치) 잡아서 amcl 활성화(navigation 창 상단 2D Pose Estimate 활용)
# ros2 run hello_ros2 patrol

import math  # 수학 함수(삼각함수 등) 사용을 위한 모듈
import time  # 시간 관련 함수 사용을 위한 모듈

import rclpy  # ROS2 Python 클라이언트 라이브러리
from action_msgs.msg import GoalStatus  # 액션의 상태(성공, 실패 등) 메시지 타입
from geometry_msgs.msg import PoseStamped  # 위치와 자세 정보를 담는 메시지 타입
from nav2_msgs.action import FollowWaypoints  # 네비게이션2에서 여러 경유지를 따라가는 액션 타입
from nav2_msgs.action._follow_waypoints import FollowWaypoints_GetResult_Response  # FollowWaypoints 액션의 결과 타입
from rclpy.action import ActionClient  # 액션 클라이언트(명령을 보내는 쪽) 클래스
from rclpy.action.client import ClientGoalHandle  # 액션 목표 핸들(GoalHandle) 클래스
from rclpy.node import Node  # ROS2 노드 기본 클래스
from rclpy.task import Future  # 비동기 작업의 결과를 담는 Future 객체

class Action_client(Node):
    """
    TurtleBot3가 여러 경유지(waypoints)를 순서대로 순찰(patrol)하도록 하는 ROS2 액션 클라이언트 노드 클래스
    """
    def __init__(self):
        super().__init__("patrol")  # 노드 이름을 'patrol'로 지정하여 Node 초기화
        # 'follow_waypoints' 액션 서버와 통신할 액션 클라이언트 생성
        self.action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.future = Future()  # 액션 목표 전송 결과를 받을 Future 객체
        self.get_result_future = Future()  # 액션 결과를 받을 Future 객체

        # 순찰할 경유지 좌표 리스트 (map 좌표계 기준, 각 튜플은 (x, y))
        self.patrol_points = [(4.0, 0.0), (4.0, 1.0), (2.0, 2.5), (0.0, 1.0)]
        # 각 경유지에서 바라볼 방향(각도, 도 단위)
        self.patrol_degree = [0, 90, 180, 90]
        self.patrol_index = 0  # 현재 순찰 경유지 인덱스

        self.goal = FollowWaypoints.Goal()  # 액션 목표 객체 생성
        self.go_next()  # 첫 번째 경유지로 이동 명령 전송

    def go_next(self):
        """
        다음 경유지로 이동 명령을 전송하는 함수
        """
        # 현재 인덱스의 경유지 좌표와 각도를 꺼내서 send_goal 함수로 전달
        self.send_goal(
            self.patrol_points[self.patrol_index][0],  # x 좌표
            self.patrol_points[self.patrol_index][1],  # y 좌표
            self.patrol_degree[self.patrol_index],     # 방향(각도)
        )
        # 다음 경유지로 인덱스 증가, 마지막이면 0으로 순환
        self.patrol_index += 1
        if self.patrol_index >= len(self.patrol_points):
            self.patrol_index = 0

    def send_goal(self, x: float, y: float, theta: int):
        """
        주어진 좌표(x, y)와 방향(theta)로 이동하는 목표를 액션 서버에 전송하는 함수
        """
        pose = PoseStamped()  # 위치와 자세 정보를 담는 메시지 객체 생성
        pose.header.frame_id = "map"  # 좌표계는 'map' 기준
        pose.header.stamp = self.get_clock().now().to_msg()  # 현재 시간(타임스탬프) 설정

        # 위치 설정
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0  # 2D 평면이므로 z는 0

        # 방향(오일러 각 -> 쿼터니언 변환)
        rad = math.radians(theta)  # 도(degree)를 라디안(radian)으로 변환
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(rad / 2.0)
        pose.pose.orientation.w = math.cos(rad / 2.0)

        self.goal.poses.clear()  # 이전 목표(경유지) 초기화
        self.goal.poses.append(pose)  # 새 목표 추가

        # 액션 서버가 준비될 때까지 대기(1초마다 확인)
        while not self.action_client.wait_for_server(timeout_sec=1):
            self.get_logger().info("nav2 서버 접속중 ...")

        # 액션 목표를 비동기로 전송, 결과는 future에 저장
        self.future: Future = self.action_client.send_goal_async(
            self.goal, feedback_callback=self.feedback_callback
        )
        # 목표 전송 결과가 오면 goal_response_callback 함수 호출
        self.future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        """
        액션 목표가 서버에 정상적으로 접수되었는지 확인하는 콜백 함수
        """
        goal_handle: ClientGoalHandle = future.result()  # 결과에서 GoalHandle 추출
        if not goal_handle.accepted:
            self.get_logger().info("골이 접수 안 되었습니다.")  # 목표가 거부된 경우
            return
        # 목표가 접수되면 결과를 비동기로 요청하고, 완료 시 done_callback 호출
        self.get_result_future: Future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.done_callback)

    def feedback_callback(self, msg):
        """
        액션 서버로부터 중간 피드백을 받을 때 호출되는 함수
        """
        feedback: FollowWaypoints.Feedback = msg.feedback  # 피드백 메시지 추출
        # 현재까지 도달한 경유지 인덱스 출력
        self.get_logger().info(f" 지금까지 처리 결과 seq{feedback.current_waypoint}")

    def done_callback(self, future: Future):
        """
        액션이 완료(성공/실패)되었을 때 호출되는 콜백 함수
        """
        result: FollowWaypoints_GetResult_Response = future.result()  # 결과 메시지 추출
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            # 목표 성공 시, 누락된 경유지 정보와 함께 성공 메시지 출력
            self.get_logger().info(f"result: {result.result.missed_waypoints} 성공!!")
            self.go_next()  # 다음 경유지로 이동
        if result.status == GoalStatus.STATUS_ABORTED:
            # 목표 실패(중단) 시 메시지 출력
            self.get_logger().info(f"result: aborted 실패!!")

def main():
    """
    ROS2 노드를 초기화하고, Action_client 노드를 실행하는 메인 함수
    """
    rclpy.init()  # ROS2 통신 초기화
    node = Action_client()  # Action_client 노드 생성
    try:
        rclpy.spin(node)  # 노드가 종료될 때까지 이벤트 루프 실행
    except KeyboardInterrupt:
        node.destroy_node()  # Ctrl+C 등으로 종료 시 노드 정리

if __name__ == "__main__":
    main()  # 메인 함수 실행