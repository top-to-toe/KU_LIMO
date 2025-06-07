# ROS2와 turtlesim 패키지를 활용하여 한 거북이가 다른 거북이를 따라가도록 하는 예제 코드입니다.
# 초보자도 이해할 수 있도록 각 줄과 함수별로 상세한 주석을 추가하였습니다.

# 필요한 패키지 설치 명령어 (주석)
# sudo apt install ros-humble-tf-transformations
# ros2 run turtlesim turtlesim_node                 # turtlesim 시뮬레이터 실행
# ros2 run hello_ros2 follow_turtlesim              # 본 코드 실행
# rviz2 -> tf 확인                                   # rviz2에서 tf 프레임 확인
# ros2 run turtlesim turtle_teleop_key              # 키보드로 거북이 조작

import rclpy  # ROS2 Python 클라이언트 라이브러리
from geometry_msgs.msg import TransformStamped, Twist  # 메시지 타입 임포트
from rclpy import time
from rclpy.node import Node  # ROS2 노드 클래스
from tf2_ros.buffer import Buffer  # tf2 변환 정보를 저장하는 버퍼
from tf2_ros.transform_broadcaster import TransformBroadcaster  # tf2 변환 브로드캐스터
from tf2_ros.transform_listener import TransformListener  # tf2 변환 리스너
from tf_transformations import euler_from_quaternion, quaternion_from_euler  # 쿼터니언-오일러 변환 함수
from turtlesim.msg import Pose  # turtlesim의 위치 메시지
from turtlesim.srv import Spawn  # turtlesim의 거북이 생성 서비스

# 거북이 따라가기 노드 클래스 정의
class Follow_turtle(Node):
    def __init__(self):
        # ROS2 노드 초기화, 노드 이름은 "follow_turtle"
        super().__init__("follow_turtle")
        
        # tf2 변환 정보를 저장할 버퍼 생성
        self.tf_buffer = Buffer()
        # tf2 변환 리스너 생성 (버퍼와 노드 연결)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 0.1초(10Hz)마다 on_timer 함수 실행하는 타이머 생성
        self.timer = self.create_timer(0.1, self.on_timer)
        
        # turtlesim의 Spawn 서비스 클라이언트 생성 (새 거북이 생성용)
        self.spawner = self.create_client(Spawn, "spawn")
        # Spawn 서비스 요청 메시지 생성 및 값 설정 (x=3, y=3, theta=0)
        request = Spawn.Request()
        request.x = 3.0
        request.y = 3.0
        request.theta = 0.0
        # 비동기로 서비스 호출 (거북이2 생성)
        self.result = self.spawner.call_async(request)
        print("done")  # 거북이 생성 요청 완료 출력
        
        # tf2 변환 브로드캐스터 생성 (프레임 정보 송신용)
        self.tf_br = TransformBroadcaster(self)
        
        # turtle1의 pose 토픽 구독 (거북이1 위치 정보 수신)
        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.sub_cb, 10)
        # turtle2의 pose 토픽 구독 (거북이2 위치 정보 수신)
        self.sub2 = self.create_subscription(Pose, "/turtle2/pose", self.sub_cb2, 10)
        
        # turtle2의 cmd_vel 토픽 퍼블리셔 생성 (거북이2 속도 명령 송신)
        self.pub = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

    # turtle1의 pose 메시지 콜백 함수
    def sub_cb(self, msg: Pose):
        # TransformStamped 메시지 생성 (프레임 변환 정보)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  # 현재 시간
        t.header.frame_id = "world"  # 부모 프레임: world
        t.child_frame_id = "turtle1"  # 자식 프레임: turtle1
        t.transform.translation.x = msg.x  # x 위치
        t.transform.translation.y = msg.y  # y 위치
        t.transform.translation.z = 0.0    # 2D이므로 z는 0
        # 오일러 각(theta)을 쿼터니언으로 변환하여 회전 정보 설정
        quat = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        # 변환 정보 브로드캐스트 (퍼블리시)
        self.tf_br.sendTransform(t)

    # turtle2의 pose 메시지 콜백 함수 (위와 동일, 프레임만 turtle2)
    def sub_cb2(self, msg: Pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "turtle2"
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_br.sendTransform(t)

    # 타이머 콜백 함수: 주기적으로 turtle1과 turtle2의 상대 위치 계산 및 제어 명령 송신
    def on_timer(self):
        try:
            # turtle2에서 turtle1로의 변환(상대 위치/자세) 조회
            t = self.tf_buffer.lookup_transform("turtle1", "turtle2", time.Time())
        except Exception:
            # 변환 조회 실패 시 로그 출력 후 함수 종료
            self.get_logger().info("lookup 실패!!")
            return
        # 상대 위치 정보 로그 출력 (디버깅용)
        self.get_logger().info(f"{t.transform.translation.x}")
        self.get_logger().info(f"{t.transform.translation.y}")
        self.get_logger().info(f"{t.transform.translation.z}")
        
        # Twist 메시지 생성 (속도 명령)
        msg = Twist()
        # 상대 회전(쿼터니언)을 오일러 각으로 변환하여 각속도에 할당
        angular = euler_from_quaternion(
            (
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )
        )
        msg.angular.x = angular[0]
        msg.angular.y = angular[1]
        msg.angular.z = angular[2]
        # 상대 위치(x, y)를 더해서 직진 속도에 할당 (간단한 추종 제어)
        msg.linear.x = t.transform.translation.x + t.transform.translation.y
        # turtle2에 속도 명령 퍼블리시
        self.pub.publish(msg)

# 메인 함수: ROS2 노드 실행
def main():
    rclpy.init()  # ROS2 초기화
    node = Follow_turtle()  # 노드 객체 생성
    try:
        rclpy.spin(node)  # 노드 실행 (콜백 함수 반복)
    except KeyboardInterrupt:
        node.destroy_node()  # Ctrl+C 시 노드 종료

# 이 파일이 메인으로 실행될 때만 main() 실행
if __name__ == "__main__":
    main()