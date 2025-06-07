# ros2 run hello_ros2 dynamic_tf
# rviz2로 확인

import rclpy
import rclpy.logging
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import quaternion_from_euler
# sudo apt install ros-humble-tf-transformations로 설치해야함.

class DynamicFramePublisher(Node):
    def __init__(self):
        # ROS2 노드 초기화, 노드 이름은 "dynamic_tf2_broadcaster"로 설정
        super().__init__("dynamic_tf2_broadcaster")
        # 1/30초(약 0.033초)마다 pub_cb 콜백 함수가 실행되도록 타이머 생성
        self.create_timer(1 / 30, self.pub_cb)
        # TransformBroadcaster 객체 생성 (좌표 변환 정보 송출용)
        self.tf_br = TransformBroadcaster(self)
        # 회전 각도를 누적할 변수 초기화
        self.t = 0.0

    def pub_cb(self):
        # 첫 번째 변환 메시지 생성: world -> map
        t = TransformStamped()
        # 현재 시간으로 타임스탬프 설정
        t.header.stamp = self.get_clock().now().to_msg()
        # 부모 프레임을 "world"로 지정
        t.header.frame_id = "world"
        # 자식 프레임을 "map"으로 지정
        t.child_frame_id = "map"
        # "map" 프레임의 위치를 (1.0, 1.0, 0.0)으로 설정
        t.transform.translation.x = 1.0
        t.transform.translation.y = 1.0
        t.transform.translation.z = 0.0
        # z축(위쪽 방향)으로 self.t 라디안만큼 회전 (roll=0, pitch=0, yaw=self.t)
        quat = quaternion_from_euler(0, 0, self.t)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # 두 번째 변환 메시지 생성: map -> joint
        t2 = TransformStamped()
        # 현재 시간으로 타임스탬프 설정
        t2.header.stamp = self.get_clock().now().to_msg()
        # 부모 프레임을 "map"으로 지정
        t2.header.frame_id = "map"
        # 자식 프레임을 "joint"로 지정
        t2.child_frame_id = "joint"
        # "joint" 프레임의 위치를 (3.0, 0.0, 0.0)으로 설정
        t2.transform.translation.x = 3.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        # 회전 없이 단위 쿼터니언(0,0,0,1)로 설정
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # self.t 값을 1/60씩 증가시켜서 회전 각도를 누적
        self.t += 1 / 60
        # 현재 각도 값을 출력 (디버깅용)
        print(self.t)

        # 변환 메시지들을 브로드캐스터를 통해 송출
        self.tf_br.sendTransform(t)
        self.tf_br.sendTransform(t2)

def main():
    # ROS2 통신 초기화
    rclpy.init()
    # DynamicFramePublisher 노드 객체 생성
    node = DynamicFramePublisher()
    try:
        # 노드가 종료될 때까지 spin (콜백 함수 반복 실행)
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C로 종료 시 노드 정리
        node.destroy_node()

if __name__ == "__main__":
    main()