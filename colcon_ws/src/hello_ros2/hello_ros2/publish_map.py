# 이 파일은 ROS2(로봇 운영체제 2) 환경에서 OccupancyGrid(점유 격자) 맵을 퍼블리시하는 노드 예제입니다.
# 초심자도 이해할 수 있도록 각 행과 파트별로 상세한 주석을 추가하였습니다.
# 이 코드는 ROS2의 퍼블리셔-서브스크라이버 구조, 메시지 타입, 타이머 사용법, OccupancyGrid 맵 구조 이해에 도움이 됩니다.

import rclpy  # ROS2 Python 클라이언트 라이브러리 임포트. ROS2 노드, 퍼블리셔, 서브스크라이버 등 기능 제공.
from nav_msgs.msg import OccupancyGrid  # OccupancyGrid 메시지 타입 임포트. 맵 데이터를 표현할 때 사용.
from rclpy.node import Node  # ROS2 노드의 기본 클래스 임포트.

class PublishMap(Node):  # ROS2 노드를 정의하는 클래스. Node를 상속받아야 ROS2에서 동작 가능.
    def __init__(self):
        super().__init__("publish_map")  # 부모 클래스(Node) 초기화, 노드 이름을 "publish_map"으로 지정.
        # 타이머 생성: 0.001초(1ms)마다 pub_cb 콜백 함수 실행. 빠른 주기로 맵을 퍼블리시함.
        # 타이머를 쓰는 이유: ROS2에서 주기적으로 작업(예: 센서 데이터, 맵 퍼블리시 등)을 수행할 때 사용.
        self.create_timer(0.001, self.pub_cb)
        # 퍼블리셔 생성: "/map" 토픽에 OccupancyGrid 타입 메시지를 퍼블리시. 큐 사이즈는 100.
        # 퍼블리셔는 ROS2에서 데이터를 다른 노드로 보내는 역할.
        self.pub = self.create_publisher(OccupancyGrid, "/map", 100)
        # OccupancyGrid 메시지 객체 생성. 퍼블리시할 맵 데이터 저장.
        self.msg = OccupancyGrid()
        # OccupancyGrid 메시지의 헤더 설정. 좌표계(frame_id)는 "odom"으로 지정.
        # "odom"은 일반적으로 로봇의 위치 추정 좌표계로 사용됨.
        self.msg.header.frame_id = "odom"
        # 맵 해상도 설정(셀 하나의 실제 크기, 단위: m). 0.1m(10cm)로 설정.
        self.msg.info.resolution = 0.1
        # 맵의 가로(너비) 셀 개수. 200개.
        self.msg.info.width = 200
        # 맵의 세로(높이) 셀 개수. 100개.
        self.msg.info.height = 100
        # 맵의 원점(origin) 위치(x, y, z). (0, 0, 0)으로 설정.
        self.msg.info.origin.position.x = 0.0
        self.msg.info.origin.position.y = 0.0
        self.msg.info.origin.position.z = 0.0
        # 맵의 원점 방향(orientation, 쿼터니언). (0, 0, 0, 1)은 회전 없음.
        self.msg.info.origin.orientation.x = 0.0
        self.msg.info.origin.orientation.y = 0.0
        self.msg.info.origin.orientation.z = 0.0
        self.msg.info.origin.orientation.w = 1.0

        # 맵 데이터(data) 초기화. OccupancyGrid의 data는 1차원 리스트로, 각 셀의 점유 상태(-1: 미확인, 0: 비어있음, 100: 점유).
        # 처음 10,000개(100x100)는 100(점유)로, 그 다음 10,000개는 0(비어있음)으로 설정.
        # 실제 맵 크기(200x100=20,000)에 맞게 데이터 길이 맞춤.
        self.msg.data = [100 for _ in range(10_000)]  # 상단 절반은 점유(100)
        self.msg.data.extend([0 for _ in range(10_000)])  # 하단 절반은 비어있음(0)

        # 셀을 순회하며 값을 바꿀 때 사용할 인덱스 변수 초기화.
        self.count = 0  # 현재 열 인덱스
        self.row = 0    # 현재 행 인덱스

    def pub_cb(self):
        # 콜백 함수: 타이머에 의해 주기적으로 호출됨.
        # 맵 메시지의 헤더에 현재 시간(타임스탬프) 갱신. ROS2 메시지는 최신 시간 정보가 중요함.
        self.msg.header.stamp = self.get_clock().now().to_msg()

        # 현재 셀의 1차원 인덱스 계산. (row * width) + count
        index = self.count + (self.msg.info.width * self.row)
        # 해당 셀의 값이 -1(미확인)이면 100(점유)로, 아니면 -1(미확인)으로 토글.
        # 맵의 일부 셀을 주기적으로 바꿔서 맵이 동적으로 변하는 예시를 보여줌.
        if self.msg.data[index] == -1:
            self.msg.data[index] = 100
        else:
            self.msg.data[index] = -1

        # 다음 셀로 이동. 열 인덱스 증가.
        self.count += 1
        # 열 인덱스가 맵 너비를 넘으면 0으로 초기화하고, 행 인덱스 증가.
        if self.count >= self.msg.info.width:
            self.count = 0
            self.row += 1
        # 행 인덱스가 맵 높이를 넘으면 0으로 초기화(전체 맵 순환).
        if self.row >= self.msg.info.height:
            self.row = 0

        # OccupancyGrid 메시지를 "/map" 토픽으로 퍼블리시.
        self.pub.publish(self.msg)

def main():
    # ROS2 노드 초기화. 여러 노드가 동시에 실행될 수 있음.
    rclpy.init()
    node = PublishMap()  # PublishMap 노드 인스턴스 생성.
    try:
        rclpy.spin(node)  # 노드가 종료될 때까지(예: Ctrl+C) 이벤트 루프 실행.
    except KeyboardInterrupt:
        node.destroy_node()  # 노드 종료 시 리소스 정리.

if __name__ == "__main__":
    main()  # 스크립트가 직접 실행될 때 main() 함수 호출.

# 전체 요약:
# 이 코드는 ROS2에서 OccupancyGrid 맵을 "/map" 토픽으로 퍼블리시하는 예제입니다.
# 타이머를 사용해 주기적으로 맵 데이터를 갱신하고, 일부 셀의 값을 토글하여 맵이 동적으로 변하는 모습을 보여줍니다.
# OccupancyGrid 메시지 구조, 퍼블리셔 생성, 타이머 사용법 등 ROS2 초심자가 실습하기에 적합합니다.
# 타이머 대신 while 루프+sleep을 쓸 수도 있지만, ROS2에서는 타이머가 이벤트 기반으로 더 효율적이고, 콜백 구조와 잘 어울립니다.
# 맵 데이터의 구조(1차원 리스트, 셀 인덱싱), 헤더 정보(좌표계, 타임스탬프) 등도 실제 로봇 맵핑에 필수적인 개념입니다.