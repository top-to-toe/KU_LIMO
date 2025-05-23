#!/usr/bin/env python3
# 쉬뱅(Shebang) 라인: 이 스크립트가 파이썬3 인터프리터로 실행되어야 함을 운영체제에 알려주는 역할임.

# 이 주석들은 이 스크립트들을 ROS2에서 실행하는 명령어를 예시로 보여줌.
# ros2 run hello_ros2 my_topic_pub # 이 노드를 실행하는 명령어임.
# ros2 topic echo /message         # '/message' 토픽으로 발행되는 메시지를 터미널에서 확인하는 명령어임.

import rclpy
# ROS2 파이썬 클라이언트 라이브러리인 'rclpy'를 가져옴.
# [보충 설명]: ROS1의 'rospy'와 비슷한 역할을 하지만, ROS2 환경에 맞춰 재설계되었음.

from rclpy.node import Node
# 'rclpy'에서 노드(Node) 클래스를 가져옴.
# [보충 설명]: 모든 ROS2 노드는 이 Node 클래스를 상속받아야 함.

from user_interface.msg import MyTopic
# 우리가 만든 'MyTopic.msg' 파일에서 정의한 메시지 타입인 'MyTopic'을 가져옴.
# [보충 설명]: 이 메시지 타입은 빌드 과정에서 파이썬 클래스로 자동 생성됨.

# Simple_pub 이라는 이름의 클래스를 정의함.
# 이 클래스가 우리가 만들 퍼블리셔(Publisher) 노드의 '설계도'임.
class Simple_pub(Node):
    def __init__(self):
        # '__init__' 메서드는 Simple_pub 객체를 만들 때 가장 먼저 실행되는 초기 설정 부분임.

        # 1. 노드 초기화:
        # 부모 클래스인 Node의 생성자를 호출해서 노드를 초기화함.
        # 노드의 이름은 "myTopicPub"으로 설정됨.
        super().__init__("myTopicPub")  # node name

        # 2. 타이머 생성:
        # 0.1초마다 'self.pub_turtle' 함수를 반복해서 실행하도록 타이머를 설정함.
        # [보충 설명]: 이 타이머 콜백에서 주기적으로 메시지를 발행할 거임.
        self.create_timer(0.1, self.pub_turtle)

        # 3. 퍼블리셔 생성:
        # 'MyTopic' 타입의 메시지를 '/message'라는 이름의 토픽으로 발행할 퍼블리셔를 만듦.
        # 마지막 '10'은 큐 사이즈(queue size)인데, 발행할 메시지가 너무 많을 때
        # 몇 개까지 큐에 저장할지 정하는 값임. (ROS2에서는 'reliability', 'durability' 같은 'QoS 설정'의 일부임)
        self.pub = self.create_publisher(MyTopic, "/message", 10)
        # [보충 설명]: create_publisher(메시지_타입, 토픽_이름, QoS_프로파일) 형태로 사용함.

        self.count = 0 # 메시지에 포함할 카운터 변수를 0으로 초기화함.

    # 0.1초마다 호출되는 타이머 콜백 함수임.
    # 이 함수 안에서 메시지를 만들고 발행하는 작업을 수행함.
    def pub_turtle(self):
        msg = MyTopic() # 'MyTopic' 타입의 빈 메시지 객체를 만듦.

        # 1. 메시지 필드에 값 할당:
        msg.a = 10 # 메시지의 'a' 필드에 10을 할당함.
        msg.b = 20 # 메시지의 'b' 필드에 20을 할당함.

        # 헤더(Header) 필드 채우기:
        # 메시지가 발행되는 현재 시간 정보를 헤더의 'stamp' 필드에 할당함.
        # [보충 설명]: self.get_clock().now().to_msg()는 현재 ROS2 노드의 시간을 ROS 메시지 형태로 변환해줌.
        msg.header.stamp = self.get_clock().now().to_msg()
        # 메시지의 'frame_id' 필드에 어떤 프레임과 관련된 메시지인지 문자열로 할당함.
        msg.header.frame_id = f"내가 만든 토픽 입니다.{self.count}"
        # [보충 설명]: frame_id는 주로 로봇의 센서 데이터가 어떤 좌표계 기준인지 알려줄 때 사용되지만,
        #              여기서는 예시로 메시지 발행 횟수를 포함한 문자열을 넣었음.

        # 2. 메시지 발행:
        self.pub.publish(msg) # 위에서 만든 메시지 객체를 '/message' 토픽으로 발행함.
        self.count += 1       # 카운터를 1 증가시킴.

# 'main' 함수는 이 ROS2 노드를 시작하고 전체 실행 로직을 담고 있음.
def main():
    rclpy.init() # rclpy 라이브러리를 초기화함. 모든 ROS2 파이썬 노드에서 가장 먼저 호출해야 함.

    node = Simple_pub() # Simple_pub 클래스의 설계도를 바탕으로 'node'라는 실제 객체를 만듦.
                        # 이 순간 '__init__' 함수가 실행되어 노드와 퍼블리셔, 타이머가 설정됨.

    try:
        # ROS2 노드가 종료되지 않는 한 계속 실행되면서 타이머 콜백을 기다림.
        rclpy.spin(node)
        # [보충 설명]: rclpy.spin()은 파이썬 ROS2 노드가 활성 상태를 유지하며
        #              퍼블리셔, 서브스크라이버, 서비스, 액션 등 모든 콜백이 호출되기를 기다리도록 함.
    except KeyboardInterrupt:
        # Ctrl+C 등으로 노드가 중단될 때 발생하는 KeyboardInterrupt 예외를 처리함.
        node.destroy_node() # 노드를 깔끔하게 종료함.

# 이 부분이 파이썬 스크립트가 직접 실행될 때만 main() 함수를 호출하도록 하는 표준 파이썬 관용구임.
if __name__ == "__main__":
    main() # main 함수를 호출하여 노드를 시작함.

# my_topic_pub.py 파일 분석: 토픽 발행자 (퍼블리셔 노드)
# 이 파이썬 스크립트는 ROS2 시스템에서 MyTopic 타입의 메시지를 특정 토픽으로 주기적으로 발행하는 역할을 함.
# 다른 노드들은 이 토픽을 구독(Subscribe)해서 메시지를 받아볼 수 있음.

    # my_topic_pub.py는 myTopicPub이라는 이름의 ROS2 노드임.
    # 이 노드는 user_interface 패키지의 MyTopic 메시지 타입을 사용해서 /message라는 토픽으로 0.1초마다 메시지를 발행함.
    # 각 메시지에는 a와 b 필드에 고정된 값(10, 20)을 넣고, header 필드에는 현재 시간과 메시지 발행 횟수를 포함한 frame_id를 채워서 보냄.
    # 이 발행된 메시지들은 /message 토픽을 구독하는 다른 노드들이 받아서 사용할 수 있음.