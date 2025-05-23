import time
# 시간 관련 함수를 사용하기 위한 모듈을 가져옴.
# [보충 설명]: 여기선 서비스 처리 시간을 시뮬레이션하기 위해 잠시 대기하는 데 사용됨.

import rclpy
# ROS2 파이썬 클라이언트 라이브러리인 'rclpy'를 가져옴.

from rclpy.callback_groups import ReentrantCallbackGroup
# ROS2의 'ReentrantCallbackGroup' 클래스를 가져옴.
# [보충 설명]: 이 콜백 그룹에 속한 콜백들은 서로 다른 쓰레드에서 동시에 실행될 수 있음.
#              즉, 한 서비스 요청을 처리하는 동안 다른 서비스 요청이 들어와도 동시에 처리할 수 있어.
#              (기본 CallbackGroup은 하나의 콜백이 실행 중일 때는 다른 콜백이 기다려야 함)

from rclpy.executors import MultiThreadedExecutor
# ROS2의 'MultiThreadedExecutor' 클래스를 가져옴.
# [보충 설명]: 여러 쓰레드를 사용해서 노드의 콜백들을 동시에 실행할 수 있게 해주는 실행자임.
#              기본 Executor는 단일 쓰레드에서 콜백을 순차적으로 처리함.

from rclpy.node import Node
# 'rclpy'에서 노드(Node) 클래스를 가져옴. 모든 ROS2 노드는 이 클래스를 상속받아야 함.

from user_interface.srv import AddTwoInts
# 우리가 만든 'AddTwoInts.srv' 파일에서 정의한 서비스 타입인 'AddTwoInts'를 가져옴.
# [보충 설명]: 이 서비스 타입은 빌드 과정에서 파이썬 클래스로 자동 생성됨.

# Service_server 라는 이름의 클래스를 정의함.
# 이 클래스가 우리가 만들 서비스 서버 노드의 '설계도'임.
class Service_server(Node):
    def __init__(self):
        # '__init__' 메서드는 Service_server 객체를 만들 때 가장 먼저 실행되는 초기 설정 부분임.

        # 1. 노드 초기화:
        # 부모 클래스인 Node의 생성자를 호출해서 노드를 초기화함.
        # 노드의 이름은 "add_service_server"로 설정됨.
        super().__init__("add_service_server")  # 노드 이름

        # 2. 콜백 그룹 생성:
        # ReentrantCallbackGroup을 생성해서 나중에 서비스 콜백에 할당할 거임.
        self.callback_group = ReentrantCallbackGroup()

        # 3. 서비스 서버 생성 및 등록:
        # 'AddTwoInts' 타입의 서비스를 'add_ints'라는 이름으로 만듦.
        # 이 서비스로 요청이 오면 'self.setBool_callback' 함수를 호출해서 처리할 거임.
        # 이때, 위에서 만든 'self.callback_group'을 할당하여 여러 요청을 동시에 처리할 수 있도록 함.
        self.create_service(
            AddTwoInts,                 # 서비스 메시지 타입
            "add_ints",                 # 서비스 이름 (클라이언트가 호출할 이름)
            self.setBool_callback,      # 요청이 오면 호출될 콜백 함수
            callback_group=self.callback_group, # 서비스 콜백에 사용할 콜백 그룹
        )
        # [보충 설명]: create_service(서비스_타입, 서비스_이름, 콜백_함수, 콜백_그룹) 형태로 사용함.

    # 클라이언트로부터 서비스 요청이 들어올 때마다 호출되는 콜백 함수임.
    # 'request' 변수에는 클라이언트가 보낸 요청 데이터('AddTwoInts.Request' 타입)가,
    # 'response' 변수에는 서버가 클라이언트에게 돌려줄 응답 객체('AddTwoInts.Response' 타입)가 전달됨.
    def setBool_callback(
        self, request: AddTwoInts.Request, response: AddTwoInts.Response
    ):
        # 서버의 작동 알고리즘...
        self.get_logger().info(f"{request.header.stamp} 시간") # 요청 헤더의 시간 정보를 로그로 출력함.
        self.get_logger().info(f"a : {request.a}")             # 요청의 'a' 값을 로그로 출력함.
        self.get_logger().info(f"b : {request.b}")             # 요청의 'b' 값을 로그로 출력함.

        # 1. 요청 처리:
        response.result = request.a + request.b # 요청받은 'a'와 'b'를 더해서 결과를 응답 객체에 저장함.
        response.success = True                 # 서비스 처리가 성공했음을 알림.
        response.message = "a 와 b 의 값을 더해서 반환 했다!" # 성공 메시지를 응답 객체에 저장함.

        time.sleep(5) # 서비스 처리 중임을 시뮬레이션하기 위해 5초 동안 프로그램을 잠시 멈춤.
        # [보충 설명]: 실제 로봇 시스템에서는 이 부분에 복잡한 계산이나 로봇 팔 동작 같은 시간이 걸리는 작업이 들어감.

        # 2. 응답 반환:
        return response # 처리된 결과가 담긴 응답 객체를 클라이언트에게 반환함.

# 'main' 함수는 이 ROS2 노드를 시작하고 전체 실행 로직을 담고 있음.
def main():
    rclpy.init() # rclpy 라이브러리를 초기화함. 모든 ROS2 파이썬 노드에서 가장 먼저 호출해야 함.

    node = Service_server() # Service_server 클래스의 설계도를 바탕으로 'node'라는 실제 객체를 만듦.
                            # 이 순간 '__init__' 함수가 실행되어 노드와 서비스 서버가 설정됨.

    # 1. 실행자(Executor) 생성:
    # MultiThreadedExecutor를 생성하고, 동시에 처리할 쓰레드 수를 5개로 지정함.
    # [보충 설명]: 이 Executor 덕분에 ReentrantCallbackGroup으로 설정된 서비스 콜백들이 동시에 실행될 수 있음.
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node) # 생성한 노드를 Executor에 추가함.

    try:
        # Executor를 실행하여 노드의 콜백들을 처리하기 시작함.
        # ROS2 시스템이 종료되지 않는 한 계속 대기하며 서비스 요청을 기다림.
        executor.spin()
        # [보충 설명]: executor.spin()은 노드의 모든 콜백 그룹에 걸쳐 콜백들을 실행하며
        #              노드가 활성 상태를 유지하도록 함.
    except KeyboardInterrupt:
        # Ctrl+C 등으로 노드가 중단될 때 발생하는 KeyboardInterrupt 예외를 처리함.
        executor.shutdown() # Executor를 안전하게 종료함.
        node.destroy_node() # 노드를 깔끔하게 종료함.

# 이 부분이 파이썬 스크립트가 직접 실행될 때만 main() 함수를 호출하도록 하는 표준 파이썬 관용구임.
if __name__ == "__main__":
    main() # main 함수를 호출하여 노드를 시작함.

# add_server.py 파일 분석: 서비스 제공자 (서비스 서버 노드)
# 이 파이썬 스크립트는 ROS2 시스템에서 AddTwoInts 타입의 서비스 요청을 받아서 처리하고, 그 결과를 응답으로 돌려주는 역할을 함.
# ROS1 서버와 비슷하지만, ROS2에서는 **콜백 그룹(Callback Groups)**과 실행자(Executor) 개념이 도입되어 동시성 처리가 더 유연해졌음.

    # add_server.py 노드는 add_service_server라는 이름의 ROS2 노드임.
    # 이 노드는 user_interface 패키지의 AddTwoInts 서비스 타입을 사용해서 add_ints라는 이름의 서비스 채널을 제공함.
    # 클라이언트로부터 두 정수(a, b)와 연산자(op)를 포함한 **요청(Request)**이 오면,
    # setBool_callback 함수가 호출되어 이들을 더하는 연산을 수행하고,
    # result, success, message를 포함하는 **응답(Response)**을 돌려줌.
    # 특히 이 서버는 ReentrantCallbackGroup과 MultiThreadedExecutor를 사용하고 있음.
    # 이 덕분에 한 서비스 요청을 처리하는 데 5초가 걸리더라도, 그 5초 동안 다른 클라이언트의 요청이 들어오면 새로운 쓰레드에서 동시에 처리할 수 있음.
    # 이는 ROS1의 단일 쓰레드 서비스 처리 방식보다 훨씬 효율적이고 유연한 동시성 처리가 가능함을 보여줌.