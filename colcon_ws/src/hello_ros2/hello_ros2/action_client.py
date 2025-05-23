# ros2 action send_goal --feedback /fibonacci user_interface/Fibonacci "{step: 15}"
# 이 명령어는 터미널에서 액션 서버에 'step'이 15인 피보나치 액션 목표를 보내는 예시임.
# 이 코드(action_client.py)를 실행하면, 이 명령어를 내부적으로 파이썬 코드로 구현해서 목표를 보냄.

import rclpy
# ROS2 파이썬 클라이언트 라이브러리인 'rclpy'를 가져옴.

from action_msgs.msg import GoalStatus
# 액션 목표의 상태(예: SUCCEEDED, ABORTED, PENDING 등)를 나타내는 상수를 포함하는 메시지 타입을 가져옴.

from rclpy.action import ActionClient
# ROS2 액션 클라이언트 기능을 사용하기 위한 'ActionClient' 클래스를 가져옴.

from rclpy.action.client import ClientGoalHandle
# 액션 클라이언트에서 목표 핸들을 다루는 데 필요한 'ClientGoalHandle' 클래스를 가져옴.
# [보충 설명]: 서버에 보낸 목표에 대한 상태, 결과 등을 추적할 수 있는 객체임.

from rclpy.callback_groups import ReentrantCallbackGroup # (이 코드에서는 직접 사용되지 않음)
# [보충 설명]: 액션 클라이언트의 콜백도 ReentrantCallbackGroup에 할당하여 동시 처리가 가능함.

from rclpy.executors import MultiThreadedExecutor
# ROS2의 'MultiThreadedExecutor' 클래스를 가져옴.

from rclpy.node import Node
# 'rclpy'에서 노드(Node) 클래스를 가져옴.

from rclpy.task import Future
# 비동기 작업의 결과를 기다리거나 콜백을 등록하는 데 사용되는 'Future' 객체를 가져옴.
# [보충 설명]: ROS2 비동기 프로그래밍의 핵심 요소 중 하나임.

from user_interface.action import Fibonacci
# 우리가 만든 'Fibonacci.action' 파일에서 정의한 액션 타입인 'Fibonacci'를 가져옴.

from user_interface.action._fibonacci import Fibonacci_GetResult_Response
# (타입 힌트용) Fibonacci 액션의 get_result 응답 타입을 명시적으로 가져옴.
# [보충 설명]: 파이썬 린터(linter)나 IDE의 타입 추론을 돕기 위해 사용될 수 있음.

# Action_client 라는 이름의 클래스를 정의함.
# 이 클래스가 우리가 만들 액션 클라이언트 노드의 '설계도'임.
class Action_client(Node):
    def __init__(self):
        # '__init__' 메서드는 Action_client 객체를 만들 때 가장 먼저 실행되는 초기 설정 부분임.

        # 1. 노드 초기화:
        # 부모 클래스인 Node의 생성자를 호출해서 노드를 초기화함.
        # 노드의 이름은 "fibonacci_client"로 설정됨.
        super().__init__("fibonacci_client")  # 노드 이름

        self.callback_group = ReentrantCallbackGroup() # (이 코드에서는 직접 사용되지 않음)
        # [보충 설명]: 클라이언트의 콜백들도 필요하다면 이 그룹에 할당하여 동시 처리할 수 있음.

        # 2. 액션 클라이언트 생성:
        # 'Fibonacci' 타입의 액션을 'fibonacci'라는 이름의 액션 서버에 보낼 클라이언트를 만듦.
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")

        self.future = Future()  # 액션 목표 전송 결과를 받을 Future 객체를 초기화함.
        self.get_result_future = Future()  # 액션 결과(Result)를 받을 Future 객체를 초기화함.

        # 3. 서버 접속 대기:
        # 액션 서버('fibonacci')가 ROS2 시스템에 등록될 때까지 최대 1초 간격으로 대기함.
        while not self.action_client.wait_for_server(timeout_sec=1):
            self.get_logger().info("피보나치 서버 접속중 ...")
            # [보충 설명]: 서버가 준비되기 전에 클라이언트가 목표를 보내면 오류가 나므로,
            #              서버가 준비될 때까지 기다리는 것은 좋은 습관임.

        self.send_goal() # 서버 접속에 성공하면 목표를 보내는 함수를 호출함.

    # 액션 목표(Goal)를 서버에 보내는 함수임.
    def send_goal(self):
        goal = Fibonacci.Goal() # 'Fibonacci.Goal' 타입의 빈 목표 객체를 만듦.
        goal.step = 8           # 피보나치 수열의 8단계까지 계산해달라는 목표를 설정함.

        # 액션 서버에 목표를 비동기적으로(Async) 보냄.
        # 이 함수는 목표를 보낸 즉시 Future 객체를 반환하므로, 클라이언트 노드는 멈추지 않고 계속 실행됨.
        # 'feedback_callback'을 지정하여 서버로부터 피드백이 올 때마다 'self.feedback_callback' 함수를 호출하도록 함.
        self.future: Future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        # 'self.future'가 완료되면(목표 수락 여부가 결정되면) 'self.goal_response_callback' 함수를 호출하도록 콜백을 등록함.
        self.future.add_done_callback(self.goal_response_callback)

    # 액션 서버가 목표를 수락했는지 또는 거부했는지 응답을 처리하는 콜백 함수임. (ROS1에 없는 중요한 체크포인트)
    def goal_response_callback(self, future: Future):
        goal_handle: ClientGoalHandle = future.result()  # type : ignore # Future 객체에서 목표 핸들을 가져옴.
        if not goal_handle.accepted: # 서버가 목표를 수락하지 않았다면
            self.get_logger().info("골이 접수 안 되었습니다.") # 접수 실패 메시지를 로그로 출력함.
            return # 함수 종료.

        # 목표가 수락되었다면, 이제 액션의 최종 결과(Result)를 받을 준비를 함.
        # 'get_result_async()'를 호출하여 결과 Future 객체를 얻고, 완료 시 'self.done_callback'을 호출하도록 등록함.
        self.get_result_future: Future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.done_callback)

    # 액션 서버로부터 중간 피드백(Feedback)이 올 때마다 호출되는 콜백 함수임.
    def feedback_callback(self, msg):
        feedback: Fibonacci.Feedback = msg.feedback # 메시지 객체에서 피드백 데이터를 가져옴.
        self.get_logger().info(f" 지금까지 처리 결과 seq{feedback.temp_seq}") # 현재까지의 피보나치 수열을 로그로 출력함.

    # 액션 서버가 모든 작업을 완료하고 최종 결과(Result)를 보냈을 때 호출되는 콜백 함수임.
    def done_callback(self, future: Future):
        result: Fibonacci_GetResult_Response = future.result()  # type : ignore # Future 객체에서 최종 결과를 가져옴.
        # 최종 상태에 따라 다른 메시지를 로그로 출력함.
        if result.status == GoalStatus.STATUS_SUCCEEDED: # 액션이 성공적으로 완료됐으면
            self.get_logger().info(f"result: {result.result.seq} 성공!!") # 성공 메시지와 최종 피보나치 수열을 출력함.
        if result.status == GoalStatus.STATUS_ABORTED: # 액션이 도중에 실패했으면
            self.get_logger().info(f"result: aborted 실패!!") # 실패 메시지를 출력함.
        # 다른 상태들(PENDING, CANCELED 등)도 처리할 수 있음.

# 'main' 함수는 이 ROS2 노드를 시작하고 전체 실행 로직을 담고 있음.
def main():
    rclpy.init() # rclpy 라이브러리를 초기화함.

    node = Action_client() # Action_client 클래스의 객체를 만듦.

    # 1. 실행자(Executor) 생성:
    # MultiThreadedExecutor를 생성하고, 동시에 처리할 쓰레드 수를 5개로 지정함.
    # [보충 설명]: 이 Executor 덕분에 클라이언트의 여러 비동기 콜백들이 동시에 실행될 수 있음.
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node) # 생성한 노드를 Executor에 추가함.

    try:
        # Executor를 실행하여 노드의 콜백들을 처리하기 시작함.
        # ROS2 시스템이 종료되지 않는 한 계속 대기하며 액션 서버로부터의 응답/피드백/결과를 기다림.
        executor.spin()
    except KeyboardInterrupt:
        # Ctrl+C 등으로 노드가 중단될 때 발생하는 KeyboardInterrupt 예외를 처리함.
        executor.shutdown() # Executor를 안전하게 종료함.
        node.destroy_node() # 노드를 깔끔하게 종료함.

# 이 부분이 파이썬 스크립트가 직접 실행될 때만 main() 함수를 호출하도록 하는 표준 파이썬 관용구임.
if __name__ == "__main__":
    main() # main 함수를 호출하여 노드를 시작함.

# action_client.py 파일 분석: 피보나치 요청자 (액션 클라이언트 노드)
# 이 파이썬 스크립트는 ROS2 시스템에서 **`Fibonacci` 타입의 액션 목표를 액션 서버에 요청하고, 그 과정에서 중간 피드백을 실시간으로 받으며, 최종 결과를 처리하는 역할**을 함.
# 마치 손님이 주문하고 중간 보고를 받으며, 다 된 음식을 최종적으로 받는 거랑 같음.
# ROS2 액션 클라이언트는 기본적으로 비동기(Asynchronous) 방식으로 동작하여, 목표를 보낸 후에도 클라이언트 노드가 다른 작업을 동시에 수행할 수 있음.

    # action_client.py 노드 (fibonacci_client):
    # fibonacci 액션 서버에 연결을 시도하고, 서버가 준비될 때까지 기다림.
    # send_goal 함수를 통해 step이 8인 피보나치 액션 목표를 비동기적으로 서버에 보냄.
    # send_goal_async를 사용하므로, 목표를 보낸 후 클라이언트 노드는 멈추지 않고 다른 작업을 계속할 수 있음.
    # 서버가 목표를 수락하면 goal_response_callback이 호출되어 최종 결과를 받을 준비를 함.
    # 서버로부터 중간 **피드백(Feedback)**이 올 때마다 feedback_callback 함수가 호출되어 현재까지의 계산 진행 상황을 출력함.
    # 서버가 모든 작업을 완료하고 **결과(Result)**를 보내면 done_callback 함수가 호출되어 최종 피보나치 수열과 액션의 최종 상태를 처리하고 출력함.