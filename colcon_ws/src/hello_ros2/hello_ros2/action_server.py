# ros2 action send_goal --feedback /fibonacci user_interface/Fibonacci "{step: 15 }"
# 이 명령어는 터미널에서 액션 서버에 'step'이 15인 피보나치 액션 목표를 보내는 예시임.
# '--feedback' 옵션을 주면 중간 피드백도 터미널에 출력됨.

import time
# 시간 관련 함수를 사용하기 위한 모듈을 가져옴.
# [보충 설명]: 여기선 피보나치 계산 진행을 시뮬레이션하기 위해 잠시 대기하는 데 사용됨.

import rclpy
# ROS2 파이썬 클라이언트 라이브러리인 'rclpy'를 가져옴.

from rclpy.action import ActionServer
# ROS2 액션 서버 기능을 사용하기 위한 'ActionServer' 클래스를 가져옴.

from rclpy.action.server import ServerGoalHandle
# 액션 서버에서 목표를 다루는 데 필요한 'ServerGoalHandle' 클래스를 가져옴.
# [보충 설명]: 이 객체를 통해 목표의 상태를 변경하고, 피드백을 발행하고, 최종 결과를 반환할 수 있음.

from rclpy.callback_groups import ReentrantCallbackGroup
# ROS2의 'ReentrantCallbackGroup' 클래스를 가져옴.
# [보충 설명]: 이 콜백 그룹에 속한 콜백들은 서로 다른 쓰레드에서 동시에 실행될 수 있어,
#              여러 액션 목표를 동시에 처리할 수 있도록 해줌.

from rclpy.executors import MultiThreadedExecutor
# ROS2의 'MultiThreadedExecutor' 클래스를 가져옴.
# [보충 설명]: 여러 쓰레드를 사용해서 노드의 콜백들을 동시에 실행할 수 있게 해주는 실행자임.

from rclpy.node import Node
# 'rclpy'에서 노드(Node) 클래스를 가져옴.

# from sympy import fibonacci # (주석 처리됨) sympy 라이브러리의 fibonacci 함수를 사용할 수도 있지만,
                              # 이 코드에서는 직접 피보나치 수열을 계산하고 있음.

from user_interface.action import Fibonacci
# 우리가 만든 'Fibonacci.action' 파일에서 정의한 액션 타입인 'Fibonacci'를 가져옴.

# Action_server 라는 이름의 클래스를 정의함.
# 이 클래스가 우리가 만들 액션 서버 노드의 '설계도'임.
class Action_server(Node):
    def __init__(self):
        # '__init__' 메서드는 Action_server 객체를 만들 때 가장 먼저 실행되는 초기 설정 부분임.

        # 1. 노드 초기화:
        # 부모 클래스인 Node의 생성자를 호출해서 노드를 초기화함.
        # 노드의 이름은 "fibonacci_server"로 설정됨.
        super().__init__("fibonacci_server")  # 노드 이름

        # 2. 콜백 그룹 생성:
        # ReentrantCallbackGroup을 생성해서 액션 콜백에 할당할 거임.
        self.callback_group = ReentrantCallbackGroup()

        # 3. 액션 서버 생성 및 등록:
        # 'Fibonacci' 타입의 액션 서비스를 'fibonacci'라는 이름으로 만듦.
        # 액션 목표가 오면 'self.execute_callback' 함수를 호출해서 처리할 거임.
        self.action_server = ActionServer(
            self,                            # 현재 노드 객체
            Fibonacci,                       # 액션 메시지 타입
            "fibonacci",                     # 액션 이름 (클라이언트가 요청할 이름)
            execute_callback=self.execute_callback, # 목표 실행 콜백 함수
            callback_group=self.callback_group # 액션 콜백에 사용할 콜백 그룹
        )
        # [보충 설명]: ActionServer는 액션 서버를 생성하는 핵심 클래스임.

    # 클라이언트로부터 액션 목표(Goal)가 들어올 때마다 호출되는 콜백 함수임.
    # 'goal_handle' 객체는 현재 처리 중인 목표에 대한 모든 정보를 담고 있으며,
    # 목표 상태 변경, 피드백 발행, 결과 반환 등에 사용됨.
    def execute_callback(self, goal_handle: ServerGoalHandle):
        # goal 을 받았을 때...
        request: Fibonacci.Goal = goal_handle.request # goal_handle에서 요청 데이터를 가져옴.
        self.get_logger().info(f"요청받은 피보나치 단계: {request.step}") # 요청받은 단계를 로그로 출력함.

        feedback = Fibonacci.Feedback() # 클라이언트에게 보낼 빈 피드백 객체를 만듦.
        feedback.temp_seq = [0, 1]      # 피보나치 수열의 초기 값 (0, 1)을 설정함.
        result = Fibonacci.Result()     # 클라이언트에게 보낼 빈 결과 객체를 만듦.

        # 피드백(Feedback) 전송 루프:
        # 요청받은 'step'까지 피보나치 수열을 계산하면서 중간중간 피드백을 보냄.
        for i in range(1, request.step):
            # 피보나치 수열 다음 항 계산: 현재 항 + 이전 항
            feedback.temp_seq.append(feedback.temp_seq[i] + feedback.temp_seq[i - 1])
            goal_handle.publish_feedback(feedback)  # 현재까지의 피보나치 수열을 피드백으로 보냄.
            time.sleep(1) # 1초 대기 (계산이 오래 걸리는 것처럼 시뮬레이션)

            # [취소 요청 처리 예시 - 이 코드에는 직접 구현되지 않음]:
            # if goal_handle.is_cancel_requested:
            #     self.get_logger().info("목표 취소 요청됨!")
            #     goal_handle.canceled()
            #     result.seq = feedback.temp_seq # 취소 시점까지의 결과도 반환할 수 있음
            #     return result

        # 결과(Result) 전송:
        # 모든 계산이 끝나면 목표를 '성공' 상태로 설정하고 최종 결과를 클라이언트에게 보냄.
        print(f"GoalHandle 타입: {type(goal_handle)}") # goal_handle의 타입을 출력 (디버깅용)
        goal_handle.succeed()  # 액션 목표의 상태를 '성공(succeeded)'으로 설정하고 클라이언트에게 알림.
        # goal_handle.abort()  # (주석 처리됨) 만약 액션이 실패했을 경우, 이 함수를 호출하여 '실패(aborted)' 상태로 설정할 수 있음.

        result.seq = feedback.temp_seq # 최종 계산된 피보나치 수열 전체를 결과 객체에 저장함.
        return result  # 액션 종료 및 결과 반환.

# 'main' 함수는 이 ROS2 노드를 시작하고 전체 실행 로직을 담고 있음.
def main():
    rclpy.init() # rclpy 라이브러리를 초기화함.

    node = Action_server() # Action_server 클래스의 객체를 만듦.

    # 1. 실행자(Executor) 생성:
    # MultiThreadedExecutor를 생성하고, 동시에 처리할 쓰레드 수를 5개로 지정함.
    # [보충 설명]: 이 Executor 덕분에 ReentrantCallbackGroup으로 설정된 액션 콜백들이
    #              여러 클라이언트의 요청을 동시에 처리할 수 있음.
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node) # 생성한 노드를 Executor에 추가함.

    try:
        # Executor를 실행하여 노드의 콜백들을 처리하기 시작함.
        # ROS2 시스템이 종료되지 않는 한 계속 대기하며 액션 목표를 기다림.
        executor.spin()
    except KeyboardInterrupt:
        # Ctrl+C 등으로 노드가 중단될 때 발생하는 KeyboardInterrupt 예외를 처리함.
        executor.shutdown() # Executor를 안전하게 종료함.
        node.destroy_node() # 노드를 깔끔하게 종료함.

# 이 부분이 파이썬 스크립트가 직접 실행될 때만 main() 함수를 호출하도록 하는 표준 파이썬 관용구임.
if __name__ == "__main__":
    main() # main 함수를 호출하여 노드를 시작함.

# action_server.py 파일 분석: 피보나치 계산기 (액션 서버 노드)
# 이 파이썬 스크립트는 ROS2 시스템에서 Fibonacci 타입의 액션 요청을 받아서 피보나치 수열을 계산하고, 그 과정에서 중간 피드백을 보내며, 최종 결과를 돌려주는 역할을 함.
# 마치 주방장이 주문(Goal)을 받아서 요리하고(처리), 중간에 진행 상황을 알려주고(Feedback), 다 된 요리(Result)를 손님에게 내어주는 거랑 같음.

    # action_server.py 노드 (fibonacci_server):
    # fibonacci라는 이름으로 Fibonacci 액션 서비스를 제공함.
    # 액션 클라이언트로부터 특정 step까지 피보나치 수열을 계산해달라는 **목표(Goal)**를 받음.
    # execute_callback 함수에서 1초에 한 번씩 피보나치 수열을 계산하고, 현재까지의 수열(temp_seq)을 **피드백(Feedback)**으로 클라이언트에게 보냄.
    # 모든 계산이 끝나면, 최종 피보나치 수열(seq)을 담은 **결과(Result)**를 클라이언트에게 보내고 액션을 성공(succeeded) 상태로 완료함.
    # ReentrantCallbackGroup과 MultiThreadedExecutor를 사용하여 여러 클라이언트의 요청을 동시에 처리할 수 있도록 설계되었음.