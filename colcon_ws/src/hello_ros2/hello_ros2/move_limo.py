# /cmd_vel [geometry_msgs/msg/Twist]
# ros2 launch limo_description gazebo_models_diff.launch.py
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> e6734c5 (move_limo.py 파일 수정: 코드 스타일 개선을 위한 공백 추가)
=======
>>>>>>> cec5998 (move_limo.py 파일 수정: 불필요한 공백 제거)
class Move_turtle(Node):
    def __init__(self):
        super().__init__("move_limo")  # node name
        self.create_timer(0.1, self.pub_turtle)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.declare_parameter("vel", 1.0)
        self.declare_parameter("angle_vel", 1.2)
        self.vel = self.get_parameter("vel").value
        self.angle_vel = self.get_parameter("angle_vel").value

        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, parameters: list[Parameter]):
        for parameter in parameters:
            if parameter.name == "vel":
                self.vel = float(parameter.value)  # type: ignore
            if parameter.name == "angle_vel":
                self.angle_vel = float(parameter.value)  # type: ignore
        return SetParametersResult(successful=True)

    def pub_turtle(self):
        msg = Twist()
        msg.angular.z = self.angle_vel  # python type 캐스팅이 자유롭다.
        msg.linear.x = self.vel  # 하지만 DDS 로 넘길 때는 type check 가 되어야 한다.
        self.pub.publish(msg)
        # self.vel += 0.01  # type: ignore

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> e6734c5 (move_limo.py 파일 수정: 코드 스타일 개선을 위한 공백 추가)
=======
>>>>>>> cec5998 (move_limo.py 파일 수정: 불필요한 공백 제거)
def main():
    rclpy.init()
    node = Move_turtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> e6734c5 (move_limo.py 파일 수정: 코드 스타일 개선을 위한 공백 추가)
=======
>>>>>>> cec5998 (move_limo.py 파일 수정: 불필요한 공백 제거)
if __name__ == "__main__":
    main()