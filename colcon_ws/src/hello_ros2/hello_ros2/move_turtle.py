# /turtle1/cmd_vel [geometry_msgs/msg/Twist]

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class Move_turtle(Node):
    def __init__(self):
        super().__init__("move_turtle")  # node name
        self.create_timer(0.1, self.pub_turtle)
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.declare_parameter("vel", 0.0)
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
        self.vel += 0.01  # type: ignore


def main():
    rclpy.init()
    node = Move_turtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
