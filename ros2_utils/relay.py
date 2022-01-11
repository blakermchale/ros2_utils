from rclpy.node import Node
import rclpy
from argparse import ArgumentParser
from sensor_msgs.msg import CameraInfo


# TODO: use topic_tools instead once it gets a release
# https://github.com/ros-tooling/topic_tools
class RelayNode(Node):
    def __init__(self, input_topic, output_topic):
        super().__init__("relay")

        # TODO: don't hardcode camera info
        self._sub_input = self.create_subscription(CameraInfo, input_topic, self._cb_relay, 1)
        self._pub_output = self.create_publisher(CameraInfo, output_topic, 1)

    def _cb_relay(self, msg):
        self._pub_output.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    parser = ArgumentParser()
    parser.add_argument("input_topic", type=str, help="Input topic.")
    parser.add_argument("output_topic", type=str, help="Output topic.")
    args, _ = parser.parse_known_args()

    relay = RelayNode(args.input_topic, args.output_topic)
    rclpy.spin(relay)


if __name__=="__main__":
    main()
