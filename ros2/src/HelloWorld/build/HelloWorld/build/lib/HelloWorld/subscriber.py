import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class SubscriberNode(Node):

    def __init__(self):
        super().__init__('subscriberNode')

        self.subscription = self.create_subscription(Int64, "topicoExemplo", self.subscriberCallback, 1)

    def subscriberCallback(self, msg):
        self.get_logger().info(f"[subscriberNode] Valor recebido: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if(__name__ == "__init__"):
    main()