import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisherNode')

        self.publisher_ = self.create_publisher(Int64, "topicoExemplo", 1)

        self.mensagem = Int64()
        self.mensagem.data = 67

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.publisher_.publish(self.mensagem)

def main(args=None):
    rclpy.init(args=args)

    node = PublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if(__name__ == "__init__"):
    main()