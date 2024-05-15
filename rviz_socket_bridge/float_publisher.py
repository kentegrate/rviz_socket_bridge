import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import pickle
import socket


class PublisherNode(Node):
    def __init__(self):
        super().__init__("float32_publisher_node")
        self.pubs = {}

    def create_or_get_publisher(self, topic_name):
        if topic_name not in self.pubs:
            self.pubs[topic_name] = self.create_publisher(Float32, topic_name, 10)

        return self.pubs[topic_name]

    def process_pickled_list(self, pickled_list):
        topic_name, value = pickle.loads(pickled_list)
        publisher = self.create_or_get_publisher(topic_name)

        msg = Float32()
        msg.data = value
        publisher.publish(msg)
        self.get_logger().info(f"Published {value} to {topic_name}")


def main():
    rclpy.init()

    # Create an instance of the node
    node = PublisherNode()

    # Create a TCP server
    port = 12345  
    s = socket.socket() 
    s.bind(("", port))  
    s.listen(5)  


    while rclpy.ok():
        conn, addr = s.accept()  
        pickled_list = conn.recv(1024)
        node.process_pickled_list(pickled_list)
        conn.close()

    rclpy.shutdown()


if __name__ == "__main__":
    main()