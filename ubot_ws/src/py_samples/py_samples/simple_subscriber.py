import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    def __init__(self):
        
        super().__init__("simple_sub")

        # Create a subscriber var and indicate what type of message to expect from std_msg
        # create_subscriber(Type, topic_name, func_exec_on_msg, queue_size)
        self.sub_ = self.create_subscription(String, "chatter",self.msgCallback, 10)


    def msgCallback(self, msg):
        # Log with logger
        self.get_logger.info("Listening: %s" % msg.data)


def main():
    # Init ROS
    rclpy.init()
    # Create simple sub
    SimpleSubscriber = SimpleSubscriber()
    # Keep the subscriber executing
    rclpy.spin(SimpleSubscriber)

    # When stopping destroy node and disconnect from ROS
    SimpleSubscriber.destroy_node()
    rclpy.shutdown()