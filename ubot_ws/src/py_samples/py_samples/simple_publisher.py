import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        # Call to inhered Node constructor with our node name
        super().__init__("simple_pub")

        # Create a publisher var and indicate what type of message to expect from std_msg
        # create_publisher(Type, topic_name, queue_size)
        self.pub_ = self.create_publisher(String, "chatter", 10)
        self.cnt_ = 0

        # Frequency of publishing in Hz (n per second)
        self.frequency_ = 1.0

        # Log with logger
        self.get_logger().info("Pub at %d Hz" % self.frequency_)
        
        # Create a timer which takes a frequency and a function name which to be executed at set frequency
        # (frequency, function_name)
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        # Type of message
        msg = String()
        
        # Message data
        msg.data = "Hello - cnt: %d" % self.cnt_

        # Pub message
        self.pub_.publish(msg)
        self.cnt_ += 1


def main():
    # Init ROS
    rclpy.init()
    # Create simple pub
    simplePublisher = SimplePublisher()
    # Keep the publisher executing
    rclpy.spin(simplePublisher)

    # When stopping destroy node and disconnect from ROS
    simplePublisher.destroy_node()
    rclpy.shutdown()


