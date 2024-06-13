import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")

        self.pub_ = self.create_publisher(String, "chatter", 10)

        self.counter_ = 0 # counting the number of messages
        self.frequency_ = 1.0 # frequency the messages are sent

        self.get_logger().info("Publishing at %d Hz" % self.frequency_)

        self.tiimer_ = self.create_timer(self.frequency_, self.timerCallback) # deifine new ros2 timer to execute the function repeatedly

    def timerCallback(self):
        msg = String()
        msg.data = "Hello ROS 2 - counter: %d" % self.counter_

        self.pub_.publish(msg) # publishing the msg to the topic
        self.counter_ += 1 # incrementing the msg counter

def main():
    rclpy.init()
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # keep the node running
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()


    