import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray

class TagDetector(Node):
    def __init__(self):
        super().__init__('turtlebot3_tag_detector')
        self.subscription = self.create_subscription(AprilTagDetectionArray, '/detections', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #print(msg.detections)
        if len(msg.detections) > 0:
            print("Tag detected %s" % (msg.detections[0].id))

def main(args=None):
    rclpy.init(args=args)

    tb3_tag_detector = TagDetector()

    rclpy.spin(tb3_tag_detector)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
