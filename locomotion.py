import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import time
import math

class Locomotion(Node):

    def __init__(self):
        super().__init__('turtlebot3_locomotion')
        # self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.subscription_scan = self.create_subscription(LaserScan, '/base_scan', self.sensor_callback, 20)

        self.subscription = self.create_subscription(
            LaserScan,
            '/front_state',
            self.listener_callback,
            10)
        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription  # prevent unused variable warning


        self.timer = self.create_timer(0.01, self.timer_callback)

        # Set baseline numbers
        self.doorway_timer = 0
        self.turn_timer = 0
        self.tick = 0
        self.previous_right = float('inf')
        self.previous_left = float('inf')
        self.state = "following-wall"

        # Laser scan data
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # Multipliers to make conversion easier
        self.distance_multiplier = 3
        self.linear_speed_multiplier = 3
        self.angular_speed_multiplier = 3

        self.move_cmd = Twist()

    # Sensor callback
    def listener_callback(self, msg: LaserScan):

        twist = Twist()

        def filter_vals(range):
            range = [v for v in range if not math.isinf(v) and not math.isnan(v)]
            return min(range) if range else float('inf')
        arr = msg.ranges[-135:]+msg.ranges[:-135]
        msg.ranges = arr[0:271]
        msg.ranges = [5.0 if x == 0.0 else (x*self.distance_multiplier) for x in msg.ranges]
        # self.get_logger().info(msg.ranges)
        # self.get_logger().info(msg.ranges)
        # self.get_logger().info(msg.crash)
        n = len(msg.ranges)
        front = n // 2
        right = n // 6
        left = n - right
    
        self.front_distance = filter_vals(msg.ranges[front-8:front+8])
        self.right_distance = filter_vals(msg.ranges[right-7:right+7])
        self.left_distance = filter_vals(msg.ranges[left-5:left+5])

        # initialize prev_right and prev_left
        if(self.previous_right == float('inf')):
            self.previous_right = self.right_distance
        if(self.previous_left == float('inf')):
            self.previous_left = self.left_distance

        match self.state:
            # base state is following a wall on right-hand side
            case "following-wall":
                self.get_logger().info("Following wall")
                twist.linear.x = 0.5 * self.linear_speed_multiplier
                twist.angular.z = 0.0

                # Course correct to stay parallel
                # Drifting away, turn slightly right toward wall 
                if(msg.ranges[right-1] < msg.ranges[right+1]):
                    twist.angular.z = -0.1 * self.angular_speed_multiplier
                # Drifting toward, turn slightly left away from wall
                if(msg.ranges[right-1] > msg.ranges[right+1]):
                    twist.angular.z = 0.1 * self.angular_speed_multiplier
                # Anti-crash mechanism - don't hug the wall too close
                if(self.right_distance < 0.3):
                    twist.angular.z = 0.2 * self.angular_speed_multiplier

                # if the front value gets too low, we are in a corner and need to turn left
                if(self.front_distance < 0.8):
                    self.state = "left-turn"

            # turn right for a little bit, then inch forward into doorway
            case "right-turn":
                self.get_logger().info("Right turn")
                twist.linear.x = 0.0
                twist.angular.z = -1.0 * self.angular_speed_multiplier
                self.turn_timer += 1

                # if we are aligned with a wall mid-turn, stop turning 
                if(abs(msg.ranges[right-1] - msg.ranges[right+1]) < 0.005 and self.turn_timer > 5):
                    self.turn_timer = 0
                    self.state = "following-wall"

                # done turning 90 degrees, so inch forward
                if(self.turn_timer > 19 and self.turn_timer < 24):
                    twist.linear.x = 0.15 * self.linear_speed_multiplier
                    twist.angular.z = 0.0

                if(self.turn_timer >= 24):
                    self.turn_timer = 0
                    self.state = "following-wall"

            # turn left until the area in front is clear
            case "left-turn":
                self.get_logger().info("left turn")
                twist.linear.x = 0.0
                twist.angular.z = 1.0 * self.angular_speed_multiplier

                if(self.front_distance > 1.0):
                    twist.linear.x = 0.1 * self.linear_speed_multiplier
                    twist.angular.z = 0.0
                    self.state = "following-wall"

            # check to see if the doorway is wide enough for the robot to fit
            case "checking-right-doorway":
                self.get_logger().info("checking right doorway")
                twist.linear.x = 0.2 * self.linear_speed_multiplier
                twist.angular.z = 0.0
                self.doorway_timer += 1

                # if timer hits a threshhold without sensing a new wall on the right, we've found a valid doorway
                if(self.doorway_timer > 8):
                    self.doorway_timer = 0
                    self.state = "right-turn"

                # if right sensor suddenly drops before hitting threshhold, doorway is too narrow
                if(self.right_distance - self.previous_right < -0.5 and self.right_distance < 1.0):
                    self.doorway_timer = 0
                    self.state = "following-wall"


        self.previous_right = self.right_distance
        self.previous_left = self.left_distance
        
        self.pub.publish(twist)

    # Timer callback
    def timer_callback(self):
        self.tick += 1

def main(args=None):
    rclpy.init(args=args)

    tb3_locomotion = Locomotion()

    rclpy.spin(tb3_locomotion)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()