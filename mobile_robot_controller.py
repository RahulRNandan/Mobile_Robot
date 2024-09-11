import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu

class MobileRobotController(Node):
    def __init__(self):
        super().__init__('mobile_robot_controller')
        # Publisher to send velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LIDAR data
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # Subscriber for IMU data
        self.imu_subscription = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        # Timer for publishing velocity commands
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Velocity settings
        self.forward_speed = 0.2  # meters per second
        self.turn_speed = 0.5     # radians per second
        self.drive_forwards = True

    def lidar_callback(self, msg):
        # Process LIDAR data to detect obstacles
        min_distance = min(msg.ranges)
        self.get_logger().info(f'LIDAR minimum distance: {min_distance}')
        if min_distance < 0.5:  # If obstacle is closer than 0.5 meters
            self.drive_forwards = False
        else:
            self.drive_forwards = True

    def imu_callback(self, msg):
        # Process IMU data for orientation and angular velocity
        self.get_logger().info(f'IMU orientation: {msg.orientation}')
    
    def timer_callback(self):
        velocity_msg = Twist()
        
        if self.drive_forwards:
            self.get_logger().info('Driving forwards')
            velocity_msg.linear.x = self.forward_speed
            velocity_msg.angular.z = 0.0
        else:
            self.get_logger().info('Turning in place')
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = self.turn_speed
        
        self.velocity_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    mobile_robot_controller = MobileRobotController()
    rclpy.spin(mobile_robot_controller)
    mobile_robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
