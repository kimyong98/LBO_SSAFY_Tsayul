import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from squaternion import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan,PointCloud

class tt(Node):

    def __init__(self):
        super().__init__('tt')
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        
        self.subscription = self.create_subscription(LaserScan,
        '/scan',self.scan_callback, qos)
        self.tt_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.is_odom = 0
        
    def scan_callback(self, msg):
        print('good')

    def odom_callback(self, msg):
        if self.is_odom < 3:
            self.is_odom += 1
            print('oooooodddddddoooooooommmm')
        
def main(args=None):
    rclpy.init(args=args)
    ttt = tt()
    rclpy.spin(ttt)
    ttt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()