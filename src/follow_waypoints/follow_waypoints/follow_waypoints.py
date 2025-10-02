import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration
import math
from geometry_msgs.msg import Pose
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time


class ReactiveWaypointPublisher(Node):
    def __init__(self):
        super().__init__('reactive_waypoint_publisher')

        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # TF2 buffer et listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.waypoints = [
            (-0.7622432708740234, -0.7395186424255371, 0.0, 0.30597624682732966),
            (-6.340085029602051, 5.2895965576171875, 0.0, 0.8693087950894443),
            (3.112633228302002, 18.122364044189453, 0.0, 0.9787240292258439),
            #(11.841577529907227, 18.903762817382812, 0.0, 0.9996453092827682),
            #(21.790807723999023, 21.812925338745117, 0.0, 0.9055835065472461),
            #(23.945878982543945, 24.401174545288086, 0.0, 0.8869977159643478),
            #(31.88422966003418, 34.15766906738281, 0.0, 0.8909804586454293)
        ]

        self.index = 0
        self.tolerance = 0.2  # Distance tolérée pour dire qu'on est "arrivé"
        self.goal_sent = False
        self.robot_pose = Pose()

    def odom_callback(self, msg: Odometry):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            translation = transform.transform.translation
            rotation = transform.transform.rotation

            self.robot_pose.position.x = translation.x
            self.robot_pose.position.y = translation.y
            self.robot_pose.position.z = translation.z
            self.robot_pose.orientation = rotation

            if self.index >= len(self.waypoints):
                return  # Tous les points atteints

            goal_x, goal_y, goal_z, _ = self.waypoints[self.index]
            distance = math.sqrt(
                (self.robot_pose.position.x - goal_x) ** 2 +
                (self.robot_pose.position.y - goal_y) ** 2 +
                (self.robot_pose.position.z - goal_z) ** 2
            )

            if not self.goal_sent:
                self.publish_goal(goal_x, goal_y, goal_z)
                self.goal_sent = True

            if distance < self.tolerance:
                self.get_logger().info(f' Waypoint atteint : {goal_x}, {goal_y}, {goal_z}')
                
                # Pause de 20 secondes
                self.get_logger().info(' Pause de 10 secondes pour diagnostic...')
                time.sleep(10)

                self.index += 1
                self.goal_sent = False

                if self.index >= len(self.waypoints):
                    self.get_logger().info(' Tous les waypoints ont été atteints.')

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Transform non disponible : {e}")

    def publish_goal(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.publisher_.publish(goal)
        self.get_logger().info(f' Nouveau waypoint envoyé : {x}, {y}, {z}')

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


