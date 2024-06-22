import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs


class OdomToTFNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # Replace 'odom_topic' with the actual odometry topic name
            self.odom_callback,
            20)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        transform_stamped = self.odom_to_tf(msg)
        # self.tf_broadcaster.sendTransform(transform_stamped)
        self.publish_static_tf(transform_stamped)

    def publish_static_tf(self, transform_stamped):
        self.tf_static_broadcaster.sendTransform(transform_stamped)
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = 'base_footprint'
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(transform_stamped)

    def odom_to_tf(self, odom_msg):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        # transform_stamped.header.stamp = odom_msg.header.stamp
        transform_stamped.header.frame_id = 'odom'
        # Replace 'base_link' with the child frame ID
        transform_stamped.child_frame_id = 'base_link'
        transform_stamped.transform.translation.x = odom_msg.pose.pose.position.x
        transform_stamped.transform.translation.y = odom_msg.pose.pose.position.y
        transform_stamped.transform.translation.z = odom_msg.pose.pose.position.z
        transform_stamped.transform.rotation = odom_msg.pose.pose.orientation
        return transform_stamped


def main(args=None):
    rclpy.init(args=args)
    odom_to_tf_node = OdomToTFNode()
    rclpy.spin(odom_to_tf_node)
    odom_to_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
