import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header, Float64MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformListener, Buffer

class TFTextPublisher(Node):
    def __init__(self):
        super().__init__('tf_text_publisher')
        self.marker_publisher = self.create_publisher(Marker, 'tf_text', 10)
        self.subscriber = self.create_subscription(Float64MultiArray, 'microros', self.microros_callback, 10)
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(0.7, self.publish_markers)
        self.num_markers = 0  # Inicjalizacja licznika markerów

    def microros_callback(self, msg):
        # Obsługa odebranych danych z tematu 'microros'
        self.get_logger().info(f'Odebrano dane z tematu microros: {msg.data}')

    def publish_markers(self):
        finger_joints = [
            ('left_hand_thumb1', 15),
            ('left_hand_thumb3', 11),
            ('left_hand_base_link', 12),
            ('left_hand_base_link', 13),
            ('left_hand_base_link', 14),
            ('left_hand_base_link', 16),
            ('left_hand_base_link', 17),
            ('left_hand_index1', 23),
            ('left_hand_index2', 22),
            ('left_hand_index3', 21),
            ('left_hand_middle1', 33),
            ('left_hand_middle2', 32),
            ('left_hand_middle3', 31),
            ('left_hand_ring1', 43),
            ('left_hand_ring2', 42),
            ('left_hand_ring3', 41),
            ('left_hand_pinky1', 53),
            ('left_hand_pinky2', 52),
            ('left_hand_pinky3', 51)
        ]

        for joint_name, joint_id in finger_joints:
            try:
                transform = self.buffer.lookup_transform('left_hand_base_link', joint_name, rclpy.time.Time())
                position = transform.transform.translation

                # position correction
                if joint_id in [21, 31, 41, 51]:
                    position.z -= 0.007

                if joint_id in [22, 32, 42, 52]:
                    position.z -= 0.006
                
                if joint_id in [23, 33, 43, 53]:
                    position.z += 0.0015

                if joint_id in [53]:
                    position.x -= 0.002

                if joint_id in [52, 51]:
                    position.x -= 0.003
                    position.z += 0.001

                if joint_id in [42, 41]:
                    position.x -= 0.001
                    position.z += 0.001

                if joint_id in [23, 22, 21]:
                    position.x += 0.001
                
                if joint_id in [22, 21]:
                    position.z += 0.001

                if joint_id in [11]:
                    position.z += 0.001
                    position.x += 0.003

                if joint_id in [12]:
                    position.x += 0.042
                    position.z += 0.0985

                if joint_id in [13]:
                    position.x += 0.011
                    position.z += 0.103

                if joint_id in [14]:
                    position.x -= 0.021
                    position.z += 0.103

                if joint_id in [15]:
                    position.x -= 0.006
                    position.z += 0.002

                if joint_id in [16]:
                    position.z += 0.069

                if joint_id in [17]:
                    position.x += 0.037
                    position.z += 0.059

                marker_msg = Marker()
                marker_msg.header = Header()
                marker_msg.header.frame_id = 'left_hand_base_link'
                marker_msg.header.stamp = self.get_clock().now().to_msg()
                marker_msg.ns = 'tf_text'

                marker_msg.id = joint_id

                marker_msg.type = Marker.TEXT_VIEW_FACING
                marker_msg.action = Marker.ADD
                marker_msg.pose.position = Point(x=position.x, y=position.y + 0.03, z=position.z)
                marker_msg.pose.orientation.x = 0.0
                marker_msg.pose.orientation.y = 0.0
                marker_msg.pose.orientation.z = 0.0
                marker_msg.pose.orientation.w = 1.0

                marker_msg.scale.x = 0.01
                marker_msg.scale.y = 0.01
                marker_msg.scale.z = 0.01
                
                marker_msg.color.r = 0.0
                marker_msg.color.g = 0.0
                marker_msg.color.b = 0.0
                marker_msg.color.a = 1.0

                # Usunięcie "ID:" z treści tekstu
                marker_msg.text = str(joint_id)

                self.marker_publisher.publish(marker_msg)
                self.num_markers += 1

            except Exception as e:
                self.get_logger().error(f"Error publishing marker for {joint_name}: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    tf_marker_publisher = TFTextPublisher()
    rclpy.spin(tf_marker_publisher)
    tf_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
