import rclpy
import numpy as np
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
        self.microros_data = np.zeros(19)

    def microros_callback(self, msg):
        # Obsługa odebranych danych z tematu 'microros'
        #self.get_logger().info(f'Odebrano dane z tematu microros: {msg.data}')
        self.microros_data = msg.data if msg.data else np.zeros(19)

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

        # Sprawdzenie, czy otrzymano dane z tematu 'microros'
        if not hasattr(self, 'microros_data'):
            self.get_logger().warn('Brak danych z tematu microros. Przypisywane zostaną wartości domyślne (0).')
            #self.microros_data = [1] * len(finger_joints)

        #for joint_name, joint_id in finger_joints:
        for i, (joint_name, joint_id) in enumerate(finger_joints):
            try:
                transform = self.buffer.lookup_transform('left_hand_base_link', joint_name, rclpy.time.Time())
                position = transform.transform.translation
                # position correction
                joint_corrections = {
                    21: {'z': -0.007},
                    22: {'z': -0.006},
                    23: {'z': 0.0015, 'x': 0.001},
                    31: {'z': -0.007},
                    32: {'z': -0.006},
                    33: {'z': 0.0015, 'x': 0.001},
                    41: {'z': -0.007},
                    42: {'z': -0.006, 'x': -0.001},
                    43: {'z': 0.0015, 'x': -0.001},
                    51: {'z': -0.007, 'x': -0.003},
                    52: {'z': -0.006, 'x': -0.003},
                    53: {'z': 0.0015, 'x': -0.002},
                    11: {'z': 0.001, 'x': 0.003},
                    12: {'z': 0.103, 'x': -0.021},
                    13: {'z': 0.103, 'x': 0.011},
                    14: {'z': 0.0985, 'x': 0.042},
                    15: {'z': 0.002, 'x': -0.006},
                    16: {'z': 0.069},
                    17: {'z': 0.059, 'x': 0.037},
                }
                if joint_id in joint_corrections:
                    corrections = joint_corrections[joint_id]
                    position.x += corrections.get('x', 0)
                    position.z += corrections.get('z', 0)


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

                marker_msg.text = str(self.microros_data[i])  # Przypisanie wartości z tablicy 'data' do tekstu markera
                self.marker_publisher.publish(marker_msg)
                #self.get_logger().error(f"to są moje texty {joint_id}: {i} {marker_msg.text}")
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
