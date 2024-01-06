import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# class GlovePositionNode(Node):
#     def __init__(self):
#         super().__init__('glove_position_node')
#         self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

#         # Ustawienie początkowych danych pozycji
#         joint_names = [
#             'left_hand_Thumb_j1', 'left_hand_Thumb_j2', 'left_hand_Thumb_j3',
#             'left_hand_Index_Finger_j1', 'left_hand_Index_Finger_j2', 'left_hand_Index_Finger_j3',
#             'left_hand_Middle_Finger_j1', 'left_hand_Middle_Finger_j2', 'left_hand_Middle_Finger_j3',
#             'left_hand_Ring_Finger_j1', 'left_hand_Ring_Finger_j2', 'left_hand_Ring_Finger_j3',
#             'left_hand_Pinky_Finger_j1', 'left_hand_Pinky_Finger_j2', 'left_hand_Pinky_Finger_j3',
#         ]
#         positions = [2.3551, -0.116810356, 1.3244985599999999,  # Dodaj resztę danych pozycji
#                      0.24958, 1.6755, 1.29502746,
#                      0.24958, 1.6755, 1.29502746,
#                      0.20362900000000003, 0.0, 0.0,
#                      0.22058500000000003, 0.0, 0.0]

#         self.joint_state_msg = JointState()
#         self.joint_state_msg.name = joint_names
#         self.joint_state_msg.position = positions

#         # Publikowanie danych pozycji co sekundę
#         self.timer = self.create_timer(1.0, self.publish_joint_state)

#     def publish_joint_state(self):
#         self.joint_state_publisher.publish(self.joint_state_msg)
#         self.get_logger().info('Publishing joint state')

# def main(args=None):
#     rclpy.init(args=args)
#     glove_position_node = GlovePositionNode()
#     rclpy.spin(glove_position_node)
#     glove_position_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from std_msgs.msg import Float64MultiArray

def main():
    rclpy.init()

    node = rclpy.create_node('publisher_node')
    publisher = node.create_publisher(Float64MultiArray, 'microros', 10)

    data = [0.1 * i for i in range(1, 20)]

    msg = Float64MultiArray()
    msg.data = data

    try:
        while rclpy.ok():
            publisher.publish(msg)
            node.get_logger().info('Wysłano wiadomość')
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
