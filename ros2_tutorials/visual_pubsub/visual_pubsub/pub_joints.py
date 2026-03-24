import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

L1 = 0.35
L2 = 0.25

def dh_matrix(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def forward_kinematics(q1, q2, d3, q4):
    T1 = dh_matrix(q1, 0,  L1, 0)
    T2 = dh_matrix(q2, 0,  L2, 0)
    T3 = dh_matrix(0,  d3, 0,  0)
    T4 = dh_matrix(q4, 0,  0,  0)
    return T1 @ T2 @ T3 @ T4

def print_H(H, logger):
    px, py, pz = H[0,3], H[1,3], H[2,3]
    lines = [
        "┌─────────────────────────────────────────────┐",
        "│        Matriz Homogénea H (base → TCP)       │",
        f"│  {H[0,0]:8.4f}  {H[0,1]:8.4f}  {H[0,2]:8.4f}  │ {H[0,3]:7.4f} │",
        f"│  {H[1,0]:8.4f}  {H[1,1]:8.4f}  {H[1,2]:8.4f}  │ {H[1,3]:7.4f} │",
        f"│  {H[2,0]:8.4f}  {H[2,1]:8.4f}  {H[2,2]:8.4f}  │ {H[2,3]:7.4f} │",
        f"│  {H[3,0]:8.4f}  {H[3,1]:8.4f}  {H[3,2]:8.4f}  │ {H[3,3]:7.4f} │",
        "└──────────────────────────────────────────────┘",
        f"  TCP → X:{px:.4f}m  Y:{py:.4f}m  Z:{pz:.4f}m",
    ]
    logger.info("\n" + "\n".join(lines))


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Posición inicial
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.t = 0.0 

    def publish_joint_states(self):
        self.t += 0.1

    
        self.q[0] = 1.0  * np.sin(self.t * 0.5)   # joint1: ±1 rad
        self.q[1] = 0.8  * np.sin(self.t * 0.3)   # joint2: ±0.8 rad
        self.q[2] = -0.1 * abs(np.sin(self.t * 0.4))  # joint3: 0 a -0.1m
        self.q[3] = 0.5  * np.sin(self.t * 0.7)   # joint4: ±0.5 rad

        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name     = ['joint1', 'joint2', 'joint3', 'joint4']
        msg.position = self.q

        self.publisher_.publish(msg)

        H = forward_kinematics(*self.q)

        self.get_logger().info(
            f"Juntas → "
            f"q1={np.degrees(self.q[0]):6.1f}°  "
            f"q2={np.degrees(self.q[1]):6.1f}°  "
            f"d3={self.q[2]*100:5.1f}cm  "
            f"q4={np.degrees(self.q[3]):6.1f}°"
        )
        print_H(H, self.get_logger())


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()