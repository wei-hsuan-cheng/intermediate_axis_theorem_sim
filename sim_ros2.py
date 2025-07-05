import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Point, Vector3, Pose, TransformStamped
from std_msgs.msg import Header, ColorRGBA
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
import numpy as np
import time

class IntermediateAxisTheoremVisualizer(Node):
    def __init__(self):
        super().__init__('intermediate_axis_theorem_visualizer')

        self.box_pub_ = self.create_publisher(Marker, '/markers/box_marker', 10)
        self.wire_pub_ = self.create_publisher(Marker, '/markers/wire_marker', 10)
        self.br = TransformBroadcaster(self)
        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.setup_dynamics()
        self.current_step = 0
        self.max_steps = len(self.t_eval)

        self.get_logger().info('\nIntermediate Axis Theorem Visualizer started.\n----------')

    
    def setup_dynamics(self):
        # === Store box dimensions as class attributes ===
        self.lx, self.ly, self.lz = 1.0, 0.2, 0.05  # [m]

        density = 1.0  # [kg/m^3]
        volume = self.lx * self.ly * self.lz
        mass = density * volume

        # === Principal moments of inertia ===
        I1 = (1/12) * mass * (self.ly**2 + self.lz**2)  # around x-axis
        I2 = (1/12) * mass * (self.lx**2 + self.lz**2)  # around y-axis
        I3 = (1/12) * mass * (self.lx**2 + self.ly**2)  # around z-axis
        self.inertia_values = (I1, I2, I3)

        # Initial conditions
        
        # # Mainly along intermediate axis (I2), with small perturbations along the other two axes
        # omega_y = 2.0  # [Hz]
        # omega0 = np.array([omega_y * 0.01, omega_y * 1.0, omega_y * 0.001]) * 2 * np.pi
        
        # Mainly along omega1 (I1), with small perturbations along the other two axes
        omega_x = 2.0  # [Hz]
        omega0 = np.array([omega_x * 1.0, omega_x * 0.01, omega_x * 0.001]) * 2 * np.pi
        
        # # Mainly along omega3 (I3), with small perturbations along the other two axes
        # omega_z = 2.0  # [Hz]
        # omega0 = np.array([omega_z * 0.01, omega_z * 0.001, omega_z * 1.0]) * 2 * np.pi
        
        # Time span
        t_span = (0, 60) # [s]
        self.t_eval = np.linspace(*t_span, int((t_span[1] - t_span[0]) / self.timer_period))

        def euler_eq(t, omega):
            w1, w2, w3 = omega
            return [
                ((I2 - I3) / I1) * w2 * w3,
                ((I3 - I1) / I2) * w3 * w1,
                ((I1 - I2) / I3) * w1 * w2
            ]

        sol = solve_ivp(euler_eq, t_span, omega0, t_eval=self.t_eval)
        self.omega_t = sol.y.T
        self.dt = self.t_eval[1] - self.t_eval[0]
        self.quats = self.integrate_quaternion(self.omega_t, self.t_eval)
        
        self.get_logger().info(
            f"\nBox: {self.lx} x {self.ly} x {self.lz} [m^3]\nInertia: I1={I1:.6f}, I2={I2:.6f}, I3={I3:.6f} [kg*m^2]\n----------"
        )


    def quat_mult(self, q, r):
        w0, x0, y0, z0 = q
        w1, x1, y1, z1 = r
        return np.array([
            w0*w1 - x0*x1 - y0*y1 - z0*z1,
            w0*x1 + x0*w1 + y0*z1 - z0*y1,
            w0*y1 - x0*z1 + y0*w1 + z0*x1,
            w0*z1 + x0*y1 - y0*x1 + z0*w1
        ])

    def integrate_quaternion(self, omega_t, t_eval):
        q = np.array([1, 0, 0, 0])  # w, x, y, z
        qs = [q.copy()]
        for i in range(1, len(t_eval)):
            w = omega_t[i]
            w_quat = np.concatenate([[0], w])
            dq = 0.5 * self.quat_mult(q, w_quat)
            q = q + dq * self.dt
            q = q / np.linalg.norm(q)
            qs.append(q.copy())
        return np.array(qs)
    
    def pub_box_marker(self, quat, now):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = now
        marker.ns = "rigid_object"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(
                x=float(quat[1]),
                y=float(quat[2]),
                z=float(quat[3]),
                w=float(quat[0])
            )
        )
        marker.scale = Vector3(x=self.lx, y=self.ly, z=self.lz)
        marker.color = ColorRGBA(r=1.0, 
                                 g=1.0, 
                                 b=1.0, 
                                 a=0.5)
        marker.lifetime.sec = 0

        self.box_pub_.publish(marker)
    
    def pub_wire_marker(self, quat, now):
        # === Wireframe edges marker ===
        # Prepare transform for rotation of vertices
        rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # x,y,z,w

        # Box half-sizes
        hx, hy, hz = self.lx / 2, self.ly / 2, self.lz / 2

        # 8 corners of box (centered at origin)
        corners = np.array([
            [-hx, -hy, -hz],  # 0
            [ hx, -hy, -hz],  # 1
            [ hx,  hy, -hz],  # 2
            [-hx,  hy, -hz],  # 3
            [-hx, -hy,  hz],  # 4
            [ hx, -hy,  hz],  # 5
            [ hx,  hy,  hz],  # 6
            [-hx,  hy,  hz],  # 7
        ])

        # Rotate corners by current quaternion
        rotated_corners = rot.apply(corners)
        
        wire_marker = Marker()
        wire_marker.header.frame_id = 'world'
        wire_marker.header.stamp = now
        wire_marker.ns = "rigid_object_wireframe"
        wire_marker.id = 1
        wire_marker.type = Marker.LINE_LIST
        wire_marker.action = Marker.ADD

        wire_marker.scale.x = 0.01  # line width

        # Define edges as pairs of corner indices
        edges = [
            # (0,1), 
            # (1,2), 
            (2,3), 
            # (3,0),  # bottom rectangle edges
            
            # (4,5), 
            # (5,6), 
            (6,7), 
            # (7,4),  # top rectangle edges
            
            # (0,4), 
            # (1,5), 
            (2,6), 
            (3,7),  # vertical edges
        ]

        wire_color = ColorRGBA(r=1.0, 
                               g=0.0, 
                               b=0.0, 
                               a=1.0)

        for (start, end) in edges:
            p_start = rotated_corners[start]
            p_end = rotated_corners[end]
            wire_marker.points.append(Point(x=float(p_start[0]), y=float(p_start[1]), z=float(p_start[2])))
            wire_marker.points.append(Point(x=float(p_end[0]), y=float(p_end[1]), z=float(p_end[2])))
            wire_marker.colors.append(wire_color)
            wire_marker.colors.append(wire_color)

        wire_marker.lifetime.sec = 0

        self.wire_pub_.publish(wire_marker)
    
    def pub_tf(self, quat, now):
        # === TF Broadcast: world -> rigid_object ===
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'rigid_object'
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = Quaternion(
            x=float(quat[1]),
            y=float(quat[2]),
            z=float(quat[3]),
            w=float(quat[0])
        )
        self.br.sendTransform(tf_msg)


    '''Main loop'''
    def timer_callback(self):
        if self.current_step >= self.max_steps:
            self.get_logger().info('\nAnimation restarted.\n----------')
            # sleep for 1 [s]
            time.sleep(1)
            self.current_step = 0  # reset to start


        quat = self.quats[self.current_step]
        now = self.get_clock().now().to_msg()

        self.pub_box_marker(quat, now)
        self.pub_wire_marker(quat, now)
        self.pub_tf(quat, now)

        self.current_step += 1
    


def main(args=None):
    rclpy.init(args=args)
    node = IntermediateAxisTheoremVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
