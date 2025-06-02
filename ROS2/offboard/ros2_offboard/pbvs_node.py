import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

from .vision_utils import estimate_homography, recover_from_homography, rodriguez, pbvs_controller

REF_IMAGE_NAME = '/home/noe/catkin_ws/src/offboard/image_reference/reference_image.jpg'

class PBVSNode(Node):
    def __init__(self):
        super().__init__('pbvs_node')
        self.declare_parameter('reference_image', REF_IMAGE_NAME)
        self.ref_image = cv2.imread(
            self.get_parameter('reference_image').get_parameter_value().string_value,
            cv2.IMREAD_COLOR)
        if self.ref_image is None:
            self.get_logger().error('Could not load reference image')
            raise RuntimeError('Failed to load reference image')
        self.current_image = None
        self.bridge = CvBridge()

        self.homography_pub = self.create_publisher(Float64MultiArray, '/homograpy_numerical', 10)
        self.ck_t_ct_pub = self.create_publisher(Float64MultiArray, '/ck_t_ct', 10)
        self.n_pub = self.create_publisher(Float64MultiArray, '/n_plane_vector', 10)
        self.Uv_pub = self.create_publisher(Float64MultiArray, '/Uv', 10)
        self.Uw_pub = self.create_publisher(Float64MultiArray, '/Uw', 10)
        self.d_pub = self.create_publisher(Float64, '/d_value', 10)

        self.sub = self.create_subscription(Image, '/iris/usb_cam/image_raw', self.image_callback, 10)

        self.K = np.array([[554.382713, 0.0, 320.0],
                           [0.0, 554.382713, 240.0],
                           [0.0, 0.0, 1.0]])
        self.counter = 0
        self.homography_solution = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.current_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def timer_callback(self):
        if self.current_image is None:
            return
        H = estimate_homography(self.ref_image, self.current_image, self.K, self.counter)
        if H is not None:
            R, t, n, d, self.homography_solution = recover_from_homography(
                H, self.K, self.counter, self.homography_solution)
            u = rodriguez(R)
            Uv, Uw = pbvs_controller(R, t, u, lambdav=0.5, lambdaw=0.5)

            self.homography_pub.publish(Float64MultiArray(data=H.flatten()))
            self.ck_t_ct_pub.publish(Float64MultiArray(data=t.flatten()))
            self.n_pub.publish(Float64MultiArray(data=n.flatten()))
            self.d_pub.publish(Float64(data=d))
            self.Uv_pub.publish(Float64MultiArray(data=Uv.flatten()))
            self.Uw_pub.publish(Float64MultiArray(data=Uw.flatten()))
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PBVSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
