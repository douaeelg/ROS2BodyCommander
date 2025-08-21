
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class WebcamWithCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('webcam_camera_info_node')

        # Change this if your external webcam is not /dev/video0
        self.cap = cv2.VideoCapture(0)  # ← Use 1 for external webcam
        if not self.cap.isOpened():
            raise RuntimeError("Could not open external webcam (try /dev/video1 or another index)")

        # Publishers
        self.image_pub = self.create_publisher(Image, '/image', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)

        # Convert OpenCV → ROS images
        self.bridge = CvBridge()

        # Set up CameraInfo message
        self.cam_info = CameraInfo()
        self.cam_info.header.frame_id = 'camera_frame'

        self.cam_info.width = 640
        self.cam_info.height = 480

        self.cam_info.distortion_model = 'plumb_bob'
        self.cam_info.d = [-1.62687928e-01,  2.15509238e+00,  1.37261966e-03,  5.05766574e-03,-7.34723922e+00]

        self.cam_info.k = [587.57774819,   0.0,         340.81662247,
 	  0.0,         586.25109604, 256.62145097,
	  0.0,           0.0,           1.0        ]

        self.cam_info.p =[6.33495136e+02, 3.34328988e+01, 2.42836848e+02, 2.81724469e+04,
 		 1.02411115e+02, 6.31680683e+02, 6.03284415e+00, 1.31869037e+04,
		 1.68257458e-01, 3.70248153e-01, 9.13567585e-01, 8.08127609e+01]

        # Timer for publishing (every 0.5 seconds)
        self.timer = self.create_timer(0.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam.")
            return

        # Timestamp
        now = self.get_clock().now().to_msg()

        # Convert to ROS Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_frame'

        # Update CameraInfo timestamp and frame
        self.cam_info.header.stamp = now

        # Publish
        self.image_pub.publish(img_msg)
        self.info_pub.publish(self.cam_info)

        self.get_logger().info("Published webcam image + CameraInfo (:)) .")

    def destroy(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamWithCameraInfoPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

