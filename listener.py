"""
import rclpy
from rclpy.node import Node
from hri_msgs.msg import IdsList
import hri_body_detect
import hri_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')
        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'camera_frame').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)
        
        self.body_sub = self.create_subscription(
        	IdsList,
        	'/humans/bodies/tracked',
        	self.make_gesture_cb,
        	10
        )


    def on_timer(self):
        from_frame_rel = self.target_frame
        to_frame_rel = 'body_izgbm'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.get_logger().info(
                f'transform of {from_frame_rel} to {to_frame_rel} : {t}')
            print(t)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    """
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from hri_msgs.msg import IdsList

class Tf2Listener(Node):

    def __init__(self):
        super().__init__('tf2_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

        self.body_sub = self.create_subscription(
            IdsList,
            '/humans/bodies/tracked',
            self.on_body_ids,  # separate subscriber callback
            10
        )
        self.latest_ids = []

    def on_body_ids(self, msg: IdsList):
        self.latest_ids = msg.ids

    def on_timer(self):
        if not self.latest_ids:
            return

        source_frame = 'camera_frame'
        body_id = self.latest_ids[0]
        target_frame = f"body_{body_id}"

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time()
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            self.get_logger().info(
                f"Transform from {source_frame} to {target_frame}: "
                f"translation=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}), "
                f"rotation=({r.x:.2f}, {r.y:.2f}, {r.z:.2f}, {r.w:.2f})"
            )
        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {source_frame} to {target_frame}: {ex}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = Tf2Listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""


import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from hri_msgs.msg import IdsList
from unitree_api.msg import Request
import math
import json


class Tf2Listener(Node):

    def __init__(self):
        super().__init__('tf2_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher to Unitree Sport API
        self.cmd_pub = self.create_publisher(Request, '/api/sport/request', 10)

        self.timer = self.create_timer(0.1, self.on_timer)  # 10 Hz

        self.body_sub = self.create_subscription(
            IdsList,
            '/humans/bodies/tracked',
            self.on_body_ids,
            10
        )

        self.latest_ids = []

    def Move(self, vx: float, vy: float, vyaw: float):
        # Construct the JSON payload
        cmd_json = {
            "x": vx,
            "y": vy,
            "z": vyaw
        }

        req = Request()
        req.parameter = json.dumps(cmd_json)
        req.header.identity.api_id = 1008  # ROBOT_SPORT_API_ID_MOVE

        self.cmd_pub.publish(req)

        self.get_logger().info(
            f"Sent Move command: x={vx:.2f}, y={vy:.2f}, yaw={vyaw:.2f}"
        )

    def on_body_ids(self, msg: IdsList):
        self.latest_ids = msg.ids

    def on_timer(self):

        if not self.latest_ids:
            self.get_logger().info("No body ID detected. Stopping robot.")
            self.Move(0.0, 0.0, 0.0)
            return

        source_frame = 'camera_frame'
        body_id = self.latest_ids[0]
        target_frame = f"body_{body_id}"

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time()
            )

            t = transform.transform.translation

            dx = t.x
            dy = t.y
            
            # Compute angle to person in horizontal plane
            angle_to_person = math.atan2(dy, dx)
            print("angle:",math.degrees(angle_to_person))            
            if (angle_to_person>=-0.12 and angle_to_person<=0.12):
               self.Move(0.0, 0.0, 0.0)
               return
            # Proportional control
            gainrot = 0.7              #initially at 1.5 then 1.0 then 0.7 then 0.5 then settled for 0.7
            vyaw = gainrot * angle_to_person

            # Clamp rotation speed to robot limits
            vyaw = max(-1.1, min(1.1, vyaw))         #initially at 0.8 too slow 1.5 too fast 1.2 still fast 1.1 seems the best
            # No linear movement yet
            gaintrain = 0.6
            vx = dx * gaintrain
            vx = max(-0.3, min(0.3, vx))
            vy = 0.0

           # if t.x <= 2:
           #     self.Move(-0.3, 0.0, vyaw)
           #    return

            self.Move(vx, vy, vyaw)

            self.get_logger().info(
                f"Rotating to face person: vyaw={vyaw:.2f} rad/s "
                f"(angle={math.degrees(angle_to_person):.1f}°)"
            )
            
            print("dx:",dx,"    dy:",dy)

        except TransformException as ex:
            self.get_logger().info(
                f"Could not transform {source_frame} to {target_frame}: {TransformException}"
            )
            self.Move(0.0, 0.0, 0.0)  # Stop movement if no valid transform
        

def main(args=None):
    rclpy.init(args=args)
    node = Tf2Listener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
