"""import rclpy
from rclpy.node import Node
from hri_msgs.msg import IdsList, Gesture

class SimpleCommander(Node):
    def __init__(self):
        super().__init__('simple_commander')

        self.target_id = None  # Will store the first detected body ID
        self.gesture_subscriber_created = False

        # Subscribe to tracked body IDs
        self.body_sub = self.create_subscription(
            IdsList,
            '/humans/bodies/tracked',
            self.body_list_cb,
            10
        )

    def body_list_cb(self, msg: IdsList):
        if not msg.ids:
            return  # No bodies detected

        if self.gesture_subscriber_created:
            return  # Already subscribed to gesture, ignore further updates

        # Get the first ID and subscribe to its gesture topic
        self.target_id = msg.ids[0]
        gesture_topic = f'/humans/bodies/{self.target_id}/gesture'
        self.get_logger().info(f'Subscribing to gesture topic: {gesture_topic}')

        self.create_subscription(
            Gesture,
            gesture_topic,
            self.gesture_cb,
            10
        )
        self.gesture_subscriber_created = True

    def gesture_cb(self, msg: Gesture):
        self.get_logger().info(f'Received gesture from {self.target_id}: {msg.gesture}')

def main():
    rclpy.init()
    node = SimpleCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""
import rclpy
import time
from rclpy.node import Node
from hri_msgs.msg import IdsList, Gesture
from unitree_api.msg import Request

class SimpleCommander(Node):
    def __init__(self):
        super().__init__('simple_commander')

        """ Add timer for gesture detection to avoid false positives"""
        
        # Will hold the ID of the first tracked person
        self.target_id = None

        # Prevent subscribing to multiple gesture topics
        self.gesture_subscriber_created = False

        # Publisher to send commands to the robot
        self.publisher_ = self.create_publisher(Request, '/api/sport/request', 10)

        # Subscribe to tracked body IDs
        self.body_sub = self.create_subscription(
            IdsList,
            '/humans/bodies/tracked',
            self.body_list_cb,
            10
        )

        # Gesture detection delay for assurance
        self.last_gesture_time = {}
        self.debounce_seconds = 10.0	# can be adjusted
        
        self.gesture_sub = None

    def body_list_cb(self, msg: IdsList):
        # Only proceed if no gesture subscription is active and there is at least one ID
        
        """
        if self.gesture_subscriber_created:
            self.get_logger().info(f'already subscribed to gesture topic :{self.gesture_topic}')
            return
        """
        
        """ Could cause a problem if the body id changes, ---> Configure dynamic subscription """

        # If ids list is empty, log and return 'no human detected'
        if not msg.ids :
            self.get_logger().info(f'no human detected')
            return
        
        # 
        if msg.ids[0]==self.target_id :
            self.get_logger().info(f'no resubscription needed {self.target_id} didn\'t change')
            return

        # Take the first ID in the list
        self.destroy_subscription(self.gesture_sub)
        self.target_id = msg.ids[0]
        print(f"ID list: {msg.ids}")
        self.gesture_topic = f'/humans/bodies/{self.target_id}/gesture'
        self.get_logger().info(f'Subscribing to gesture topic: {self.gesture_topic}')

        # Subscribe to that person's gesture topic
        self.gesture_sub = self.create_subscription(
            Gesture,
            self.gesture_topic,
            self.gesture_cb,
            10
        )
        #self.gesture_subscriber_created = True

    def gesture_cb(self, msg: Gesture):
        # Get the gesture ID (integer)
        gesture_id = msg.gesture
        self.get_logger().info(f"Received gesture from {self.target_id}: {gesture_id}")

        now = time.time()
        last_time = self.last_gesture_time.get(self.target_id, 0)
        self.get_logger().warn(str(self.last_gesture_time))

        # Map gesture ID to corresponding action
        gesture_actions = {
            8: self.send_hello,     # Open Palm
            10: self.send_sit,      # Thumb Don
            11: self.send_stand,    # Thumb Up
            12: self.send_dance,     # Victory
            7: self.send_Stretch    # Fist
        }

        # Call the matching function if it exists
        action = gesture_actions.get(gesture_id)
        self.get_logger().warn(str(now - last_time))

        if now - last_time >= self.debounce_seconds:
            self.last_gesture_time[self.target_id] = now
            if action:
                action()
            else:
                self.get_logger().warn(f"Unknown gesture ID: {gesture_id}")
        else:
            self.get_logger().info(f"[{self.target_id}] Gesture {gesture_id} ignored (debounce)")

    # --- Action Methods ---

    def send_hello(self):
        msg = Request()
        msg.header.identity.api_id = 1016  # Replace with correct API ID if needed
        self.publisher_.publish(msg)
        self.get_logger().info("🖐️ Hello command sent")

    def send_sit(self):
        msg = Request()
        msg.header.identity.api_id = 1005  # Replace with correct API ID if needed
        self.publisher_.publish(msg)
        self.get_logger().info("🪑 Sit command sent")

    def send_stand(self):
        msg = Request()
        msg.header.identity.api_id = 1004  # Replace with correct API ID if needed
        self.publisher_.publish(msg)
        self.get_logger().info("🧍 Stand command sent")

    def send_dance(self):
        msg = Request()
        msg.header.identity.api_id = 1022  # Replace with correct API ID if needed
        self.publisher_.publish(msg)
        self.get_logger().info("💃 Dance command sent")



    def send_Stretch(self):
        msg = Request()
        msg.header.identity.api_id = 1017  # Replace with correct API ID if needed
        self.publisher_.publish(msg)
        self.get_logger().info("🧘 Stretch command sent")

        
# Entry point
def main():
    rclpy.init()
    node = SimpleCommander()
    rclpy.spin(node)         # Keeps the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

