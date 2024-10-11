import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from evk4_msg.msg import EventArray  # Make sure to import your custom message type


class Evk4Subscriber(Node):
    def __init__(self):
        super().__init__('evk4_subscriber')

        # Create a subscriber to the topic_cd_event_buffer
        self.subscriber_cd_events = self.create_subscription(
            EventArray,
            'topic_cd_event_buffer',
            self.listener_callback,
            500
        )

        self.get_logger().info("Subscriber to topic_cd_event_buffer initialized.")

    def listener_callback(self, msg):
        # Process the received EventArray message
        self.get_logger().info(f"Received message with {len(msg.events)} events.")
        
        for event in msg.events:
            self.get_logger().info(f"Event: x={event.x}, y={event.y}, polarity={event.polarity}, timestamp={event.ts.sec}.{event.ts.nanosec}")

def main(args=None):
    rclpy.init(args=args)

    evk4_subscriber = Evk4Subscriber()

    rclpy.spin(evk4_subscriber)

    # Clean up
    evk4_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
