import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from evk4_msg.msg import EventArray  # Make sure to import your custom message type

import h5py
import numpy as np
import time 


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

        self.current_file = None
        self.start_time = None 

    def listener_callback(self, msg):
        # Process the received EventArray message
        for event in msg.events:
            self.get_logger().info(f"Event: x={event.x}, y={event.y}, polarity={event.polarity}, timestamp={event.ts.sec}.{event.ts.nanosec}")

    def destroy_node(self):
        # Close the current HDF5 file before shutting down if it exists
        if self.current_file is not None:
            self.current_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    evk4_subscriber = Evk4Subscriber()

    rclpy.spin(evk4_subscriber)

    # Clean up
    evk4_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
