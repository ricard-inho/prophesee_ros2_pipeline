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
        

        current_time = time.time()
        current_second = int(current_time)

        # Check if we need to create a new file
        if self.start_time is None or current_second != self.start_time:
            if self.current_file is not None:
                self.current_file.close()

            self.start_time = current_second

            # Use the timestamp of the first event to create the file name
            first_event_timestamp = msg.events[0].ts.sec  # Get the timestamp from the first event
            file_name = f"/workspace/events/events_{first_event_timestamp}.h5"

            self.current_file = h5py.File(file_name, 'w')

            # Set file attributes
            self.current_file.attrs['format'] = 'HDF5 Event File'
            self.current_file.attrs['camera_integrator_name'] = 'YourCameraIntegrator'
            self.current_file.attrs['plugin_integrator_name'] = 'YourPluginIntegrator'

            # Create groups for event data
            cd_group = self.current_file.create_group('CD')
            ext_trigger_group = self.current_file.create_group('EXT_TRIGGER')

            event_dtype = np.dtype([
                ('x', 'u4'),  # unsigned int (32 bits)
                ('y', 'u4'),  # unsigned int (32 bits)
                ('p', 'u1'),  # unsigned char (1 bit for polarity)
                ('t', 'u8')   # unsigned long long (64 bits for timestamp)
            ])

            # Create datasets for CD events
            cd_events_data = cd_group.create_dataset('events', (0,), maxshape=(None,), dtype=event_dtype)  # Placeholder for events
            cd_indexes_data = cd_group.create_dataset('indexes', (0,), maxshape=(None,), dtype='u8')  # Placeholder for indexes

            # Initialize datasets
            self.cd_events = cd_events_data
            self.cd_indexes = cd_indexes_data
        
        self.get_logger().info(f"Received message with {len(msg.events)} events. Timestamps ({msg.events[0].ts.nanosec},{msg.events[-1].ts.nanosec}")

        # Prepare data for HDF5
        new_events = []
        new_indexes = []

        for event in msg.events:
            self.get_logger().info(f"Event: x={event.x}, y={event.y}, polarity={event.polarity}, timestamp={event.ts.sec}.{event.ts.nanosec}")
            # Prepare the event data in ECF format
            timestamp = event.ts.sec * 1_000_000 + event.ts.nanosec // 1000  # Convert timestamp to microseconds
            new_events.append((event.x, event.y, event.polarity, timestamp))

        # Expand the datasets and add new events
        current_event_size = self.cd_events.shape[0]
        new_event_size = current_event_size + len(new_events)

        # Resize datasets
        self.cd_events.resize((new_event_size,))
        self.cd_indexes.resize((new_event_size,))

        # Store the events and their indexes
        for idx, (x, y, polarity, ts) in enumerate(new_events):
            self.cd_events[current_event_size + idx] = (x, y, polarity, ts)  # Store event data
            self.cd_indexes[current_event_size + idx] = current_event_size + idx  # Store index


        # for event in msg.events:
        #     self.get_logger().info(f"Event: x={event.x}, y={event.y}, polarity={event.polarity}, timestamp={event.ts.sec}.{event.ts.nanosec}")

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
