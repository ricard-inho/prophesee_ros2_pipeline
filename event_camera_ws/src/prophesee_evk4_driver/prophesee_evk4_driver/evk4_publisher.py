import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from builtin_interfaces.msg import Time # type: ignore
from evk4_msg.msg import EventArray, Event

from metavision_core.event_io.raw_reader import initiate_device
from metavision_core.event_io import EventsIterator
import time
from metavision_hal import DeviceDiscovery

class Evk4Publisher(Node):

    def __init__(self):
        super().__init__('evk4_publisher')

        self.declare_parameter("bias_file", "")
        self.declare_parameter("raw_file_to_read", "")

        self.biases_file = self.get_parameter("bias_file").get_parameter_value().string_value
        self.raw_file_to_read = self.get_parameter("raw_file_to_read").get_parameter_value().string_value

        self.publisher_cd_events = self.create_publisher(EventArray, 'topic_cd_event_buffer', 500)

        while not self.open_device():
            self.get_logger().info("Trying to open device...")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))


        self.process_events()



    def process_events(self):
        # Events iterator on Device
        mv_iterator = EventsIterator.from_device(device=self.device)
        self.device_height, self.device_width = mv_iterator.get_size()  # Camera Geometry
        while rclpy.ok():
            try:
                 # Process events
                for evs in mv_iterator:
                    self.publish_cd_event_buffer(evs)
                    self.get_logger().info(f"New buffer of size {evs.size} with timestamp range: "
                                   f"({evs[0]['t']},{evs[-1]['t']})")

            except KeyboardInterrupt:
                self.device.get_i_events_stream().stop_log_raw_data()
                break
        

    def publish_cd_event_buffer(self, event_buffer):
        """Publish the CD events to the ROS topic."""
        
        msg = EventArray()

        msg.header.stamp = Time(sec=int(event_buffer[0]['t'] * 1e-6), 
                                nanosec=int(event_buffer[0]['t'] % 1e6) * 1000)
        msg.height = self.device_height
        msg.width = self.device_width
        

        for event in event_buffer:
            ros_event = Event()
            ros_event.x = int(event['x'])  
            ros_event.y = int(event['y']) 
            ros_event.polarity = bool(event['p'])
            ros_event.ts = Time()
            ros_event.ts.nanosec = int((event['t']* 1e3) % 1e9) #nanoseconds
            msg.events.append(ros_event)

        self.publisher_cd_events.publish(msg)
        # self.get_logger().info(f'Publishing: ({msg})')



    def open_device(self) -> bool:
        try:
            if self.raw_file_to_read:
                self.device = DeviceDiscovery.open(self.raw_file_to_read)
                self.get_logger().info(f"[CONF] Reading from raw file: {self.event_file_path}")
            else:
                # HAL Device on live camera
                self.device = initiate_device("")
                self.get_logger().info("[CONF] Using live device stream.")

                # Start the recording
                if self.device.get_i_events_stream():
                    log_path = "/workspace/events/recording_" + time.strftime("%y%m%d_%H%M%S", time.localtime()) + ".raw"
                    
                self.device.get_i_events_stream().log_raw_data(log_path)

            return True
        except Exception as e:
            self.get_logger().warn(str(e))
            return False
        
    def destroy_node(self):
        self.device.get_i_events_stream().stop_log_raw_data()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    evk4_publisher = Evk4Publisher()

    rclpy.spin(evk4_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    evk4_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()