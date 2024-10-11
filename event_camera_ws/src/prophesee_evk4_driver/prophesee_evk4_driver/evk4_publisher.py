import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from builtin_interfaces.msg import Time # type: ignore
from evk4_msg.msg import EventArray, Event


from metavision_hal import DeviceDiscovery

class Evk4Publisher(Node):

    def __init__(self):
        super().__init__('evk4_publisher')

        self.declare_parameter("bias_file", "")
        self.declare_parameter("raw_file_to_read", "")

        self.biases_file = self.get_parameter("bias_file").get_parameter_value().string_value
        self.raw_file_to_read = self.get_parameter("raw_file_to_read").get_parameter_value().string_value

        self.publisher_cd_events = self.create_publisher(EventArray, 'topic_cd_event_buffer', 500)

        while not self.open_camera():
            self.get_logger().info("Trying to open camera...")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))


        self.process_events()



    def process_events(self):
        while rclpy.ok():
            try:
                ret = self.i_eventsstream.poll_buffer()
                if ret < 0:
                    break
                elif ret > 0:
                    raw_data = self.i_eventsstream.get_latest_raw_data()
                    if raw_data is not None:
                        self.i_eventsstreamdecoder.decode(raw_data)
            except KeyboardInterrupt:
                break
        
        self.i_eventsstream.stop()

    def print_cd_events(self, event_buffer):
        if event_buffer.size > 0:
            self.get_logger().info(f"New buffer of size {event_buffer.size} with timestamp range: "
                                   f"({event_buffer[0]['t']},{event_buffer[-1]['t']})")
            self.publish_cd_event_buffer(event_buffer)

    def publish_cd_event_buffer(self, event_buffer):
        """Publish the CD events to the ROS topic."""
        
        msg = EventArray()

        msg.header.stamp = Time(sec=int(event_buffer[0]['t'] * 1e-6), 
                                nanosec=int(event_buffer[0]['t'] % 1e6) * 1000)
        msg.height = 0#TODO
        msg.width = 0#TODO
        

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



    def open_camera(self) -> bool:
        try:
            if self.raw_file_to_read:
                self.camera = DeviceDiscovery.open(self.raw_file_to_read)
                self.get_logger().info(f"[CONF] Reading from raw file: {self.event_file_path}")
            else:
                self.camera = DeviceDiscovery.open("")
                self.get_logger().info("[CONF] Using live camera stream.")
            
            self.i_cddecoder = self.camera.get_i_event_cd_decoder()
            self.i_cddecoder.add_event_buffer_callback(self.print_cd_events)
            self.i_eventsstreamdecoder = self.camera.get_i_events_stream_decoder()
            self.i_eventsstream = self.camera.get_i_events_stream()
            self.i_eventsstream.start()

            return True
        except Exception as e:
            self.get_logger().warn(str(e))
            return False
        
    def __del__(self):
        self.i_eventsstream.stop()


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