import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from builtin_interfaces.msg import Time # type: ignore
from evk4_msg.msg import EventArray, Event

from metavision_core.event_io.raw_reader import initiate_device
from metavision_core.event_io import EventsIterator
import time
import numpy as np
import threading
from queue import Queue
import hydra
from hydra.core.config_store import ConfigStore
from dataclasses import dataclass, field
import os


@dataclass
class Config:
    log_path: str = "/workspace/events"

cs = ConfigStore.instance()
cs.store(name="config", node=Config)

class Evk4Publisher(Node):

    def __init__(self, cfg: Config):
        super().__init__('evk4_publisher')

        self.cfg = cfg

        self.publisher_cd_events = self.create_publisher(EventArray, 'topic_cd_event_buffer', 500)

        self.event_queue = Queue(maxsize=10000)
        self.stop_event = threading.Event()

        while not self.open_device():
            self.get_logger().info("Trying to open device...")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        self.get_logger().info("[INFO] Starting publisher loop")
        self.publisher_thread = threading.Thread(target=self.publisher_loop)
        self.publisher_thread.start()
        
        self.get_logger().info("[INFO] Starting process events loop")
        self.processing_thread = threading.Thread(target=self.process_events)
        self.processing_thread.start()


    def publisher_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                evs = self.event_queue.get()
                self.publish_cd_event_buffer(evs)
            except Queue.Empty:
                continue
        

    def process_events(self):
        # Events iterator on Device
        mv_iterator = EventsIterator.from_device(device=self.device)
        self.device_height, self.device_width = mv_iterator.get_size()  # Camera Geometry
        try:
            for evs in mv_iterator:
                if self.stop_event.is_set():
                    break
                self.event_queue.put(evs)  

        except KeyboardInterrupt:
            self.device.get_i_events_stream().stop_log_raw_data()
            
        

    def publish_cd_event_buffer(self, event_buffer):
        
        if event_buffer.size > 0:
            start_time = time.time()

            msg = EventArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height = self.device_height
            msg.width = self.device_width

            # Convert and assign arrays directly
            msg.x = event_buffer['x'].astype(np.int16).tolist()
            msg.y = event_buffer['y'].astype(np.int16).tolist()
            msg.polarity = event_buffer['p'].astype(bool).tolist()
            # msg.ts = event_buffer['t'].astype(np.int64).tolist()
            secs = (event_buffer['t'] * 1e-6).astype(np.int32)
            nsecs = ((event_buffer['t'] % 1e6) * 1000).astype(np.int32)
            msg.ts = [Time(sec=int(s), nanosec=int(ns)) for s, ns in zip(secs, nsecs)]
            # msg.ts.nanosec = int((t['t']* 1e3) % 1e9 for t in event_buffer['t'])

            self.publisher_cd_events.publish(msg)

            
            self.get_logger().debug(f"New buffer of size {event_buffer.size} with timestamp range: "
                                    f"({event_buffer[0]['t']},{event_buffer[-1]['t']}) - Queue size {self.event_queue.qsize()}")
            end_time = time.time()
            print(f"Execution time publish_cd_event_buffer: {end_time - start_time:.4f} seconds - Queue size {self.event_queue.qsize()}")



    def open_device(self) -> bool:
        try:
            # HAL Device on live camera
            self.device = initiate_device("")
            self.get_logger().info("[CONF] Using live device stream.")

            # Start the recording
            if self.device.get_i_events_stream():
                # log_path = "/workspace/events/recording_" + time.strftime("%y%m%d_%H%M%S", time.localtime()) + ".raw"
                log_path = f"{self.cfg.log_path}/recording_" + time.strftime("%y%m%d_%H%M%S", time.localtime()) + ".raw"
                    
                self.device.get_i_events_stream().log_raw_data(log_path)

            return True
        except Exception as e:
            self.get_logger().warn(str(e))
            return False
        
    def destroy_node(self):
        self.device.get_i_events_stream().stop_log_raw_data()
        super().destroy_node()

@hydra.main(config_path="/workspace/config", config_name="evk4_cfg", version_base="1.2")
def main(cfg: Config):
    rclpy.init()

    evk4_publisher = Evk4Publisher(cfg)

    try:
        rclpy.spin(evk4_publisher)
    except KeyboardInterrupt:
        evk4_publisher.get_logger().info("KeyboardInterrupt received, shutting down...")
        evk4_publisher.stop_event.set()  # Signal threads to stop
        evk4_publisher.publisher_thread.join()
        evk4_publisher.processing_thread.join()
    finally:
        evk4_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()