# prophesee_ros2_pipeline

```
./docker/build_docker.sh
./docker/run_docker.sh
```


```
cd event_camera_ws
colcon build
source install/setup.bash
ros2 run prophesee_evk4_driver evk4_subscriber 
ros2 run prophesee_evk4_driver evk4_publisher  
```