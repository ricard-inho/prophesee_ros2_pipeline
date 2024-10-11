# prophesee_ros2_pipeline

```
docker build -t ros2-humble .
docker run -it --rm ros2-humble
docker exec -it ros2-humble bash

```


```
cd event_camera_ws
colcon build
source install/setup.bash
ros2 run prophesee_evk4_driver evk4_subscriber 
ros2 run prophesee_evk4_driver evk4_publisher  
```