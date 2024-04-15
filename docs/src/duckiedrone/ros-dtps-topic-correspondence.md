# DTPS-ROS topic correspondence


## IMU
In the duckiedrone the IMU topics are published to DTPS by the `flight_controller_node`.

The topics are picked up by the `imu_driver_node` in `dt-ros-interface` and relayed on the same topics as the duckiebot.

|DTPS topic name                |ROS topic name|
|:---                           |---:         |
|`/ "sensor" / "imu" / "raw"`   |Black        |