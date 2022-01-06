# ros2modbus
Control servo control via modbus and ROS2

git clone https://github.com/ros2/teleop_twist_keyboard
Build:
colcon build --packages-select ros2modbus teleop_twist_keyboard

In terminal 1:
ros2 run ros2modbus ros2modbus_subscriber
In terminal 2:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
