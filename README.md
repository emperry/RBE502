ROS 2 Node:
RBE502/controller.py
Within the node, there is a function "group_controller". Treat this as your workspace. We can do something similar to what we did in class and have an if statement/switch case type situation to select what controller we have.


To run the code:
In one terminal, access the PX4 software

cd final-project/PX4-Autopilot

To start the Pixhawk Simulation:
make px4_sitl gz_x500


In another terminal,
MicroXRCEAgent udp4 -p 8888


In a third terminal:
source /opt/ros/humble/setup.bash
cd final-project/colcon_ws

The first time you build:
colcon build

Since you're not changing px4_msgs, after the first run you can
colcon build --packages-select RBE502

source install/setup.bash
ros2 run RBE502 controller