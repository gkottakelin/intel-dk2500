# jetarm_description

Initial ROS 2 description package for displaying the measured JetArm prototype in RViz.

The URDF joint angles are model/URDF angles, not raw servo positions. Use
`project/src/arm_model.py` to convert raw positions and to validate
`joint_angles -> tcp_pose`.

The launch file uses a small joint-state mapper for J6. The GUI slider shows the
real raw bus-servo range: `0` is fully open, `700` is geometrically closed, and
`700..1000` stays visually closed while representing extra gripping force on
real hardware. The display URDF exposes one `joint6_gripper` visual joint; the
opposite scissor arm is a mimic joint, not another control parameter. Both arms
start at J5. At raw `0`, the two fingers are fully open in one straight line.

## Build In Ubuntu 22.04 / ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cp -r /mnt/d/jetarm/project/ros2_ws/src/jetarm_description ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select jetarm_description
source install/setup.bash
```

## Display In RViz

```bash
ros2 launch jetarm_description display.launch.py
```

The default launch uses `joint_state_publisher_gui`, so install the GUI package
first if it is missing:

```bash
sudo apt update
sudo apt install -y ros-humble-joint-state-publisher-gui
ros2 launch jetarm_description display.launch.py
```

To launch without the slider GUI:

```bash
ros2 launch jetarm_description display.launch.py use_gui:=false
```

## FK Verification From The Project Root

```powershell
python project\src\arm_model.py --angle-deg J1=0 --angle-deg J2=0 --angle-deg J3=0 --angle-deg J4=0 --angle-deg J5=0 --json
python project\src\arm_model.py --angle-deg J1=90 --angle-deg J2=90 --json
```

At all-zero model angles, `tcp_link` should be at `[0, 0, 0.527]` meters.
