### Description
A learning project for autonomous drone steering which utilize ROS2, ardupilot SITL and gazebo simulation.
The system launches a drone and navigates towards the selected flying target.
Drone control is achieved through visual servoing based on object tracking from onboard camera images.

### Design Patterns:
- OpenCvTracker implement an Adapter
- RcOverrider implement a Facade
- SteeringUnit or TrackerManager implements Observers (by using ROS topics)
- (TODO) RcOverride and ModeChanger implement Proxy Design Pattern

### TODO
Main functionality:
- Use racing drone or power up iris. Currently it struggle leaning and flying fast or climbing up quickly.
- Implement [advanced tracking](https://www.computer.org/csdl/journal/ec/2022/03/09519550/1wc8Vbe1r7G). Opencv trackers lost target when camera rotates. Currently decided to bypass tracking problem by implementing black spot tracker and black sphere as a target.
- Implement advanced steering, take drone intertia into account

Additional Functionalities:
- Steering from gamepad
- Following a target moving on the ground

Code related:
- Unit tests
- Auto simulation tests
- C++ Node

### Installation:
follow [ardupilot ros manual](https://ardupilot.org/dev/docs/ros.html) (using docker)
- Install ROS 2
- ROS 2 with SITL
- ROS 2 Interfaces
- ROS 2 with SITL in Gazebo

run docker with display and nvidia: \
`docker run -it --env DISPLAY=$DISPLAY  --volume /tmp/.X11-unix:/tmp/.X11-unix --runtime=nvidia --gpus all --name ardupilot-dds3 ardupilot/ardupilot-dev-ros2`

### Run:
**start stopped docker**:\
`docker start ardupilot-dds3`

**Open 3 terminals:**\
enter docker\
`sudo xhost +Local:* && docker container exec -it ardupilot-dds3 /bin/bash`\
source ros\
`cd ~/ardu_ws && source /opt/ros/humble/setup.bash && source install/setup.bash`

**1st terminal:**\
start simulation\
`ros2 launch ardupilot_gz_bringup iris_runway.launch.py`

**2nd terminal:**\
move black sphere up - based on [wrench](https://community.gazebosim.org/t/is-there-a-way-to-give-initial-velocity-or-force-to-the-model-in-the-gazebo/2397/6)
`gz topic -t "/world/map/wrench/persistent" -m gz.msgs.EntityWrench -p "entity: {name: 'sphere', type: MODEL}, wrench: {force: {x: 0, y:0, z:9.80}}" && gz topic -t "/world/map/wrench" -m gz.msgs.EntityWrench -p "entity: {name: 'sphere', type: MODEL}, wrench: {force: {x: 0, y:0, z:40000}}" && sleep 3 && gz topic -t "/world/map/wrench" -m gz.msgs.EntityWrench -p "entity: {name: 'sphere', type: MODEL}, wrench: {force: {x: 0, y:0, z:-40000}}"`\
build collider\
`colcon build --packages-select collider`\
run collider (wait till simulation start using GPS)\
`ros2 run collider collider_bin`

**3rd terminal:**\
Run Pycharm if you want to edit the code\
`cd /opt/pycharm-2024.3.5/bin/`\
`sh pycharm.sh`

### Problems worth to mention:
**Gazebo doesn't show image:**\
probably old gazebo process still running\
`ps aux | grep gazebo`\
`kill -9 <PID>`

**Pycharm doesn't see ros dependencies:**\
source ros before running pycharm\
`cd ~/ardu_ws && source /opt/ros/humble/setup.bash && source install/setup.bash`
