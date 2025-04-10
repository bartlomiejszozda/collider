### DESCRIPTION
Project to make some fun by steering drone using ROS2, ardupilot SITL and gazebo simulation
Collider is starting a drone, than steering it in order to hit a flying object.
Steering is based on object tracking which uses camera image (visual servoing).

Difficulties:\
Steering should be more robust, however iris is not powerful and it doesn't manage leaning too much while flying fast or climbing up quickly.
Decided to do simple steering algorithms, but it still manages to hit stationary/slightly moving targets.
Need to use racing drone or power up iris, then refine steering algorithms.

Tested various off-the-shelf trackers, but they barely manage to track an object from a moving camera.
Usually off-the-shelf trackers lost target when drone rotates too much
Currently decided to bypass tracking problem by using BlackSphere and using BlackSpotTracker
Need to implement dedicated solution


### TODO
#### Code related:
unit tests \
RcOverride and ModeChanger implement Proxy Design Pattern \
c++ Node \
#### Main functionality:
Implement [advanced tracking](https://www.computer.org/csdl/journal/ec/2022/03/09519550/1wc8Vbe1r7G) \
Implement advanced steering, take drone intertia into account \
#### Additional Functionalities:
Steering from gamepad \
Following a target moving on the ground \

### DESIGN PATTERNS:
Cant use many design patterns However, some design patterns are applied:
OpenCvTracker implement an Adapter \
RcOverrider implement a Facade \
SteeringUnit, TrackerManager implement Observer (by using ROS topics) \

### INSTALLATION:
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

**Open terminal:**\
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
Run Pycharm If you want to edit code\
`cd /opt/pycharm-2024.3.5/bin/`\
`sh pycharm.sh`

### Problems worth to mention:
**Gazebo doesn't show image:**\
probably old gazebo process still running\
`ps aux | grep gazebo`
`kill -9 <PID>`

**Pycharm doesn't see ros dependencies:**\
source ros before running pycharm\
`cd ~/ardu_ws && source /opt/ros/humble/setup.bash && source install/setup.bash`

