# collider
Companion Computer steering drone based on image. 
It is a ROS2 Node that communicates with ardupilot.

when gazebo freaks out with some logs (drained packets etc.) you should kill old process
ps aux | grep gazebo
kill -9 PID
