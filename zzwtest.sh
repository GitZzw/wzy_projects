#!/bin/bash
### BEGIN INIT INFO
# Provides: test
# Short-Description: mlkk test
# Description:  
# Required-Start: $remote_fs $local_fs
# Required-Stop: $remote_fs $local_fs
# Default-Start: 2 3 4 5
# Default-Stop: 0 1 6
### END INIT INFO
source ~/.bashrc   ### zzw_source
source ~/catkin_ws/devel/setup.bash

#px4_run
roslaunch px4_realsense_bridge bridge_mavros.launch

### zzw_nodes_run
cd /home/gf/zzw/catkin_ws/src/wzy_projects/scripts
python3 /home/gf/zzw/catkin_ws/src/wzy_projects/scripts/yolo_realsense_tcp.py & sleep 5
python3 /home/gf/zzw/catkin_ws/src/wzy_projects/scripts/yolo_client.py & sleep 15

roslaunch pnp_target_node pnp_target_node_time.launch
exit 0
