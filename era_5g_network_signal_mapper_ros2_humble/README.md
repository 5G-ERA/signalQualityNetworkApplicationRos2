# signal_quality_network_app_ROS2_Humble_dev_Branch
Code developed  under ROS2 Humble

## Installation


Install ROS2 Humble environment.

Here is sugested source for instalation 
```
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```


After instalation check if the ROS Humble is installed
```
printenv ROS_DISTRO
```

Output should be: 

Humble

If other distro is displayed then humble, run this command
```
source /opt/ros/humble/setup.bash
```

Or open Home Directory and find file .bashrc, if is not visible press ctr+H to display hidden files
and replace/comment the 
"source /opt/ros/(your_ros_distribution)/setup.bash"

and add following to ".bashrc" file:
```
source /opt/ros/humble/setup.bash
```


Create Workspace for ROS2. "ros2_ws" name is not mandatory, you can add different name or if is already available Workspace for ros2, just add to your workspace downloaded code.
```
mkdir -p ros2_ws/src
```

Go inside ros2_ws directory
```
cd ros2_ws/
```


Download the code by pressing green button in top right corrner "<> Code" by pressing Download ZIP from folding menu.

Or Git-clone, you should be in "src" folder when run command

```
git clone https://github.com/5G-ERA/signal_quality_network_application_ros2.git
```

Unzip and copy Folder "era_5g_network_signal_mapper_ros2" 
which is inside of this folder: "signal_quality_network_application-era_5g_network_signal_mapper_ros2_humble_dev"
to folder: src which is in folder: "ros2_ws"


From terminal run command for compiling ros2 package.
Make sure you in the "ros2_ws/" directory in terminal, not in "ros2_ws/src/" or other location
```
colcon build
```
Source Workspace
```
source ~/ros2_ws/install/setup.bash
```

Or open Home Directory and find file .bashrc, if is not visible press ctr+H to display hidden files
and replace/comment the 
"source "other source"/setup.bash"

and add following to ".bashrc" file:
```
source ~/ros2_ws/install/setup.bash
```

## Run Semantic Map pkg
While Simulation in ROS2 is running - RUN network_signal_mapper_ros2

```
ros2 run era_5g_network_signal_mapper_ros2 signal_mapper
```
In another terminal run
```
ros2 run era_5g_network_signal_mapper_ros2 sub_signal_mapper
```

To save map as **ocupancyGrid map** run, this rosnode is for merging the map with pointcloud and colours specified prior will be seen as obstacle.

```
ros2 run era_5g_network_signal_mapper_ros2 costmap_translate
```

save map current folder
```
ros2 run nav2_map_server map_saver_cli -t map_Semantic
```

save map and specify location of saving
```
ros2 run nav2_map_server map_saver_cli -t map_Semantic -f ~/robot_ws/src/test/maps/test04-08-2023
```

“~/robot_ws/src/test/maps/test04-08-2023” is location for saving map and name of the map.

Location “~/robot_ws/src/test/maps/”

Name of the map test04-08-2023”

### Load saved map

```
ros2 service call /semantic_map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /robot_ws/src/test/maps/test04-08-2023.yaml}"
```

### Get current colour of the semantic map
In order t use this feature the pakage must be run before the pointcloud is loaded either via saving and loading semantic poitcloud as a static file or running signal_mapper package.
from termianl:
```
ros2 run era_5g_network_signal_mapper_ros2 current_color
```

The topic "/current_color" will publish current color of the location of the robot


### ROS Service for saving and loading semantic poitcloud as a static file

from termianl:

```
cd era_5g_network_signal_mapper_ros2"
```
From this location run in terminal to run node for starting service server:

```
source /opt/ros/humble/setup.bash
```

```
python3 tools.py
```
Save poitcloud as a file

```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```
```
ros2 service call /save_pointcloud std_srvs/Trigger "{}"
```
To load sematic pointcloud from file run:
```
ros2 service call /load_and_publish_pointcloud std_srvs/Trigger "{}"
```
And select topic: "/loaded_pointcloud"

### ROS Service for for updating confidence of semanitc PCL from static file


```
python3 tools.py
```

```
ros2 service call /append_pcl pcl_interfaces/srv/AppendPcl "{pcl1: pcl1.txt, pcl_out: pcl3.txt}"
```
### ROS Service for requestic pcl from an area (from static file)


```
python3 tools.py
```

The area of interest must be specified:

It needs 4 coordinates corners

min_x = left side border line

max_x = right side border line

min_y = bottom border line

max_y = top border line


pcl1 represent filename of semanitc  source map

pcl_out represent filename of target semantic map, if file does not exists, new file will be created, if file exists, the content will be overwritten.


```
ros2 service call /update_pcl pcl_interfaces/srv/PartialMapRequest "{pcl1: pcl1.txt, min_x: -1.3, max_x: 1, min_y: -2, max_y: -1, pcl_out: pcl_out.txt}"
```
### ROS Service for combining two semantic PCL files in another specified file


```
python3 tools.py
```

```
ros2 service call /append_pcl pcl_interfaces/srv/AppendPcl "{pcl1: pcl1.txt, pcl2: pcl2.txt, pcl_out: pcl3.txt}"
```

### Multirobot support
client:
```
docker run --network host --rm -e ROS_DOMAIN_ID=21 -e NETAPP_ADDRESS=http://192.168.0.80:5896 -e TOPICS_TO_SERVER='[{"name":"/tb1/amcl_pose","type":"geometry_msgs/msg/PoseWithCovarianceStamped","qos":{"depth":1,"history":1,"reliability":1,"durability":1}},{"name":"/tb2/amcl_pose","type":"geometry_msgs/msg/PoseWithCovarianceStamped","qos":{"depth":1,"history":1,"reliability":1,"durability":1}}]' -e TOPICS_FROM_SERVER='[{"name": "/semantic_pcl","type": "sensor_msgs/msg/PointCloud2","compression": "lz4"}]' but5gera/ros2_relay_client:1.0.0
```

server:
```
docker run --network host --rm -e ROS_DOMAIN_ID=30 -e TOPICS_FROM_CLIENT='[{"name":"/tb1/amcl_pose","type":"geometry_msgs/msg/PoseWithCovarianceStamped","qos":{"depth":1,"history":1,"reliability":1,"durability":1}},{"name": "/tb2/amcl_pose","type":"geometry_msgs/msg/PoseWithCovarianceStamped","qos":{"depth":1,"history":1,"reliability":1,"durability":1}}]' -e TOPICS_TO_CLIENT='[{"name": "/semantic_pcl","type": "sensor_msgs/msg/PointCloud2","compression": "lz4"}]' but5gera/ros2_relay_server:1.0.0
```